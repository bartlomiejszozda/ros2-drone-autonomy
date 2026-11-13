"""
Answer questions about the camera image with an AI model through OpenRouter.

    ros2 action send_goal /collider/ask collider_interfaces/action/Ask \
      "{question: 'Where is the red car?'}" --feedback

Every question gets the newest camera frame attached, plus the earlier questions and
answers of this conversation (text only). One question runs at a time, so the
conversation stays in order. The node only answers; it sends nothing to the vehicle.
"""

import base64
import os
import signal
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout
from threading import Lock

import cv2
import rclpy
from collider_interfaces.action import Ask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.assistant.Conversation import Conversation, HistoryLog
from collider.src.assistant.OpenRouterClient import AssistantError, OpenRouterClient
from collider.src.node_time import age, now_sec
from collider.src.tracker.TrackerNode import SUPPORTED_ENCODINGS, image_to_frame


def image_to_data_url(msg: Image, quality: int = 85) -> str:
    """JPEG-encode an rgb8/bgr8 image as a data URL, the form OpenRouter accepts inline."""
    frame = image_to_frame(msg)
    if msg.encoding == 'rgb8':
        # OpenCV expects BGR; without this a red car would arrive blue.
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise AssistantError('could not encode the camera image as JPEG')
    return 'data:image/jpeg;base64,' + base64.b64encode(jpeg.tobytes()).decode('ascii')


class AssistantNode(Node):
    def __init__(self, client=None, **kwargs):
        super().__init__('assistant', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        self._model = parameter('model', 'qwen/qwen3.8-27b')
        self._max_image_age = float(parameter('max_image_age_sec', 2.0))
        timeout = float(parameter('request_timeout_sec', 60.0))
        self._conversation = Conversation(int(parameter('max_history_turns', 1000)),
                                          parameter('system_prompt', ''))
        self._log = HistoryLog(parameter('history_file', '~/.ros/collider/ai_history.jsonl'))
        api_key = os.environ.get('OPENROUTER_API_KEY', '')
        self._client = client if client is not None else OpenRouterClient(api_key, timeout)
        if client is None and not api_key:
            self.get_logger().warning('OPENROUTER_API_KEY is not set; questions will fail')
        # The HTTP request blocks, so it runs in its own thread while the action keeps
        # publishing feedback and can still be cancelled.
        self._http = ThreadPoolExecutor(max_workers=1)

        # Always subscribed: the newest frame is ready the moment a question arrives.
        # Its own group, so images keep arriving while a question waits for the model.
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        image_topic = parameter('image_topic', '/iris/static_camera_sensor/image')
        self.create_subscription(Image, image_topic, self._on_image, latest_only,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self._image = None
        self._image_time = 0.0

        self._busy = False
        self._lock = Lock()
        self._server = ActionServer(
            self, Ask, '/collider/ask', execute_callback=self._execute,
            goal_callback=self._accept, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'Answering questions on /collider/ask with {self._model}; '
            f'images from {image_topic}; history in {self._log.path or "(off)"}')

    def _on_image(self, msg):
        # Only a reference is kept; encoding happens once per question, not per frame.
        self._image = msg
        self._image_time = now_sec(self)

    def _accept(self, request):
        if not request.question.strip():
            self.get_logger().warning('Rejecting question: it is empty')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warning('Rejecting question: another one is being answered')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._answer(goal_handle)
        finally:
            with self._lock:
                self._busy = False

    def _answer(self, goal_handle):
        request = goal_handle.request
        model = request.model or self._model
        if request.new_conversation:
            self._conversation.clear()
        result = Ask.Result(model=model, history_turns=self._conversation.turns)
        try:
            messages = self._conversation.messages(request.question, self._fresh_image_url())
            answer = self._wait_for_model(goal_handle, model, messages, result)
        except AssistantError as error:
            result.message = str(error)
            goal_handle.abort()
            self.get_logger().error(f'Question failed: {error}')
        else:
            if answer is None:
                result.message = 'cancelled; the answer is discarded'
                goal_handle.canceled()
            else:
                result.success = True
                result.answer = answer
                self._conversation.add(request.question, answer)
                goal_handle.succeed()
                self.get_logger().info(f'Q: {request.question}\nA: {answer}')
        self._log.append(question=request.question, answer=result.answer,
                         success=result.success, message=result.message, model=model,
                         latency_sec=round(result.latency_sec, 2),
                         history_turns=result.history_turns,
                         new_conversation=request.new_conversation)
        return result

    def _fresh_image_url(self) -> str:
        image = self._image
        if image is None:
            raise AssistantError('no camera image received yet; is the simulator running?')
        if age(now_sec(self), self._image_time) > self._max_image_age:
            raise AssistantError(
                f'newest camera image is older than {self._max_image_age:g} s')
        if image.encoding not in SUPPORTED_ENCODINGS:
            raise AssistantError(f'unsupported image encoding {image.encoding}')
        return image_to_data_url(image)

    def _wait_for_model(self, goal_handle, model, messages, result):
        """Return the answer, or None if cancelled. Raises AssistantError on failure."""
        future = self._http.submit(self._client.chat, model, messages)
        # Wall clock on purpose: the request takes real time, also when the simulation
        # is paused or runs at another speed.
        start = time.monotonic()
        while True:
            try:
                return future.result(timeout=0.5)
            except FutureTimeout:
                pass
            finally:
                # Also when the request failed: a slow error is worth knowing about.
                result.latency_sec = time.monotonic() - start
            if goal_handle.is_cancel_requested:
                # The HTTP request cannot be interrupted; its answer is ignored.
                result.latency_sec = time.monotonic() - start
                return None
            goal_handle.publish_feedback(Ask.Feedback(
                stage=f'waiting for {model}', elapsed_sec=time.monotonic() - start))

    def destroy_node(self):
        self._http.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    # Three threads: a question waiting for the model, images, and cancel requests.
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = AssistantNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch);
        # ignore the second one so it cannot interrupt the clean-up below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
