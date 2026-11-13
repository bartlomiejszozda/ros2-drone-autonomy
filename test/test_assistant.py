"""Tests for the assistant node, its conversation history and the OpenRouter client."""

import base64
import json
import threading
import time
from uuid import uuid4

import cv2
import numpy as np
import pytest
import requests
import rclpy
from action_msgs.msg import GoalStatus
from collider_interfaces.action import Ask
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.assistant.AssistantNode import AssistantNode, image_to_data_url
from collider.src.assistant.Conversation import Conversation, HistoryLog
from collider.src.assistant.OpenRouterClient import AssistantError, OpenRouterClient

WIDTH, HEIGHT = 64, 48

# --- Conversation and HistoryLog ------------------------------------------------


def test_conversation_sends_system_prompt_earlier_turns_and_image_last():
    conversation = Conversation(max_turns=10, system_prompt='You see a drone camera.')
    conversation.add('q1', 'a1')
    messages = conversation.messages('q2', image_url='data:image/jpeg;base64,xx')
    assert [m['role'] for m in messages] == ['system', 'user', 'assistant', 'user']
    assert messages[1]['content'] == 'q1'  # earlier turns are text only
    assert messages[-1]['content'] == [
        {'type': 'text', 'text': 'q2'},
        {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,xx'}}]


def test_conversation_keeps_only_the_newest_turns():
    conversation = Conversation(max_turns=2)
    for n in range(5):
        conversation.add(f'q{n}', f'a{n}')
    assert conversation.turns == 2
    texts = [m['content'] for m in conversation.messages('next')[:-1]]
    assert texts == ['q3', 'a3', 'q4', 'a4']
    conversation.clear()
    assert conversation.messages('next') == [
        {'role': 'user', 'content': [{'type': 'text', 'text': 'next'}]}]


def test_history_log_appends_json_lines(tmp_path):
    log = HistoryLog(str(tmp_path / 'sub' / 'history.jsonl'))
    log.append(question='q1', answer='a1')
    log.append(question='q2', answer='ż')
    lines = (tmp_path / 'sub' / 'history.jsonl').read_text(encoding='utf-8').splitlines()
    records = [json.loads(line) for line in lines]
    assert [r['question'] for r in records] == ['q1', 'q2']
    assert records[1]['answer'] == 'ż'
    assert 'time' in records[0]


def test_history_log_with_empty_path_writes_nothing():
    HistoryLog('').append(question='q')  # must not raise or create files


# --- OpenRouterClient -----------------------------------------------------------


class FakeResponse:
    def __init__(self, status_code, body):
        self.status_code = status_code
        self._body = body
        self.text = body if isinstance(body, str) else json.dumps(body)

    def json(self):
        if isinstance(self._body, str):
            raise ValueError('not JSON')
        return self._body


def client_returning(response=None, error=None, api_key='key'):
    calls = []

    def post(url, **kwargs):
        calls.append(kwargs)
        if error is not None:
            raise error
        return response
    return OpenRouterClient(api_key, timeout_sec=5.0, post=post), calls


def test_client_returns_answer_and_sends_key_and_model():
    client, calls = client_returning(FakeResponse(200, {
        'choices': [{'message': {'content': 'centre at (320, 240)'}}]}))
    assert client.chat('qwen/x', [{'role': 'user', 'content': 'hi'}]) == 'centre at (320, 240)'
    assert calls[0]['headers']['Authorization'] == 'Bearer key'
    assert calls[0]['json']['model'] == 'qwen/x'
    assert calls[0]['timeout'] == 5.0


@pytest.mark.parametrize('response, error, expected', [
    (FakeResponse(401, {'error': {'message': 'No auth credentials found'}}), None,
     'HTTP 401: No auth credentials found'),
    (FakeResponse(200, {'error': {'message': 'provider down'}}), None, 'provider down'),
    (FakeResponse(502, '<html>bad gateway</html>'), None, 'not JSON'),
    (FakeResponse(200, {'choices': []}), None, 'unexpected response'),
    (FakeResponse(200, {'choices': [{'message': {'content': None}}]}), None, 'empty answer'),
    (None, requests.Timeout(), 'no answer within 5 s'),
    (None, requests.ConnectionError('offline'), 'request failed'),
])
def test_client_turns_failures_into_readable_errors(response, error, expected):
    client, _ = client_returning(response, error)
    with pytest.raises(AssistantError, match=expected):
        client.chat('m', [])


def test_client_without_key_does_not_send():
    client, calls = client_returning(FakeResponse(200, {}), api_key='')
    with pytest.raises(AssistantError, match='OPENROUTER_API_KEY'):
        client.chat('m', [])
    assert calls == []


# --- image encoding -------------------------------------------------------------


def make_image(encoding='rgb8', red=(255, 0, 0)):
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:] = red if encoding == 'rgb8' else red[::-1]
    msg = Image()
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = encoding
    msg.step = WIDTH * 3
    msg.data = frame.tobytes()
    return msg


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_red_image_stays_red_in_jpeg(encoding):
    url = image_to_data_url(make_image(encoding))
    assert url.startswith('data:image/jpeg;base64,')
    jpeg = np.frombuffer(base64.b64decode(url.split(',', 1)[1]), dtype=np.uint8)
    blue, green, red = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)[HEIGHT // 2, WIDTH // 2]
    assert red > 200 and green < 50 and blue < 50


# --- the node -------------------------------------------------------------------


class FakeModel:
    """Stands in for OpenRouterClient: records what it was sent, answers 'answer N'."""

    def __init__(self):
        self.calls = []
        self.delay = 0.0
        self.error = None

    def chat(self, model, messages):
        self.calls.append((model, messages))
        time.sleep(self.delay)
        if self.error is not None:
            raise AssistantError(self.error)
        return f'answer {len(self.calls)}'


class AssistantRig:
    def __init__(self, tmp_path, max_image_age=5.0):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/assistant_test_' + uuid4().hex
        self.model = FakeModel()
        self.history_file = tmp_path / 'history.jsonl'
        self.assistant = AssistantNode(client=self.model, context=self.context,
                                       parameter_overrides=[
                                           Parameter('image_topic', value=prefix + '/image'),
                                           Parameter('history_file',
                                                     value=str(self.history_file)),
                                           Parameter('max_image_age_sec', value=max_image_age)])
        self.client_node = Node('assistant_test_client', context=self.context)
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_pub = self.client_node.create_publisher(Image, prefix + '/image', latest_only)
        self.client = ActionClient(self.client_node, Ask, '/collider/ask')
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.assistant)
        self.executor.add_node(self.client_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        if not self.client.wait_for_server(timeout_sec=10):
            self.close()
            raise AssertionError('assistant action server not discovered')

    def publish_image(self):
        deadline = time.monotonic() + 3.0
        while self.image_pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'image subscriber not discovered'
            time.sleep(0.02)
        self.image_pub.publish(make_image())
        while self.assistant._image is None:
            assert time.monotonic() < deadline, 'image not received'
            time.sleep(0.02)

    def send(self, question, **fields):
        return wait(self.client.send_goal_async(Ask.Goal(question=question, **fields)))

    def ask(self, question, **fields):
        handle = self.send(question, **fields)
        assert handle.accepted
        return wait(handle.get_result_async())

    def close(self):
        self.executor.shutdown()
        self.assistant.destroy_node()
        self.client_node.destroy_node()
        rclpy.shutdown(context=self.context)


def wait(future, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


@pytest.fixture
def rig(tmp_path):
    rig = AssistantRig(tmp_path)
    yield rig
    rig.close()


def test_answers_with_image_and_remembers_the_conversation(rig):
    rig.publish_image()
    first = rig.ask('where is the red car?')
    assert first.status == GoalStatus.STATUS_SUCCEEDED
    assert first.result.success
    assert first.result.answer == 'answer 1'
    assert first.result.model == 'qwen/qwen3.8-27b'
    assert first.result.history_turns == 0
    second = rig.ask('and the blue one?', model='other/model')
    assert second.result.history_turns == 1
    model, messages = rig.model.calls[1]
    assert model == 'other/model'
    assert [m['content'] for m in messages[:2]] == ['where is the red car?', 'answer 1']
    assert messages[-1]['content'][1]['image_url']['url'].startswith('data:image/jpeg')


def test_new_conversation_forgets_but_the_file_keeps_everything(rig):
    rig.publish_image()
    rig.ask('q1')
    result = rig.ask('q2', new_conversation=True).result
    assert result.history_turns == 0
    assert len(rig.model.calls[1][1]) == 1  # only the new question
    records = [json.loads(line) for line in rig.history_file.read_text().splitlines()]
    assert [(r['question'], r['answer']) for r in records] == [
        ('q1', 'answer 1'), ('q2', 'answer 2')]
    assert records[1]['new_conversation'] is True


def test_without_image_the_question_fails_and_the_model_is_not_called(rig):
    result = rig.ask('where is the red car?')
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'no camera image' in result.result.message
    assert rig.model.calls == []


def test_model_error_is_reported_and_not_remembered(rig):
    rig.publish_image()
    rig.model.error = 'HTTP 401: No auth credentials found'
    result = rig.ask('q1')
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.message == 'HTTP 401: No auth credentials found'
    rig.model.error = None
    assert rig.ask('q2').result.history_turns == 0


def test_cancel_discards_the_answer_and_busy_rejects_a_second_question(rig):
    rig.publish_image()
    rig.model.delay = 1.5
    handle = rig.send('slow question')
    assert handle.accepted
    time.sleep(0.2)
    assert not rig.send('second question').accepted
    wait(handle.cancel_goal_async())
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    rig.model.delay = 0.0
    time.sleep(1.5)  # let the discarded request finish
    assert rig.ask('q2').result.history_turns == 0


def test_empty_question_is_rejected(rig):
    assert not rig.send('   ').accepted
