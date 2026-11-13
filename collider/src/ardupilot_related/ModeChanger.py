from ardupilot_msgs.srv import ModeSwitch
from rclpy.node import Node


class ModeChanger:
    mode_map = {
        "stabilize": 0,
        "acro": 1,
        "alt_hold": 2,
        "guided": 4,
        "rtl": 6,
        "land": 9,
        "brake": 17,
    }

    def __init__(self, node: Node, service_name='/ap/v1/mode_switch',
                 callback_group=None):
        # Reuse one client for all asynchronous mode-change attempts.
        self.client = node.create_client(
            ModeSwitch, service_name, callback_group=callback_group)

    def call_mode_async(self, mode: str):
        """Send a mode request immediately and return its Future."""
        request = ModeSwitch.Request(mode=self.mode_map[mode])
        return self.client.call_async(request)
