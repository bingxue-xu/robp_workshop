import rclpy
from rclpy.node import Node
from datetime import datetime
import json
import os

class BaseTest(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.robot_name = 'unset'
        self.domain_id = -1
        self.json_folder = None
        self.json_path = None

    def _init_json(self):
        os.makedirs(self.json_folder, exist_ok=True)
        self.json_path = os.path.join(
            self.json_folder,
            f"{self.robot_name}_domain_{self.domain_id}.json"
        )
        if not os.path.isfile(self.json_path):
            skeleton = {
                "robot_name": self.robot_name,
                "domain_id": self.domain_id,
                "results": {},
                "last_updated": None
            }
            with open(self.json_path, 'w') as f:
                json.dump(skeleton, f, indent=2)

    def update_config(self, robot_name=None, domain_id=None, json_folder=None):

        if robot_name:
            self.robot_name = robot_name
        if domain_id is not None:
            self.domain_id = domain_id
        if json_folder:
            self.json_folder = os.path.expanduser(json_folder)
        else:
            self.json_folder = os.path.expanduser('~/hardware_test_results')

        self._init_json()

    def save_result(self, hardware_key, passed, detail=""):
        if not self.json_path:
            self.get_logger().error("JSON path not configured. Call update_config() first.")
            return

        with open(self.json_path, 'r') as f:
            data = json.load(f)
        data.setdefault("results", {})
        data["results"][hardware_key] = {
            "status": "PASS" if passed else "FAIL",
            "detail": detail
        }
        data["last_updated"] = datetime.now().isoformat()
        with open(self.json_path, 'w') as f:
            json.dump(data, f, indent=2)

        if passed:
            self.get_logger().info(f"[{hardware_key}] PASS ({detail})-> result saved at {self.json_path}")
        else:
            self.get_logger().warn(f"[{hardware_key}] FAIL ({detail}) -> result saved at {self.json_path}")

        self.get_logger().info(f"Results saved to {self.json_path}")

    def wait_for_topic(self, topic_name, timeout_s):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout_s:
            topics = [t[0] for t in self.get_topic_names_and_types()]
            if topic_name in topics:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def run_test(self, hardware_key, msg_type, topic_name, timeout_s, **kwargs):
        if not self.json_path:
            self.get_logger().error("Missing JSON path. Call update_config() first.")
            return False

        if not self.wait_for_topic(topic_name, timeout_s):
            detail = f"topic {topic_name} did not appear"
            self.get_logger().error(detail)
            self.save_result(hardware_key, False, detail)
            return False

        # sub only once
        self.received_msg = None
        sub = self.create_subscription(msg_type, topic_name, self._generic_callback, 10)
        start = self.get_clock().now()
        while self.received_msg is None and \
              (self.get_clock().now() - start).nanoseconds / 1e9 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.received_msg is None:
            detail = f"topic {topic_name} did not publish any message"
            self.get_logger().error(detail)
            self.save_result(hardware_key, False, detail)
            return False

        try:
            passed, detail = self.is_passing(self.received_msg, **kwargs)
        except Exception as e:
            passed = False
            detail = str(e)
            self.get_logger().error(f"is_passing exception: {detail}")

        self.save_result(hardware_key, passed, detail)
        return passed

    def _generic_callback(self, msg):
        if self.received_msg is None:
            self.received_msg = msg

    def is_passing(self, msg, **kwargs):
        raise NotImplementedError("Subclasses must implement is_passing()")

    def main_spin_and_exit(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        try:
            ok = self.perform_test()
        except Exception as e:
            self.get_logger().error(f"perform_test exception: {e}")
            ok = False

        rclpy.spin_once(self, timeout_sec=0.1)
        rclpy.shutdown()
        exit(0 if ok else 1)
