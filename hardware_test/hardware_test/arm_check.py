import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from hardware_test.base_test import BaseTest

from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

import time

class ArmCheck(BaseTest):
    def __init__(self):
        super().__init__(node_name='arm_check')
        self.hardware_key = 'Arm'
        self.servo_pos = None
        self.servo_temp = None
        self.servo_volt = None
        self.feedback_timeout = 4.0
        self.move_wait = 2.5  # seconds, a bit longer than 2000ms
        self.feedback_received = {'pos': False, 'temp': False, 'volt': False}

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter('json_folder', '').value

        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

    def check_nonzero(self, arr, name):
        if not arr or all(x == 0 for x in arr):
            detail = f"All values in {name} are zero or missing: {arr}"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

    def feedback_cb_pos(self, msg):
        self.servo_pos = msg
        self.feedback_received['pos'] = True

    def feedback_cb_temp(self, msg):
        self.servo_temp = msg
        self.feedback_received['temp'] = True

    def feedback_cb_volt(self, msg):
        self.servo_volt = msg
        self.feedback_received['volt'] = True

    def wait_for_feedback(self):
        # Wait for all feedback topics to be received
        start = self.get_clock().now().seconds_nanoseconds()[0]
        while not all(self.feedback_received.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - start > self.feedback_timeout:
                missing = [k for k, v in self.feedback_received.items() if not v]
                raise RuntimeError(f"Timeout waiting for feedback topics: {missing}")

    def move_servos(self, positions, times):
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label='', size=0, stride=0)], data_offset=0)
        msg.data = positions + times
        self.servo_cmd_pub.publish(msg)
        self.get_logger().info(f"Published servo move: {msg.data}")
        time.sleep(self.move_wait)

    def perform_test(self):
        self.setup_parameters()

        # Subscribe to feedback topics
        self.create_subscription(JointState, '/servo_pos_publisher', self.feedback_cb_pos, 10)
        self.create_subscription(Float64MultiArray, '/servo_temp_publisher', self.feedback_cb_temp, 10)
        self.create_subscription(Int16MultiArray, '/servo_volt_publisher', self.feedback_cb_volt, 10)
        self.servo_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)

        # Wait for feedback
        self.wait_for_feedback()

        # Check feedback values
        self.check_nonzero(getattr(self.servo_pos, 'position', []), 'servo_pos_publisher')
        self.check_nonzero(getattr(self.servo_temp, 'data', []), 'servo_temp_publisher')
        self.check_nonzero(getattr(self.servo_volt, 'data', []), 'servo_volt_publisher')

        # Move all servos upright in 2000ms
        upright_pos = [12000] * 6
        upright_time = [2000] * 6
        self.move_servos(upright_pos, upright_time)

        # Move each servo individually and check movement
        results = {}
        for i in range(6):
            servo_name = f"servo_{i+1}"
            # Record initial position
            initial_pos = list(getattr(self.servo_pos, 'position', []))
            # Command movement
            pos = [-1] * 6
            pos[i] = 7000 if i % 2 == 0 else 24000
            self.move_servos(pos, upright_time)
            # Wait and get new feedback
            rclpy.spin_once(self, timeout_sec=0.1)
            moved_pos = list(getattr(self.servo_pos, 'position', []))
            # Command back to center
            pos[i] = 12000
            self.move_servos(pos, upright_time)
            rclpy.spin_once(self, timeout_sec=0.1)
            # Check if servo moved
            try:
                before = initial_pos[i]
                after = moved_pos[i]
                moved = abs(after - before) > 100  # threshold in centi-degrees
            except Exception as e:
                moved = False
                before = after = None
            if moved:
                results[servo_name] = {
                    'moved': True,
                    'detail': f"Moved from {before} to {after}"
                }
            else:
                results[servo_name] = {
                    'moved': False,
                    'detail': f"No movement detected (from {before} to {after})"
                }
                self.get_logger().error(f"{servo_name} failed to move: {results[servo_name]['detail']}")

        detail = {"feedback_check": "passed", "servo_results": results}
        return all(v['moved'] for v in results.values()), detail

def main(args=None):
    rclpy.init(args=args)
    node = ArmCheck()
    node.main_spin_and_exit()

if __name__ == '__main__':
    main()