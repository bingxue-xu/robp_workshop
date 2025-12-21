import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from hardware_test.base_test import BaseTest

from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

import time
import threading

class ArmCheck(BaseTest):
    def __init__(self):
        super().__init__(node_name='arm_check')
        self.hardware_key = 'Arm'
        self.servo_pos = None
        self.servo_temp = None
        self.servo_volt = None
        self.feedback_timeout = 15.0  # Increased timeout for phidgets initialization
        self.move_wait = 1.0  # wait a bit after moving servos
        self.feedback_received = {'pos': False, 'temp': False, 'volt': False}

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter(
            'json_folder', '~/dd2419/workshop_ws/src/hardware_test/test_results/components_test'
        ).value
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
        msg.data = positions + times # single command: ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int16MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [1000,-1,2000,-1,-1,-1,2000,2000,2000,2000,2000,2000]}"

        self.servo_cmd_pub.publish(msg)
        self.get_logger().info(f"Published servo move: {msg.data}")
        time.sleep(self.move_wait)

    def check_servo_movement(self, i, initial_pos, results, servo_name):
        # Wait up to 2 seconds for feedback to change
        timeout = 2.0
        interval = 0.2
        elapsed = 0.0
        moved = False
        before = initial_pos[i]
        after = before
        time.sleep(interval)  # Ensure we have time to receive initial position
        while elapsed < timeout:
            rclpy.spin_once(self, timeout_sec=interval)
            moved_pos = list(getattr(self.servo_pos, 'position', []))
            if len(moved_pos) > i:
                after = moved_pos[i]
                if abs(after - before) > 100:
                    moved = True
                    break
            time.sleep(interval)
            elapsed += interval
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

    def perform_test(self):
        self.setup_parameters()

        # Subscribe to feedback topics
        self.create_subscription(JointState, '/servo_pos_publisher', self.feedback_cb_pos, 10)
        self.create_subscription(Int16MultiArray, '/servo_temp_publisher', self.feedback_cb_temp, 10)
        self.create_subscription(Int16MultiArray, '/servo_volt_publisher', self.feedback_cb_volt, 10)
        self.servo_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)

        # Wait for feedback
        self.wait_for_feedback()

        # Check feedback values and collect topic results
        topic_results = {}

        # Servo positions
        pos_values = list(getattr(self.servo_pos, 'position', []))
        if pos_values and not all(x == 0 for x in pos_values):
            topic_results["servo_pos_publisher"] = {
                "status": "PASS",
                "detail": f"Range: {min(pos_values)} ~ {max(pos_values)}"
            }
        else:
            topic_results["servo_pos_publisher"] = {
                "status": "FAIL",
                "detail": f"All values zero or missing: {pos_values}"
            }

        # Servo temperatures
        temp_values = list(getattr(self.servo_temp, 'data', []))
        if temp_values and not all(x == 0 for x in temp_values):
            topic_results["servo_temp_publisher"] = {
                "status": "PASS",
                "detail": f"Range: {min(temp_values)} ~ {max(temp_values)}"
            }
        else:
            topic_results["servo_temp_publisher"] = {
                "status": "FAIL",
                "detail": f"All values zero or missing: {temp_values}"
            }

        # Servo voltages
        volt_values = list(getattr(self.servo_volt, 'data', []))
        if volt_values and not all(x == 0 for x in volt_values):
            topic_results["servo_volt_publisher"] = {
                "status": "PASS",
                "detail": f"Range: {min(volt_values)} ~ {max(volt_values)}"
            }
        else:
            topic_results["servo_volt_publisher"] = {
                "status": "FAIL",
                "detail": f"All values zero or missing: {volt_values}"
            }

        # Move all servos upright in 2000ms
        upright_pos = [12000] * 6
        upright_time = [2000] * 6
        self.move_servos(upright_pos, upright_time)

        # Move each servo individually to both sides and check movement asynchronously
        results = {}
        threads = []
        center = 12000
        min_pos = 8000
        max_pos = 16000
        upright_time = [1000] * 6

        for i in range(6):
            servo_name = f"servo_{i+1}"

            # Move to min
            pos = [-1] * 6
            pos[i] = min_pos
            self.move_servos(pos, upright_time)
            time.sleep(0.3)  # 等待反馈刷新
            initial_pos = list(getattr(self.servo_pos, 'position', []))
            t_min = threading.Thread(target=self.check_servo_movement, args=(i, initial_pos, results, servo_name + "_min"))
            t_min.start()
            threads.append(t_min)

            # Move to max
            pos[i] = max_pos
            self.move_servos(pos, upright_time)
            time.sleep(0.3)  # 等待反馈刷新
            initial_pos = list(getattr(self.servo_pos, 'position', []))
            t_max = threading.Thread(target=self.check_servo_movement, args=(i, initial_pos, results, servo_name + "_max"))
            t_max.start()
            threads.append(t_max)

            # Return to center
            pos[i] = center
            self.move_servos(pos, upright_time)

        for t in threads:
            t.join()

        all_pass = all(v['moved'] for v in results.values())
        # clap for all passed 
        if all_pass:
            old_wait = self.move_wait
            self.move_wait = 0.3  
            for _ in range(3):
                pose = [center] * 6
                pose[0] = max_pos
                self.move_servos(pose, [1] * 6)
                pose[0] = min_pos
                self.move_servos(pose, [1] * 6)
            self.move_servos([center] * 6, [1] * 6)
            self.move_wait = old_wait

        status = "PASS" if all_pass else "FAIL"
        summary = "All servos moved" if all_pass else "Some servos failed: " + \
            ", ".join([k for k, v in results.items() if not v['moved']])
        detail = {
            "status": status,
            "detail": summary,
            **topic_results,
            "servo_results": results
        }

        self.save_result(self.hardware_key, all_pass, detail)
        return all_pass, detail

def main(args=None):
    rclpy.init(args=args)
    node = ArmCheck()
    node.main_spin_and_exit()

if __name__ == '__main__':
    main()
