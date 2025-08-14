import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Twist
from hardware_test.base_test import BaseTest

class PhidgetsCheck(BaseTest):
    def __init__(self):
        super().__init__(node_name='phidgets_check')
        self.hardware_key = 'Phidgets'
        self.timeout_s = 20.0

        self.results = {
            "Encoders": {"status": "FAIL", "detail": "No data received"},
            "Motors": {"status": "FAIL", "detail": "No data received"},
            "Spatial": {"status": "FAIL", "detail": "No data received"},
            "Temperature": {"status": "FAIL", "detail": "No data received"}
        }
        self._received = {k: False for k in self.results}
        self._encoders_value = None
        self._motors_value = None
        self._imu_value = None
        self._temp_value = None

        self.duty_pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.duty_msg = DutyCycles()
        self.duty_msg.duty_cycle_left = 0.1
        self.duty_msg.duty_cycle_right = 0.1
        self.duty_timer = self.create_timer(0.1, self.publish_duty_cycle)  

    def publish_duty_cycle(self):
        self.duty_pub.publish(self.duty_msg)
        # self.get_logger().info(f"Publishing duty cycles: left={self.duty_msg.duty_cycle_left}, right={self.duty_msg.duty_cycle_right}")

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter(
            'json_folder', '~/dd2419/workshop_ws/src/hardware_test/test_results/components_test'
        ).value
        self.timeout_s = self.declare_parameter('timeout_s', 30.0).value

        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

    def is_passing(self, **kwargs):

        # 2. sub topic
        self._last_encoder_left = None
        self._last_encoder_right = None

        def enc_cb(msg):
            changed = False
            if self._last_encoder_left is not None and self._last_encoder_right is not None:
                if (msg.delta_encoder_left != self._last_encoder_left) or (msg.delta_encoder_right != self._last_encoder_right):
                    changed = True
            self.get_logger().info(
                f"Encoders received: left={msg.delta_encoder_left}, right={msg.delta_encoder_right}, changed={changed}"
            )
            if changed and (msg.delta_encoder_left != 0 or msg.delta_encoder_right != 0):
                self._received["Encoders"] = True
                self._encoders_value = msg
            self._last_encoder_left = msg.delta_encoder_left
            self._last_encoder_right = msg.delta_encoder_right

        enc_sub = self.create_subscription(Encoders, '/motor/encoders', enc_cb, 1)

        def mot_cb(msg):
            self._received["Motors"] = True
            self._motors_value = msg
        mot_sub = self.create_subscription(DutyCycles, '/motor/duty_cycles', mot_cb, 1)

        def imu_cb(msg):
            self._received["Spatial"] = True
            self._imu_value = msg
        imu_sub = self.create_subscription(Imu, '/imu/data_raw', imu_cb, 1)

        def temp_cb(msg):
            self._received["Temperature"] = True
            self._temp_value = msg
        temp_sub = self.create_subscription(Temperature, '/imu/temperature', temp_cb, 1)

        # 3. wait
        start = self.get_clock().now()
        while (not all(self._received.values())) and \
              (self.get_clock().now() - start).nanoseconds / 1e9 < self.timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 4. check
        # Encoders
        if self._received["Encoders"] and self._encoders_value:
            left = self._encoders_value.delta_encoder_left
            right = self._encoders_value.delta_encoder_right
            if left != 0 or right != 0:
                self.results["Encoders"] = {
                    "status": "PASS",
                    "detail": f"delta_encoder_left={left}, delta_encoder_right={right}"
                }
            else:
                self.results["Encoders"] = {
                    "status": "FAIL",
                    "detail": f"Encoder values are zero: left={left}, right={right}"
                }
        else:
            self.results["Encoders"] = {
                "status": "FAIL",
                "detail": "No encoder data received"
            }

        # Motors
        if self._received["Motors"] and self._motors_value:
            left = self._motors_value.duty_cycle_left
            right = self._motors_value.duty_cycle_right
            if abs(left) > 0.01 or abs(right) > 0.01:
                self.results["Motors"] = {
                    "status": "PASS",
                    "detail": f"duty_cycle_left={left:.3f}, duty_cycle_right={right:.3f}"
                }
            else:
                self.results["Motors"] = {
                    "status": "FAIL",
                    "detail": f"Duty cycles are zero: left={left:.3f}, right={right:.3f}"
                }
        else:
            self.results["Motors"] = {
                "status": "FAIL",
                "detail": "No motor duty cycle data received"
            }

        # Spatial (IMU)
        if self._received["Spatial"] and self._imu_value:
            acc = self._imu_value.linear_acceleration
            if abs(acc.x) > 1e-3 or abs(acc.y) > 1e-3 or abs(acc.z) > 1e-3:
                self.results["Spatial"] = {
                    "status": "PASS",
                    "detail": f"acc=({acc.x:.2f},{acc.y:.2f},{acc.z:.2f})"
                }
            else:
                self.results["Spatial"] = {
                    "status": "FAIL",
                    "detail": f"IMU acceleration values are zero: ({acc.x:.2f},{acc.y:.2f},{acc.z:.2f})"
                }
        else:
            self.results["Spatial"] = {
                "status": "FAIL",
                "detail": "No IMU data received"
            }

        # Temperature
        if self._received["Temperature"] and self._temp_value:
            temp = self._temp_value.temperature
            if temp > -20 and temp < 100 and temp != 0:
                self.results["Temperature"] = {
                    "status": "PASS",
                    "detail": f"Temperature={temp:.2f}°C"
                }
            else:
                self.results["Temperature"] = {
                    "status": "FAIL",
                    "detail": f"Temperature value out of range: {temp:.2f}°C"
                }
        else:
            self.results["Temperature"] = {
                "status": "FAIL",
                "detail": "No temperature data received"
            }

        all_pass = all(self.results[k]["status"] == "PASS" for k in self.results)
        status = "PASS" if all_pass else "FAIL"
        detail = "All phidgets passed" if all_pass else "Some phidgets failed: " + \
            ", ".join([k for k in self.results if self.results[k]["status"] != "PASS"])
        return (all_pass, {"status": status, "detail": detail, **self.results})

    def perform_test(self):
        self.setup_parameters()
        
        passed, phidgets_detail = self.is_passing()
        self.save_result(self.hardware_key, passed, phidgets_detail)
        return passed, phidgets_detail

def main(args=None):
    rclpy.init(args=args)
    node = PhidgetsCheck()
    node.main_spin_and_exit()

if __name__ == '__main__':
    main()
