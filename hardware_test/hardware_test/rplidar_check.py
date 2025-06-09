import rclpy
from sensor_msgs.msg import LaserScan
from hardware_test.base_test import BaseTest

class RPLidarCheck(BaseTest):
    def __init__(self):
        super().__init__(node_name='rplidar_check')
        self.hardware_key = 'RPLidar'

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter('json_folder', '').value
        topic_name = self.declare_parameter('topic_name', '/scan').value
        timeout_s = self.declare_parameter('timeout_s', 4.0).value
        point_threshold = self.declare_parameter('point_count_threshold', 100).value
        range_threshold = self.declare_parameter('range_threshold', 1.0).value

        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

        self.topic_name = topic_name
        self.timeout_s = timeout_s
        self.point_count_threshold = point_threshold
        self.range_threshold = range_threshold

    def is_passing(self, msg: LaserScan, **kwargs):
        total_points = len(msg.ranges)
        finite_ranges = [r for r in msg.ranges if r == r and r != float('inf')]

        if not finite_ranges:
            detail = "No finite range values in scan"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

        measured_max = max(finite_ranges)

        self.get_logger().info(
            f"Total rplidar points: {total_points}, "
            f"Max measured distance: {measured_max:.2f} m, "
            f"Thresholds -> points: {self.point_count_threshold}, range: {self.range_threshold:.2f}"
        )

        if total_points < self.point_count_threshold:
            detail = f"Insufficient points: {total_points} (< {self.point_count_threshold})"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

        if measured_max < self.range_threshold:
            detail = f"Max measured distance too low: {measured_max:.2f} m (< {self.range_threshold:.2f} m)"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

        detail = f"Total rplidar points: {total_points}, Max measured distance: {measured_max:.2f} m"
        return True, detail

    def perform_test(self):
        self.setup_parameters()

        return self.run_test(
            hardware_key=self.hardware_key,
            msg_type=LaserScan,
            topic_name=self.topic_name,
            timeout_s=self.timeout_s
        )


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarCheck()
    node.main_spin_and_exit()

if __name__ == '__main__':
    main()