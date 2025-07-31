import rclpy
from geometry_msgs.msg import Twist
import time
import math
import numpy as np
from hardware_test.base_test import BaseTest
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Header
import struct
from sensor_msgs.msg import PointField

class PoseTracker:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            return {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'yaw': self.quaternion_to_yaw(transform.transform.rotation)
            }
        except Exception as e:
            self.node.get_logger().error(f"Failed to get transform: {e}")
            return None

    def quaternion_to_yaw(self, quaternion):
        """convert quaternion to yaw angle"""
        from tf_transformations import euler_from_quaternion
        _, _, yaw = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw


class OdometryTest(BaseTest):
    def __init__(self):
        super().__init__(node_name='odometry_test')
        self.cmd_pub = None
        self.pose_tracker = None
        self.test_results = []

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.theoretical_square_pub = self.create_publisher(PointCloud2, '/theoretical_square', latching_qos)

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter('json_folder', '').value

        self.square_size = self.declare_parameter('square_size', 1.0).value
        self.laps = self.declare_parameter('laps_per_direction', 2).value
        self.speed = self.declare_parameter('speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value

        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_tracker = PoseTracker(self) 

        self.pub_theoretical_square_pc2()

    def pub_theoretical_square_pc2(self):
        """Publish a theoretical square path as PointCloud2 for visualization."""

        points = []

        start_x, start_y = 0.0, 0.0
        size = self.square_size 

        square_corners = [
            (start_x, start_y),
            (start_x + size, start_y),
            (start_x + size, start_y + size),
            (start_x, start_y + size)           
        ]

        points_per_edge = 100

        for i in range(4):
            start_corner = square_corners[i]
            end_corner = square_corners[(i + 1) % 4]

            for j in range(points_per_edge):
                t = j / points_per_edge
                x = start_corner[0] + t * (end_corner[0] - start_corner[0])
                y = start_corner[1] + t * (end_corner[1] - start_corner[1])
                z = 0.0
                rgb = struct.unpack('I', struct.pack('BBBB', 255, 0, 0, 255))[0]  
                points.append([x, y, z, rgb])

        for dx in [-0.01, 0.0, 0.01]:
            for dy in [-0.01, 0.0, 0.01]:
                x = start_x + dx
                y = start_y + dy
                z = 0.0
                rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
                points.append([x, y, z, rgb])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'

        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        self.theoretical_square_pub.publish(pc2_msg)

    def perform_test(self):
        try:
            self.setup_parameters()
            self.get_logger().info("Starting Odometry Test")

            if not self.wait_for_odometry():
                self.save_result('Odometry', False, "Failed to receive odometry data")
                return False
            
            success = self.run_all_squares()

            if success:
                return self.analyze_and_save_results()
            else:
                self.save_result('Odometry', False, "Failed to complete square path")
                return False    
            
        except Exception as e:
            self.get_logger().error(f"Error during Odometry Test: {e}")
            self.save_result('Odometry', False, str(e))
            return False
        finally:
            self.stop_robot()


    def wait_for_odometry(self):
        retry_count = 0
        while retry_count < 20:
            if self.pose_tracker and self.pose_tracker.get_pose():
                return True
            rclpy.spin_once(self, timeout_sec=1.0)
            retry_count += 1
        return False
    
    def run_all_squares(self):
        for lap in range(self.laps):
            result = self.run_single_square('ccw')
            if result:
                self.test_results.append(result)
                self.get_logger().info(f"CCW lap {lap + 1}: {result['closure_error']:.4f}m")
            else:
                return False
        for lap in range(self.laps):
            result = self.run_single_square('cw')
            if result:
                self.test_results.append(result)
                self.get_logger().info(f"CW lap {lap + 1}: {result['closure_error']:.4f}m")
            else:
                return False
        return True
    
    def run_single_square(self, direction='ccw'):
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            return None
        
        try:
            for edge in range(4):
                if not self.move_straight(self.square_size):
                    return None
                if not self.turn(direction):
                    return None
                    
            end_pose = self.pose_tracker.get_pose()
            if not end_pose:
                return None
            
            dx = end_pose['x'] - start_pose['x']
            dy = end_pose['y'] - start_pose['y']
            closure_error = math.sqrt(dx**2 + dy**2)

            return {
                'direction': direction,
                'closure_error': closure_error,
                'dx': dx,
                'dy': dy,
            }

        except Exception as e:
            self.get_logger().error(f"Error during square run: {e}")
            return None
        
    def move_straight(self, distance):
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            self.get_logger().error("Failed to get start pose for straight movement")
            return False
        
        twist = Twist()
        twist.linear.x = self.speed
        self.get_logger().info(f"Moving straight for {distance}m at speed {self.speed}m/s")

        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

            current_pose = self.pose_tracker.get_pose()
            if not current_pose:
                self.get_logger().warn("Temporarily lost pose tracking, continuing...")
                continue

            moved = math.sqrt((current_pose['x'] - start_pose['x'])**2 +
                              (current_pose['y'] - start_pose['y'])**2)
            if moved >= distance:
                self.get_logger().info(f"Moved {moved:.2f}m, stopping")
                break

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # Allow time for stop command to take effect 
        self.get_logger().info("Straight movement complete")
        return True

    def turn(self, direction='ccw'):
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            self.get_logger().error("Failed to get start pose for turn")
            return False
        
        twist = Twist()
        twist.angular.z = self.angular_speed if direction == 'ccw' else -self.angular_speed
        target_angle = math.pi / 2  if direction == 'ccw' else -math.pi / 2

        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

            current_pose = self.pose_tracker.get_pose()
            if not current_pose:
                self.get_logger().warn("Temporarily lost pose tracking, continuing...")
                continue

            angle_diff = current_pose['yaw'] - start_pose['yaw']
            self.get_logger().info(f"Current angle: {math.degrees(angle_diff):.2f}°")
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            if abs(angle_diff) >= abs(target_angle):
                turned_degrees = math.degrees(abs(angle_diff))
                self.get_logger().info(f"Turned {turned_degrees:.1f} ° / 90°, stopping")
                break

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # Allow time for stop command to take effect

        final_angle_degrees = math.degrees(abs(angle_diff))
        self.get_logger().info(f"Turn complete, final angle: {final_angle_degrees:.1f} °")
        return True
    

    def analyze_and_save_results(self):
        if not self.test_results:
            self.save_result('Odometry', False, "No test results to analyze")
            return False
        
        all_errors = [result['closure_error'] for result in self.test_results]
        ccw_errors = [result['closure_error'] for result in self.test_results if result['direction'] == 'ccw']
        cw_errors = [result['closure_error'] for result in self.test_results if result['direction'] == 'cw']

        mean_error = np.mean(all_errors)
        std_error = np.std(all_errors)
        ccw_mean_error = np.mean(ccw_errors) if ccw_errors else 0.0
        cw_mean_error = np.mean(cw_errors) if cw_errors else 0.0
        relative_accuracy = (1 - mean_error / (self.square_size * 4)) * 100

        detail_dict = {
            "test_config": f"{self.square_size:.1f}m square path, {len(self.test_results)} laps bidirectional",
            "mean_error_m": f"{round(mean_error, 4)} out of {self.square_size}",
            "std_error_m": f"{round(std_error, 4)}",
            "ccw VS cw_mean_error_m": f"{round(ccw_mean_error, 4)} VS {round(cw_mean_error, 4)}",
            "relative_accuracy_percent": f"{round(relative_accuracy, 4)}% out of total {self.square_size * 4} m",
        }

        self.save_result("Odometry", True, detail_dict)

        return True
    
    def stop_robot(self):
        if self.cmd_pub:
            twist = Twist()
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    test_node = OdometryTest()

    try:
        success = test_node.perform_test()
        if success:
            test_node.get_logger().info("Odometry Test completed successfully")
        else:
            test_node.get_logger().error("Odometry Test failed")
    except Exception as e:
        test_node.get_logger().error(f"Exception in Odometry Test: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

