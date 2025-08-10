import rclpy
from geometry_msgs.msg import Twist
import time
import math
import numpy as np
from hardware_test.base_test import BaseTest
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from robp_interfaces.msg import Encoders
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Header
import struct
from sensor_msgs.msg import PointField
import json, csv, os
from datetime import datetime

class PoseTracker:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def get_pose(self):
        current_time = self.node.get_clock().now()  
        tf_future = self.tf_buffer.wait_for_transform_async(
            'odom', 'base_link', time=current_time)
        rclpy.spin_until_future_complete(self.node, tf_future, timeout_sec=0.05)

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

    @staticmethod
    def quaternion_to_yaw(quaternion):
        """convert quaternion to yaw angle"""
        from tf_transformations import euler_from_quaternion
        _, _, yaw = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw


class UMBmarkOdometryTest(BaseTest):
    def __init__(self):
        super().__init__(node_name='umbmark_odometry_test')
        self.cmd_pub = None
        self.pose_tracker = None
        self.last_delta_left = 0
        self.last_delta_right = 0
        self.enc_sub = self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.theoretical_square_pub = self.create_publisher(PointCloud2, '/theoretical_square', latching_qos)

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter('json_folder', '').value
        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

        self.square_size = self.declare_parameter('square_size', 4.0).value
        self.speed = self.declare_parameter('speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.direction = str(self.declare_parameter('direction', 'ccw').value).lower()
        self.lap_index = int(self.declare_parameter('lap', 1).value)       

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_tracker = PoseTracker(self) 

        base_dir = self.json_folder if self.json_folder else os.getcwd()
        base_dir = os.path.join(base_dir, 'odometry_test')
        os.makedirs(base_dir, exist_ok=True)
        self.json_path = os.path.join(base_dir, f"{self.robot_name}_umbmark.json")

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

    def encoder_callback(self, msg):
        """Callback for encoder messages to update the last delta values."""
        self.last_delta_left = msg.delta_encoder_left
        self.last_delta_right = msg.delta_encoder_right

    def perform_test(self):
        self.setup_parameters()
        while True:
            pose = self.pose_tracker.get_pose()
            if pose:
                self.get_logger().info(f"TF ready")
                break
            time.sleep(0.1)  
        self.get_logger().info(f"Starting test {self.robot_name}: lap={self.lap_index}, direction={self.direction}, size={self.square_size}")

        if not self.run_single_square(self.direction):
            self.get_logger().error("Square run failed")
            return False

        # Manual measurement input
        x_abs = float(input("Enter measured x_abs [m]: "))
        y_abs = float(input("Enter measured y_abs [m]: "))
        theta_abs = input("Enter measured theta_abs [deg] (optional, Enter to skip): ")
        theta_abs = float(theta_abs) if theta_abs.strip() else None

        odom_pose = self.pose_tracker.get_pose() or {'x': 0, 'y': 0, 'yaw': 0}
        dx = x_abs - odom_pose['x']
        dy = y_abs - odom_pose['y']
        dtheta = None
        if theta_abs is not None:
            dtheta = theta_abs - math.degrees(odom_pose['yaw'])

        record = {
            'x_abs': x_abs,
            'y_abs': y_abs,
            'theta_abs_deg': theta_abs,
            'x_calc': odom_pose['x'],
            'y_calc': odom_pose['y'],
            'theta_calc_deg': math.degrees(odom_pose['yaw']),
            'dx': dx,
            'dy': dy,
            'dtheta_deg': dtheta,
            'closure_error': math.sqrt(dx**2 + dy**2),
            'timestamp': datetime.now().strftime("%Y-%m-%dT%H:%M:%S"), 
            }

        results = {}
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        results = data 
            except Exception as e:
                self.get_logger().error(f"Failed to load JSON: {e}")
                results = {}

        if self.robot_name not in results:
            results[self.robot_name] = {
            'square_size': self.square_size,
            'speed': self.speed,
            'angular_speed': self.angular_speed,
            'ccw': {},
            'cw': {}
            }
        if self.direction not in results[self.robot_name]:
            results[self.robot_name][self.direction] = {}
        
        results[self.robot_name][self.direction][str(self.lap_index)] = {
            'detailed_results': record,
        }

        with open(self.json_path, 'w') as f:
            json.dump(results, f, indent=2)

        self.get_logger().info(f"Saved lap {self.lap_index} {self.direction} → {self.json_path}")
        return True

    def run_single_square(self, direction='cw'):
        for edge in range(4):
            if not self.move_straight(self.square_size):
                return False
            if not self.turn(direction):
                return False
        return True
    
    def move_straight(self, distance):
        twist = Twist()
        twist.linear.x = self.speed
        moved = 0.0
        last_pose = self.pose_tracker.get_pose()
        if not last_pose:
            return False
        
        while moved < distance:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            current_pose = self.pose_tracker.get_pose()
            if current_pose and last_pose:
                dx = current_pose['x'] - last_pose['x']
                dy = current_pose['y'] - last_pose['y']
                moved += math.sqrt(dx**2 + dy**2)
                last_pose = current_pose
            else:
                self.get_logger().error("Failed to get current pose during movement")
                continue
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.1)  # Allow time for stop command to take effect
        stopped = (abs(self.last_delta_left) < 1 and abs(self.last_delta_right) < 1)
        if not stopped:
            time.sleep(0.1)  
        return True
    
    def turn(self, direction='cw'):
        twist = Twist()
        twist.angular.z = self.angular_speed if direction == 'ccw' else -self.angular_speed
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            return False

        turned = 0.0
        while abs(turned) < math.pi/2 : 
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            current_pose = self.pose_tracker.get_pose()
            if current_pose:
                turned = self.normalize_angle(current_pose['yaw'] - start_pose['yaw'])
            else:
                self.get_logger().error("Failed to get current pose during turn")
                continue
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # Allow time for stop command to take effect
        return True
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
        
def main(args=None):
    rclpy.init(args=args)
    node = UMBmarkOdometryTest()
    try:
        node.perform_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

