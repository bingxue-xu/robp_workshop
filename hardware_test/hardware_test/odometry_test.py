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
import json, csv, os

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
        self.test_results = [] # store manual error entries

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
        self.laps = self.declare_parameter('laps_per_direction', 5).value
        self.speed = self.declare_parameter('speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_theoretical_square_pc2()
        self.pose_tracker = PoseTracker(self) 


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
        self.setup_parameters()
        self.get_logger().info("Starting UMBmark Odometry Test (Manual Measurement Mode)")

        for direction in ['cw', 'ccw']:
            for lap in range(self.laps):
                self.get_logger().info(f"Starting lap {lap+1}/{self.laps} ({direction})")
                self.run_single_square(direction)

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

                self.results.append({
                    'lap': lap + 1,
                    'direction': direction,
                    'x_abs': x_abs,
                    'y_abs': y_abs,
                    'theta_abs_deg': theta_abs,
                    'x_calc': odom_pose['x'],
                    'y_calc': odom_pose['y'],
                    'theta_calc_deg': math.degrees(odom_pose['yaw']),
                    'dx': dx,
                    'dy': dy,
                    'dtheta_deg': dtheta,
                    'closure_error': math.sqrt(dx**2 + dy**2)
                })

                input("Reposition robot to origin and press Enter to start next run...")

        self.analyze_results()
        self.save_json_csv()
        self.save_result("UMBmark_Odometry", True, self.results)
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
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            return False
        
        while moved < distance:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            current_pose = self.pose_tracker.get_pose()
            if current_pose:
                moved = math.sqrt((current_pose['x'] - start_pose['x'])**2 + (current_pose['y'] - start_pose['y'])**2)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # Allow time for stop command to take effect
        return True
    
    def turn(self, direction='cw'):
        twist = Twist()
        twist.angular.z = self.angular_speed if direction == 'ccw' else -self.angular_speed
        start_pose = self.pose_tracker.get_pose()
        if not start_pose:
            return False

        turned = 0.0
        while abs(turned) < math.pi/2:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            current_pose = self.pose_tracker.get_pose()
            if current_pose:
                turned = self.normalize_angle(current_pose['yaw'] - start_pose['yaw'])

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
    
    def analyze_results(self):
        cw  = [(r['dx'], r['dy']) for r in self.results if r['direction'] == 'cw']
        ccw = [(r['dx'], r['dy']) for r in self.results if r['direction'] == 'ccw']
               
        def center_of_gravity(points):
            if not points:
                return (0.0, 0.0)
            xs, ys = zip(*points)
            return (np.mean(xs), np.mean(ys))
        
        cg_cw = center_of_gravity(cw)
        cg_ccw = center_of_gravity(ccw)

        r_cw = math.sqrt(cg_cw[0]**2 + cg_cw[1]**2)
        r_ccw = math.sqrt(cg_ccw[0]**2 + cg_ccw[1]**2)
        e_max_syst = max(r_cw, r_ccw)

        self.get_logger().info(f"CW cluster center: {cg_cw}, radius {r_cw:.3f} m")
        self.get_logger().info(f"CCW cluster center: {cg_ccw}, radius {r_ccw:.3f} m")
        self.get_logger().info(f"E_max_system: {e_max_syst:.3f} m")

        self.results.append({
            'analysis': {
                'cw_cluster_center': cg_cw,
                'ccw_cluster_center': cg_ccw,
                'cw_radius_m': r_cw,
                'ccw_radius_m': r_ccw,
                'E_max_syst_m': e_max_syst
            }
        })

    def save_json_csv(self):
        os.makedirs(self.json_folder, exist_ok=True)
        umbmark_json_path = os.path.join(self.json_folder, f"{self.robot_name}_umbmark.json")
        umbmark_csv_path = os.path.join(self.json_folder, f"{self.robot_name}_umbmark.csv")

        with open(umbmark_json_path, 'w') as f:
            json.dump(self.results, f, indent=2)

        keys = list(self.results[0].keys())
        with open(umbmark_csv_path, 'w', newline='') as cf:
            writer = csv.DictWriter(cf, fieldnames=keys)
            writer.writeheader()
            for r in self.results:
                if isinstance(r.get('analysis'), dict):
                    continue
                writer.writerow(r)

        self.get_logger().info(f"Saved UMBmark results to {umbmark_json_path} and {umbmark_csv_path}")
        
        
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

