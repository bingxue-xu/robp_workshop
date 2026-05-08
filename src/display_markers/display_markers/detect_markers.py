
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler
from aruco_msgs.msg import MarkerArray  
from geometry_msgs.msg import TransformStamped, PointStamped


class DisplayMarkers(Node) :

    def __init__(self) :
        super().__init__('display_markers')


        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer(rclpy.time.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.on_timer) 

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        # Subscribe to aruco marker topic and call callback function on each recieved message
        self.subscription = self.create_subscription(MarkerArray, '/aruco/markers', self.aruco_callback, 10)

        self.publisher = self.create_publisher(PointStamped, '/marker_pose', 10)
        

    def aruco_callback(self, msg: MarkerArray) :
		# look up transform from map_frame to marker frame and publish the position of each marker
        # here we will get coordinates of the marker in the camera frame
        # camera -> base_link -> map -> marker
        

        self.target_frame = 'map'
        self.source_frame = msg.header.frame_id

        self.get_logger().info('timestamp: {}. \n rclpy.time.Time: {}'.format((msg.header.stamp.nanosec + msg.header.stamp.sec * math.pow(10,9)), self.get_clock().now().nanoseconds))
#
        # timeout = rclpy.time.Duration(seconds=0.5)
        if self.tf_buffer.can_transform(self.target_frame, self.source_frame, msg.header.stamp):
            try: 
                t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                msg.header.stamp,
                timeout=rclpy.time.Duration(seconds=0.5),
                )
                self.get_logger().info("It works")
            except (TransformException, LookupException, ConnectivityException,) as ex:
                self.get_logger().warn('Could not transform due to {}'.format(ex))
                return
            except ExtrapolationException as ex:
                self.get_logger().warn('Could not transform due to {}'.format(ex))
                return
    
            new_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg.markers[0].pose, t)

            point = PointStamped()
            point.header = new_pose.header
            point.point = new_pose.pose.position

            self.publisher.publish(point)

            
            t2 = TransformStamped()
            t2.header.stamp = msg.header.stamp
            t2.transform.translation.x = new_pose.pose.position.x
            t2.transform.translation.y = new_pose.pose.position.y
            t2.transform.translation.z = new_pose.pose.position.z
            t2.transform.rotation = new_pose.pose.orientation
            t2.header.frame_id = 'map'
            t2.child_frame_id = 'aruco/est_marker' + str(msg.markers[0].id)

            self.tf_broadcaster.sendTransform(t2)


def main() :
    rclpy.init()
    node = DisplayMarkers()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# class Aruco_detect(Node):
#     def __init__(self):
#         super().__init__('display_markers')
#         self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self._tf_broadcaster = TransformBroadcaster(self) #Broadcaster 1
#         self.subscription = self.create_subscription(MarkerArray, '/aruco/markers', self.callback, 10)        
#         self.subscription

#         self.publish = self.create_publisher(PointStamped, '/object/detected', 10)

#     def callback(self, msg:MarkerArray):
#         for marker in msg.markers:
#             id = int(marker.id)
#             #Getting transform from aruco_temp_detected to map
#             source_frame = msg.header.frame_id
#             target_frame = 'map' 
#             trans = None

#             if self.tf_buffer.wait_for_transform_async(target_frame, source_frame,  marker.header.stamp):
#                 try:
#                     trans = self.tf_buffer.lookup_transform(
#                                 target_frame,
#                                 source_frame,
#                                  marker.header.stamp,timeout=rclpy.duration.Duration(seconds=0.1)) #this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
#                 except(ConnectivityException,LookupException,ExtrapolationException,TransformException):
#                     print('Human is friend')

#             #if we succed in gettting transform, broadcast transform to new frame with map as parent
#             if trans is not None:
#                 formed = do_transform_pose_stamped(marker.pose, trans)

#                 #if id ==2 :
#                 #     rotate = quaternion_from_euler(0,np.pi/2,-np.pi/2)
#                 #elif id == 1:
#                 #    rotate = quaternion_from_euler(0,-np.pi/2,-np.pi/2)
#                 #quat = quaternion_multiply(rotate,[formed.pose.orientation.x,formed.pose.orientation.y,formed.pose.orientation.z,formed.pose.orientation.w])
# #
#                 #formed.pose.orientation.x = quat[0]
#                 #formed.pose.orientation.y = quat[1]
#                 #formed.pose.orientation.z = quat[2]
#                 #formed.pose.orientation.w = quat[3]
# #
#                 self.broadcast_transform(formed,id) #broadcasting

        
#     def broadcast_transform(self,pose, id):

#         t1 = TransformStamped()
#         t1.header.stamp = pose.header.stamp
#         t1.header.frame_id = 'map'
#         t1.child_frame_id = 'aruco/detected%d' %id

#         t1.transform.translation.x = pose.pose.position.x
#         t1.transform.translation.y = pose.pose.position.y
#         t1.transform.translation.z = pose.pose.position.z
#         t1.transform.rotation.x = pose.pose.orientation.x
#         t1.transform.rotation.y = pose.pose.orientation.y
#         t1.transform.rotation.z = pose.pose.orientation.z
#         t1.transform.rotation.w = pose.pose.orientation.w

#         self._tf_broadcaster.sendTransform(t1)

#         point = PointStamped()
#         point.header = pose.header
#         point.point.x = pose.pose.position.x
#         point.point.y = pose.pose.position.y
#         point.point.z = pose.pose.position.z

#         self.publish.publish(point)


# def main():
#     rclpy.init()
#     node = Aruco_detect()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     rclpy.shutdown()





# if __name__ == '__main__':
#     main()