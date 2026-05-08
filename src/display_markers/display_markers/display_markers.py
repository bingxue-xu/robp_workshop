
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler
from aruco_msgs.msg import MarkerArray  
from geometry_msgs.msg import TransformStamped


class DisplayMarkers(Node) :

    def __init__(self) :
        super().__init__('display_markers')

        # Initialize the transform listernerand assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listner = TransformListener(self.tf_buffer,self)

        # Initialize the transform broadcaster
        self.tf_broadcast = TransformBroadcaster(self)
        # Subscribe to aruco marker topic and call callback function on each recieved message
        self.aruco_sub = self.create_subscription(MarkerArray,'/aruco/markers',self.aruco_callback,10)

    def aruco_callback(self, msg: MarkerArray) :
    
    # look up transform from map_frame to marker frame and publish the position of each marker
            #except TransformException:
        trans = TransformStamped()
        for msg_marker in msg.markers:
            trans.header.stamp = msg_marker.header.stamp
            trans.header.frame_id = 'map'
            trans.child_frame_id = f'/aruco/detected{msg_marker.id}'
            rotation_angle = math.pi/2


            traget ='map'
            from_frame = 'base_link'
            try:
                t = self.tf_buffer.lookup_transform(traget,from_frame, msg_marker.header.stamp)
            except TransformException:
                print(f'Failed transfrom from {from_frame} to {traget}')
                return
            

            
            x =  msg_marker.pose.pose.position.z 
            y = -msg_marker.pose.pose.position.x 
            z = -msg_marker.pose.pose.position.y

            msg_marker.pose.pose.position.x = x + 0.08987
            msg_marker.pose.pose.position.y = y + 0.0175
            msg_marker.pose.pose.position.z = z + 0.10456

            q=quaternion_from_euler(0, 0, 2*rotation_angle)
            msg_marker.pose.pose.orientation.x = q[0]
            msg_marker.pose.pose.orientation.y = q[1]
            msg_marker.pose.pose.orientation.z = q[2]
            msg_marker.pose.pose.orientation.w = q[3]
            
            base_link_marker= tf2_geometry_msgs.do_transform_pose(msg_marker.pose.pose,t)

            trans.transform.translation.x = base_link_marker.position.x 
            trans.transform.translation.y = base_link_marker.position.y 
            trans.transform.translation.z = base_link_marker.position.z 
            trans.transform.rotation.x = base_link_marker.orientation.x
            trans.transform.rotation.y =base_link_marker.orientation.y
            trans.transform.rotation.z =base_link_marker.orientation.z
            trans.transform.rotation.w =base_link_marker.orientation.w
            
       
            self.tf_broadcast.sendTransform(trans)
    

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


'''#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, quaternion_multiply
from tf2_geometry_msgs import do_transform_pose_stamped
import time
from geometry_msgs.msg import TransformStamped

from aruco_msgs.msg import MarkerArray

class Aruco_detect(Node):
    def __init__(self):
        super().__init__('display_markers')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._tf_broadcaster = TransformBroadcaster(self) #Broadcaster 1
        self.subscription = self.create_subscription(MarkerArray, '/aruco/markers', self.callback, 10)        
        self.subscription
    def callback(self, msg:MarkerArray):
        for marker in msg.markers:
            id = int(marker.id)
            #Getting transform from aruco_temp_detected to map
            source_frame = msg.header.frame_id
            target_frame = 'map' 
            trans = None

            if self.tf_buffer.wait_for_transform_async(target_frame, source_frame,  marker.header.stamp):
                try:
                    trans = self.tf_buffer.lookup_transform(
                                target_frame,
                                source_frame,
                                 marker.header.stamp,timeout=rclpy.duration.Duration(seconds=0.1)) #this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
                except(ConnectivityException,LookupException,ExtrapolationException,TransformException):
                    print('Human is friend')

            #if we succed in gettting transform, broadcast transform to new frame with map as parent
            if trans is not None:
                formed = do_transform_pose_stamped(marker.pose, trans)

                #if id ==2 :
                #     rotate = quaternion_from_euler(0,np.pi/2,-np.pi/2)
                #elif id == 1:
                #    rotate = quaternion_from_euler(0,-np.pi/2,-np.pi/2)
                #quat = quaternion_multiply(rotate,[formed.pose.orientation.x,formed.pose.orientation.y,formed.pose.orientation.z,formed.pose.orientation.w])
#
                #formed.pose.orientation.x = quat[0]
                #formed.pose.orientation.y = quat[1]
                #formed.pose.orientation.z = quat[2]
                #formed.pose.orientation.w = quat[3]
#
                self.broadcast_transform(formed,id) #broadcasting

        
    def broadcast_transform(self,pose, id):

        t1 = TransformStamped()
        t1.header.stamp = pose.header.stamp
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'aruco/detected%d' %id

        t1.transform.translation.x = pose.pose.position.x
        t1.transform.translation.y = pose.pose.position.y
        t1.transform.translation.z = pose.pose.position.z
        t1.transform.rotation.x = pose.pose.orientation.x
        t1.transform.rotation.y = pose.pose.orientation.y
        t1.transform.rotation.z = pose.pose.orientation.z
        t1.transform.rotation.w = pose.pose.orientation.w

        self._tf_broadcaster.sendTransform(t1)


def main():
    rclpy.init()
    node = Aruco_detect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()





if __name__ == '__main__':
    main()
'''