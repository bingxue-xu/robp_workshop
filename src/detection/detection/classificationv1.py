import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo,Imu
from detection.utils import load_model
from detection.detector import Detector
from arian_interfaces.srv import AddCamObs
from arian_interfaces.msg import PointWithRadiusArray, PointWithRadius
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from torchvision.transforms import v2
from torchvision.ops import nms
from message_filters import ApproximateTimeSynchronizer, Subscriber
from slam.slam import cv
import tf2_ros
import cv2
import torch
from collections import Counter
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import std_msgs.msg as std_msgs
class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        model_path = "/home/rosuser/dd2419_ws/src/detection/detection/det_2024-05-10_BEST.pt"
        self.detector = Detector()
        self.model = load_model(self.detector,model_path,torch.device("cuda"))
        self.bridge = CvBridge()
        dummy_input = torch.rand(1, 3, 480, 640).to(torch.device("cuda"))
        self.model = self.model.to(torch.device("cuda"))
        self.traced_model = torch.jit.trace(self.model, dummy_input.to(torch.device("cuda")))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.ang_speed = 0
        self.k = 0

        self.input_transforms = v2.Compose(
            [
                v2.ToImage(),
                v2.ToDtype(torch.float32, scale=True),
                #v2.Normalize(mean=[0.4, 0.4, 0.4], std=[0.25, 0.25, 0.25]),
            ])
        
        self.sub = self.create_subscription(Imu,"/imu/data_raw",self.imu_callback,10,callback_group=ReentrantCallbackGroup())
        

        self.image_depth_subscription = Subscriber(self,Image,'/camera/aligned_depth_to_color/image_raw')
        self.camera_info_subscription = Subscriber(self, CameraInfo,'/camera/color/camera_info')
        self.image_color_subscription = Subscriber(self,Image, '/camera/color/image_raw')

        queue_size = 10
        self.ts = ApproximateTimeSynchronizer([self.image_color_subscription, self.camera_info_subscription,self.image_depth_subscription],queue_size,0.01)
        self.ts.registerCallback(self.image_color_callback)


        self.bbox_publisher = self.create_publisher(Image, '/object_bounding_boxes', 10)
        self.point = self.create_publisher(PointWithRadius, '/estimated_pose', 10)

        self.add_objects = self.create_client(AddCamObs, '/add_cam_obs')
        while not self.add_objects.wait_for_service(timeout_sec=2.0):
           self.get_logger().info('add_objects not available, waiting again...')




        self.animals = ["Binky","Hugo", "Slush", "Muddles", "Kiki", "Oakie"]
        self.color_ranges = {
        "red": [([0, 172, 63], [6, 255, 201]), ([170, 172, 63], [180, 255, 201])],
        "green": [([63, 145, 45], [96, 255, 255])],
        "blue": [([98, 153, 74], [101, 255, 255])],
        "wooden": [([9, 10, 67], [15, 79, 252])]
        }

        self.sub_sleep = self.create_subscription(std_msgs.Bool, '/sleep', self.sleep_callback, 10 ,callback_group=ReentrantCallbackGroup())

    def sleep_callback(self, msg: std_msgs.Bool):
        if msg.data == True:
            time.sleep(0.8)
        else:
            time.sleep(0.3)
    def image_color_callback(self, msg_color: Image,msg_camerainfo: CameraInfo,msg_depth: Image):
        if self.ang_speed <2: #and self.k % 2 == 0:
            cv_image= self.bridge.imgmsg_to_cv2(msg_color, "rgb8")
            self.cv_image_array = np.array(cv_image, dtype=np.uint8)
            cv_image= self.bridge.imgmsg_to_cv2(msg_depth,"16UC1")
            self.cv_depth_image_array = np.array(cv_image, dtype=np.float32)
            self.camera_focal_length_x = msg_camerainfo.k[0]
            self.camera_focal_length_y = msg_camerainfo.k[4]
            self.camera_principal_point_x = msg_camerainfo.k[2]
            self.camera_principal_point_y = msg_camerainfo.k[5]
            self.bounding_box(self.cv_image_array, self.cv_depth_image_array,msg_color.header.stamp,msg_color.header.frame_id)
        self.k+=1
        
    def imu_callback(self,msg: Imu):
        self.ang_speed=abs(msg.angular_velocity.z)

    def bounding_box(self,cv_image,depth_image,time_stamp,frame_id):
        '''
        Args:
            cv_image (np.array): Image in OpenCV format.
            time_stamp (int): Time stamp of the image.
            frame_id (str): Frame ID of the image.
            point_cloude (np.array): Point cloud in OpenCV format.
        Returns:
            Publishes the bounding boxes and object positions.
        '''
        

        #point_cloude_cuda = torch.tensor(point_cloude).to("cuda")
        input_image = torch.stack([self.input_transforms(cv_image)]).to(torch.device("cuda"))
        # input_image = torch.tensor(cv_image).to(torch.device("cuda"))
        with torch.no_grad():    
            out = self.traced_model(input_image)
            SCORE_THRESHOLD = 0.92
            bbs = self.model.out_to_bbs(out,SCORE_THRESHOLD)
            try:
                bbs = self.nms(bbs[0],iou_threshold=0.5)
            except:
                return
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
            tem_list = []
            #bbs = max(bbs, key=lambda x: x['score'])
            for bb in bbs:
                #######creating the PouisitionWithRadiusArray message ###########

                object = PointWithRadius()
                pose = PointStamped()
                neighborhood_size = 3
                x_min = int(bb['x'])
                y_min = int(bb['y'])
                center_y = int(bb['y']+bb['height']/2)
                center_x = int(bb['x']+bb['width']/2)
                x_max = int(bb['x'] + bb['width'])
                y_max = int(bb['y'] + bb['height'])
                marginx = 70
                marginy = 70
                if center_x - marginx < 0 or center_x + marginx >= cv_image.shape[1] or center_y - marginy < 0 or center_y + marginy >= cv_image.shape[0]:
                    continue
                x,y,z = self.get_depth(depth_image,center_x, center_y, neighborhood_size)
                z += 0.015 #adding offset of object center
                
                THRESHOLD_DISTANCE = 1.0
                if z > THRESHOLD_DISTANCE or y<0.03 or y> 0.09:
                    continue
                pose.header.stamp = time_stamp 
                pose.header.frame_id = frame_id
                if not all([x,y,z]):
                    continue
                pose.point.x = float(x)
                pose.point.y = float(y)
                pose.point.z = float(z) #+0.02 #Adding depth of object to not just get the f
                object.point = pose
                #self.point.publish(object)
                #self.get_logger().info(str(object.point.point.x)+' ' +str(object.point.point.y)
                #                    +' '+str(object.point.point.z))
                #######Assigning the object type########
                category = bb["category"]

                if category == 7 :
                    object_color = self.compute_color(hsv_image, center_x, center_y, x_max, y_max) 
                    object.radius = 0.04
                    object.target = True
                    if object_color != None:
                        object.type = f"{object_color} cube"
                    else: continue
                elif category == 8 :

                    object_color = self.compute_color(hsv_image, center_x, center_y, x_max, y_max) 
                    object.target = True
                    object.radius = 0.04
                    if object_color != "wooden" and object_color != None:
                        object.type = f"{object_color} ball"
                    else: continue
                elif category == 9:
                    object.radius = 0.20
                    object.target = False
                    object.type = "box"
                elif category < len(self.animals) +1:
                    object.target = True
                    object.radius = 0.06
                    object.type = self.animals[category-1]
                else:
                    continue 
                # self.get_logger().info('object type: '+object.type)
                image=cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                label = '{}'.format(object.type) #, bb["score"])
                image=cv2.putText(image,label,(center_x,y_max+10),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 255, 0),2)
                image_with_boxes = self.bridge.cv2_to_imgmsg(image, "rgb8")
                image_with_boxes.header.stamp = time_stamp
                object.image = image_with_boxes
                self.bbox_publisher.publish(image_with_boxes)
                #Image with boxes borde vara här, även object.image
                tem_list.append(object)
            #if len(tem_list) != 0:
            #    object_types = [obj.type for obj in tem_list]
            #    most_common_type = Counter(object_types).most_common(1)[0][0]
#
            #    for obj in tem_list:
            #        obj.type = most_common_type
            if tem_list != []:
                srv_message = AddCamObs.Request()
                srv_message.radpoints.points = tem_list
                self.future = self.add_objects.call_async(srv_message)

    def compute_color(self, hsv_image, x_min,y_min,x_max,y_max):
            '''
            Args:
                cv_image (np.array): Image in OpenCV format.
                x_min (int): top left x-coordinate of the bounding box.
                y_min (int): top left y-coordinate of the bounding box.
                x_max (int): bottom right x-coordinate of the bounding box.
                y_max (int): bottom right y-coordinate of the bounding box.
            Returns:
                str: Color label of the object in the bounding box.
            '''

            hsv_bbox = hsv_image[y_min:y_max, x_min:x_max]
            
            # Display the cropped image
            #cv2.imwrite("/home/rosuser/dd2419_ws/src/detection/detection/cropped_image.jpg", hsv_bbox)
            color_pixels = {}
            if hsv_bbox.size != 0:


                for color, ranges in self.color_ranges.items():
                    mask = sum([cv2.inRange(hsv_bbox, np.array(low), np.array(high)) for low, high in ranges])
                    color_pixels[color] = np.count_nonzero(mask)

                color_label = max(color_pixels, key=color_pixels.get)
                return color_label if color_pixels[color_label] > 0 else None

            else:
                return None
      

    def get_depth(self, cv_depth_image, center_x, center_y, neighborhood_size=3):
        '''
        Args:
            cv_depth_image (np.array): Depth image in OpenCV format.
            center_x (int): X-coordinate of the center of the neighborhood.
            center_y (int): Y-coordinate of the center of the neighborhood.
            neighborhood_size (int): Size of the neighborhood.
            fx (float): Focal length of the camera in the x direction.
            fy (float): Focal length of the camera in the y direction.
            cx (float): X-coordinate of the principal point.
            cy (float): Y-coordinate of the principal point.
        Returns:
            tuple: Mean x, y, z coordinates of the neighborhood.
        '''
        x_min = max(0, center_x - neighborhood_size // 2)
        y_min = max(0, center_y - neighborhood_size // 2)
        x_max = min(cv_depth_image.shape[1], center_x + neighborhood_size // 2)
        y_max = min(cv_depth_image.shape[0], center_y + neighborhood_size // 2)

        depth_bbox = cv_depth_image[y_min:y_max, x_min:x_max]
        depth = np.mean(depth_bbox)/1000
    

        # Calculate the mean x, y, z coordinates
        x = (center_x - self.camera_principal_point_x) * depth / self.camera_focal_length_x
        y = (center_y - self.camera_principal_point_y) * depth / self.camera_focal_length_y
        z = depth

        return x, y, z



      
    def nms(self,bbs,iou_threshold = 0.5):
        '''Apply non-maximum suppression to bounding boxes.
        Args:
            bbs (list): List of bounding boxes in the format (x, y, width, height, score).
            iou_threshold (float): Intersection over union threshold to suppress bounding boxes.
        Returns:
            list: List of bounding boxes after applying non-maximum suppression.
        '''
        boxes = [[item['x'], item['y'], item['x'] + item['width'], item['y'] + item['height']] for item in bbs]
        scores = [item['score'] for item in bbs]
        boxes = torch.tensor(boxes, dtype=torch.float32)
        scores = torch.tensor(scores)
        keep = nms(boxes, scores, iou_threshold)
        return [bbs[i] for i in keep]
    
    def get_mean_point(self,center_x, center_y, neighborhood_size, point_cloude):
        '''
        Args:
            center_x (int): X-coordinate of the center of the neighborhood.
            center_y (int): Y-coordinate of the center of the neighborhood.
            neighborhood_size (int): Size of the neighborhood.
            point_cloude (np.array): Point cloud in OpenCV format.
        Returns:
            np.array: Mean point of the neighborhood.
        '''
        half_neighborhood = neighborhood_size // 2
        y_min = max(0, center_y - half_neighborhood)
        y_max = min(point_cloude.shape[0], center_y + half_neighborhood + 1)
        x_min = max(0, center_x - half_neighborhood)
        x_max = min(point_cloude.shape[1], center_x + half_neighborhood + 1)

        neighborhood_points = point_cloude[y_min:y_max, x_min:x_max, :]
        mask = np.all(neighborhood_points != 0, axis=2)
        neighborhood_points = neighborhood_points[mask]

        if neighborhood_points.size > 0:
            return np.mean(neighborhood_points, axis=0, dtype=np.float64)
        else:
            return None


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()


