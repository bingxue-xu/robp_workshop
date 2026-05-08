

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
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
import sensor_msgs_py.point_cloud2 as pc2
class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        model_path = "/home/rosuser/dd2419_ws/src/detection/detection/det_2024-04-18_Baug_multiple.pt"
        self.detector = Detector()
        self.model = load_model(self.detector,model_path,torch.device("cuda"))
        self.bridge = CvBridge()
        dummy_input = torch.rand(1, 3, 480, 640).to(torch.device("cuda"))
        self.model = self.model.to(torch.device("cuda"))
        self.traced_model = torch.jit.trace(self.model, dummy_input.to(torch.device("cuda")))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)


        self.input_transforms = v2.Compose(
            [
                v2.ToImage(),
                v2.ToDtype(torch.float32, scale=True),
                v2.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ])


        # self.add_objects = self.create_client(AddCamObs, '/add_cam_obs')
        # while not self.add_objects.wait_for_service(timeout_sec=2.0):
        #    self.get_logger().info('add_objects not available, waiting again...')
       

        self.point_cloude_subscription = Subscriber(self, PointCloud2, '/camera/depth/color/points')
        self.image_color_subscription = Subscriber(self,Image, '/camera/color/image_raw')
        queue_size = 10
        self.ts = ApproximateTimeSynchronizer([self.image_color_subscription, self.point_cloude_subscription],queue_size,0.01)
        self.ts.registerCallback(self.image_color_callback)


        self.point = self.create_publisher(PointStamped, '/object_position', 10)
        self.bbox_publisher = self.create_publisher(Image, '/object_bounding_boxes', 10)
        self.point = self.create_publisher(PointStamped, '/estimated_pose', 10)


        self.animals = ["Binky","Hugo", "Slush", "Muddles", "Kiki", "Oakie"]
        self.color_ranges = {
        "red": [([0, 172, 63], [6, 255, 201]), ([170, 172, 63], [180, 255, 201])],
        "green": [([63, 145, 45], [96, 255, 255])],
        "blue": [([98, 153, 74], [101, 255, 255])],
        "wooden": [([9, 10, 67], [15, 79, 252])]
        }



    def image_color_callback(self, msg: Image, msg2: PointCloud2):
        '''
        Args:
            msg (Image): Image message from the camera.
            msg2 (PointCloud2): Point cloud message from the camera.
        Returns:
            Publishes the bounding boxes and object positions.
        '''
        cv_image= self.bridge.imgmsg_to_cv2(msg, "rgb8")
        gen = pc2.read_points_numpy(msg2,skip_nans=True)
        height, width, _= cv_image.shape
        point_cloude = gen[:,:3]
        point_cloud_reshaped = point_cloude.reshape(height, width, 3)
        self.bounding_box(cv_image,msg.header.stamp,msg.header.frame_id,point_cloud_reshaped)

    
   
    def bounding_box(self,cv_image,time_stamp,frame_id,point_cloude):
        '''
        Args:
            cv_image (np.array): Image in OpenCV format.
            time_stamp (int): Time stamp of the image.
            frame_id (str): Frame ID of the image.
            point_cloude (np.array): Point cloud in OpenCV format.
        Returns:
            Publishes the bounding boxes and object positions.
        '''
        if cv_image is not None and point_cloude is not None: #and self.camera_principal_point_x is not None and self.camera_principal_point_y is not None:
            

            #point_cloude_cuda = torch.tensor(point_cloude).to("cuda")
            input_image = torch.stack([self.input_transforms(cv_image)]).to(torch.device("cuda"))
            # input_image = torch.tensor(cv_image).to(torch.device("cuda"))
            with torch.no_grad():    
                out = self.traced_model(input_image)
                SCORE_THRESHOLD = 0.98
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
                    center_y = int(bb['y'])#+bb['height']/2)
                    center_x = int(bb['x'])#+bb['width']/2)
                    x_max = int(bb['x'] + bb['width'])
                    y_max = int(bb['y'] + bb['height'])
                    marginx = 50
                    marginy = 10
                    if center_x - marginx < 0 or center_x + marginx >= cv_image.shape[1] or center_y - marginy < 0 or center_y + marginy >= cv_image.shape[0]:
                        return
                    mean_point = self.get_mean_point(center_x, center_y, neighborhood_size, point_cloude)

                    if mean_point is not None:
                        distance = np.linalg.norm(mean_point)
                        #print(distance)
                    else:
                        return
                    THRESHOLD_DISTANCE = 1.3
                    if distance > THRESHOLD_DISTANCE:
                        return
                    pose.header.stamp = time_stamp 
                    pose.header.frame_id = frame_id
                    pose.point.x = mean_point[0]
                    pose.point.y = mean_point[1]
                    pose.point.z = mean_point[2]
                    object.point = pose
                    self.point.publish(pose)
                    self.get_logger().info(str(object.point.point.x)+' ' +str(object.point.point.y)
                                       +' '+str(object.point.point.z))
                    #######Assigning the object type########
                    category = bb["category"]

                    if category == 6 :
                        object_color = self.compute_color(hsv_image, center_x, center_y, x_max, y_max) 
                        object.radius = 0.05
                        object.target = True
                        object.type = f"{object_color} cube"
                    elif category == 7 :

                        object_color = self.compute_color(hsv_image, center_x, center_y, x_max, y_max) 
                        object.target = True
                        object.radius = 0.05
                        if object_color != "wooden":
                            object.type = f"{object_color} ball"
                        
                    elif category == 8:
                        object.radius = 0.20
                        object.target = False
                        object.type = "box"
                    elif category < len(self.animals):
                        object.target = True
                        object.radius = 0.05
                        object.type = self.animals[category]
                    else:
                        return 
                    self.get_logger().info('object type: '+object.type)
                    image=cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    label = '{}'.format(object.type) #, bb["score"])
                    image=cv2.putText(image,label,(center_x,y_max+10),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 255, 0),2)
                    tem_list.append(object)
                image_with_boxes = self.bridge.cv2_to_imgmsg(image, "rgb8")
                image_with_boxes.header.stamp = time_stamp
                object.image = image_with_boxes
                self.bbox_publisher.publish(image_with_boxes)
            
                srv_message = AddCamObs.Request()
                srv_message.radpoints.points = tem_list
                #self.future = self.add_objects.call_async(srv_message)
                #rclpy.spin_until_future_complete(self, self.future)


  
    def nms(self,bbs,iou_threshold = 0.5):
        '''Apply non-maximum suppression to bounding boxes.
        Args:
            bbs (list): List of bounding boxes in the format (x, y, width, height, score).
            iou_threshold (float): Intersection over union threshold to suppress bounding boxes.
        Returns:
            list: List of bounding boxes after applying non-maximum suppression.
        '''
        # boxes = []
        # scores = []
        # for item in bbs:
        #     x_min = item['x']
        #     y_min = item['y']
        #     x_max = item['x'] + item['width']
        #     y_max = item['y'] + item['height']
        #     boxes.append([x_min, y_min, x_max, y_max])
        #     scores.append(item['score'])
        # boxes = torch.tensor(boxes, dtype=torch.float32)
        # scores = torch.tensor(scores)
        # keep = nms(boxes, scores, iou_threshold)
        # bbs = [bbs[i] for i in keep]
        # return bbs
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
        # for dy in range(-neighborhood_size // 2, neighborhood_size // 2 + 1):
        #     for dx in range(-neighborhood_size // 2, neighborhood_size // 2 + 1):
        #         x = center_x + dx
        #         y = center_y + dy
        #         indices.append([y, x]) 

        # indices = np.array(indices)
        # neighborhood_points = point_cloude[indices[:,0], indices[:,1], :]
        # mask = np.all(neighborhood_points !=0, axis=1)
        # neighborhood_points = neighborhood_points[mask]

        # if neighborhood_points.size > 0:
        #     return np.mean(neighborhood_points, axis=0)
        # else:
        #     return None
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

            #     # low_red = np.array([0, 200, 100])  
            #     # high_red = np.array([10, 255, 255]) 
            #     low_red = np.array([0, 172, 63])  
            #     high_red = np.array([6, 255, 201]) 
            #     red_mask1 = cv2.inRange(hsv_bbox, low_red, high_red)
                
            #     low_red2 = np.array([170, 172, 63])  
            #     high_red2 = np.array([180, 255, 201]) 
            #     red_mask2 = cv2.inRange(hsv_bbox, low_red2, high_red2)
            #     red_mask=red_mask1+red_mask2
            #     red_pixels = np.count_nonzero(red_mask)

            #     low_green = np.array([63, 145, 45]) 
            #     high_green = np.array([96, 255, 255])  
            #     green_mask = cv2.inRange(hsv_bbox, low_green, high_green)
            #     green_pixels = np.count_nonzero(green_mask)

    
            #     low_blue = np.array([98, 153, 74])
            #     high_blue = np.array([101, 255, 255])
            #     blue_mask = cv2.inRange(hsv_bbox, low_blue, high_blue)
            #     blue_pixels = np.count_nonzero(blue_mask)

            #     low_wooden = np.array([9, 10, 67])  
            #     high_wooden = np.array([15, 79, 252]) 
            #     wooden_mask = cv2.inRange(hsv_bbox, low_wooden, high_wooden)
            #     wooden_pixels = np.count_nonzero(wooden_mask)

            #     # Check which color has more pixels
            #     if red_pixels > green_pixels and red_pixels > blue_pixels and red_pixels > wooden_pixels:
            #         color_label = "red"
            #     elif green_pixels > red_pixels and green_pixels > blue_pixels and green_pixels > wooden_pixels:
            #         color_label = "green"
            #     elif blue_pixels > red_pixels and blue_pixels > green_pixels and blue_pixels > wooden_pixels:
            #         color_label = "blue"
            #     elif wooden_pixels > red_pixels and wooden_pixels > green_pixels and wooden_pixels > blue_pixels:
            #         color_label = "wooden"
            #     else:
            #         color_label = None
            #     return color_label
            # else:
            #     return

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