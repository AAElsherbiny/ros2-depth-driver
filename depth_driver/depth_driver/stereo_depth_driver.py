from depth_driver.depth_driver import depth_driver
from camera_drivers.cameradriver import cameradriver
import cv2
import rclpy
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge 


BASELINE = 6.3
FOCAL_LENGTH = 194
Fx = Fy = FOCAL_LENGTH 
CX = 320
CY = 180

qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

class stereoDepthPub(depth_driver):
    def __init__(self, node_name = "stereo_depth_node", left_stereo_object=None, right_stereo_object=None, topic_name = "stereo_depth", compressed = False):
        
        self.canvas_dispar = None
        self.compressed_flag=compressed
        self.frame_timestamp = None

        self.left_subscriber = left_stereo_object
        self.right_subscriber = right_stereo_object

        super().__init__(node_name)

        self.timer = self.create_timer(1.0 / 30.0, self.__subscribeToCamera)

        self.topic_name = topic_name
        self.compressed = compressed
        self.br = CvBridge()
        self.init_publisher()

    def __subscribeToCamera(self):
        left_frame = self.left_subscriber.get_frame()
        right_frame = self.right_subscriber.get_frame()

        # Capture timestamp from left frame's message header
        if self.left_subscriber.msg is not None:
            self.frame_timestamp = self.left_subscriber.msg.header.stamp

        imgL = left_frame
        imgR = right_frame
        
        if imgL is None or imgR is None:
            print("No frame recieved")
            return

        # Resize to 1280x720
        fixed_dim = (640, 360)
        imgL, imgR = cv2.resize(imgL, fixed_dim), cv2.resize(imgR, fixed_dim)
        
        self.__proccess_depth(imgL, imgR)

    def __proccess_depth(self, imgL, imgR):
        # Convert to grayscale
        gray_imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        gray_imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        # Calculate disparity
        disparity_map = self.__return_disparity(gray_imgL, gray_imgR)
        self.depth_frame = self.__return_depth(disparity_map)
        self.publish()

        # Create canvas for visualization
        self.canvas_dispar = super().normalize(disparity_map)

    def __return_disparity(self, imgL, imgR):
        """
        Calculate the disparity of the input images
        """
        dim = imgL.shape[::-1]

        low_params = {"dimension": (320, 180), "kernel": 5, "block": 15,
                "min_disparity": 0, "disparity": 5}
        low_disparity = self.__calculate_disparity(imgL, imgR, low_params)

        low = self.__scale(cv2.resize(low_disparity, dim), low_params["min_disparity"], low_params["disparity"])
        low = low[:,160:]
        return low
    
    def __calculate_disparity(self, imgL, imgR, params):
        """
        Calculate the disparity given a specific dimension and parameters
        """
        # Deep copy
        imgL, imgR = imgL.copy(), imgR.copy()
        
        # Resize
        dim = params["dimension"]
        resized_imgL, resized_imgR = cv2.resize(imgL, dim), cv2.resize(imgR, dim) # Resize

        # Get parameters
        kernel, min_dispar, dispar, block = params["kernel"], params["min_disparity"], params["disparity"], params["block"]

        # Blur images to remove noises
        blur_imgL = cv2.GaussianBlur(resized_imgL, (kernel, kernel), 0)
        blur_imgR = cv2.GaussianBlur(resized_imgR, (kernel, kernel), 0)

        # Calculate disparity
        stereo = cv2.StereoSGBM_create(
            minDisparity = 16 * min_dispar,
            numDisparities = 16 * dispar,
            blockSize = block,
            speckleRange = 2,
        P1 = 8 * 3 * kernel ** 2, P2 = 32 * 3 * kernel ** 2)

        # Return disparity
        return stereo.compute(blur_imgL, blur_imgR) 
    
    def __scale(self, value, min_dispar, num_dispar):
        """
        Scale the disparity from pixels to pixel/pixel
        """
        return (value / (16 ** 2) - min_dispar) / num_dispar  #cm/cm
    
    def __return_depth(self, disparity):
        """
        Calculate depth map knowing the disparity map, the camera's focal lens, and the baseline.
        Thanks to maths, we can also use this function to calculate disparity map knowing the rest.
        """
        
        # Cast type of disparity
        disparity = float(disparity) if type(disparity) == int else disparity
        
        # Calculate depth map
        depth = np.divide(BASELINE * FOCAL_LENGTH, 10*disparity, out = np.zeros_like(disparity), where = disparity != 0.0)

        return depth  # mm # hopefully
    
    def get_depth(self, y, x, normalize = True, draw_circle = True):
        
        depth_value = self.depth_frame[y,x]

        # Draw the circle
        if draw_circle:
            center_coordinates = (x, y)
            radius = 10
            color = (255, 255, 255)
            thickness = 2
            if normalize:
                cv2.circle(self.canvas_dispar, center_coordinates, radius, color, thickness)    
                return self.canvas_dispar, depth_value
            else:
                cv2.circle(self.depth_frame, center_coordinates, radius, color, thickness)    
                return self.depth_frame, depth_value

        else:
            if normalize:
                return self.canvas_dispar, depth_value
            else:
                return self.depth_frame, depth_value

    def normalize(self,frame):
        return cv2.bitwise_not(self.canvas_dispar)

    def spin(self):
        rclpy.spin_once(self)
        rclpy.spin_once(self.left_subscriber, timeout_sec=0.1)
        rclpy.spin_once(self.right_subscriber, timeout_sec=0.1)


    def init_publisher(self): 
        if self.compressed:
            self.publisher = self.create_publisher(CompressedImage, self.topic_name, qos_profile)
        else:
            self.publisher = self.create_publisher(Image, self.topic_name, qos_profile)
        print("initialized depth_publisher node")


    def publish(self):
    
        while self.depth_frame is None:
            self.get_logger().error("Depth frame is empty")
        
        if self.compressed:
            compressed_frame = self.compress_frame(self.depth_frame)
            if compressed_frame is not None:
                img_msg = CompressedImage()
                img_msg.header.stamp = self.frame_timestamp
                img_msg.format = "jpeg"
                img_msg.data = compressed_frame
                self.publisher.publish(img_msg)     
                self.get_logger().info("Published a compressed frame")
                return
        else:
            img_msg = self.br.cv2_to_imgmsg(self.depth_frame, encoding='64FC1')
            img_msg.header.stamp = self.frame_timestamp
            self.publisher.publish(img_msg)
            self.get_logger().info("Published a frame")

    def compress_frame(self, frame):
        if frame is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            if result:
                return encimg.tobytes()
            else:
                self.get_logger().warn("Failed to compress frame")



class StereoDepthDriver(cameradriver):
    def __init__(self,subscriber=True, client=False, topic="stereo_depth", node_name="stereoDepth_node_sub", service="stereoDepth_service"):
        super().__init__(subscriber, client, topic, node_name, service)
        super().set_mode()
        self.depth_frame = None

    def get_depth(self, y, x, normalize = True, draw_circle = True):
        self.depth_frame = super().get_frame()

        if self.depth_frame is None:
            self.get_logger().warn("No depth frame received yet to get depth value")
            return None
        
        depth_estimation = self.depth_frame
        depth_value = depth_estimation[y, x]

        if normalize:
            depth_estimation = self.normalize(depth_estimation)
        if draw_circle:
            center_coordinates = (x, y)  
            radius = 10
            color = (255, 255, 255)
            thickness = 2                   
            cv2.circle(depth_estimation, center_coordinates, radius, color, thickness)
        
        return depth_estimation, depth_value

    def get_frame(self, normalize = False):
        self.depth_frame = super().get_frame()
        if self.depth_frame is None:
            self.get_logger().warn("No depth frame received yet to get depth value")
            return None
        
        if normalize:
            return self.normalize(self.depth_frame)
        else:
            return self.depth_frame

    def normalize(self, frame):

        if frame is None:
            self.get_logger().warn("No frame received yet to normalize")
            return
        self.normalized_frame = frame
        (min_val, max_val, _, _) = cv2.minMaxLoc(self.normalized_frame)
        if max_val == 0: max_val = 1
        self.normalized_frame = cv2.convertScaleAbs(self.normalized_frame, alpha=255.0/max_val, beta=- min_val * 255.0/max_val)

        return self.normalized_frame
    
    def display(self, frame, frame_name = "frame"):
        cv2.imshow(frame_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    def get_internsic_values(self):
        return Fx , Fy, CX, CY