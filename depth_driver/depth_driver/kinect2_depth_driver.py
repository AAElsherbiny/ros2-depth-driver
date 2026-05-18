from depth_driver.depth_driver import depth_driver
import rclpy

Fx =  365.6
Fy =  365.36
Cx =  248.82
Cy =  208.63

class kinect_v2_depth(depth_driver):
    def __init__(self, kinect_object, node_name = "kinect2_depth_node"):

        self.kinect_v2_subscriber = kinect_object
        super().__init__(node_name)
    
    def spin(self, timeout):
        rclpy.spin_once(self.kinect_v2_subscriber, timeout_sec=timeout)

        frame = self.kinect_v2_subscriber.get_depth_frame()
        timestamp = self.kinect_v2_subscriber.get_timestamp()
        if frame is not None:
            self.depth_frame = frame
            self.depth_timestamp = timestamp

    def get_internsic_values(self) :
        return Fx , Fy ,Cx , Cy