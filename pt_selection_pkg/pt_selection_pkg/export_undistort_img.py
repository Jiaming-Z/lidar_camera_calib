import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

class exportUndistortImg(Node):

    def __init__(self):
        super().__init__('undistort_img_node')

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        
        # subscribe to image file to be undistorted
        self.ros_img_front = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_left_center/image",  # Subscribes from front left center image
            self.sub_callback_img,
            self.qos_profile)

        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])

        self.bridge = CvBridge()
        self.latest_im  = None
        self.img_undistorted = None


    def sub_callback_img(self, Image):
        #print("subscribed to image")
        self.latest_im  = None
        try:
            self.latest_im  = self.bridge.imgmsg_to_cv2(Image)
            self.img_undistorted = self.undistort(self.latest_im)
            cv2.imwrite("new_image_undistorted.png", self.img_undistorted)
            print("successfully exported undistorted image as new_image_undistorted.png")

        except CvBridgeError as e:
            print(e)

    def undistort(self, image):
    
        K = self.camera_info
        dist_coeffs = np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000])
        img_size = (image.shape[1], image.shape[0])
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, img_size, alpha=1)
        f = open('undistorted_camera_matrix.txt', 'w')
        f.write('undistorted K matrix: '+'\n')
        f.write(np.array2string(new_K, precision=3, separator=',')+'\n')
        image_undistorted = cv2.undistort(image, K, dist_coeffs, None, new_K)
        return image_undistorted


def main(args=None):
    rclpy.init(args=args)
    
    undistort_node = exportUndistortImg()
    
    rclpy.spin_once(undistort_node)
    
    undistort_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
