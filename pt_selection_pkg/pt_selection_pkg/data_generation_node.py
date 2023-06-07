#collect 5-10 pt clouds from rosbag, merge them, and then publish the merged pointcloud (similar to calib_test_node) to RVIZ
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
from pt_selection_pkg.pointcloud2_to_pcd_file import *
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pickle
import os

class pt_collection_node(Node):

    def __init__(self):
        super().__init__('calibration_subscriber_node')

        # ---------------------------------------------------------------------------- #
        #                                  PARAMETERS                                  #
        # ---------------------------------------------------------------------------- #
        self.threshold         = 10 # number of pointclouds to accumulate
        self.collection_period = 0.2  # seconds
        self.output_path = os.path.join(os.getcwd(), str(self.threshold) + "_pdc_calib_data.pkl") # If you change this, also change the one in calibration.py
        print(f"output path: {self.output_path}")
        self.out_data = {
            'points': [],
            'rot_numpy': np.array([]),
            'trans_numpy': np.array([]),
            'dist_coeffs_numpy': np.array([]),
            'camera_images_flc': [],
            'camera_images_frc': [],
            'camera_images_fr': [],
            'camera_images_fl': [],
            'camera_images_rl': [],
            'camera_images_rr': []
        }
        # ---------------------------------------------------------------------------- #
        #                                PARAMETERS END                                #
        # ---------------------------------------------------------------------------- #

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # merged pointcloud "buffer", as a PointCloud2 object
        self.merged_pcd = None
        self.merged_lst = []
        
        # latest pointcloud 
        self.latest_pc = None
        self.counter = 0
        self.saved_header = None
        self.saved_field = None
        
        # ---------------------------------------------------------------------------- #
        #                                  SUBSCRIBER                                  #
        # ---------------------------------------------------------------------------- #
        # LIDAR POINTCLOND SUBSCRIBER
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_front_points",  # Subscribes from front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file  # Prevent unused variable warning
        
        # CAMERA IMAGE SUBSCRIBERS
        # subscribe to image file to be undistorted
        self.flc = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_left_center/image",  # Subscribes from front left center image
            self.flc_sub,
            self.qos_profile)
        self.flc

        self.frc = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_right_center/image",  # Subscribes from front left center image
            self.frc_sub,
            self.qos_profile)
        self.frc
        
        self.fl = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_left/image",  # Subscribes from front left center image
            self.fl_sub,
            self.qos_profile)
        self.fl
        
        self.fr = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_right/image",  # Subscribes from front left center image
            self.fr_sub,
            self.qos_profile)
        self.fr
    
        self.rl = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_rear_left/image",  # Subscribes from front left center image
            self.rl_sub,
            self.qos_profile)
        self.rl
        
        self.rr = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_rear_right/image",  # Subscribes from front left center image
            self.rr_sub,
            self.qos_profile)
        self.rr

        # collect 10 frames of point cloud, one per second, merge them
        self.timer_collect_10 = self.create_timer(self.collection_period, self.collect_10_callback)

        print("Calibration subscriber node initialized")

    # ---------------------------------------------------------------------------- #
    #                              CALLBACK FUNCTIONS                              #
    # ---------------------------------------------------------------------------- #
    # Just collect the images as it is. 
    def flc_sub(self, msg):
        print("Received flc image")
        self.out_data['camera_images_flc'] = self.bridge.imgmsg_to_cv2(msg)
    
    def frc_sub(self, msg):
        self.out_data['camera_images_frc'] = self.bridge.imgmsg_to_cv2(msg)
        
    def fl_sub(self, msg):
        self.out_data['camera_images_fl'] = self.bridge.imgmsg_to_cv2(msg)
    
    def fr_sub(self, msg):
        self.out_data['camera_images_fr'] = self.bridge.imgmsg_to_cv2(msg)
        
    def rl_sub(self, msg):
        self.out_data['camera_images_rl'] = self.bridge.imgmsg_to_cv2(msg)
        
    def rr_sub(self, msg):  
        self.out_data['camera_images_rr'] = self.bridge.imgmsg_to_cv2(msg)

    # collect 1 pointcloud at a time, and add to merged_pcd
    # https://answers.ros.org/question/58626/merging-multiple-pointcloud2/   seems like we can just add pointclouds together?
    def sub_callback_pcd(self, msg): 
        print("Received pointcloud")
        #calibration_subscriber_node.glob_pcd_file = None
        gen2 = read_points(msg, skip_nans=True) # returns a pointcloud Generator
        self.saved_header = msg.header
        self.saved_field = msg.fields
        self.latest_pc = list(gen2)
        print('collecting pointcloud number', self.counter)
        self.collect_10_callback()

    def collect_10_callback(self):
        if self.latest_pc is None:
            return
        if self.counter < self.threshold:
            self.merged_lst+= self.latest_pc
            self.merged_pcd = np.array(self.merged_lst)
            self.counter += 1
        
        else: 
            
            # We have collected threshold number of pointclouds
            self.out_data['points'] = self.merged_pcd
            print("Collected", self.counter, "pointclouds")
            
            # Save all the data collected
            with open(self.output_path, 'wb') as f:
                pickle.dump(self.out_data, f)
                print("Saved data to", self.output_path)
            # Shutdown the node
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
            
    def on_shutdown(self):
        # Perform any necessary cleanup operations here
        pass

def main(args=None):
    print("Starting collection node")
    rclpy.init(args=args)

    node = pt_collection_node()
    rclpy.spin(node)

    # Call the on_shutdown() method for cleanup
    node.on_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
