#collect 5-10 pt clouds from rosbag, merge them, and then publish the merged pointcloud (similar to calib_test_node) to RVIZ
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
from sensor_msgs_py import point_cloud2 as pc2
from pt_selection_pkg.pointcloud2_to_pcd_file import *
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pickle
import os
import cv2

DISTANCE_THRESHOLD = 200000 #UNIT IN METERS
PCD_COUNT = 1

# COMMAND TO PLAY ROS BAG IN ROAR MACHINE: ros2 bag play /home/roar/rosbag2_2023_01_06-15_39_44/rosbag2_2023_01_06-15_39_44_0.mcap -s mcap -l

class pt_collection_node(Node):

    def __init__(self):
        super().__init__('calibration_subscriber_node')

        # ---------------------------------------------------------------------------- #
        #                                  PARAMETERS                                  #
        # ---------------------------------------------------------------------------- #
        self.threshold         = PCD_COUNT # number of pointclouds to accumulate
        self.collection_period = 0.2  # seconds
        self.output_path = os.path.join("/home/art-berk/race_common/src/perception/IAC_Perception/src/lidar_camera_calib/" + f"{self.threshold}_LIVE_pdc_calib_data.pkl") # If you change this, also change the one in calibration.py
        print(f"output path: {self.output_path}")
        self.out_data = {
            'front points': [],
            'left points': [],
            'right points': [],
            'rot_numpy': np.array([]),
            'trans_numpy': np.array([]),
            'camera_info_numpy': np.array([]), # enter the camera matrix K, either here or in pt_selection_calibration.py
            'dist_coeffs_numpy': np.array([]), # enter the distortion coefficients, either here or in pt_selection_calibration.py
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
        self.received_flc = False
        self.received_frc = False
        self.received_fl = False
        self.received_rl = False
        self.received_rr = False
        self.received_fr = False
        
        
        # latest pointcloud 
        self.latest_pc = None
        self.counter = 0
        self.saved_header = None
        self.saved_field = None
        
        # ---------------------------------------------------------------------------- #
        #                                  SUBSCRIBER                                  #
        # ---------------------------------------------------------------------------- #
        # LIDAR POINTCLOND SUBSCRIBER
        ####### FOR NOW ONLY FRONT LIDAR???
        self.pcd_file_front = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_front_points",  # Subscribes from front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file_front  # Prevent unused variable warning

        
        self.pcd_file_left = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_left_points",  # Subscribes from left lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file_left  # Prevent unused variable warning


        self.pcd_file_right = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_right_points",  # Subscribes from right lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file_right  # Prevent unused variable warning
        
        
        # CAMERA IMAGE SUBSCRIBERS
        # subscribe to image file to be undistorted
        self.flc = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_front_left_center/image/compressed",  # Subscribes from front left center image
            self.flc_sub,
            self.qos_profile)
        self.flc

        self.frc = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_front_right_center/image/compressed",  # Subscribes from front right center image
            self.frc_sub,
            self.qos_profile)
        self.frc
        
        self.fl = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_front_left/image/compressed",  # Subscribes from front left image
            self.fl_sub,
            self.qos_profile)
        self.fl
        
        self.fr = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_front_right/image/compressed",  # Subscribes from front right image
            self.fr_sub,
            self.qos_profile)
        self.fr
    
        self.rl = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_rear_left/image/compressed",  # Subscribes from rear left image
            self.rl_sub,
            self.qos_profile)
        self.rl
        
        self.rr = self.create_subscription(
            sensor_msgs.CompressedImage,
            "/vimba_rear_right/image/compressed",  # Subscribes from rear right image
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
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_flc = True
        self.out_data['camera_images_flc'] = cv2_msg
    
    def frc_sub(self, msg):
        print("Received frc image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_frc = True
        self.out_data['camera_images_frc'] = cv2_msg
        #self.out_data['camera_images_frc'] = self.bridge.imgmsg_to_cv2(msg)
        
    def fl_sub(self, msg):
        print("Received fl image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_fl = True
        self.out_data['camera_images_fl'] = cv2_msg
    
    def fr_sub(self, msg):
        print("Received fr image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_fr = True
        self.out_data['camera_images_fr'] = cv2_msg
        #self.out_data['camera_images_fr'] = self.bridge.imgmsg_to_cv2(msg)
        
    def rl_sub(self, msg):
        print("Received rl image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_rl = True
        self.out_data['camera_images_rl'] = cv2_msg
        #self.out_data['camera_images_rl'] = self.bridge.imgmsg_to_cv2(msg)
        
    def rr_sub(self, msg):  
        print("Received rr image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_msg = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.received_rr = True
        self.out_data['camera_images_rr'] = cv2_msg
        #self.out_data['camera_images_rr'] = self.bridge.imgmsg_to_cv2(msg)

    # collect 1 pointcloud at a time, and add to merged_pcd
    # https://answers.ros.org/question/58626/merging-multiple-pointcloud2/   seems like we can just add pointclouds together?
    def sub_callback_pcd(self, msg): 
        print("Received pointcloud")
        gen2 = read_points(msg, skip_nans=True, field_names=["x", "y", "z"]) # returns a pointcloud Generator
        pts_arr = pc2.read_points(msg, skip_nans=True, field_names=["x", "y", "z"]) # the build in ros2 read_points, return np array
        self.saved_header = msg.header
        self.saved_field = msg.fields
        self.latest_pc = []
        for i in pts_arr:
            if (i[0]*i[0] + i[1]*i[1] + i[2]*i[2]) < DISTANCE_THRESHOLD**2:
                self.latest_pc.append(i)
        print('collecting pointcloud number', self.counter)
        self.collect_10_callback()

    def collect_10_callback(self):
        if self.latest_pc is None:
            return
        if self.counter < self.threshold:
            self.merged_lst+= self.latest_pc
            self.merged_pcd = np.array(self.merged_lst)
            self.counter += 1
        #self.received_flc and self.received_frc and self.received_fl and  self.received_fr and self.received_rl and self.received_rr
        if self.counter >= self.threshold and self.received_frc and self.received_fr:
            
            # We have collected threshold number of pointclouds
            self.out_data['front points'] = self.merged_pcd
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
