#collect 5-10 pt clouds from rosbag, merge them, and then publish the merged pointcloud (similar to calib_test_node) to RVIZ
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.timer import Timer
class pt_collection_node(Node):

    def __init__(self):
        super().__init__('calibration_subscriber_node')

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # merged pointcloud "buffer", as a PointCloud2 object
        self.merged_pcd = None
        # latest pointcloud 
        self.latest_pc = None

        
        self.counter = 0
        self.threshold = 5
        
        
        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_front_points",  # Subscribes from front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file  # Prevent unused variable warning

        # publish merged pointclouds
        self.merged_pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2,
            "/merged_pointcloud",
            rclpy.qos.qos_profile_sensor_data)

        # collect 5 frames of point cloud, one per second, merge them
        self.timer_collect_5 = self.create_timer(1, self.sub_callback_pcd)

    # collect 1 pointcloud, add to merged_pcd
    # https://answers.ros.org/question/58626/merging-multiple-pointcloud2/   seems like we can just add pointclouds together?
    def sub_callback_pcd(self, PointCloud2):

        #calibration_subscriber_node.glob_pcd_file = None
        self.latest_pc = None
        gen2 = read_points(PointCloud2, field_names=['x', 'y', 'z'], skip_nans=True) # returns a pointcloud Generator
        
        self.latest_pc = np.array(list(gen2))
        
        if self.counter < self.threshold:
            self.merged_pcd += self.latest_pc
        else:
            self.merged_publisher_callback()
            self.timer_collect_5.cancel()

        self.counter += 1

        
    
    # publish the 5 merged pointclouds 
    def merged_publisher_callback(self):
        
        self.merged_pcd_publisher.publish(self.merged_pcd)
