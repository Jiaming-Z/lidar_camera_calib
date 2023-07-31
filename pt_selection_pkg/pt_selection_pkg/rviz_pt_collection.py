#collect 5-10 pt clouds from rosbag, merge them, and then publish the merged pointcloud (similar to calib_test_node) to RVIZ
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
from pt_selection_pkg.pointcloud2_to_pcd_file import *
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.timer import Timer
from sensor_msgs_py import point_cloud2 as pc2
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
        self.merged_lst = []
        # latest pointcloud 
        self.latest_pc = None

        
        self.counter = 0
        self.threshold = 10
        self.saved_header = None
        self.saved_field = None
        
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

        # collect 10 frames of point cloud, one per second, merge them
        self.timer_collect_10 = self.create_timer(1, self.collect_10_callback)

    # collect 1 pointcloud, add to merged_pcd
    # https://answers.ros.org/question/58626/merging-multiple-pointcloud2/   seems like we can just add pointclouds together?
    def sub_callback_pcd(self, PointCloud2): 
        
        gen2 = pc2.read_points(PointCloud2, skip_nans=True) # returns a pointcloud Generator
        self.saved_header = PointCloud2.header
        self.saved_field = PointCloud2.fields

        self.latest_pc = np.array(list(gen2))
        if len(self.latest_pc) == 0:
            print("ERROR: Latest Pointcloud is empty, check Lidar Messages")
            return

        print('collecting pointcloud number', self.counter)
        self.collect_10_callback()

    def collect_10_callback(self):
        
        if self.counter < self.threshold:

            if self.latest_pc is None:
                print("ERROR: Latest Pointcloud is empty, check Lidar Messages")
                return

            self.merged_lst+=list(self.latest_pc)
            self.merged_pcd = np.array(self.merged_lst)
            self.counter += 1
        else:
            self.merged_publisher_callback()
            self.timer_collect_10.cancel()
    
    # publish the merged pointclouds 
    def merged_publisher_callback(self):
        
        self.merged_pcd_publisher.publish(create_cloud(self.saved_header, self.saved_field, self.merged_pcd))

def main(args=None):
    rclpy.init(args=args)

    node = pt_collection_node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
