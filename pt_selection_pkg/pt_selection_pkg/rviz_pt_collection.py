#collect 5-10 pt clouds from rosbag, merge them, and then publish the merged pointcloud (similar to calib_test_node) to RVIZ

class pt_collection_node(Node):

    def __init__(self):
        super().__init__('calibration_subscriber_node')

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest_im = None
        self.latest_pc = None

        # Pointcloud buffer
        self.buffer_front = []
        self.buffer_left = []
        self.buffer_right = []
        self.counter = 0
        self.threshold = 5
        
        
        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_front_points",  # Subscribes from front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file  # Prevent unused variable warning

        self.merged_pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2,
            "/merged_pointcloud",
            rclpy.qos.qos_profile_sensor_data)


    def sub_callback_pcd(self, PointCloud2):

        #calibration_subscriber_node.glob_pcd_file = None
        self.latest_pc = None
        gen2 = read_points(PointCloud2, field_names=['x', 'y', 'z'], skip_nans=True) # returns a pointcloud Generator
        #gen = pointcloud2_to_pcd_file.read_points(PointCloud2, skip_nans=True) # returns a pointcloud Generator
        #self.pc_total = np.array(list(gen2))
        self.latest_pc = np.array(list(gen2))
        file_path = '/home/jiamingzhang/temp.pkl'
        with open(file_path, 'wb') as f:
            pickle.dump(self.latest_pc, f)
        self.buffer_front.append(PointCloud2)
        
    
    # merge 5 point clouds, publish them
    def merged_publisher_callback(self, PointCloud2):
        self.merged_pcd = 
        for i in range(5):
            self.merged_pcd = 
        
        self.merged_pcd_publisher.publish(merged_pcd)
