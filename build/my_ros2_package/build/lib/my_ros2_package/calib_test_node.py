import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
import threading
import matplotlib.pyplot as plt

import pickle

# import pointcloud2_to_pcd_file
import os 
#print(os.getcwd())
# from pointcloud2_to_pcd_file import read_points
from my_ros2_package.pointcloud2_to_pcd_file import *
#from my_ros2_package.params import *

from scipy.spatial.transform import Rotation as R

class calibration_subscriber_node(Node):

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
        
        self.latest_projection = None
        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/luminar_front_points",  # Subscribes from front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.pcd_file  # Prevent unused variable warning

        # Subscribe to image file
        self.ros_img_front = self.create_subscription(
            sensor_msgs.Image,
            "/vimba_front_left_center/image",  # Subscribes from front left center image
            self.sub_callback_img,
            self.qos_profile)
        self.ros_img_front  # Prevent unused variable warning

        self.bridge = CvBridge()


        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publish image to rvidz
        self.im_publisher = self.create_publisher(
            sensor_msgs.Image,
            "/vimba_front_left_center/image_undistorted",
            rclpy.qos.qos_profile_sensor_data)

        self.timer_pub = self.create_timer(0.5, self.publisher_im_callback)

        self.overlay_publisher = self.create_publisher(
            sensor_msgs.Image,
            "/vimba_front_left_center/image_overlay",
            rclpy.qos.qos_profile_sensor_data)

        self.timer_pub_overlay = self.create_timer(0.5, self.publisher_overlay_callback)


        ###PLOT THREAD!
        #self.plot_thread = PlotThread(self.x_data, self.y_data)


    def sub_callback_img(self, Image):
        print("subscribed to image")
        self.latest_im  = None
        try:
            self.latest_im  = self.bridge.imgmsg_to_cv2(Image)
            
        except CvBridgeError as e:
            print(e)

    def sub_callback_pcd(self, PointCloud2):
        print("subscribed to pointcloud")

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
        

        if(self.latest_im is not None and self.latest_pc is not None):
            self.projection()
        
    def publish_static_merged_pointcloud():
        # Merge the pointclouds in self.buffer_front
        pass
        # Publish it indefinitely

    def undistort(self):
        # https://github.com/Chrislai502/Lidar_Camera_Calibration/blob/main/pcd_image_overlay_Chris.py
    
        cam_image = self.latest_im
        camera_info = self.camera_info
        dist_coeffs = np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000])
        img_size = (cam_image.shape[1], cam_image.shape[0])
        new_K, _ = cv.getOptimalNewCameraMatrix(camera_info, dist_coeffs, img_size, alpha=1)
        image_undistorted = cv.undistort(cam_image, camera_info, dist_coeffs, None, new_K)
        return image_undistorted

    def publisher_im_callback(self):
        im_undistorted_cv = self.undistort()
        try:
            im_undistorted_imgmsg = self.bridge.cv2_to_imgmsg(im_undistorted_cv)
        except CvBridgeError as e:
            print(e) 
            return
        print('published in rviz as topic "/vimba_front_left_center/image"')
        self.im_publisher.publish(im_undistorted_imgmsg)

    def publisher_overlay_callback(self):
        im_undistorted_cv = self.undistort()

        #ptc_z_camera = np.transpose(self.pc_total)[:,2]
        ptc_xy_camera = self.latest_projection
        # z_min=np.min(ptc_z_camera)
        # z_range=np.max(ptc_z_camera)-z_min
        # ptc_z_camera=(ptc_z_camera-z_min)*255/z_range
        # ptc_z_camera=ptc_z_camera.astype(np.uint8)
        # color=cv2.applyColorMap(ptc_z_camera[:,np.newaxis],cv2.COLORMAP_HSV)
        #r=color.shape[0]
        print("accessed publisher")
        # file_path = '/home/jiamingzhang/temp.pkl'
        # with open(file_path, 'wb') as f:
        #     pickle.dump(ptc_xy_camera, f)

        for j in range(ptc_xy_camera.shape[0]):
            #print(ptc_xy_camera.shape)
            i=ptc_xy_camera[j]
            #print(i)
            c=np.array([0.0, 0.0, 255.0])
            a = int(np.floor(i[0]))
            b = int(np.floor(i[1]))
            #print("a,b", a, b)
            img_shape = im_undistorted_cv.shape
            if a>0 and b>0 and a < img_shape[1] and b < img_shape[0]:
                print("a,b", a, b)
                print('shape', im_undistorted_cv.shape)
                # try:

                # temp = im_undistorted_cv[b,a]
                
                alp = 0.5
                im_undistorted_cv[b-1:b+1,a-1:a+1] = c*alp + im_undistorted_cv[b-1:b+1,a-1:a+1] * (1-alp)
                
                print('within range')
                # except:
                #     continue
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(im_undistorted_cv))
 

    def projection(self):
        im = self.latest_im
        #im = self.undistort()
        pc = self.latest_pc
        #print("hi")
        self.undistort()
        
        from_frame_rel = 'luminar_front' #params.lidar_frame
        to_frame_rel = 'vimba_front_left_center' #params.camera_frame

        tf_transform = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time())
        
        t_vec = tf_transform.transform.translation
        t_lid = np.array([t_vec.x, t_vec.y, t_vec.z])

        quat_R = tf_transform.transform.rotation

        r = R.from_quat([quat_R.x, quat_R.y, quat_R.z, quat_R.w])
        
        R_matrix = r.as_dcm()

        print("R", R_matrix)
        print("t in lidar frame", t_lid)
        t_cam = np.array([t_lid[2], -t_lid[0], -t_lid[1]])
        print("t in camera frame", t_cam)
        t_chris = np.array([ 0.017, -0.016, 0.156])
        t_chris_cam = np.array([t_chris[2], -t_chris[0], -t_chris[1]])
        t_tiled = np.tile(t_chris_cam, (pc.shape[0], 1))
        projected_points = np.dot(self.camera_info, np.dot(R_matrix, (pc.transpose() + t_tiled.transpose()))).transpose()
        #projected_points = np.dot(self.camera_info, np.dot(R_matrix, (pc.transpose() + t_tiled.transpose()))).transpose()
        #print(projected_points.shape) # (28263, 3)
        normalized_points = np.array([[i[0]/i[2], i[1]/i[2], 1] for i in projected_points])

        print("normalized projected image vector", normalized_points)
        #print("actual image vector", im)
        self.latest_projection = normalized_points

   

'''
class PlotThread(threading.Thread):
    def __init__(self, x_data, y_data):
        threading.Thread.__init__(self)
        self.x_data = x_data
        self.y_data = y_data
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.line, = self.ax.plot(self.x_data, self.y_data)

    def getxyz(event):
    
        # store the current mousebutton
        b = ax.button_pressed
        # set current mousebutton to something unreasonable
        ax.button_pressed = -1
        # get the coordinate string out
        s = ax.format_coord(event.xdata, event.ydata)
        #    set the mousebutton back to its previous state
        ax.button_pressed = b
        out = ""
        print(s)
    # Connect the mouse click event to the onclick function
    cid = fig.canvas.mpl_connect('button_press_event', getxyz)

    # Show the plot
    plt.show()

    def run(self):
        while not rospy.is_shutdown():
            # Update the plot data
            self.line.set_xdata(self.x_data)
            self.line.set_ydata(self.y_data)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

'''

##NEXT STEPS:
### this is point collecting node, 
### clicking points: node subscribe to /clicked_point topic, and in rviz "publish point", click to see point
### Merge point clouds: collect 5 pt clouds, merge them, then publish them (in a seperate node or seperate thread)
### 

def main(args=None):
    
    rclpy.init(args=args)
    
    calibration_subscriber = calibration_subscriber_node()
    
    rclpy.spin(calibration_subscriber)


