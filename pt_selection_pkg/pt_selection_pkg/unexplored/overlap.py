import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
from my_ros2_package.pointcloud2_to_pcd_file import *

# publish a overlaped image of lidar and image points, with blue=lidar pts clicked, green=image pts clicked
# also print out in terminal how much are the clicked lidar points in rviz differ in distance from actual lidar points
class overlap_pub_node(Node):
    
    def __init__(self):
        super().__init__('overlap_pub_node')
        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # publish the overlap
        self.overlay_publisher = self.create_publisher(
                sensor_msgs.Image,
                "/vimba_front_left_center/image_overlay",
                rclpy.qos.qos_profile_sensor_data)

        self.latest_pc = None
        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            "/merged_pointcloud",  # Subscribes from MERGED 10 FRAME OF pcd of front lidar
            self.sub_callback_pcd,
            self.qos_profile)
        self.latest_projection = None

        self.bridge = CvBridge()

        pic1_path = "new_image_undistorted.png"
        self.img_undistorted = cv2.imread(pic1_path) # pass in the undistorted (not cropped) image

        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])
        self.undistorted_camera_info = np.array([[827.1930542,   0,        275.14427751],
                                                [  0,        828.20013428, 145.27567362],
                                                [  0,           0,          1     ]])
        self.R = np.array([[-0.02055, 0.99975, 0.00859],
                        [-0.05978,-0.00981, 0.99816],
                        [-0.998  ,-0.02   ,-0.05997]])
        self.t = np.array([-0.01169,-0.29431, 2.32696])

        self.timer_pub_overlay = self.create_timer(0.5, self.publisher_overlay_callback)

        # input this manually
        self.all_pt_list =  [[7.20000000e+01, 4.90000000e+01, 1.41783791e+01, 3.26228428e+00, 2.60112667e+00],
                            [1.70000000e+02, 1.04000000e+02, 1.28549576e+01, 1.64912844e+00, 1.63924956e+00],
                            [9.00000000e+01,  1.45000000e+02,  1.42244320e+01,  3.10702419e+00, 1.13780963e+00],
                            [1.73000000e+02,  1.88000000e+02,  1.28062191e+01,  1.61688614e+00, 5.05442739e-01],
                            [3.09000000e+02,  1.86000000e+02,  2.98137779e+01, -4.27837074e-01, 6.66600466e-01],
                            [3.56000000e+02,  1.88000000e+02,  2.95880947e+01, -1.89676654e+00, 6.55570626e-01],
                            [3.36000000e+02,  1.66000000e+02,  3.11740074e+01, -1.44641638e+00, 1.41120648e+00],
                            [2.86000000e+02,  1.84000000e+02,  5.44212914e+01,  4.86706495e-01, 1.19493735e+00],
                            [1.85000000e+02,  1.02000000e+02,  2.99020710e+01,  3.72472095e+00, 3.55019283e+00],
                            [2.30000000e+02,  1.26000000e+02,  2.85917034e+01,  2.12543201e+00, 2.62642193e+00],
                            [4.05000000e+02,  1.09000000e+02,  1.91215729e+02, -2.54982567e+01, 2.07946014e+01],
                            [4.08000000e+02,  1.49000000e+02,  1.91588745e+02, -2.55737705e+01, 1.15379143e+01],
                            [4.97000000e+02,  9.90000000e+01,  1.67729050e+02, -4.03239059e+01, 1.99132977e+01],
                            [4.99000000e+02,  1.39000000e+02,  1.69607391e+02, -4.12023735e+01, 1.04788647e+01],
                            [1.66000000e+02,  7.60000000e+01,  1.63135773e+02,  2.43768444e+01, 2.40905285e+01],
                            [1.74000000e+02,  2.21000000e+02,  1.27924595e+01,  1.58866215e+00, 5.18536568e-02],
                            [1.08000000e+02,  7.40000000e+01,  1.38679991e+01,  2.64059973e+00, 2.17800713e+00],
                            [2.94000000e+02,  1.54000000e+02,  2.15639252e+02,  3.96171212e-01, 1.09125872e+01],
                            [1.72000000e+02,  1.24000000e+02,  1.28065834e+01,  1.65250087e+00, 1.40220273e+00],
                            [1.18000000e+02,  1.64000000e+02,  1.40416698e+01,  2.57688522e+00, 8.96251678e-01]]
        
        self.sel_img, self.sel_pcd = self.breakup(self.all_pt_list)

        

    def sub_callback_pcd(self, PointCloud2):
        
        self.latest_pc = None
        gen2 = read_points(PointCloud2, field_names=['x', 'y', 'z'], skip_nans=True) # returns a pointcloud Generator
        
        self.latest_pc = np.array(list(gen2))
        print('Wonder why there lidar and image do not exactly align? It is because there are currently inaccuracies when clicking points in rviz')
        print('Here are the minimum distances (in lidar unit) of the selected lidar points (clicked in rviz) and the closest real recieved lidar point to them')
        for pt in self.sel_pcd:
            
            print('min dist for point ',pt, ':')
            print(min([np.linalg.norm(np.array(p)-np.array(pt)) for p in list(self.latest_pc)]))
        
        

        if self.latest_pc is not None:
            self.latest_projection = self.projection(self.R, self.t, self.latest_pc)


    def publisher_overlay_callback(self):
        im_undistorted_cv = self.img_undistorted
        ptc_xy_camera = self.latest_projection
        
        #print(ptc_xy_camera.shape[0])
        for j in range(ptc_xy_camera.shape[0]):
            #print(ptc_xy_camera.shape)
            i=ptc_xy_camera[j]
            #print(i)
            c=np.array([0.0, 0.0, 255.0])
            a = int(np.floor(i[0]))
            b = int(np.floor(i[1]))
            
            img_shape = im_undistorted_cv.shape
            
            if a>0 and b>0 and a < img_shape[1] and b < img_shape[0]:
                #print("a,b", a, b)
                #print('shape', im_undistorted_cv.shape)
                # try:

                # temp = im_undistorted_cv[b,a]
                
                alp = 0.5
                im_undistorted_cv[b-1:b+1,a-1:a+1] = c*alp + im_undistorted_cv[b-1:b+1,a-1:a+1] * (1-alp)
                
                # except:
                #     continue
        for i in range(len(self.sel_img)):
            im_pt = (int(self.sel_img[i][0]), int(self.sel_img[i][1]))
            pc_pt = (int(self.projection(self.R, self.t, self.sel_pcd)[i][0]), int(self.projection(self.R, self.t, self.sel_pcd)[i][1]))
            cv2.circle(im_undistorted_cv, im_pt, 1, (0, 255, 0), 2)
            cv2.circle(im_undistorted_cv, pc_pt, 1, (255, 0, 0), 2)
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(im_undistorted_cv))

    # project the lidar frame into camera image frame, using the undistorted camera info matrix
    def projection(self, R, t, pcd):
        K = self.undistorted_camera_info 
        def toCartes(a):
            return np.array([a[0]/a[2], a[1]/a[2]])
        
        rotated = np.dot(R, np.transpose(pcd))

        transformed = np.transpose(rotated)+t
        
        result = np.transpose(toCartes(np.dot(K, np.transpose((np.transpose(rotated)+t)))))
        #print('projected result', result)
        return result

    # Break up the [[u,v,x,y,z]...] self.all_pt_list into [[u,v]...] and [[x,y,z]...]
    def breakup(self, l):
        l1 = []
        l2 = []
        for a in l:
            l1.append([a[0],a[1]])
            l2.append([a[2],a[3],a[4]])
        return l1, l2
        






def main(args=None):
    
    rclpy.init(args=args)
    
    node = overlap_pub_node()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
