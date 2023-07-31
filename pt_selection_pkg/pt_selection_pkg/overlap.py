import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
from sensor_msgs_py import point_cloud2 as pc2

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
                "/vimba_front_right_center/image_overlay",
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

        # input this parameter
        self.undistorted_camera_info = np.array([[1.57700e+03,0.00000e+00,5.36020e+02],
 [0.00000e+00,1.58130e+03,3.20674e+02],
 [0.00000e+00,0.00000e+00,1.00000e+00]])
        # input this parameter
        self.R = np.array([[-2.55018e-02,-9.99675e-01,-2.50639e-04],
 [ 4.77680e-02,-9.68132e-04,-9.98858e-01],
 [ 9.98533e-01,-2.54847e-02, 4.77771e-02]])
        # input this parameter
        self.t = np.array([-0.12104,-0.05424,-0.22951])

        self.timer_pub_overlay = self.create_timer(0.5, self.publisher_overlay_callback)

        # input this parameter
        self.all_pt_list = [[501, 563, 3.7860517501831055, -0.1314089149236679, -0.36725127696990967], [520, 350, 3.9691531658172607, -0.17884023487567902, 0.08687365055084229], [847, 344, 3.713683843612671, -0.9109978079795837, 0.07670542597770691], [844, 573, 3.5421509742736816, -0.8619372248649597, -0.3618428409099579], [564, 378, 3.879394054412842, -0.28289592266082764, 0.00736922025680542], [819, 370, 3.6950583457946777, -0.8444523811340332, 0.022561609745025635], [869, 437, 2.133234977722168, -0.6032557487487793, -0.07292886823415756], [915, 448, 1.7366533279418945, -0.5685082674026489, -0.06734936684370041], [816, 458, 3.596477508544922, -0.816709578037262, -0.17254026234149933], [796, 411, 3.6517751216888428, -0.7801579833030701, -0.06444092094898224], [648, 318, 7.293030738830566, -0.781611442565918, 0.2882116436958313], [431, 408, 9.324764251708984, 0.24158716201782227, -0.20837688446044922], [479, 411, 8.994609832763672, -0.011698246002197266, -0.09594982862472534], [442, 332, 9.475360870361328, 0.20123255252838135, 0.32087957859039307], [491, 322, 9.084478378295898, -0.07679009437561035, 0.32754647731781006], [464, 308, 9.379240036010742, 0.08596083521842957, 0.44552862644195557], [546, 310, 9.29450798034668, -0.4120352864265442, 0.4373873472213745], [498, 470, 3.829281806945801, -0.13060981035232544, -0.2137560248374939], [583, 417, 3.8361122608184814, -0.3209671378135681, -0.07248884439468384], [646, 376, 3.803565740585327, -0.4659099578857422, 0.008674204349517822]]
        self.sel_img, self.sel_pcd = self.breakup(self.all_pt_list)

        
    def sub_callback_pcd(self, PointCloud2):
        
        self.latest_pc = None
        gen2 = pc2.read_points(PointCloud2, field_names=['x', 'y', 'z'], skip_nans=True) # returns a pointcloud Generator
        
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
        
        # Add the color coding based on z (depth, which is actually x in lidar)
        ptc_z_camera = self.latest_pc[:,0]
        z_min=np.min(ptc_z_camera)
        z_range=np.max(ptc_z_camera)-z_min
        ptc_z_camera=(ptc_z_camera-z_min)*255/z_range
        ptc_z_camera=ptc_z_camera.astype(np.uint8)
        color=cv2.applyColorMap(ptc_z_camera[:,np.newaxis],cv2.COLORMAP_HSV)
        r=color.shape[0]

        # overlay the colorcoated lidar point onto the image
        for j in range(r):
            i=ptc_xy_camera[j]
            c=color[np.newaxis,np.newaxis,j,0]
            a = int(np.floor(i[0]))
            b = int(np.floor(i[1]))
            
            img_shape = im_undistorted_cv.shape
            
            # check if the projected point is in scope of image
            if a>0 and b>0 and a < img_shape[1] and b < img_shape[0]:
                alp = 0.5
                im_undistorted_cv[b-1:b+1,a-1:a+1] = c*alp + im_undistorted_cv[b-1:b+1,a-1:a+1] * (1-alp)
        
        # overlay the selected lidar and image points over the image
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
        
        result = np.transpose(toCartes(np.dot(K, np.transpose((np.transpose(rotated)+t)))))
        
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
