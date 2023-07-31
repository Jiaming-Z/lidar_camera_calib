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

        self.camera_info = np.array([[1658.482492, 0.000000, 535.224910], 
                                [0.000000, 1659.144364, 324.466514], 
                                [0.000000, 0.000000, 1.000000]])
        self.undistorted_camera_info = np.array([[1.57700e+03,0.00000e+00,5.36020e+02],
 [0.00000e+00,1.58130e+03,3.20674e+02],
 [0.00000e+00,0.00000e+00,1.00000e+00]])
        self.R = np.array([[-2.55018e-02,-9.99675e-01,-2.50639e-04],
 [ 4.77680e-02,-9.68132e-04,-9.98858e-01],
 [ 9.98533e-01,-2.54847e-02, 4.77771e-02]])
        self.t = np.array([-0.12104,-0.05424,-0.22951])

        self.timer_pub_overlay = self.create_timer(0.5, self.publisher_overlay_callback)

        # input this manually
        # self.all_pt_list =  [[189, 542, 2.5588274002075195, 0.32432475686073303, -0.22267985343933105], [472, 464, 3.0802817344665527, -0.10874482989311218, -0.12248444557189941], [251, 212, 2.916322708129883, 0.27344876527786255, 0.29067516326904297], [390, 207, 2.9206297397613525, 0.026939332485198975, 0.33234333992004395], [509, 203, 3.9990525245666504, -0.12196928262710571, -0.002295970916748047], [398, 324, 2.8816561698913574, 0.017968297004699707, 0.1359933614730835], [437, 332, 9.422277450561523, 0.21041691303253174, 0.31528764963150024], [483, 323, 9.132172584533691, -0.06010794639587402, 0.3209947645664215], [431, 407, 9.199897766113281, 0.2925487756729126, -0.09332311153411865], [473, 403, 8.975249290466309, 0.007119655609130859, -0.09218466281890869], [644, 318, 7.305987358093262, -0.7675980925559998, 0.2907868027687073], [624, 399, 7.310792446136475, -0.6649692058563232, -0.10174232721328735], [578, 574, 7.0894455909729, -0.44930770993232727, -0.7948412895202637], [635, 483, 7.108160972595215, -0.6930181980133057, -0.4349815845489502], [540, 307, 9.32555866241455, -0.3597373962402344, 0.4512764811515808], [533, 730, 3.1510424613952637, -0.18575918674468994, -0.662308931350708], [539, 435, 9.738765716552734, -0.5898716449737549, -0.834537148475647], [310, 70, 4.474766731262207, 0.3405618667602539, 0.8392564058303833], [459, 307, 9.369884490966797, 0.08786337077617645, 0.4290320873260498], [450, 432, 8.744592666625977, 0.15017716586589813, -0.28535914421081543]]
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



        #print(ptc_xy_camera.shape[0])
        for j in range(r):
            #print(ptc_xy_camera.shape)
            i=ptc_xy_camera[j]
            #print(i)
            c=color[np.newaxis,np.newaxis,j,0]
            print('color: ', c)
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
