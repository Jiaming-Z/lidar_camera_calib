import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import PointStamped
import numpy as np


### point selection using the topic : clicking points: node subscribe to /clicked_point topic to rviz node, and in rviz "publish point", click to see point, record clicked point

### integrate calibration algorithum here, output R,t
class InteractiveMarkerNode(Node):

    def __init__(self):
        super().__init__('interactive_marker_node')

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # select points in camera image
        self.img_file = self.create_subscription(
            geometry_msgs.msg.PointStamped,
            "/clicked_point",  # Subscribes from clicked point in rviz
            self.sub_callback_clicked_img,
            self.qos_profile)

        # select points in lidar 
        self.pcd_file = self.create_subscription(
            geometry_msgs.msg.PointStamped,
            "/clicked_point",  # Subscribes from clicked point in rviz
            self.sub_callback_clicked_pcd,
            self.qos_profile)

        self.point_count = 0
        self.want_point_number = 10    
        
        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])

        self.all_selected_pts = np.array() # in [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, select 10 points
    
    # first select a point from image, then select a point from lidar pointcloud
    # (click img point, click pcd point), repeat for 10 sets, after all 10 sets of points selected, do calibration algorithum
    def sub_callback_clicked_img(self, PointStamped_img):
        u = PointStamped_img.point.x
        v = PointStamped_img.point.y
        self.selected_pts = np.array() # a new array in [u,v,x,y,z] format
        self.selected_pts = np.append(self.selected_pts, u)
        self.selected_pts = np.append(self.selected_pts, v)


    def sub_callback_clicked_pcd(self, PointStamped_pcd):
        
        x = PointStamped_pcd.point.x
        y = PointStamped_pcd.point.y 
        z = PointStamped_pcd.point.z
        self.selected_pts = np.append(self.selected_pts, x)
        self.selected_pts = np.append(self.selected_pts, y)
        self.selected_pts = np.append(self.selected_pts, z)
        self.all_selected_pts = np.append(self.all_selected_pts, self.selected_pts)
        self.selected_pts = np.array() # clear the old array
        
        self.point_count += 1
        if self.point_count >= self.want_point_number :
            self.write_R_t_into_file()

    #def undo_clicked_pcd(self):
        # undo the last clicked point

    # write R and t into txt file
    def write_R_t_into_file(self):
        R, t = self.calibration_algorithum()
        f = open('R_t.txt', 'w')
        f.write('R matrix: ')
        f.write(R)
        f.write('t vector: ')
        f.write(t)
        f.close()

    # get R and t
    def calibration_algorithum(self):
        K = self.camera_info
        K_inv = np.linalg.inv(K)
        def make_A(input2):
            A_lst_T = [] # use list for flexibility
            for i in range(len(input2)):
                n = input2[i]
                I = np.dot(K_inv, np.array([n[0], n[1], 1])) #[X',Y',1]
                A_lst_T.append([0*n[2], 0*n[3], 0*n[4], 0, -1*n[2], -1*n[3], -1*n[4], -1, I[1]*n[2], I[1]*n[3], I[1]*n[4], I[1]])
                A_lst_T.append([1*n[2], 1*n[3], 1*n[4], 1, 0*n[2], 0*n[3], 0*n[4], 0, -I[0]*n[2], -I[0]*n[3], -I[0]*n[4], -I[0]])
            
            A_T = np.array(A_lst_T)
            A = np.transpose(A_T)
            return A
        
        A = make_A(self.all_selected_pts)

        def get_raw_Rt(A):
            U, sig, VT = np.linalg.svd(A)

            rt_vec = U[:,11]
            R_raw = np.array([[rt_vec[0], rt_vec[1], rt_vec[2]],
                        [rt_vec[4], rt_vec[5], rt_vec[6]],
                        [rt_vec[8], rt_vec[9], rt_vec[10]]])
            t_raw = np.array([rt_vec[3], rt_vec[7], rt_vec[11]])
            return R_raw, t_raw
        
        R_raw, t_raw = get_raw_Rt(A)

        def get_real_Rt(R_raw, t_raw):
            UR, sigR, VTR = np.linalg.svd(R_raw)
            D = np.array([[1,0,0],
                        [0,1,0],
                        [0,0,1]])
            R = np.dot(UR, np.dot(D, VTR))

            s = (sigR[0] + sigR[1] + sigR[2])/3
            t = t_raw/s
            return R, t

        R, t = get_real_Rt(R_raw, t_raw)
        return R, t




def main(args=None):
    rclpy.init(args=args)

    node = InteractiveMarkerNode()

    node.main_loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
