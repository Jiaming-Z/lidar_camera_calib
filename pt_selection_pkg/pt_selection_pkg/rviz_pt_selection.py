import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import numpy as np

import cv2
import matplotlib as plt
import threading


### point selection using the topic : clicking points: node subscribe to /clicked_point topic to rviz node, and in rviz "publish point", click to see point, record clicked point

### integrate calibration algorithum here, output R,t

## in rviz: set display size 0.05, use MIDDLE MOUSE BUTTON to move frame around


class InteractiveMarkerNode(Node):
    
    def __init__(self):
        super().__init__('interactive_marker_node')

        self.qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # select points in lidar from /clicke_point
        self.img_file = self.create_subscription(
            PointStamped,
            "/clicked_point",  # Subscribes from clicked point in rviz
            self.sub_callback_clicked,
            self.qos_profile)

        '''# select points in lidar 
        self.pcd_file = self.create_subscription(
            PointStamped,
            "/clicked_point",  # Subscribes from clicked point in rviz
            self.sub_callback_clicked_pcd,
            self.qos_profile)'''

        self.point_count = 0
        self.want_point_number = 10    
        
        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])

        self.all_selected_pts = np.array([]) # in [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, select 10 points
        self.pcd_pt_list = []
        
        pic1_path = "image_undistorted.png"
        self.img1 = cv2.imread(pic1_path)
        self.counter1 = 0
        self.camera_pt_list = []
        
                
                    
        thread = threading.Thread(target=self.thread_function, args=(pic1_path,))
        thread.start()
        
        
        if self.counter1 >= self.want_point_number:
            print("you have selected enough camera points ("+str(self.counter1)+"), move on to rviz and select same amount of lidar points in same order!")
        
        
    def thread_function(self, pic1_path):
            print("created a seperate window for selecting image points")
            pic1 = cv2.imread(pic1_path)
            
            def pic1_clickback(event, x, y, flags, param):
                #print("at least in func")
                #global img1
                
                # ---------------------------------------------------------------------------- #
                #                                 LeftClick = 1                                #
                # ---------------------------------------------------------------------------- #
                if event == cv2.EVENT_LBUTTONDBLCLK:
                    # ---------------------------------------------------------------------------- #
                    #                  Record the coordinate of the clicked point                  #
                    # ---------------------------------------------------------------------------- #
                    self.camera_pt_list.append([x, y])
                    print("Target Frame Points: ", self.camera_pt_list)
                    
                    
                    # ---------------------------------------------------------------------------- #
                    #                      Draw the circle and label the point                     #
                    # ---------------------------------------------------------------------------- #
                    cv2.circle(self.img1, (x, y), 1, (0, 0, 255), 2)
                    cv2.putText(self.img1, str(self.counter1), (x-5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    self.counter1 += 1

                # ---------------------------------------------------------------------------- #
                #                               Middle_click = 1                               #
                # ---------------------------------------------------------------------------- #
                if event == cv2.EVENT_MBUTTONDOWN:
                    
                    # ---------------------------------------------------------------------------- #
                    #                              Undo the last point                             #
                    # ---------------------------------------------------------------------------- #
                    self.camera_pt_list.pop()
                    print("Original Frame Points: ", self.camera_pt_list)
                    self.img1 = pic1.copy()

                    # ---------------------------------------------------------------------------- #
                    #                Redrawing the remaining points in a fresh image               #
                    # ---------------------------------------------------------------------------- #
                    for i, point in enumerate(self.camera_pt_list):
                        cv2.circle(self.img1, point, 1, (0, 0, 255), 2)
                        cv2.putText(self.img1, str(i), (point[0]-5, point[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    self.counter1 -= 1
                    
                #if self.counter1 >= self.want_point_number:
                #    print("you have selected enough camera points ("+str(self.counter1)+"), move on to rviz and select same amount of lidar points in same order!")
            
                    
            cv2.namedWindow("pic1_target_frame")
            cv2.setMouseCallback("pic1_target_frame", pic1_clickback)
            while True:
                cv2.imshow("pic1_target_frame", self.img1)

                # press q to exit
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    
    # first select 10 points from image (matplotlib thread), with the 10 point clicked marked on the window
    # then select 10 corresponding point from lidar pointcloud in rviz2
    # after all 10 sets of points selected, calibration algorithum will be triggered
    def sub_callback_clicked(self, PointStamped):    
        x = PointStamped.point.x
        y = PointStamped.point.y 
        z = PointStamped.point.z
        self.selected_pcd_pts = []# a new list in [x,y,z] format
        self.selected_pcd_pts.append(x)
        self.selected_pcd_pts.append(y)
        self.selected_pcd_pts.append(z)
        self.pcd_pt_list.append(self.selected_pcd_pts) # a list of lists, [x1,y1,z1], [x2,y2,z2],...]
        print("pcd point number", self.point_count)
        print(self.pcd_pt_list)
        self.point_count += 1
        if self.point_count >= self.want_point_number :
            self.write_R_t_into_file()

    #def undo_clicked_pcd(self):
        # undo the last clicked point

    # function combining self.camera_pt_list and self.pcd_pt_list to 
    # [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, assuming same length
    def combine_lists(self, l1, l2):
        new_l = []
        for i in range(len(l1)):
            new_l.append(l1[i]+l2[i])
        return new_l
    
    # write R and t into txt file
    def write_R_t_into_file(self):
        R, t = self.calibration_algorithum()
        f = open('R_t.txt', 'w')
        f.write('R matrix: ')
        f.write(np.array2string(R, precision=2, separator=','))
        f.write('t vector: ')
        f.write(np.array2string(t, precision=2, separator=','))
        f.close()

    # get R and t
    def calibration_algorithum(self):
        K = self.camera_info
        K_inv = np.linalg.inv(K)
        self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list)
        self.all_selected_pts = np.array(self.all_pt_list) # convert list to np array
        print(self.all_selected_pts)
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
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
