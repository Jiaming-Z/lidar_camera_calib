import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs.msg as sensor_msgs
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib as plt
import threading
import math as m
from std_msgs.msg import String

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

        # select points in lidar from /clicked_point
        self.img_file = self.create_subscription(
            PointStamped,
            "/clicked_point",  # Subscribes from clicked point in rviz
            self.sub_callback_clicked,
            self.qos_profile)
        

        # keyboard input "r" to remove the latest pointcloud point selected
        # currently not functional 04/25/23
        self.keyboard_rm_pcd = self.create_subscription(
            String, 
            '/keyboard_input', 
            self.undo_clicked_pcd,
            self.qos_profile)

        self.best_R = None
        self.best_t = None

        self.point_count = 0
        self.want_point_number = 20
        self.min_point_number = 10    
        
        self.camera_info = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])

        self.undistorted_camera_info = np.array([[827.1930542,   0,        275.14427751],
                                                [  0,        828.20013428, 145.27567362],
                                                [  0,           0,          1     ]])

        self.all_selected_pts = np.array([]) # in [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, select 10 points
        self.pcd_pt_list = []
        
        
        pic1_path = "new_image_undistorted.png"
        self.img_undistorted = cv2.imread(pic1_path) # pass in the undistorted (not cropped) image

        self.counter1 = 0 #counter for selecting camera points
        self.camera_pt_list = []
        


        # set count to 20, replace all_pt_list with the list below, click any 1 point in rviz, triggers calculation for R, t using these points
        # ## for testing only:
        # self.all_pt_list =  [[7.20000000e+01, 4.90000000e+01, 1.41783791e+01, 3.26228428e+00, 2.60112667e+00],
        #                     [1.70000000e+02, 1.04000000e+02, 1.28549576e+01, 1.64912844e+00, 1.63924956e+00],
        #                     [9.00000000e+01,  1.45000000e+02,  1.42244320e+01,  3.10702419e+00, 1.13780963e+00],
        #                     [1.73000000e+02,  1.88000000e+02,  1.28062191e+01,  1.61688614e+00, 5.05442739e-01],
        #                     [3.09000000e+02,  1.86000000e+02,  2.98137779e+01, -4.27837074e-01, 6.66600466e-01],
        #                     [3.56000000e+02,  1.88000000e+02,  2.95880947e+01, -1.89676654e+00, 6.55570626e-01],
        #                     [3.36000000e+02,  1.66000000e+02,  3.11740074e+01, -1.44641638e+00, 1.41120648e+00],
        #                     [2.86000000e+02,  1.84000000e+02,  5.44212914e+01,  4.86706495e-01, 1.19493735e+00],
        #                     [1.85000000e+02,  1.02000000e+02,  2.99020710e+01,  3.72472095e+00, 3.55019283e+00],
        #                     [2.30000000e+02,  1.26000000e+02,  2.85917034e+01,  2.12543201e+00, 2.62642193e+00],
        #                     [4.05000000e+02,  1.09000000e+02,  1.91215729e+02, -2.54982567e+01, 2.07946014e+01],
        #                     [4.08000000e+02,  1.49000000e+02,  1.91588745e+02, -2.55737705e+01, 1.15379143e+01],
        #                     [4.97000000e+02,  9.90000000e+01,  1.67729050e+02, -4.03239059e+01, 1.99132977e+01],
        #                     [4.99000000e+02,  1.39000000e+02,  1.69607391e+02, -4.12023735e+01, 1.04788647e+01],
        #                     [1.66000000e+02,  7.60000000e+01,  1.63135773e+02,  2.43768444e+01, 2.40905285e+01],
        #                     [1.74000000e+02,  2.21000000e+02,  1.27924595e+01,  1.58866215e+00, 5.18536568e-02],
        #                     [1.08000000e+02,  7.40000000e+01,  1.38679991e+01,  2.64059973e+00, 2.17800713e+00],
        #                     [2.94000000e+02,  1.54000000e+02,  2.15639252e+02,  3.96171212e-01, 1.09125872e+01],
        #                     [1.72000000e+02,  1.24000000e+02,  1.28065834e+01,  1.65250087e+00, 1.40220273e+00],
        #                     [1.18000000e+02,  1.64000000e+02,  1.40416698e+01,  2.57688522e+00, 8.96251678e-01]]  
        # ## for testing only:
        # self.point_count = 20     
                    
        thread = threading.Thread(target=self.thread_function, args=(pic1_path,))
        thread.start()
        
        
        
        
    def thread_function(self, pic1_path):
            print("created a seperate window for selecting image points")
            
            pic1 = cv2.imread(pic1_path)
            
            def pic1_clickback(event, x, y, flags, param):
                #print("at least in func")
                
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
                    cv2.circle(self.img_undistorted, (x, y), 1, (0, 0, 255), 2)
                    cv2.putText(self.img_undistorted, str(self.counter1), (x-5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    self.counter1 += 1

                # ---------------------------------------------------------------------------- #
                #                               Middle_click = 1                               #
                # ---------------------------------------------------------------------------- #
                if event == cv2.EVENT_MBUTTONDOWN:
                    
                    # ---------------------------------------------------------------------------- #
                    #                              Undo the last point, currently not functional 04/25/23                            #
                    # ---------------------------------------------------------------------------- #
                    self.camera_pt_list.pop()
                    print("Original Frame Points: ", self.camera_pt_list)
                    self.img_undistorted = pic1.copy()

                    # ---------------------------------------------------------------------------- #
                    #                Redrawing the remaining points in a fresh image               #
                    # ---------------------------------------------------------------------------- #
                    for i, point in enumerate(self.camera_pt_list):
                        cv2.circle(self.img_undistorted, point, 1, (0, 0, 255), 2)
                        cv2.putText(self.img_undistorted, str(i), (point[0]-5, point[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    self.counter1 -= 1
                    
                #if self.counter1 >= self.want_point_number:
                #    print("you have selected enough camera points ("+str(self.counter1)+"), move on to rviz and select same amount of lidar points in same order!")
            
                    
            cv2.namedWindow("pic1_target_frame")
            cv2.setMouseCallback("pic1_target_frame", pic1_clickback)
            while True:
                cv2.imshow("pic1_target_frame", self.img_undistorted)

                # press q to exit
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    '''
    def sub_callback_img(self, Image):
        #print("subscribed to image")
        self.latest_im  = None
        try:
            self.latest_im  = self.bridge.imgmsg_to_cv2(Image)
            
        except CvBridgeError as e:
            print(e)
    '''

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
        self.selected_pcd_pts = [] # clear list just in case
        self.point_count += 1
        if self.point_count >= self.want_point_number :
            self.write_R_t_into_file()
            self.write_R_t_discard_worst()
            self.ransac_R_t()

    # undo the last published lidar point when keyboard input == 'r'
    # currently not functional 04/25/23
    def undo_clicked_pcd(self, msg):
        if msg.data == 'r':
            self.point_count -= 1
            self.pcd_pt_list.pop()
            print("point removed: ", self.selected_pcd_pts, ", current point count: ", self.point_count)

    '''
    # https://github.com/Chrislai502/Lidar_Camera_Calibration/blob/main/pcd_image_overlay_Chris.py
    # undistort the image, so fisheye form is used
    def undistort(self, image):
    
        K = self.camera_info
        dist_coeffs = np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000])
        img_size = (image.shape[1], image.shape[0])
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, img_size, alpha=1)
        image_undistorted = cv2.undistort(image, K, dist_coeffs, None, new_K)
        return image_undistorted
    '''


    # function combining self.camera_pt_list and self.pcd_pt_list to 
    # [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, assuming same length
    def combine_lists(self, l1, l2):
        new_l = []
        for i in range(len(l1)):
            new_l.append(l1[i]+l2[i])
        return new_l
    
    # write R and t into txt file
    def write_R_t_into_file(self):
        self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
        self.all_selected_pts = np.array(self.all_pt_list)
        R, t, e = self.calibration_algorithum(self.all_selected_pts)
        f = open('R_t.txt', 'w')
        f.write('R matrix: ')
        f.write(np.array2string(R, precision=3, separator=',')+'\n')
        f.write('t vector: ')
        f.write(np.array2string(t, precision=3, separator=',')+'\n')
        f.write('reprojection error (in lidar unit): ')
        f.write(str(e)+'\n')
        f.write('reprojection error for all 20 pts(in pixels): ')
        f.write(str(self.reprojection_err(self.all_selected_pts, R, t))+'\n')
        f.write('')
        f.write(str(self.all_pt_list)+'\n')
        f.close()

    # discard the "outlier" point pair to find the R, t with the smallest reprojection error
    def write_R_t_discard_worst(self):
        self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
        self.all_selected_pts = np.array(self.all_pt_list)

        all_pts = list(self.all_selected_pts)
        bad_ind = 0
        final_R, final_t, last_e = self.calibration_algorithum(self.all_selected_pts)

        for i in range(len(self.all_selected_pts)):
            all_pts.pop(i)
            all_pts_minus_1 = all_pts
            all_pts = list(self.all_selected_pts)
            R, t, e = self.calibration_algorithum(np.array(all_pts_minus_1))
            if e < last_e:
                last_e = e
                bad_ind = i
                final_R = R
                final_t = t

        f = open('R_t_remove_outlier.txt', 'w')
        f.write('The following is the result of removing the '+str(bad_ind)+ 'th point selected\n')
        f.write('R matrix: ')
        f.write(np.array2string(final_R, precision=3, separator=',')+'\n')
        f.write('t vector: ')
        f.write(np.array2string(final_t, precision=3, separator=',')+'\n')
        f.write('smallest reprojection error (in pixels): ')
        f.write(str(last_e)+'\n')
        f.write('reprojection error for all 20 pts(in pixels): ')
        f.write(str(self.reprojection_err(self.all_selected_pts, final_R, final_t))+'\n')
        f.close()

    def ransac_R_t(self):
        self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
        self.all_selected_pts = np.array(self.all_pt_list)
        final_R, final_t, last_e = self.calibration_algorithum(self.all_selected_pts)

        l = len(self.all_selected_pts)
        used_pts = []
        used_pt_cord = None
        for a in range(0, l):
            for b in range(a+1, l):
                for c in range(b+1, l):
                    for d in range(c+1, l):
                        for x in range(d+1, l):
                            for f in range(x+1, l):
                                #print("WTMD", self.all_selected_pts)
                                #print(a, b, c, d, x, f)
                                #print("aaaa", self.all_selected_pts[a])
                                temp_l = list(self.all_selected_pts)
                                A = temp_l[a]
                                B = temp_l[b]
                                C = temp_l[c]
                                D = temp_l[d]
                                E = temp_l[x]
                                F = temp_l[f]
                                sel_pts = [A, B, C, D, E, F]
                                
                                print(sel_pts)
                                R, t, e = self.calibration_algorithum(np.array(sel_pts))
                                if e < last_e :
                                    last_e = e
                                    final_R = R
                                    final_t = t
                                    used_pts = [a,b,c,d,x,f]
                                    used_pts_cord = np.array(sel_pts)
        self.best_R = final_R
        self.best_t = final_t
        f = open('R_t_ransac.txt', 'w')
        f.write('The following is the result of using these 6 points' + str(used_pts)+'\n')
        f.write('R matrix: \n')
        f.write(np.array2string(final_R, precision=5, separator=',')+'\n')
        f.write('t vector: ')
        f.write(np.array2string(final_t, precision=5, separator=',')+'\n')
        f.write('smallest reprojection error for all 20 pts (in pixels): ')
        f.write(str(last_e)+'\n')
        f.write('reprojection error for the 6 selected points (in pixels): ')
        f.write(str(self.reprojection_err(used_pts_cord, final_R, final_t))+'\n')
        f.close()

    
    '''
    # get reprojection error, preject to camera frame and reproject back to lidar frame, in unit of lidar
    def reprojection_err(self, input1, R, t):
        K = self.camera_info
        def toCartes(a):
            return [a[0]/a[2], a[1]/a[2]], a[2]
        def calc_proj(x):
            return toCartes(np.dot(K, (np.dot(R, x)+t)))
        e_sum = 0
        for a in input1:
            lidar_points = np.array([a[2],a[3],a[4]])
            calib_data, z = calc_proj(lidar_points)

            projected_non_carte = np.array([calib_data[0]*z, calib_data[1]*z, z])
            undo_K = np.dot(np.linalg.inv(K), projected_non_carte)
            lidar_reprojected = np.dot(np.linalg.inv(R), (undo_K-t))

            x_diff = abs(a[2] - lidar_reprojected[0])
            y_diff = abs(a[3] - lidar_reprojected[1])
            z_diff = abs(a[4] - lidar_reprojected[2])

            diff = m.sqrt(m.sqrt(x_diff**2 + y_diff**2)**2 + z_diff**2)
            
            e_sum += diff
            
        e = m.sqrt(e_sum/len(input1))
        return e
    '''
    # projection error in terms of pixels, in camera frame
    def reprojection_err(self, input1, R, t):
        K = self.undistorted_camera_info
        def toCartes(a):
            return [a[0]/a[2], a[1]/a[2]]
        def calc_proj(x):
            return toCartes(np.dot(K, (np.dot(R, x)+t)))
        e_sum = 0
        for a in input1:
            lidar_points = np.array([a[2],a[3],a[4]])
            calib_data = calc_proj(lidar_points)

            u_diff = abs(a[0] - calib_data[0])
            v_diff = abs(a[1] - calib_data[1])
            
            diff = m.sqrt(u_diff**2 + v_diff**2)
            
            e_sum += diff
            
        e = e_sum/len(input1)
        return e
    
    # get R and t
    def calibration_algorithum(self, selected_points):
        K = self.undistorted_camera_info
        K_inv = np.linalg.inv(K)
        
        print("selected: ", selected_points)
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
        
        A = make_A(selected_points)

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
        e = self.reprojection_err(self.all_selected_pts, R, t)
        return R, t, e



def main(args=None):
    rclpy.init(args=args)
    
    node = InteractiveMarkerNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
