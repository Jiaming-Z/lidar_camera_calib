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
import itertools

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
        self.min_point_number = 6   

        # input this parameter
        self.undistorted_camera_info = np.array([[1.57700e+03,0.00000e+00,5.36020e+02],
 [0.00000e+00,1.58130e+03,3.20674e+02],
 [0.00000e+00,0.00000e+00,1.00000e+00]])

        self.all_selected_pts = np.array([]) # in [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, select 10 points
        self.pcd_pt_list = []
        
        
        pic1_path = "new_image_undistorted.png"
        self.img_undistorted = cv2.imread(pic1_path) # pass in the undistorted (not cropped) image

        self.counter1 = 0 #counter for selecting camera points
        self.camera_pt_list = []
        

        # input this parameter ONLY IF you want to use previously selected points
        # set count to 20, replace all_pt_list with the list below, click any 1 point in rviz, triggers calculation for R, t using these points
        # ## for testing only:
        self.all_pt_list =  [[501, 563, 3.7860517501831055, -0.1314089149236679, -0.36725127696990967], 
        [520, 350, 3.9691531658172607, -0.17884023487567902, 0.08687365055084229], 
        [847, 344, 3.713683843612671, -0.9109978079795837, 0.07670542597770691], 
        [844, 573, 3.5421509742736816, -0.8619372248649597, -0.3618428409099579], 
        [564, 378, 3.879394054412842, -0.28289592266082764, 0.00736922025680542], 
        [819, 370, 3.6950583457946777, -0.8444523811340332, 0.022561609745025635], 
        [869, 437, 2.133234977722168, -0.6032557487487793, -0.07292886823415756], 
        [915, 448, 1.7366533279418945, -0.5685082674026489, -0.06734936684370041], 
        [816, 458, 3.596477508544922, -0.816709578037262, -0.17254026234149933], 
        [796, 411, 3.6517751216888428, -0.7801579833030701, -0.06444092094898224], 
        [648, 318, 7.293030738830566, -0.781611442565918, 0.2882116436958313], 
        [431, 408, 9.324764251708984, 0.24158716201782227, -0.20837688446044922], 
        [479, 411, 8.994609832763672, -0.011698246002197266, -0.09594982862472534], 
        [442, 332, 9.475360870361328, 0.20123255252838135, 0.32087957859039307], 
        [491, 322, 9.084478378295898, -0.07679009437561035, 0.32754647731781006], 
        [464, 308, 9.379240036010742, 0.08596083521842957, 0.44552862644195557], 
        [546, 310, 9.29450798034668, -0.4120352864265442, 0.4373873472213745], 
        [498, 470, 3.829281806945801, -0.13060981035232544, -0.2137560248374939], 
        [583, 417, 3.8361122608184814, -0.3209671378135681, -0.07248884439468384], 
        [646, 376, 3.803565740585327, -0.4659099578857422, 0.008674204349517822]]  
        # ## for testing only:
        self.point_count = 20     
                    
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
                    
                
                    
            cv2.namedWindow("pic1_target_frame")
            cv2.setMouseCallback("pic1_target_frame", pic1_clickback)
            while True:
                cv2.imshow("pic1_target_frame", self.img_undistorted)

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
        self.selected_pcd_pts = [] # clear list just in case
        self.point_count += 1
        if self.point_count >= self.want_point_number :
            self.write_R_t_into_file()
            self.write_R_t_discard_worst()
            self.ransac_R_t()
            self.all_combs_R_t()

    # undo the last published lidar point when keyboard input == 'r'
    # currently not functional 04/25/23
    def undo_clicked_pcd(self, msg):
        if msg.data == 'r':
            self.point_count -= 1
            self.pcd_pt_list.pop()
            print("point removed: ", self.selected_pcd_pts, ", current point count: ", self.point_count)


    # function combining self.camera_pt_list and self.pcd_pt_list to 
    # [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, assuming same length
    def combine_lists(self, l1, l2):
        new_l = []
        for i in range(len(l1)):
            new_l.append(l1[i]+l2[i])
        return new_l
    
    # write R and t into txt file
    def write_R_t_into_file(self):
        #self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
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
        #self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
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

    #select the group of 6 points that gives the smallest reprojection error, write its R, t
    def ransac_R_t(self):
        #self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
        print('Ransac running')
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
                                temp_l = list(self.all_selected_pts)
                                A = temp_l[a]
                                B = temp_l[b]
                                C = temp_l[c]
                                D = temp_l[d]
                                E = temp_l[x]
                                F = temp_l[f]
                                sel_pts = [A, B, C, D, E, F]
                                
                                #print(sel_pts)
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
        print('Ransac done, check text file for R and t')

    #select all possible combinations of 7-20 points and find the selection with minimum reprojection error, write its R, t
    def all_combs_R_t(self):
        #self.all_pt_list = self.combine_lists(self.camera_pt_list, self.pcd_pt_list) # comment out this line to use manually entered testing all_pt_list
        print('all combs running')
        self.all_selected_pts = np.array(self.all_pt_list)
        final_R, final_t, last_e = self.calibration_algorithum(self.all_selected_pts)
        num_pts_used = 20
        used_pt_cord = None
        e_list = []
        e_all_20_list = []
        f = open('R_t_all_combs.txt', 'w')
        for r in range(7, self.want_point_number + 1):
            min_e_for_this_r = float("inf")
            min_e_for_all_20_pts = float("inf")
            for sel_pts in itertools.combinations(self.all_selected_pts, r):
                R, t, _ = self.calibration_algorithum(np.array(sel_pts))
                e = self.reprojection_err(sel_pts, R, t)
                if e < min_e_for_this_r : #select based on minimum e of this number of points
                    min_e_for_this_r = e
                    min_e_for_all_20_pts = _
                    final_R = R
                    final_t = t
                    used_pts_cord = sel_pts
                    num_pts_used = r
            
            e_list.append(min_e_for_this_r)
            e_all_20_list.append(min_e_for_all_20_pts)

            print('minimum error for ' + str(r) + ' points selected: ' + str(min_e_for_this_r))
            f.write('The following is the result of using '+ str(num_pts_used) + ' points\n')
            f.write('R matrix: \n')
            f.write(np.array2string(final_R, precision=5, separator=',')+'\n')
            f.write('t vector: ')
            f.write(np.array2string(final_t, precision=5, separator=',')+'\n')
            f.write('smallest reprojection error for all 20 pts (in pixels): ')
            f.write(str(min_e_for_all_20_pts)+'\n')
            f.write('reprojection error for the selected points (in pixels): ')
            f.write(str(min_e_for_this_r)+'\n\n')
            
        last_e_for_selected_pts = min(e_list)
        last_e_for_all_pts = min(e_all_20_list)
        f.write('\n\n\n')
        f.write('The following is the result of using '+ str(num_pts_used) + ' points\n')
        f.write('R matrix: \n')
        f.write(np.array2string(final_R, precision=5, separator=',')+'\n')
        f.write('t vector: ')
        f.write(np.array2string(final_t, precision=5, separator=',')+'\n')
        f.write('smallest reprojection error for all 20 pts (in pixels): ')
        f.write(str(last_e_for_all_pts)+'\n')
        f.write('reprojection error for the selected points (in pixels): ')
        f.write(str(last_e_for_selected_pts)+'\n')
        
        
        f.close()
        print('All combinations done, check text file for R and t')


    # reprojection error in terms of pixels, in camera frame
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
    
    # get R, t, and reprojection error on all selected points
    def calibration_algorithum(self, selected_points):
        K = self.undistorted_camera_info
        K_inv = np.linalg.inv(K)
        
        #print("selected: ", selected_points)
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
