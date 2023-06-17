# lidar_camera_calib

Steps in using this repo to calibrate lidar(3D)-camera(2D):

0, Install ros2, and rviz2, preferably in linux machine

1, In your working directory, pull this repo and run 'source install/setup.bash' in all terminals opened.

2, In terminal 1, play the ros bag (folder than contains metadata) in loop, for example: 'ros2 bag play /path_to_file/name_of_bag_folder/ -l'.

3, In terminal 2, run 'rviz2'.

4, collect lidar points

  4a, In rviz_pt_collection.py, make sure the self.pcd_file is subscribed to the correct topic of lidar pointcloud (for example, "/luminar_front_points"). 

  4b, In terminal 3, 'ros2 run pt_selection_pkg pt_collection_node' to collect 10 pointclouds and stack them together,
  so in rviz the pointclouds are not moving and are stationary, we will use this to select lidar points, don't close this terminal, let it keep running.

5, undistort camera image

  5a, In export_undistort_img.py, input the camera K matrix for image before undistortion as self.camera_info, also input the distortion coefficients in      self.undistort function as dist_coeffs, also make sure the self.ros_img_front is subscribed to the correct topic of image info (for example,                "/vimba_front_left_center/image").
  
  5b, In terminal 4, 'ros2 run pt_selection_pkg undistort_img_node', it will write the undistorted image as a png file in your workspace, and will also        write a 'undistorted_camera_matrix.txt' containing the new K matrix of the undistorted image. The node will finish running after 1 spin, so feel free to    reuse this terminal for othing commands.

6, do the calibration

  6a, In rviz_pt_selection.py, paste the new K matrix contained in undistorted_camera_matrix.txt to assign it as self.undistorted_camera_info.
  
  6b, In terminal 4, 'ros2 run pt_selection_pkg pt_selection_node', a window showing the undistorted image will pop up.
  
  6c, In the pop up window, carefully select 20 points at 20 distinguishable pixels of that image by double clicking left button of mouse, example good       points to select include tip of tent, fin of race car.. etc, make sure these points are easily identifiable at both lidar and image views. Once you         select a point, there should be a number on top of it telling you it is that number of point you clicked (0 to 19). Keep the window open after all 20       points are clicked. After each point clicked, you should see update of all currently selected image points in the terminal.
  
  6d, In rviz, we should see the /merged_pointcloud topic in XYOrbit view showing the lidar points. Select the 20 lidar points in the same order as you       clicked on the camera points (refer to the still-open pop up window to see the order of points selected). To select a lidar point, click 'Publish Point'   and click the lidar dot closest to the corresponding image point selected. Once you see a pink ball on screen, it means you published the point             sucessfully. After each point clicked, you should see update of all currently selected lidar points in the terminal.
  
  6e, Once all 20 points selected in lidar, the algorithum will automatically run, and the R,t data will be written into txt files.
  
 7, verify the calibration
 
  7a, There will be 3 txt files written: R_t.txt, R_t_remove_outlier.txt, and R_t_ransac.txt, each containing the rotation matrix R, translation vector t,    as well as reprojection errors. Usually the R_t_ransac.txt will contain the R and t with minimum reprojection error, if the reprojection error is too     large, you can try reselect the points and run again.
  
  7b, In addition to the reprojection error, you can also visually examine how accurate the R and t are. In overlap.py, paste R and t from the txt files      and assign as self.R and self.t, also paste the new K matrix from the earlier 'undistorted_camera_matrix.txt' and assign as                         self.undistorted_camera_info, lastly, from the R_t.txt, paste the list of all select points and assign as self.all_pt_list. Then In terminal 5, run 'ros2     run pt_selection_pkg overlay_node'. There should be a new topic in rviz called "/image_overlay", open that up and you can see the lidar points (red) overlayed on the distorted camera image, with green points representing the selected image points and blue points representing the selected lidar points. You can also see from the terminal the distances between the clicked lidar points in rviz and the closest real lidar point subscribed to.
