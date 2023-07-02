import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import os
import pickle
import cv2
import math as m
########################     CLICK AT MINIMUM 6 POINTS, DESIGNED TO WORK AROUND 20 POINTS SELECTED

# set the number points to select, once reached, cross out the matplotlib window to run calibration algorithm automatically
points_to_select = 20

# Name of the files
input_filename = "1NEW_moving_DATA_pdc_calib_data.pkl"

# "/home/roar/ros2_ws/src/pt_selection_pkg/pt_selection_pkg/calib_databags/" 
# "C:/Users/amazi/ros2_ws/lidar_camera_calib/pt_selection_pkg/pt_selection_pkg/calib_databags/
with open(f"/home/roar/ros2_ws/src/pt_selection_pkg/pt_selection_pkg/calib_databags/{input_filename}", 'rb') as file:
    # Read the Pickle file
    data = pickle.load(file)

# Get the data
# if camera matrix and distortion coefficients not set in data_generation_node, re-affirm here
data['camera_info_numpy'] = np.array([[1732.571708*0.5 , 0.000000, 549.797164*0.5], 
                                [0.000000, 1731.274561*0.5 , 295.484988*0.5], 
                                [0.000000, 0.000000, 1.000000]])
data['dist_coeffs_numpy'] = np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000])
lidar_points = data['front points'] # 3D points
lidar_points = np.array(lidar_points) # Assume it is comin in x, y, z
print (lidar_points.shape) 

# PROBLEM: the generator in data_generation (gen2 = read_points(msg, skip_nans=True, field_names=["x", "y", "z"]))
# this might cause the lidar points to be extrapolated somehow and make farther away points' dimensions larger (in x, y, and z)
# testing perspective in matplotlib, see why x and z and y axis are scaled (height of tents not the same)
'''
x_2 = lidar_points[:, 0]
z_2 = lidar_points[:, 2]

plt.scatter(x_2, z_2)
plt.xlabel('X')
plt.ylabel('Z')
plt.title('Plot of Points')
plt.show()
'''


lidar_points_left = data['left points'] # 3D points
lidar_points_left = np.array(lidar_points_left)
lidar_points_right = data['right points'] # 3D points
lidar_points_right = np.array(lidar_points_right)


# undistort image to get new image and new camera matrix
def undistort(image):
    print('image', image)
    
    K = data['camera_info_numpy']
    dist_coeffs = data['dist_coeffs_numpy']
    img_size = (image.shape[1], image.shape[0])
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, img_size, alpha=1)
    
    image_undistorted = cv2.undistort(image, K, dist_coeffs, None, new_K)
    return image_undistorted, new_K
# mini_batch = lidar_points[:20]


# # Checking for order of the points coming in
# from scipy import sparse

# # Create a sample dense array
# dense_arr = np.array([[1, 0, 0],
#                       [0, 2, 0],
#                       [0, 0, 3]])

# # Convert the dense array to a sparse matrix
# sparse_mat = sparse.csr_matrix(dense_arr)

# # Check if the array is sparse
# is_sparse = sparse.issparse(sparse_mat)

# for i in range(6):
#     sparse_mat = sparse.csr_matrix(lidar_points -  lidar_points[lidar_points[:, i].argsort()])
#     print(f"{i}: {sparse.issparse(sparse_mat)}, {sparse_mat.nnz}")


# Your initial 3D data
n = 100
# x = lidar_points[30000:-30000, 0]
# y = lidar_points[30000:-30000, 1]
# z = lidar_points[30000:-30000, 2]
x = lidar_points[:, 0]
y = lidar_points[:, 1]
z = lidar_points[:, 2]

# Normalize the axis for color coding
norm_z = Normalize(vmin=min(z), vmax=max(z)) # for the x-y plot
norm_x = Normalize(vmin=min(x), vmax=max(x)) # for the z-y plot
norm_y = Normalize(vmin=min(y), vmax=max(y)) # for the z-y plot
cmap = get_cmap("rainbow")

# Load your images, undistorted images and new undistorted camera matrix
img_flc, K_flc = undistort(data['camera_images_flc']) #mpimg.imread('image_undistorted.png')
#img_frc, K_frc = undistort(data['camera_images_frc'])
#img_fl, K_fl = undistort(data['camera_images_fl'])
#img_fr, K_fr = undistort(data['camera_images_fr'])
#img_rl, K_rl = undistort(data['camera_images_rl'])
#img_rr, K_rr = undistort(data['camera_images_rr'])
#all_imgs = [img_flc, img_frc, img_fl, img_fr, img_rl, img_rr]
#all_K = [K_flc, K_frc, K_fl, K_fr, K_rl, K_rr]

'''lid_im_pairs = [[lidar_points, img_flc, K_flc], [lidar_points, img_frc, K_frc], 
                [lidar_points, img_fl, K_fl], [lidar_points, img_fr, K_fr], 
                [lidar_points_left, img_fl, K_fl], [lidar_points_right, img_fr, K_fr], 
                [lidar_points_left, img_rl, K_rl], [lidar_points_right, img_rr, K_rr]]'''

####TODO: Make a for loop for all lidar-image pairs


# Creating the plots and image subplot in the same figure
fig = plt.figure(figsize=(15, 10))
gspec = fig.add_gridspec(2, 3)
view_ax = plt.subplot(gspec[0, :])
axs = [plt.subplot(gspec[1, 0]), plt.subplot(gspec[1, 1]), plt.subplot(gspec[1, 2])]

fig.patch.set_facecolor('black')

# Create boxes around the axes
for ax in axs:
    for spine in ax.spines.values():
        spine.set_edgecolor('white')

view_ax.set_axis_off()
view_ax.spines['left'].set_color('white')
view_ax.spines['right'].set_color('white')
view_ax.spines['top'].set_color('white')
view_ax.spines['bottom'].set_color('white')

# Inverting color for dark mode
for ax in axs:
    ax.set_facecolor('black')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')

# Plot the initial data as scatter plot
colors = cmap(norm_x(x))
axs[0].scatter(x, z, c=colors, s = 0.1)
axs[0].set_xlabel('X')
axs[0].set_ylabel('Z')
axs[0].axis('equal')

# Plot the second data as scatter plot
colors = cmap(norm_x(x))
axs[1].scatter(-y, z, c=colors, s = 0.1)
axs[1].set_xlabel('Y')
axs[1].set_ylabel('Z')
axs[1].axis('equal')

# Show the image
# FOR NOW ONLY FRONT LEFT CENTER
img_plot = axs[2].imshow(img_flc) 

# New points data, annotations and its plot, initialized as empty and None. X-Z plot front view
new_points = []
new_points_plot = None
annotations = []
guidelines0 = []

# New points for the second scatter plot Y-Z plot side view
new_points2 = []
new_points_plot2 = None
annotations2 = []
guidelines1 = []

# New points for the image, with their plot and annotations
new_image_points = []
new_image_points_plot = None
image_annotations = []

# Function to create or update views
def create_or_update_views():
    # TODO: Make this better, add a functionality to capture the view you want to save
    view_ax.clear()
    view_ax.set_axis_off()
    view_ax.imshow(fig.canvas.renderer.buffer_rgba()[500:], aspect='auto')

def update_plot():
    global new_points_plot, new_image_points_plot, new_points_plot2

    # Remove previous new points, guidelines and their annotations
    for plot, points, annots, guidelines, ax in [
        (new_points_plot, new_points, annotations, guidelines0, axs[0]), 
        (new_points_plot2, new_points2, annotations2, guidelines1, axs[1]), 
        (new_image_points_plot, new_image_points, image_annotations, [], axs[2])]:
        
        if plot:
            plot.remove()
            if ax == axs[0]:
                new_points_plot = None
            elif ax == axs[1]:
                new_points_plot2 = None
            else:
                new_image_points_plot = None
        for annotation in annots:
            annotation.remove()
        annots.clear()
        for guideline in guidelines:
            guideline.remove()
        guidelines.clear()

        # If there are new points, plot them and add their annotations
        if points:
            x_values, y_values = zip(*points)
            # plot = ax.scatter(x_values, y_values, color='white')
            annotation_color = None
            if ax == axs[0]:
                annotation_color = 'white'
                plot = ax.scatter(x_values, y_values, color='white')
                new_points_plot = plot
            elif ax == axs[1]:
                annotation_color = 'white'
                plot = ax.scatter(x_values, y_values, color='white')
                new_points_plot2 = plot
            else:
                annotation_color = 'black'
                plot = ax.scatter(x_values, y_values, color='red')
                new_image_points_plot = plot
            for i, txt in enumerate(points):
                annotation = ax.annotate(i+1, (x_values[i] + 0.01, y_values[i] + 0.01), color=annotation_color)
                annots.append(annotation)

    # Draw new guidelines
    if new_points:
        guideline = axs[1].axhline(new_points[-1][1], color='orange', alpha=0.7)
        guidelines1.append(guideline)
    if new_points2:
        guideline = axs[0].axhline(new_points2[-1][1], color='orange', alpha=0.7)
        guidelines0.append(guideline)

    # Display total selected points
    for i, points in enumerate([new_points, new_points2, new_image_points]):
        axs[i].set_title(f"Total selected points: {len(points)}")

    create_or_update_views()
    plt.draw()
prev_event = None
def onclick(event):
    global prev_event

    # run the calibration algorithm once enough points selected
    print('PTS_SELECTED', min(len(new_points), len(new_points2), len(new_image_points)))
    if points_to_select <= min(len(new_points), len(new_points2), len(new_image_points)):
        #ransac_R_t(K_flc)
        print('WOOOHOOO!!!')
        return 
    
    # Add a point with double left click
    if event.button == 1 and event.xdata and event.ydata:
        if prev_event and event.dblclick:
            if event.inaxes == axs[0]:
                new_points.append((event.xdata, event.ydata))
            elif event.inaxes == axs[1]:
                new_points2.append((event.xdata, event.ydata))
            elif event.inaxes == axs[2]:
                new_image_points.append((event.xdata, event.ydata))

    # Remove last point with right click
    elif event.button == 3:
        if event.inaxes == axs[0] and new_points:
            new_points.pop()
        elif event.inaxes == axs[1] and new_points2:
            new_points2.pop()
        elif event.inaxes == axs[2] and new_image_points:
            new_image_points.pop()

    update_plot()
    prev_event = event

# Connect the mouse click event to the onclick function.
if points_to_select > min(len(new_points), len(new_points2), len(new_image_points)):
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    
    plt.show()
    print('HI, DONE')
else:
    print('finished point selection')
    plt.close()
print('finished point selection')

####### FINISH POINT SELECTION, START CALIBRATION 

# combine the xz and yz lidar points selected into xyz lidar points
def combine_xyz(xz, yz):
    xyz = []
    if len(xz) != len(yz):
        print("number of xz and yz points don't match")
        return
    for i in range(len(xz)):
        pt = [xz[i][0], yz[i][0], yz[i][1]]
        xyz.append(pt)
    return xyz


# function combining [[u1,v1],[u2,v2]...] and [[x1,y1,z1],[x2,y2,z2]...] to 
# [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2],...] format, assuming same length
def combine_lists(l1, l2):
    new_l = []
    for i in range(len(l1)):
        new_l.append([l1[i][0], l1[i][1], l2[i][0], l2[i][1], l2[i][2]])
    return new_l

# reprojection error in terms of pixels, in camera frame
# inputs:
# input1: numpy array of [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2]...], data used to calculated reprojection error, usually all 20 selected points
# R: numpy array of 3x3 rotation matrix
# t: numpy array of 3x1 translation vector
# undist_K: numpy array of undistorted camera matrix, passed in for this specific image
def reprojection_err(input1, R, t, undist_K):
    K = undist_K
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


# calibration algorithm to get R and t
# inputs: 
# selected_points: numpy array of [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2]...], selected points (minimum of 6) used to calibrate R and t 
# undist_K: numpy array of undistorted camera matrix, passed in for this specific image
# all_selected_pts: numpy array of [[u1,v1,x1,y1,z1], [u2,v2,x2,y2,z2]...], all 20 selected points used in calculating reprojection error
def calibration_algorithum(selected_points, undist_K, all_selected_pts):
    K = undist_K
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
    # find a way to calculate error using all_selected_points
    e = reprojection_err(all_selected_pts, R, t, undist_K)
    return R, t, e

# write R and t in txt file, using the 6 best fit points that give minimum reprojection error
def ransac_R_t(undist_K):
    print('RANSAC OPENNNNED!!!!!')
    all_pt_list = combine_lists(new_image_points, combine_xyz(new_points, new_points2)) # comment out this line to use manually entered testing all_pt_list
    all_selected_pts = np.array(all_pt_list)
    final_R, final_t, last_e = calibration_algorithum(all_selected_pts, undist_K, all_selected_pts)

    l = len(all_selected_pts)
    used_pts = []
    used_pts_cord = all_selected_pts
    for a in range(0, l):
        for b in range(a+1, l):
            for c in range(b+1, l):
                for d in range(c+1, l):
                    for x in range(d+1, l):
                        for f in range(x+1, l):
                            
                            temp_l = list(all_selected_pts)
                            A = temp_l[a]
                            B = temp_l[b]
                            C = temp_l[c]
                            D = temp_l[d]
                            E = temp_l[x]
                            F = temp_l[f]
                            sel_pts = [A, B, C, D, E, F]
                            #print(sel_pts)
                            R, t, e = calibration_algorithum(np.array(sel_pts), undist_K, all_selected_pts)
                            if e < last_e :
                                last_e = e
                                final_R = R
                                final_t = t
                                used_pts = [a,b,c,d,x,f]
                                used_pts_cord = np.array(sel_pts)
    print('RANSAC FINISHED!!!!!')
    f = open('R_t_ransac.txt', 'w')
    f.write('The following is the result of using these 6 points ' + str(used_pts)+'\n')
    f.write('R matrix: \n')
    f.write(np.array2string(final_R, precision=5, separator=',')+'\n')
    f.write('t vector: ')
    f.write(np.array2string(final_t, precision=5, separator=',')+'\n')
    f.write('smallest reprojection error for all '+ str(points_to_select) +' pts (in pixels): ')
    f.write(str(last_e)+'\n')
    f.write('reprojection error for the 6 selected points (in pixels): ')
    f.write(str(reprojection_err(used_pts_cord, final_R, final_t, undist_K))+'\n')
    f.close()

ransac_R_t(K_flc)

# print out in terminal how much are the clicked lidar points in matplotlib differ in distance from actual lidar points
print('potential matplotlib errors: ')
for pt in combine_xyz(new_points, new_points2):
            
    print('min dist for point ',pt, ':')
    print(min([[np.linalg.norm(np.array(p)-np.array(pt)), p] for p in list(lidar_points)]))

    
