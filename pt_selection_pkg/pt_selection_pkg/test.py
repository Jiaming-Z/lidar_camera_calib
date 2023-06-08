import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import os
import pickle

# Name of the files
input_filename = "1_pdc_calib_data.pkl"
with open(f"/home/zhihao/chris/ros_workspace/src/lidar_camera_calib/pt_selection_pkg/pt_selection_pkg/calib_databags/{input_filename}", 'rb') as file:
    # Read the Pickle file
    data = pickle.load(file)

# Get the data
lidar_points = data['points'] # 3D points
lidar_points = np.array(lidar_points) # Assume it is comin in x, y, z
print (lidar_points.shape) 
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

# Load your image
img = data['camera_images_flc']#mpimg.imread('image_undistorted.png')

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
img_plot = axs[2].imshow(img)

# New points data, annotations and its plot, initialized as empty and None.
new_points = []
new_points_plot = None
annotations = []
guidelines0 = []

# New points for the second scatter plot
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
cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()