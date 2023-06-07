import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

# Your initial 3D data
n = 100
x = np.random.rand(n)
y = np.random.rand(n)
z = np.random.rand(n)

# Normalize the axis for color coding
norm_xz = Normalize(vmin=min(z), vmax=max(z)) # for the x-y plot
norm_yz = Normalize(vmin=min(x), vmax=max(x)) # for the z-y plot
cmap = get_cmap("rainbow")

# Load your image
img = mpimg.imread('image_undistorted.png')

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
colors = cmap(norm_xz(z))
axs[0].scatter(x, y, c=colors)
axs[0].set_xlabel('X')
axs[0].set_ylabel('Y')

# Plot the second data as scatter plot
colors = cmap(norm_yz(x))
axs[1].scatter(z, y, c=colors)
axs[1].set_xlabel('Z')
axs[1].set_ylabel('Y')

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