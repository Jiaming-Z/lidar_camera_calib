import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap
from mpl_toolkits.axes_grid1.inset_locator import inset_axes


lidar_points = []
for b in [25, 50, 100, 200]:
    pts = []
    for x_p in range(6):
        for y_p in range(6):
            for z_p in range(6):
                pts.append([b + x_p, y_p, z_p])
    lidar_points.extend(pts)
lidar_points = np.array(lidar_points)
print(lidar_points)

x = lidar_points[:, 0]
y = lidar_points[:, 1]
z = lidar_points[:, 2]

# Normalize the axis for color coding
norm_z = Normalize(vmin=min(z), vmax=max(z)) # for the x-y plot
norm_x = Normalize(vmin=min(x), vmax=max(x)) # for the z-y plot
norm_y = Normalize(vmin=min(y), vmax=max(y)) # for the z-y plot
cmap = get_cmap("rainbow")


# Creating the plots and image subplot in the same figure
fig = plt.figure(figsize=(15, 10))
gspec = fig.add_gridspec(2, 3)
view_ax = plt.subplot(gspec[0, :])
axs = [plt.subplot(gspec[1, 0]), plt.subplot(gspec[1, 1])]

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

'''
# Function to create or update views
def create_or_update_views():
    # TODO: Make this better, add a functionality to capture the view you want to save
    view_ax.clear()
    view_ax.set_axis_off()
    view_ax.imshow(fig.canvas.renderer.buffer_rgba()[500:], aspect='auto')

create_or_update_views()
'''
#view_ax.imshow()
plt.show()