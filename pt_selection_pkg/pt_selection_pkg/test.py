import matplotlib.pyplot as plt
import numpy as np

# Your initial data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Creating the plot
fig, ax = plt.subplots()

# Plot the initial data
ax.plot(x, y, color='blue')

# New points data, annotations and its plot, initialized as empty and None.
new_points = []
new_points_plot = None
annotations = []

def update_plot():
    global new_points_plot

    # If a plot for the new points exists, remove it.
    if new_points_plot is not None and len(annotations) > 0:
        print(f"NewPointsPlot: {new_points_plot}")
        new_points_plot.remove()

        # Remove existing annotations.
        for annotation in annotations:
            annotation.remove()
        annotations.clear()

    # If there are new points, plot them.
    if new_points:
        x_values, y_values = zip(*new_points)
        new_points_plot = ax.scatter(x_values, y_values, color='red')
        for i, txt in enumerate(new_points):
            annotation = ax.annotate(i+1, (x_values[i], y_values[i]))
            annotations.append(annotation)
    
    # Display total selected points
    total_points = len(new_points)
    plt.title(f"Total selected points: {total_points}")

    plt.draw()

def onclick(event):
    # If the left mouse button is clicked, add a point.
    if event.button == 1 and event.xdata and event.ydata:
        new_points.append((event.xdata, event.ydata))

    # If the right mouse button is clicked, remove the last point if it exists.
    elif event.button == 3 and new_points:
        new_points.pop()

    update_plot()

# Connect the mouse click event to the onclick function.
cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()