import datashader as ds
import datashader.transfer_functions as tf
from datashader.utils import export_image
import pandas as pd
import numpy as np

# Your initial 3D data
n = 100
x = np.random.rand(n)
y = np.random.rand(n)
z = np.random.rand(n)

# Create a dataframe
df = pd.DataFrame({'x': x, 'y': y, 'z': z})

# Create a canvas with the range of your data
canvas = ds.Canvas(x_range=(df.x.min(), df.x.max()), y_range=(df.y.min(), df.y.max()), plot_width=800, plot_height=600)

# Aggregate the points on your canvas
agg = canvas.points(df, 'x', 'y')

# Shade your data
img = tf.shade(agg, cmap=['lightblue', 'darkblue'], how='log')

# You can now display or export your image
export_image(img, 'my_image')