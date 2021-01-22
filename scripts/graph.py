#!/usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os
from matplotlib.colors import ListedColormap

pwd = os.getcwd()


data_3d_output = pwd + "/build/results.csv"

# AFTER CLUSTERING:
# import data
df = pd.read_csv(data_3d_output)

# figure container
fig = plt.figure()

# add axes to the figure in subplot arrangement
ax = fig.add_subplot(111, projection='3d')

x = df["x"]
y = df["y"]
z = df["z"]
c = df["label"]

# figure labels
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# get colormap from seaborn see:
# https://stackoverflow.com/questions/52285104/3d-scatterplots-in-python-with-hue-colormap-and-legend
cmap = ListedColormap(sns.color_palette("hls", n_colors=12))

# draw scatter plot
#ax.scatter(x, y, z)

# plot
sc = ax.scatter(x, y, z, s=25, c=c, marker='o', cmap=cmap)

# legend
plt.legend(*sc.legend_elements(), bbox_to_anchor=(1.05, 1), loc=7)


# show scatter plot
plt.show()
