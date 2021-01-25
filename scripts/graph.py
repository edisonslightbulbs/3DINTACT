#!/usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os
from matplotlib.colors import ListedColormap

pwd = os.getcwd()

# KNN:
data_file = pwd + "/build/output_data.txt"
plt.figure()
df = pd.read_csv(data_file)
sns.lineplot(x=df.id, y=df.nn1, hue = 100)
plt.xlabel("id")
plt.ylabel("Euclidean dist")
plt.title("knn")
# plt.show()

# DBSCAN:
data_3d_output = pwd + "/build/results.csv"
df = pd.read_csv(data_3d_output)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = df["x"]
y = df["y"]
z = df["z"]
c = df["label"]

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

cmap = ListedColormap(sns.color_palette("hls", n_colors=12))
sc = ax.scatter(x, y, z, s=25, c=c, marker='o', cmap=cmap)
plt.legend(*sc.legend_elements(), bbox_to_anchor=(1.05, 1), loc=7)
plt.show()


# get colormap from seaborn see:
# https://stackoverflow.com/questions/52285104/3d-scatterplots-in-python-with-hue-colormap-and-legend
