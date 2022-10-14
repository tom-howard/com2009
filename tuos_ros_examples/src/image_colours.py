#!/usr/bin/env python3
# adapted from realpython.com:
# https://realpython.com/python-opencv-color-spaces/

import argparse
import cv2
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import os

cli = argparse.ArgumentParser(description="Obtain pixel colours from an image")

cli.add_argument("Path", metavar="path", type=str, help="Path to the image file")

args = cli.parse_args()

img = cv2.imread(args.Path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

if os.path.getsize(args.Path) > 100000:
    msg = """
    This image is quite big, so it might take some time to process.
    Consider cropping it down a bit to make things faster... 
    """
    print(msg)

print("Processing the image, a plot will appear shortly...")

pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

h, s, v = cv2.split(hsv)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.scatter(h.flatten(), s.flatten(), facecolors=pixel_colors, marker=".")
ax.set_xlabel("Hue")
ax.set_ylabel("Saturation")
ax.grid(True)
plt.show()
