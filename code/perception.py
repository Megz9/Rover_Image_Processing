#%matplotlib qt # Choose %matplotlib qt to plot to an interactive window (note it may show up behind your browser)
# Make some of the relevant imports
import cv2 # OpenCV for perspective transform
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import scipy.misc # For saving images as needed
import glob  # For reading in a list of images from a folder
import imageio
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
def find_rocks(img,levels=(110,110,50)):
    above_thresh = (img[:,:,0] > levels[0]) \
    & (img[:,:,1] > levels[1]) \
    & (img[:,:,2] < levels[2])
    
    color_select = np.zeros_like(img[:,:,0])
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))#
    return warped, mask
    
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst = 10
    bottom_offset = 5
    source = np.float32([[14, 140],
                         [300, 140],
                         [200, 95],
                         [120, 95]])

    destination = np.float32([[image.shape[1] / 2 - dst, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst, image.shape[0] - 2*dst - bottom_offset],
                              [image.shape[1] / 2 - dst, image.shape[0] - 2*dst - bottom_offset]])


    # 2) Apply perspective transform
    warped,mask = perspect_transform(image, source, destination)
