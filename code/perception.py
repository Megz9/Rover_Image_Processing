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

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask =cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))#mask needed to get the obstacle 
    return warped, mask
def find_rocks(img,levels=(110,110,50)):
    above_thresh = (img[:,:,0] > levels[0]) \
    & (img[:,:,1] > levels[1]) \
    & (img[:,:,2] < levels[2])
    color_select = np.zeros_like(img[:,:,0])
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst = 15
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

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obs_map=np.absolute(np.float32(threshed)-1)*mask
    bottom_offset = 60
    threshed_goodfidlity = threshed
    obs_map_goodfility = obs_map
    threshed_goodfidlity[0:bottom_offset, :] = threshed[0:bottom_offset, :] * 0
    obs_map_goodfility[0:bottom_offset, :] = obs_map[0:bottom_offset, :] * 0
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,2] = threshed_goodfidlity*255
    #Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    Rover.vision_image[:,:,0] = obs_map_goodfility*255

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    obsx, obsy = rover_coords(obs_map)
    # 6) Convert rover-centric pixel values to world coordinates
    xpos,ypos=Rover.pos
    yaw=Rover.yaw
    world_size=Rover.worldmap.shape[0]
    scale=2* dst
    x_world,y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    obsx_world,obsy_world = pix_to_world(obsx, obsy, xpos, ypos, yaw, world_size, scale)
    ###good fidlity
    xpix_goodfidlity, ypix_goodfidlity = rover_coords(threshed_goodfidlity)
    obsx_goodfidlity, obsy_goodfility = rover_coords(obs_map_goodfility)
    x_world_goodfidlity,y_world_goodfidlity = pix_to_world(xpix_goodfidlity, ypix_goodfidlity, xpos, ypos, yaw, world_size, scale)
    obsx_world_goodfidlity,obsy_world_goodfidlity = pix_to_world(obsx_goodfidlity, obsy_goodfility, xpos, ypos, yaw, world_size, scale)
    if ((Rover.roll > 359.5) or (Rover.roll < 0.5)) and ((Rover.pitch > 359.5) or (Rover.pitch < 0.5)):

        # 7) Update Rover worldmap (to be displayed on right side of screen)
        Rover.worldmap[obsy_world_goodfidlity, obsx_world_goodfidlity, 0] = 255
        #          Rover.fworldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[y_world_goodfidlity, x_world_goodfidlity, 2] = 255
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)

    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_dists=dist
    # Rover.nav_angles = rover_centric_angles
    Rover.nav_angles=angles    
         # finding rocks functions
    rock_map = find_rocks(warped)
    if rock_map.any():
        xrocks, yrocks = rover_coords(rock_map)
        x_world_rock,y_world_rock = pix_to_world(xrocks, yrocks, xpos, ypos, yaw, world_size, scale)
        rocks_dist,rocks_angles = to_polar_coords(xrocks, yrocks)
        rock_idx=np.argmin(rocks_dist)
        rock_xcen = x_world_rock[rock_idx]
        rock_ycen=y_world_rock[rock_idx]
        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:,:,1] = rock_map*255
        # 8) Convert rover-centric pixel positions to polar coordinates
        distRock, anglesRock = to_polar_coords(xrocks, yrocks)
        Rover.nav_anglesrock=anglesRock
        Rover.nav_distrock=distRock
        
    else:
        Rover.vision_image[:,:,1] = 0
        Rover.nav_distrock=None

    
    obsDist, obsAngles = to_polar_coords(obsx, obsy)
    Rover.obs_angles=obsAngles
    Rover.obs_dist=obsDist   
    
    return Rover
