import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_above=(160, 160, 160), rgb_below=(255,255,255)):
    # make sure all values are in range:
    rgb_above = np.clip(rgb_above,0,255)
    rgb_below = np.clip(rgb_below,0,255)
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be between all three threshold values in RGB
    # rgb_above and rgb_below will now contain a boolean array with "True"
    # where threshold was met
	
	# ok, I'll need to understand this in detail...
	
    between_thresh = (img[:,:,0] > rgb_above[0]) \
                & (img[:,:,1] > rgb_above[1]) \
                & (img[:,:,2] > rgb_above[2]) \
                & (img[:,:,0] < rgb_below[0]) \
                & (img[:,:,1] < rgb_below[1]) \
                & (img[:,:,2] < rgb_below[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[between_thresh] = 1
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
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    # 1) Define source and destination points for perspective transform.
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])

    # 2) apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3a) apply col threshold for navigable terrain
    nav_min = [160,160,160]
    nav_max = [255,255,255]
    nav_thres = color_thresh(warped,nav_min,nav_max)

    # 3b) apply col threshold for rocks
    rock_col_delta = 50
    rock_col_code = [162,133,7]
    rock_min = np.zeros_like(rock_col_code) #init
    rock_min[0] = rock_col_code[0]-rock_col_delta
    rock_min[1] = rock_col_code[1]-rock_col_delta
    rock_min[2] = rock_col_code[2]-rock_col_delta
    rock_max = np.zeros_like(rock_col_code) #init
    rock_max[0] = rock_col_code[0]+rock_col_delta
    rock_max[1] = rock_col_code[1]+rock_col_delta
    rock_max[2] = rock_col_code[2]+rock_col_delta
 
    rock_thres = color_thresh(warped,rock_min,rock_max)
    rock_x_debug = np.average(rock_thres)

    # 3c) apply col threshold for obstacles
    obst_min = [0,0,0]
    obst_max = nav_min
    obst_thres = color_thresh(warped,obst_min,obst_max)
	
	# 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    #Rover.vision_image[:,:,0] = obst_thres
    #Rover.vision_image[:,:,1] = rock_thres
    #Rover.vision_image[:,:,2] = nav_thres
    Rover.vision_image[:,:,0] = np.ones_like(obst_thres)
    Rover.vision_image[:,:,1] = np.zeros_like(rock_thres)
    #Rover.vision_image[:,:,2] = nav_thres

    
    # 5) Convert map image pixel values to rover-centric coords
    navX, navY = rover_coords(nav_thres) #navigable terrain
    rockX, rockY = rover_coords(rock_thres) #rocks
    obstX, obstY = rover_coords(obst_thres) #obstacles
    
    
    # 6) Convert rover-centric pixel values to world coordinates
    
    world_size = Rover.worldmap.shape[0]
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(navX, navY, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rockX, rockY, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstX, obstY, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
		# dist, angles = to_polar_coords(x_pixel, y_pixel)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navX, navY)
    
    
    return Rover