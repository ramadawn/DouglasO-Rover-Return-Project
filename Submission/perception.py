import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, thresh_type , rgb_thresh=(160, 160, 160),):
    # Create an array of zeros same xy size as img, but single channel
    thresh = np.zeros_like(img[:,:,0])
    
    if thresh_type == "navigation":
        thresh_programmer = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
       
    
    elif thresh_type == "features":
        thresh_programmer = (img[:,:,0] < rgb_thresh[0]) \
                    & (img[:,:,1] < rgb_thresh[1]) \
                    & (img[:,:,2] < rgb_thresh[2])
    
    else:
        thresh_programmer =(img[:,:,0] > 184) \
                & (img[:,:,1] > 164) \
                & (img[:,:,2] < 73)
        
         
    thresh[thresh_programmer] = 1
   
    return thresh
    

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    yaw_rad = yaw * np.pi / 180
    
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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
    img = Rover.img
    half_square_transition = 5
    bottom_offset = 9
    source = np.float32([[118,99],[205,99],[2,151],[315,151]])
    dest = np.float32([[img.shape[1]/2 - half_square_transition, img.shape[0] - 2*half_square_transition - bottom_offset], 
                      [img.shape[1]/2 + half_square_transition, img.shape[0] - 2*half_square_transition - bottom_offset],
                      [img.shape[1]/2 - half_square_transition, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + half_square_transition, img.shape[0] - bottom_offset]])
    # 2) Apply perspective transform
    
    
    warped = perspect_transform(img, source, dest)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = color_thresh(warped, "navigation")
    obstacle = color_thresh(warped, "features" ) 
    rock_sample = color_thresh(warped, "stone")
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle 
    Rover.vision_image[:,:,1] = rock_sample 
    Rover.vision_image[:,:,2] = navigable 

    # 5) Convert map image  pixel values to rover-centric coords
    colour_tran_x, colour_tran_y = rover_coords(navigable)
    terrian_tran_x, terrian_tran_y = rover_coords(obstacle)
    stone_tran_x, stone_tran_y = rover_coords(rock_sample)
    # 6) Convert rover-centric pixel values to world coordinates
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    scale = 10
    world_size = 200
    navigable_y_world, navigable_x_world = pix_to_world(colour_tran_x, colour_tran_y, xpos, ypos, yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(terrian_tran_x, terrian_tran_y, xpos, ypos, yaw, world_size, scale)
    rock_y_world, rock_x_world = pix_to_world(stone_tran_x, stone_tran_y, xpos, ypos, yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if abs(Rover.roll) > 0.5 or abs(Rover.pitch) > 0.5:
        Rover.nomapmake +=1
    else: # Rover.roll < 0.5 or Rover.pitch < 0.5:
        Rover.mapmake += 1
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(colour_tran_x, colour_tran_y)
    # Update Rover pixel distances and angles
    
    Rover.nav_dists = rover_centric_pixel_distances
    
    Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover
