import numpy as np
import cv2


LOW_OBSTACLE_THRESHOLD = (0, 0, 0)
LOW_DRIVABLE_THRESHOLD = (160, 160, 160)
HIGH_DRIVABLE_THRESHOLD = (256, 256, 256)
FIDELITY_LIMIT = 0.4

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, low_threshold=LOW_DRIVABLE_THRESHOLD, high_threshold=HIGH_DRIVABLE_THRESHOLD):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be between all three threshold values in RGB
    # within_range will now contain a boolean array with "True"
    # where threshold was met
    within_range = (img[:, :, 0] >= low_threshold[0]) \
                & (img[:, :, 1] >= low_threshold[1]) \
                & (img[:, :, 2] >= low_threshold[2]) \
                & (img[:, :, 0] < high_threshold[0]) \
                & (img[:, :, 1] < high_threshold[1]) \
                & (img[:, :, 2] < high_threshold[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[within_range] = 1
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
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))
    
    return warped, mask


def mark_rocks(img, low_threshold=(128, 95, 0), high_threshold=(230, 200, 80)):
    rock_map = ((img[:, :, 0] > low_threshold[0]) & (img[:, :, 1] > low_threshold[1]) & (img[:, :, 2] > low_threshold[2]) &
                (img[:, :, 0] < high_threshold[0]) & (img[:, :, 1] < high_threshold[1]) & (img[:, :, 2] < high_threshold[2]))
    result = np.zeros_like(img[:, :, 0])
    result[rock_map] = 1
    return result


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 0) Reject processing if rover is not stable enough
    fidelity = True
    if (Rover.roll > FIDELITY_LIMIT and Rover.roll < 360 - FIDELITY_LIMIT) or (Rover.pitch > FIDELITY_LIMIT and Rover.pitch < 360 - FIDELITY_LIMIT):
        print("*** Roll=", Rover.roll,"*** Pitch=",Rover.pitch)
        fidelity = False
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    image = Rover.img
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset]
                              ])
    # 2) Apply perspective transform4
    flattened_image, effective_map = perspect_transform(image, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    drivable_area = color_thresh(flattened_image)
    obstacle_area = color_thresh(flattened_image, LOW_OBSTACLE_THRESHOLD, HIGH_DRIVABLE_THRESHOLD) * effective_map

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:, :, 0] = obstacle_area * 255
    Rover.vision_image[:, :, 2] = drivable_area * 255

    # 5) Convert map image pixel values to rover-centric coords

    x_pixel, y_pixel = rover_coords(drivable_area)
    obstacle_x_pixel, obstacle_y_pixel = rover_coords(obstacle_area)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size

    x_world, y_world = pix_to_world(x_pixel, y_pixel, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_pixel, obstacle_y_pixel, Rover.pos[0], Rover.pos[1],
                                                      Rover.yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    if fidelity:
        Rover.worldmap[y_world, x_world, 2] += 10
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
    
    distance, angles = to_polar_coords(x_pixel, y_pixel)
    Rover.nav_angles = angles

    # rock_map = mark_rocks(flattened_image, levels=(110, 110, 50))
    rock_map = mark_rocks(flattened_image, low_threshold=(128, 95, 0), high_threshold=(230, 200, 80))
    if rock_map.any():
        rock_x_pixel, rock_y_pixel = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x_pixel, rock_y_pixel, Rover.pos[0], Rover.pos[1],
                                                      Rover.yaw, world_size, scale)
        rock_distance, rock_angle = to_polar_coords(rock_x_pixel, rock_y_pixel)
        rock_index = np.argmin(rock_distance)
        rock_x_center = rock_x_world[rock_index]
        rock_y_center = rock_y_world[rock_index]

        if fidelity:
            Rover.worldmap[rock_y_center, rock_x_center, 1] = 255
        Rover.vision_image[:, :, 1] = rock_map * 255
    else:
        Rover.vision_image[:, :, 1] = 0

    return Rover
