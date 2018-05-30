import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, hsv_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    thresh_min = hsv_thresh[0]
    thresh_max = hsv_thresh[1]
    within_thresh = (hsv_img[:, :, 0] >= thresh_min[0]) & (hsv_img[:, :, 0] <= thresh_max[0]) & \
                    (hsv_img[:, :, 1] >= thresh_min[1]) & (hsv_img[:, :, 1] <= thresh_max[1]) & \
                    (hsv_img[:, :, 2] >= thresh_min[2]) & (hsv_img[:, :, 2] <= thresh_max[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[within_thresh] = 1
    # Return the binary image
    return color_select


def rock_thresh(img, rgb_thresh=(110, 110, 50)):
        # Create an array of zeros same xy size as img, but single channel
        color_select = np.zeros_like(img[:, :, 0])
        # Require that each pixel be above all three threshold values in RGB
        # above_thresh will now contain a boolean array with "True"
        # where threshold was met
        rock_thresh = (img[:, :, 0] > rgb_thresh[0]) \
            & (img[:, :, 1] > rgb_thresh[1]) \
            & (img[:, :, 2] > rgb_thresh[2])
        # Index the array of zeros with the boolean array and set to 1
        color_select[rock_thresh] = 1
        # Return the binary image
        return color_select


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
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
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1],
                                                               img.shape[0]))
    # mask[80:, :] = 1
    return warped, mask


# Apply the above functions in succession and update Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst_size = 5
    bottom_offset = 8
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])

    # 2) Apply perspective transform
    warped, mask = perspect_transform(image, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    thresholded = color_thresh(warped, hsv_thresh=([0, 0, 211], [255, 255, 255]))
    #thresholded[:30, :] = 0
    obs_map = np.absolute(np.float32(thresholded) - 1) * mask
    rock_map = color_thresh(warped, hsv_thresh=([60, 120, 110], [135, 255, 255]))

    # 4) Update Rover.vision_image
    Rover.vision_image[:, :, 0] = obs_map * 255
    Rover.vision_image[:, :, 1] = rock_map * 255
    Rover.vision_image[:, :, 2] = thresholded * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(thresholded)
    obsxpix, obsypix = rover_coords(obs_map)
    rockxpix, rockypix = rover_coords(rock_map)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1],
                                                        Rover.yaw, world_size, scale = 10)
    obstacle_x_world, obstacle_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1]
                                                      , Rover.yaw, world_size, scale = 40)
    rock_x_world, rock_y_world = pix_to_world(rockxpix, rockypix, Rover.pos[0], Rover.pos[1]
                                                          , Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.pitch < 0.5 or Rover.pitch > 359.5) and \
       (Rover.roll < 0.5 or Rover.roll > 359.5):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    if rock_map.any():
        Rover.mode = 'rock_found'
        dist, angles = to_polar_coords(rockxpix, rockypix)
    else:
        dist, angles = to_polar_coords(xpix, ypix)

    Rover.nav_angles = angles
    Rover.nav_dists = dist

    return Rover
