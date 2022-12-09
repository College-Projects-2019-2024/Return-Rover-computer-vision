import numpy as np
import cv2

def sub(arr, window):   
    m = arr.shape[1]
    supressedarr = np.ndarray(shape =(0, m), dtype = arr.dtype)
    

    for element in arr:
        u = True
        for suppressed in supressedarr:
            c = 0      
            for k in range (m):
                if ( abs (element[k] - suppressed[k])  < window) :
                    c+=1 
            e=0
            for k in range (m):
                if ( abs (element[k] - suppressed[k])  == 0) :
                    e+=1
            if (c==m and e!=m):
                    u = False
                    for i in range (m):
                        suppressed[i] =( suppressed[i] + element[i] )/2

        if (u):
            supressedarr = np.append(np.array(  [element]  ), supressedarr, axis=0)


    return supressedarr

def get_src():
    example_grid = "../calibration_images/example_grid1.jpg"

    grid_img = cv2.imread(example_grid)


    gray = cv2.cvtColor(grid_img,cv2.COLOR_BGR2GRAY)


    blurred = cv2.GaussianBlur(gray, (3,3), 0)

    edges = cv2.Canny(blurred, 70,30)


    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 10, minLineLength=80, maxLineGap=60)

    lines = lines.reshape(  (lines.shape[0], lines.shape[2])   )  

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[


    (0,     height),

    (0,     int(height*0.8)),                         
    # Bottom-left point
    (int(width*0.2),  int(height*0.55)),    # Top-left point
    (int(width*0.8), int(height*0.55)),    # Top-right point
    (width,     int(height*0.8)),
    (width, height),                        # Bottom-right point
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)


    edges = cv2.bitwise_and(mask, edges)


    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 10, minLineLength=100, maxLineGap=60)

    lines = lines.reshape(  (lines.shape[0], lines.shape[2])   )  


    img = np.zeros_like(edges, dtype=None, shape=None)


    lines = sub(lines,30)

    out= []

    for i in range (4):

        gg = np.zeros_like(edges, dtype=None, shape=None)

        x1, y1, x2, y2 = lines[i]

        cv2.line(gg, (x1, y1), (x2, y2), 50, 1)

        out.append(gg)
    
    points = np.ndarray(shape=(0,2),dtype=np.int32)


    for k in range (1,4):
        out[0] += out[k]

    out[0][out[0] <=50] = 0

    for j in range (out[0].shape[0]):
        for i in range (out[0].shape[1]):
            if (out[0][j][i]>80):
                points = np.append(points, np.array(  [[i,j]]  ), axis=0)

    subressedpoints =   sub(points, 4)               
    
    
    subressedpoints[[0, 1]] = subressedpoints[[1, 0]]
    

    return subressedpoints






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

def impose(xpix, ypix, range=80):      # to make x and y more clear and detected more accuarte
    dist = np.sqrt(xpix**2 + ypix**2)
    return xpix[dist < range], ypix[dist < range]

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
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]),M,(img.shape[1], img.shape[0]))
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped,mask

def rock_thresh(img, yellow_thresh=(100, 100, 20)):
    x = np.zeros_like(img[:,:,0])

    rock = (img[:,:,0] > yellow_thresh[0]) \
                & (img[:,:,1] > yellow_thresh[1]) \
                & (img[:,:,2] < yellow_thresh[2])

    x[rock] = 1
    return x

src = get_src()


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    warped,mask = perspect_transform(Rover.img,source,destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = color_thresh(warped)
    rocks = rock_thresh(warped)
    obstacles = np.absolute(np.float32(navigable)-1)*mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,1] = rocks *255
    Rover.vision_image[:,:,2] = navigable*255
    Rover.vision_image[:,:,0] = obstacles*255

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

   dist, angles = to_polar_coords(xpix_navigable, ypix_navigable)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    
 
    
    
    return Rover
