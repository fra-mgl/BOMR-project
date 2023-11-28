import numpy as np
import cv2 as cv
from module.VisionObjects import VisionObject, create_VisionObject, general_color_text, VisionObjectsTypes
from module.utils import print_error, display_image, draw_on_image
from module.constants import morph_kernel_dim

corner_color = general_color_text


def colored_object_extraction(image, color):
    """
    given an image and a color, returns the image with only the objects of that color
    :param image: openCV image to process
    :param color: color to extract
    :return: opnCV contours of the objects of that color
    """

    # convert to HSV
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # color range for HSV space

    # red
    # note: we define two range of red because it is the only color that is not contiguous in the HSV space
    # (i.e. it's on the border of the space)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # green
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # blue
    lower_blue = np.array([100, 40, 40])
    upper_blue = np.array([140, 255, 255])

    # magenta
    lower_magenta = np.array([140, 40, 40])
    upper_magenta = np.array([160, 255, 255])

    # if to choose color range
    if color == "red":
        lower = None
        upper = None
    elif color == "green":
        lower = lower_green
        upper = upper_green
    elif color == "blue":
        lower = lower_blue
        upper = upper_blue
    elif color == "magenta":
        lower = lower_magenta
        upper = upper_magenta
    else:
        print_error("Error in module.utils.colored_object_extraction: color not recognized")
        return

    #  create the mask
    if color == "red":
        mask = cv.inRange(hsv, lower_red1, upper_red1) | cv.inRange(hsv, lower_red2, upper_red2)
    else:
        mask = cv.inRange(hsv, lower, upper)
    """
     From OpenCV documentation:
     In OpenCV, finding contours is like finding white object from black background.
     So object to be found should be white and background should be black.
    """

    # Morphology open -> to remove small objects (generated by noisy environment) we can apply a morphological
    # opening filter (erosion followed by dilation)
    kernel = np.ones((morph_kernel_dim, morph_kernel_dim), np.uint8)  # Kernel definition
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    """
    From OpenCV documentation:
    If you pass cv.CHAIN_APPROX_NONE, all the boundary points are stored. But actually do we need all the points? For instance, you found the contour of a straight line.
    Do you need all the points on the line to represent that line? No, we need just two end points of that line.
    This is what cv.CHAIN_APPROX_SIMPLE does. It removes all redundant points and compresses the contour, thereby saving memory."""
    # find contours
    ret, thresh = cv.threshold(mask, 40, 255, 0)
    if int(cv.__version__[0]) > 3:
        contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    else:
        _, contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    return contours


def grid_extraction(image):
    """
        given the image, extract the grid in which the Thymio moves
        and return a cropped image with just the grid
        :param image: openCV image to process
        :return corners: all 4 corners of the grid
        :return check: debug variable
        :return success: True or False. If False, second return argument used for debugging
        """

    #  grayscale
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # extract the four corner identifiers: they are 4 aruco markers
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_1000)
    arucoParams = cv.aruco.DetectorParameters_create()
    positions, ids, rejected = cv.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    print("------------------")
    print("grid_extraction: markers found")
    print("IDS:")
    print(ids)
    print("------------------")

    if ids is None:
        print_error("Error in module.utils.grid_extraction: no markers found")
        return None, None, False

    top_left = None
    top_right = None
    bottom_left = None
    bottom_right = None

    # extract aruco with id = 1, 2, 3 or 4
    for id_list in ids:  # each id is inside a list
        id_ = id_list[0]  # get the id
        pos_idx = np.nonzero(ids == id_list)  # get the index of the id
        pos_idx = pos_idx[0][0]
        pos_int = positions[pos_idx].astype(int)
        pos_int = pos_int.reshape((-1, 1, 2))   # to be compliant with opencv's contours definition
        if id_ == 1:
            top_left = VisionObject(pos_int)
        elif id_ == 2:
            top_right = VisionObject(pos_int)
        elif id_ == 3:
            bottom_left = VisionObject(pos_int)
        elif id_ == 4:
            bottom_right = VisionObject(pos_int)
        else:
            pass

    # check if all 4 identifiers have been found
    # all 4 corners are needed to compute perspective transform
    check = [top_left, top_right, bottom_left, bottom_right]
    count_of_none = sum(1 for element in check if element is None)
    if count_of_none > 0:  # need to detect all 4 markers
        print_error("Error in module.utils.grid_extraction: cannot classify corners correctly")
        return None, check, False

    top_left.compute_min_max()
    top_right.compute_min_max()
    bottom_left.compute_min_max()
    bottom_right.compute_min_max()

    corners = np.array([
        [top_left.min[0], top_left.min[1]],
        [top_right.max[0], top_right.min[1]],
        [bottom_left.min[0], bottom_left.max[1]],
        [bottom_right.max[0], bottom_right.max[1]]
    ])

    return corners, None, True


def perspective_transform(source, src_points, dst_dim):
    """
    given the source image and the destination image, compute the perspective transform
    :param source: source image - format example: np.float32([[0, 0], [500, 0], [0, 400], [500, 400]])
    :param src_points: source corners
    :param dst_dim: destination dimension - tuple (width, height)
    :return: corrected image
    """
    src_points = np.float32(src_points)
    # destination points
    dst_points = np.float32([
        [0, 0],
        [dst_dim[0], 0],
        [0, dst_dim[1]],
        [dst_dim[0], dst_dim[1]]
    ])

    # compute the perspective transform
    M = cv.getPerspectiveTransform(src_points, dst_points)
    dst = cv.warpPerspective(source, M, dst_dim)
    return dst



