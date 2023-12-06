import numpy as np
import cv2 as cv
from math import floor
from enum import Enum
from vision.utils import print_error
from vision.constants import grid_height_cells, grid_width_cells

# Define an enumeration class called Color
class VisionObjectsTypes(Enum):
    GENERAL = 0
    OBSTACLE = 1
    GOAL = 2
    TARGET = 3

general_color_text = "magenta"
general_color_RGB = (255, 0, 255) # magenta
obstacle_color_text = "red"
obstacle_color_RGB = (0, 0, 255) # red
goal_color_text = "blue"
goal_color_RGB = (255, 0, 0) # blue
target_color_text = "green"
target_color_RGB = (0, 255, 0) # green

class VisionObject:
    color = general_color_RGB
    contour = None
    center_pixel = None
    center_grid = None
    min = None
    max = None

    def __init__(self, contour):
        self.contour = contour
        self.find_center()

    def __str__(self):
        return "Vision object: type = general, pixel = " + str(self.center_pixel)

    # Instance method
    def find_center(self):
        """
        given a contour, returns the coordinates of its center
        :param contour: list of points of a contour
        :return: coordinates of the center (list)
        """
        # extract first and second coordinate of each point of the contour
        first_coord = [sublist[0][0] for sublist in self.contour]
        second_coord = [sublist[0][1] for sublist in self.contour]

        # create numpy array and compute mean
        x_coord = np.array(first_coord)
        y_coord = np.array(second_coord)

        x_mean = np.mean(x_coord)
        y_mean = np.mean(y_coord)

        self.center_pixel = [int(x_mean), int(y_mean)]

    def compute_min_max(self):
        # extract first and second coordinate of each point of the contour
        first_coord = [sublist[0][0] for sublist in self.contour]
        second_coord = [sublist[0][1] for sublist in self.contour]

        # create numpy array and compute mean
        x_coord = np.array(first_coord)
        y_coord = np.array(second_coord)

        self.min = [int(np.min(x_coord)), int(np.min(y_coord))]
        self.max = [int(np.max(x_coord)), int(np.max(y_coord))]

    def compute_grid_coordinates(self, grid_width_pixel, grid_height_pixel):
        # use floor to select cell
        y = floor(self.center_pixel[0] / grid_width_pixel * grid_width_cells)
        x = floor(self.center_pixel[1] / grid_height_pixel * grid_height_cells)
        self.center_grid = [x,y]  # coordinates are flipped to be compliant with the reference frame

    def generate_label(self):
        text = type(self).__name__ + ", " + str(self.center_grid)
        text_position = (self.center_pixel[0] - 100, self.center_pixel[1] + 50)
        return text, text_position, self.color


class Obstacle(VisionObject):
    color = obstacle_color_RGB

    def __init__(self, contour):
        super().__init__(contour)

    def __str__(self):
        return "Vision object: type = " + type(self).__name__ + ", grid = " + str(self.center_grid)


class Goal(VisionObject):
    color = goal_color_RGB

    def __init__(self, contour):
        super().__init__(contour)

    def __str__(self):
        return "Vision object: type = " + type(self).__name__  + ", grid = " + str(self.center_grid)


class Target(VisionObject):
    color = target_color_RGB

    def __init__(self, contour):
        super().__init__(contour)

    def __str__(self):
        return "Vision object: type = " + type(self).__name__  + ", grid = " + str(self.center_grid)


def create_VisionObject(contour, t):
    if t == VisionObjectsTypes.GENERAL:
        return VisionObject(contour)
    elif t == VisionObjectsTypes.OBSTACLE:
        return Obstacle(contour)
    elif t == VisionObjectsTypes.GOAL:
        return Goal(contour)
    elif t == VisionObjectsTypes.TARGET:
        return Target(contour)
    else:
        print_error("Error in module.VisionObject.create_VisionObject: type not recognized")
        return None



# ------------ Thymio class ------------
class Thymio():
    state = [0.0, 0.0, 0.0]  # x, y, theta [-180, 180]
    color = general_color_RGB
    corners = []  # each corner is a list [x, y] w.r.t. opencv's reference
    center_pixel = None
    center_grid = None

    def __init__(self, corners):
        self.corners = corners
        self.find_center()

    def __str__(self):
        return "Vision object: type = " + type(self).__name__  + ", grid = " + str(self.center_grid)

    def find_center(self):
        """
        given a contour, returns the coordinates of its center
        :param contour: list of points of a contour
        :return: coordinates of the center (list)
        """
        # extract first and second coordinate of each point of the contour
        first_coord = [sublist[0] for sublist in self.corners]
        second_coord = [sublist[1] for sublist in self.corners]

        # create numpy array and compute mean
        x_coord = np.array(first_coord)
        y_coord = np.array(second_coord)

        x_mean = np.mean(x_coord)
        y_mean = np.mean(y_coord)

        self.center_pixel = [int(x_mean), int(y_mean)]

    def compute_grid_coordinates(self, grid_width_pixel, grid_height_pixel):
        y = round(self.center_pixel[0] / grid_width_pixel * grid_width_cells, 3)
        x = round(self.center_pixel[1] / grid_height_pixel * grid_height_cells, 3)
        self.center_grid = [x,y]   # coordinates are flipped to be compliant with the reference frame
        self.state = [x,y, self.state[2]]

    def compute_theta(self):
        """
        compute the orientation of the robot
        given two points (from corners 0-3 and 1-2) get the angle between the line and the reference
        compute the angle using both the pairs of coordinates and return the average
        :return: theta
        """
        point1 = self.corners[0]
        point2 = self.corners[3]
        theta1 = np.arctan2(point1[1] - point2[1], point1[0] - point2[0])

        point1 = self.corners[1]
        point2 = self.corners[2]
        theta2 = np.arctan2(point1[1] - point2[1], point1[0] - point2[0])

        theta1 = np.rad2deg(theta1)
        theta2 = np.rad2deg(theta2)

        print("Before: ", theta1, theta2)
        theta1 = fix_theta(theta1)
        theta2 = fix_theta(theta2)
        print("After: ", theta1, theta2)

        theta = (theta1 + theta2) / 2

        # normalization
        if theta > 180:
            theta -= 360
        elif theta < -180:
            theta += 360 

        if theta < -165:
            theta = 180

        return theta

    def pose_estimation(self, grid_width_pixel, grid_height_pixel):
        """
        compute the pose of the robot
        :return: x, y, theta
        """
        self.find_center()
        self.compute_grid_coordinates(grid_width_pixel, grid_height_pixel)
        theta = self.compute_theta()
        self.state = [self.center_grid[0], self.center_grid[1], round(theta, 2)]
        return self.state

    def generate_label(self):
        text = type(self).__name__ + ", x=" + str(self.state[0]) + ", y=" + str(self.state[1]) + ", th=" + str(self.state[2])
        text_position = (self.center_pixel[0] - 700, self.center_pixel[1] + 50)
        return text, text_position, self.color

    def draw_thymio(self, image, text_choice=False):
        """
        draw a point on the image
        :param image: image on which draw the point
        :param text_choice: if True, add a label with the coordinates
        :return: image with the point
        """
        # draw boundaries: create a line between each pairs of corners and draw them
        cv.line(image, self.corners[0], self.corners[1], self.color, 10)
        cv.line(image, self.corners[1], self.corners[2], self.color, 10)
        cv.line(image, self.corners[2], self.corners[3], self.color, 10)
        cv.line(image, self.corners[3], self.corners[0], self.color, 10)

        # draw the central point and add a label with the coordinates
        cv.circle(image, self.center_pixel, 20, self.color, -1)
        if text_choice:
            text, text_position, font_color = self.generate_label()
            cv.putText(image, text, text_position, cv.FONT_HERSHEY_SIMPLEX, 2, font_color, 2)

        # draw the orientation line
        theta = self.compute_theta()
        theta = np.radians(theta)
        x = self.center_pixel[0] + 200 * np.cos(theta)
        y = self.center_pixel[1] + 200 * np.sin(theta)
        cv.arrowedLine(image, self.center_pixel, (int(x), int(y)), self.color, 10)

        return image


def fix_theta(th):

    th = -th

    new_theta = th + 90
     

    return new_theta