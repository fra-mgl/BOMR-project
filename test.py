import asyncio
import time
import math
import numpy as np
from cv2 import VideoCapture
import matplotlib.pyplot as plt
from matplotlib import colors
import Global
# from vision.module import detection, utils, constants
# from vision.functions import vision_init, get_thymio
from vision.functions import vision_init, get_thymio
# import vision_nav
import KF
from Control import Control
import cv2
from itertools import chain
import math
import MotionControl

from vision.utils import display_image

# lock the camera resource
cap = VideoCapture(1)

grid = None
obs = []
obs_grid = None
targets = []
goal = []

# import constants
from vision.constants import grid_height_cells, grid_width_cells

#  vision initialization

flag, grid, obs, obs_grid, targets, goal = vision_init(cap)
print(obs_grid)

if not flag:
    assert 0

print(targets)

display_image("", grid)

get_thymio(cap)
# if you get to this point, the vision is initialized and you have acquired information about the environment