import cv2 as cv
import numpy as np
from vision import detection, utils, constants
from vision.VisionObjects import Thymio, create_VisionObject, VisionObjectsTypes, obstacle_color_text, goal_color_text, target_color_text



def recognition(grid):
    """
    recognition of obstacles, targets and goal
    :param grid: environment
    :return flag: True if all objects are found, False otherwise
    :return obstacles: occupacy grid (1 if obstacle, 0 if free)
    :return targets: list of targets
    :return goal: goal
    """

    obstacles = []  # list of obstacles
    targets = []  # list of targets
    goal = []  # goal


    # extract obstacles
    contours = detection.colored_object_extraction(grid, obstacle_color_text)
    if contours is None:
        utils.print_error("Error in vision.functions.recognition: no obstacle found")
        return False, None, None, None
    for c in contours:
        obstacles.append(create_VisionObject(c, VisionObjectsTypes.OBSTACLE))

    # extract target
    contours = detection.colored_object_extraction(grid, target_color_text)
    if contours is None:
        utils.print_error("Error in vision.functions.recognition: no target found")
        return False, None, None, None
    for c in contours:
        targets.append(create_VisionObject(c, VisionObjectsTypes.TARGET))

    # extract goal
    contours = detection.colored_object_extraction(grid, goal_color_text)
    if contours is None:
        utils.print_error("Error in vision.functions.recognition: no goal found")
        return False, None, None, None
    for c in contours:
        goal.append(create_VisionObject(c, VisionObjectsTypes.GOAL))

    grid_width_pixel = grid.shape[1]
    grid_height_pixel = grid.shape[0]

    # compute position on grid
    for o in obstacles:
        o.compute_grid_coordinates(grid_width_pixel, grid_height_pixel)
    for t in targets:
        t.compute_grid_coordinates(grid_width_pixel, grid_height_pixel)
    for g in goal:
        g.compute_grid_coordinates(grid_width_pixel, grid_height_pixel)

    return True, obstacles, targets, goal


def perspective(source, grid_corners):
    dst_dim = [constants.grid_width_cells * constants.cell_resolution_in_pixel,
               constants.grid_height_cells * constants.cell_resolution_in_pixel]
    grid = detection.perspective_transform(source, grid_corners, dst_dim)
    return grid


def thymio_recognition(env):
    """
    recognition of thymio
    :param env: environment (image from camera)
    :return: Thymio object
    """
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_1000)
    arucoParams = cv.aruco.DetectorParameters_create()
    positions, ids, rejected = cv.aruco.detectMarkers(env, arucoDict, parameters=arucoParams)
    print("------------------")
    print("thymio_recognition: markers found")
    print("IDS:")
    print(ids)
    print("------------------")
    if ids is None:
        utils.print_error("Error in vision.functions.thymio_recognition: no markers found")
        return False, None
    thymio = None
    for id_list in ids:  # each id is inside a list
        id_ = id_list[0]  # get the id
        if id_ == 5:
            pos_idx = np.nonzero(ids == id_list)  # get the index of the id
            pos_idx = pos_idx[0][0]
            pos_int = positions[pos_idx].astype(int)
            thymio = Thymio(pos_int[0])

    if thymio is None:
        utils.print_error("Error in vision.functions.thymio_recognition: Thymio not found")
        return False, None

    thymio.pose_estimation(env.shape[1], env.shape[0])

    return True, thymio


def prepare_output(thymio, obs, tar, g):
    """
    prepare the output to send to the robot
    :param thymio: thymio VisionObject
    :param obs: list of obstacles VisionObject
    :param tar: list of targets VisionObject
    :param g: goal VisionObject
    :return: output
    """

    state = thymio.state

    # obstacles
    obs_grid = np.zeros((constants.grid_height_cells, constants.grid_width_cells))
    for o in obs:
        obs_grid[o.center_grid[0], o.center_grid[1]] = 1

    # targets
    # create a list with all the targets' position
    targets = []
    for t in tar:
        targets.append(t.center_grid)

    # goal
    goal = g.center_grid

    return state, obs_grid, targets, goal


def env_init(source):
    """
    environment initialization: grid extraction, perspective transform and object recognition
    :param source: image from camera
    :return flag: Successful flag
    :return grid: grid
    :return obs: list of obstacles
    :return obs_grid: occupation grid (1 if obstacle, 0 if free)
    :return targets: list of targets
    :return goal: list of goals
    """
    grid_corners, _, flag = detection.grid_extraction(source)
    if not flag:
        return flag, None, None, None, None, None
    # perspective transform
    grid = perspective(source, grid_corners)
    flag, obs, tar, g = recognition(grid)
    if not flag:
        return flag, None, None, None, None, None

    # PREPARE DATA
    # obstacles
    obs_grid = np.zeros((constants.grid_height_cells, constants.grid_width_cells))
    for o in obs:
        obs_grid[o.center_grid[0], o.center_grid[1]] = 1

    # targets
    targets = []
    for t in tar:
        targets.append(t.center_grid)

    # goal
    goal = []
    for gg in g:
        goal.append(gg.center_grid)

    return flag, grid, obs, obs_grid, targets, goal


# -----------------------------------------------------------------------------------------------
#                           TO BE CALLED FROM OUTSIDE THE MODULE
# -----------------------------------------------------------------------------------------------

def vision_init(cap):
    """
    initialize vision
    :param cap: camera object
    :return flag: True if vision is initialized, False otherwise
    :return other variables: see env_init (above)
    """
    flag = False

    for _ in range(10):
            _, env = cap.read()
            utils.display_image("Test", env)
            flag, grid, obs, obs_grid, targets, goal = env_init(env)
            if flag:
                break

    if not flag:
        utils.print_error("Error in vision.functions.vision_init: cannot initialize vision")
        return False, None, None, None, None, None
    return flag, grid, obs, obs_grid, targets, goal


def get_thymio(cap):
    """
    get Thymio's pose
    :param cap: camera object
    :return flag: True if Thymio is found, False otherwise
    :return thymio: Thymio object
    :return state: [x, y, theta]
    """
    flag_grid = False
    flag_thymio = False

    for _ in range(10):
        _, env = cap.read()
        grid_corners, _, flag_grid = detection.grid_extraction(env)
        if not flag_grid:
            continue
        # perspective transform
        grid = perspective(env, grid_corners)
        utils.display_image("test", grid)
        flag_thymio, thymio = thymio_recognition(grid)
        if flag_thymio == True:
            break

    if not flag_grid:
        utils.print_error("Error in vision.functions.get_thymio: cannot find grid")
        return False, None, None
    if not flag_thymio:
        utils.print_error("Error in vision.functions.get_thymio: cannot find Thymio")
        return False, None, None
    
    return True, thymio, (thymio.state[0], thymio.state[1], thymio.state[2])


def visualize_data(source, thymio, obstacles, obs_grid, targets, goal):
    """
    prepare data for visualization
    :param source:
    :param thymio:
    :param obstacles:
    :param obs_grid:
    :param targets:
    :param goal:
    :return out_img: image to show
    :return out_text: text to show
    """
    out_img = source.copy()
    out_img = utils.draw_on_image(out_img, obstacles, True)
    out_img = utils.draw_on_image(out_img, targets, True)
    out_img = utils.draw_on_image(out_img, goal, True)
    out_img = thymio.draw_thymio(out_img)

    out_text = "State: " + str(thymio.state) + "\n"
    out_text += "Obstacles (" + str(np.count_nonzero(obs_grid == 1)) + "): \n"
    out_text += str(obs_grid) + "\n"
    out_text += "Targets (" + str(len(targets)) + "): " + str(targets) + "\n"
    out_text += "Goal (" + str(len(targets)) + "): " + str(goal) + "\n"
    # print("State: " + str(state))
    # print("Obstacles (" + str(np.count_nonzero(obs_grid == 1)) + "): ")
    # print(obs_grid)
    # print("Targets (" + str(len(targets)) + "): " + str(targets))
    # print("Goal (" + str(len(targets)) + "): " + str(goal))

    return out_img, out_text

