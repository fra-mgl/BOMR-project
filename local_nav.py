import time
import math
import numpy as np

Speed_Factor = 0.35
Long_distance = 275
Short_distance = 130
Threshold_detection = 20

def switch_sens(sens):
    """
    Switches the direction value between "Left" and "Right". 
    :param sens: The current direction value ('Left' or 'Right' assumed).
    :return: The switched direction value.
    """
    # Check the current value of 'sens'
    if sens == "Left":
        sens = "Right"
    else:
        sens = "Left"
    
    # Return the updated 'sens' value
    return sens

def switch_axe(axe):
    if(axe == 0):
        axe = 1
    else:
        axe = 0
    return axe

def rotate_robot(angle_degrees, rotation_speed, sens, thymio):
    """
    Rotates the Thymio robot by a specified angle at a given rotation speed and direction.
    The robot calculates the distance each wheel must travel based on the provided angle.
    The motor speeds are adjusted depending on the rotation direction ('Left' or 'Right').
    The Thymio robot is then set to move according to the calculated motor targets, and the function
    waits for the robot to complete the rotation before returning.

    :param angle_degrees: The desired angle of rotation in degrees.
    :param rotation_speed: The speed at which the robot should rotate.
    :param sens: The rotation direction ('Left' or 'Right').
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :return: None
    """
    # convert the angle to radians
    angle_radians = math.radians(angle_degrees)

    # Calculate the distance each wheel must travel
    wheel_distance = 100  # Remplacez par la distance réelle entre les roues de votre robot
    wheel_circumference = 2 * math.pi * (wheel_distance / 2)
    distance_to_travel = (angle_radians / (2 * math.pi)) * wheel_circumference
    # Adjust motor speed
    if (sens == "Right"): 
        motor_speed = rotation_speed
        motor_left_target = motor_speed - 18
        motor_right_target = -motor_speed 
    if (sens == "Left"):
        motor_speed = rotation_speed
        motor_left_target = -motor_speed
        motor_right_target = motor_speed + 3

    # set motor speed
    thymio.set_motors_PID(motor_left_target, motor_right_target)
    # waiting to travel the computer distance
    time.sleep(distance_to_travel / (motor_speed*Speed_Factor))


def forward_robot(motor_speed, distance, thymio):
    """
    Moves the Thymio robot forward at a specified motor speed for a duration of 3.6 seconds.
    The left and right motors are set to the given speed, and the Thymio robot is then instructed
    to move forward. The function pauses for the specified duration to allow the robot to complete
    its forward movement before returning.

    :param motor_speed: The speed at which the robot should move forward.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :return: None
    """
    motor_left_target = motor_speed
    motor_right_target = motor_speed
    thymio.set_motors_PID(motor_left_target, motor_right_target) 
    time_to_travel = distance / (motor_speed*Speed_Factor)

    # Wait for the robot to travel the specified distance
    time.sleep(time_to_travel)

    
def determine_special_path(path, special_cases,axe, special_step):
    """
    Determines a special step based on conditions in the given path and special cases.
    :param path: The path representation, likely a list or data structure.
    :param special_cases: A list of special cases for comparison.
    :param axe: The specific axis to consider in the path.
    :param special_step: The variable to determine the special step, initially provided as an argument.
    :return: The updated value of special_step.
    """
    if path[axe][2] == special_cases[axe][0]:
       special_step= 1
    elif path[axe][2] == special_cases[axe][1]:
       special_step= 2
    return special_step

def navigate_obstacle(special_step, sens, k, thymio,obstacle_coordinates,position,orientation,axe):
    """
    Navigates the Thymio robot around an obstacle based on specified conditions.
    :param special_step: A value indicating a special step condition.
    :param sens: The initial direction of movement ('Right' or 'Left').
    :param k: A counter variable, likely tracking iterations.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :param obstacle_coordinates: Coordinates of the detected obstacle.
    :param position: The current position of the Thymio robot.
    :param orientation: The current orientation of the Thymio robot.
    :param axe: The specific axis to consider in the path.
    :return: Updated counter variable 'k' and the state after navigating the obstacle.
    """
    # Check for special step condition
    if (special_step == 1):
        sens = "Right"
    else :
        sens = "Left"
    
    # Flag to track direction during obstacle navigation
    Left = False

    # Loop through obstacle coordinates
    for point in obstacle_coordinates:
        # Adjust direction depending on the orientation
        if orientation == -90 or  orientation == 0:
            sens = switch_sens(sens)

        # Check if Thymio is next to the obstacle
        if (np.floor(position[axe]) == (point[axe]-1)) and (np.floor(position[switch_axe(axe)]) == (point[switch_axe(axe)])):
            Left = True
            rotate_robot(90, 100, sens,thymio)
            forward_robot(100, Short_distance, thymio)
            rotate_robot(90, 100, switch_sens(sens),thymio)
            forward_robot(100, Long_distance, thymio)

        # Check if Thymio is on the other side of the obstacle
        if (np.floor(position[axe]) == (point[axe]+1)) and (np.floor(position[switch_axe(axe)]) == (point[switch_axe(axe)])):
            Left = False
            rotate_robot(100, 100, switch_sens(sens),thymio)
            forward_robot(100, Short_distance, thymio)
            rotate_robot(90, 100, sens,thymio)
            forward_robot(100, Long_distance, thymio)

    # Check for special step condition
    if special_step == 0:
        if not Left:
            sens = switch_sens(sens)
    
        rotate_robot(90, 100, switch_sens(sens),thymio)
        forward_robot(100, Short_distance, thymio)
        rotate_robot(90, 100, sens,thymio)

        state = "orientation" # Update state

    else:
        k += 1
        state = "orientation" # Update state

    return k, state

def handle_target_left(thymio):
    """
    Handles the left-side target encountered by the Thymio robot.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :return: The updated state after handling the left-side target.
    """
    rotate_robot(90, 100, "Left",thymio)
    forward_robot(100, Short_distance, thymio)
    rotate_robot(90, 100, "Right",thymio)
    forward_robot(100, Short_distance, thymio)

    state = "orientation" # Update state

    return state

def handle_target_right(thymio):
    """
    Handles the right-side target encountered by the Thymio robot.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :return: The updated state after handling the right-side target.
    """
    rotate_robot(90, 100, "Right",thymio)
    forward_robot(100, Short_distance, thymio)
    rotate_robot(90, 100, "Left",thymio)
    forward_robot(100, Short_distance, thymio)

    state = "orientation" # Update state

    return state

def handle_final_orientation(path, k, axe, thymio):
    """
    Handles the final orientation adjustment for the Thymio robot based on the provided path and iteration count.
    :param path: The path representation, likely a list or data structure.
    :param k: The current iteration count.
    :param axe: The specific axis to consider in the path.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :return: The updated state after handling the final orientation.
    """
    # Check if the current iteration aligns with the desired final orientation
    if path[axe][k] == path[axe][1]:
        state = "end"
    # If the iteration count is less than the desired final orientation, rotate the robot left
    elif path[axe][1] > path[axe][k]:
        rotate_robot(90, 100, "Left",thymio)
        state = "end"
    # If the iteration count is greater than the desired final orientation, rotate the robot right
    elif path[axe][1] < path[axe][k]:
        rotate_robot(90, 100, "Right",thymio)
        state = "end"
    
    return state
    
async def local_nav(thymio):
    """
    Performs local navigation for the Thymio robot based on proximity sensor readings.
    Reads proximity sensor values, scales them, and checks for obstacles.
    If an obstacle is detected (based on a threshold), stops the robot's motors.
    :param thymio: An instance of the Thymio robot class providing sensor readings and motor control.
    :return: A boolean indicating whether an obstacle is detected during local navigation.
    """
    global prox_horizontal

    # Initialize obstacle flag
    obstacle = False
    #w_l = [0,  20, -20, -20, 0, 0, 0]
    # w_r = [0, -20, -20,  20, 0, 0,  0]
    # Scale factors for sensors and constant factor
    sensor_scale = 200
    
    # Placeholder variables for sensor readings
    y = [0,0]
    x = [0,0,0,0,0,0,0]

    # Get proximity sensor readings from the Thymio robot
    _,_,prox,= await thymio.get_sensors()
        
    for i in range(len(x)):
        # Get and scale proximity sensor readings
        x[i] = prox[i] // sensor_scale

        # Check if obstacle is detected based on the front proximity sensor
        if (x[2] > Threshold_detection):
            obstacle = True 
            thymio.set_motors_PID(0, 0) # Stop the robot's motors if an obstacle is detected


    return obstacle
    
def obstacle_function(path, position, angle, thymio,obstacle_coordinates):
    """
    Handles obstacle detection and navigation for the Thymio robot.
    Detects the presence of obstacles based on the robot's position and orientation.
    Initiates obstacle navigation strategies, including special cases, and updates the robot's state.
    :param path: The path representation, likely a list or data structure.
    :param position: The current position of the Thymio robot.
    :param angle: The current orientation angle of the Thymio robot.
    :param thymio: An instance of the Thymio robot class providing motor control methods.
    :param obstacle_coordinates: Coordinates of the detected obstacle.
    :return: The status indicating the current step of the path after the obstacle handling process.
    """
    # Initialize variables
    special_step = 0
    status = 0
    orientation = 0

    # Check if Thymio is facing forward or backward
    if ((angle < -170) or (angle > 170) or (-10 < angle < 10)):
        if ((angle < -170) or (angle > 170)): orientation = 180
        else: orientation = 0
        axe = 1
        x_s=[np.floor(position[0])-2,np.floor(position[0])-2]
        y_s=[np.floor(position[1])+1,np.floor(position[1])-1]
    
    # Check if Thymio is facing left or right
    if ((-100 < angle < -80) or (80 < angle < 100)):
        if (-100 < angle < -80): orientation = -90
        else: orientation = 90
        axe = 0
        x_s=[np.floor(position[0])+1,np.floor(position[0])-1]
        y_s=[np.floor(position[1])+2,np.floor(position[1])+2]

    special_cases = [x_s,y_s]
    k = 2
    time.sleep(0.2)
    obstacle_position = path[:,0]

    # Check if Thymio is behind the target
    if (np.floor(position[axe]) == path[axe][1]):
        special_step = determine_special_path(path, special_cases,axe, special_step)
        if(special_step == 1) or (special_step == 2):
            status = 4
        else:
            status = 3
        state = "obstacle_behind"

    # Check if Thymio is to the left of the target
    if (np.floor(position[axe]) > path[axe][1]):
        state = "obstacle_left"
        status = 3

    # Check if Thymio is to the right of the target
    if (np.floor(position[axe]) < path[axe][1]):
        state = "obstacle_right"
        status = 3

    # Handle obstacle navigation based on the detected state
    if state == "obstacle_behind":
        k, state = navigate_obstacle(special_step, "Right", k, thymio,obstacle_coordinates,position,orientation, axe)
    if state == "obstacle_left":
        state = handle_target_left(thymio)
    if state == "obstacle_right":
        state = handle_target_right(thymio)
    if state == "orientation":
        state = handle_final_orientation(path, k,axe, thymio)

    # Check if the obstacle handling process is complete
    if state == "end":
        time.sleep(0.2)
        k = 1
        return status
    
def obstacle_extraction(obs_grid):
    """
    Extracts the coordinates of ones (obstacles) and zeros (free spaces) from an occupancy grid.
    Iterates through each element of the grid, categorizing coordinates based on the presence of obstacles (1) or free spaces (0).
    Returns a list of coordinates representing the locations of obstacles in the grid.
    :param obs_grid: The occupancy grid representing the environment with ones (obstacles) and zeros (free spaces).
    :return: A list of coordinates corresponding to the locations of obstacles in the grid.
    """
    # Initialize lists to store coordinates of ones (obstacles) and zeros (free spaces)
    ones_coordinates = []
    zeros_coordinates = []

    # Iterate through each row and column of the occupancy grid
    for row_index, row in enumerate(obs_grid):
        for col_index, value in enumerate(row):
            # Categorize coordinates based on the presence of obstacles (1) or free spaces (0)
            if value == 1:
                ones_coordinates.append((row_index, col_index))
            elif value == 0:
                zeros_coordinates.append((row_index, col_index))

    # Return a list of coordinates representing the locations of obstacles in the grid
    occupancy_grid = ones_coordinates
    return occupancy_grid
