import time
import math
import numpy as np

Rotate_Factor = 0.35

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
    if (sens == "Left"): 
        motor_speed = rotation_speed
        motor_left_target = motor_speed - 5
        motor_right_target = -motor_speed 
    if (sens == "Right"):
        motor_speed = rotation_speed
        motor_left_target = -motor_speed
        motor_right_target = motor_speed + 3

    # set motor speed
    thymio.set_motors_PID(motor_left_target, motor_right_target)
    # waiting to travel the computer distance
    time.sleep(distance_to_travel / (motor_speed*Rotate_Factor))

# function to make the robot go forward
def forward_robot(motor_speed,thymio):
    motor_left_target = motor_speed
    motor_right_target = motor_speed
    thymio.set_motors_PID(motor_left_target, motor_right_target) 
    time.sleep(3.6)

# function that define the speed at the beginning 
def beginning_robot(motor_speed,thymio):
    motor_left_target = motor_speed
    motor_right_target = motor_speed
    thymio.set_motors_PID(motor_left_target,motor_right_target)
    
def handle_obstacle_behind(path, special_cases,axe, special_step):
    if path[axe][2] == special_cases[axe][0]:
       special_step= 1
    elif path[axe][2] == special_cases[axe][1]:
       special_step= 2
    return target

def handle_obstacle_left(position, obstacle_position):
    return position[0] > obstacle_position[0]

def handle_obstacle_right(position, obstacle_position):
    return position[0] < obstacle_position[0]

def navigate_obstacle(special_step, sens, k, thymio,obstacle_cooredinates,position,orientation,axe):
    if (special_step == 1):
        sens = "Left"
    else :
        sens = "Right"
    for point in obstacle_cooredinates:
        if orientation == -90 or  orientation == 0:
            sens = switch_sens(sens)
        if (np.floor(position[axe]) == (point[axe]-1)) and (np.floor(position[switch_axe(axe)]) == (point[switch_axe(axe)])):
            Left = True
            rotate_robot(90, 100, sens,thymio)
            forward_robot(100,thymio)
            rotate_robot(90, 100, switch_sens(sens),thymio)
            forward_robot(100,thymio)
            time.sleep(4.5)
        if (np.floor(position[axe]) == (point[axe]+1)) and (np.floor(position[switch_axe(axe)]) == (point[switch_axe(axe)])):
            Left = False
            rotate_robot(00, 100, switch_sens(sens),thymio)
            forward_robot(100,thymio)
            rotate_robot(90, 100, sens,thymio)
            forward_robot(100,thymio)
            time.sleep(4.5)
    if special_step == 0:
        if not Left:
            sens = switch_sens(sens)
        rotate_robot(90, 100, switch_sens(sens),thymio)
        forward_robot(100,thymio)
        rotate_robot(90, 100, sens,thymio)
        state = "orientation"
    else:
        k += 1
        state = "orientation"
    return k, state

def handle_target_left(thymio):
    rotate_robot(90, 100, "Right",thymio)
    forward_robot(100,thymio)
    rotate_robot(90, 100, "Left",thymio)
    forward_robot(100,thymio)
    state = "orientation"
    return state

def handle_target_right(thymio):
    rotate_robot(90, 100, "Left",thymio)
    forward_robot(100,thymio)
    rotate_robot(90, 100, "Right",thymio)
    forward_robot(100,thymio)
    state = "orientation"
    return state

def handle_target_state(path, k, axe, thymio):
    if path[axe][k] == path[axe][1]:
        state = "end"
    elif path[axe][1] > path[axe][k]:
        rotate_robot(90, 100, "Right",thymio)
        state = "end"
    elif path[axe][1] < path[axe][k]:
        rotate_robot(90, 100, "Left",thymio)
        state = "end"
    return state
    
async def local_nav(thymio):
    global prox_horizontal
    obstacle = False
    w_l = [0,  20, -20, -20, 0, 0, 0]
    w_r = [0, -20, -20,  20, 0, 0,  0]
    # Scale factors for sensors and constant factor
    sensor_scale = 200
    
    
    y = [0,0]
    x = [0,0,0,0,0,0,0]
    _,_,proxi,= await thymio.get_sensors()
        
    for i in range(len(x)):
        # Get and scale inputs
        x[i] = proxi[i] // sensor_scale
        if (x[2] > 12):
            obstacle = True 
    return obstacle
    
def obstacle_function(path, position, angle, thymio,obstacle_cooredinates):
    special_step = 0
    status = 0
    orientation = 0
    if ((angle < -170) or (angle > 170) or (-10 < angle < 10)):
        if ((angle < -170) or (angle > 170)): orientation = 180
        else: orientation = 0
        axe = 1
        x_s=[np.floor(position[0])-2,np.floor(position[0])-2]
        y_s=[np.floor(position[1])+1,np.floor(position[1])-1]
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
    if (np.floor(position[axe]) == path[axe][1]):
        special_step = handle_obstacle_behind(path, special_cases,axe, special_step)
        state = "obstacle_behind"
        if(special_step == 1) or (special_step == 2):
            status = 4
        else:
            status = 3
    if (np.floor(position[axe]) > path[axe][1]):
        handle_obstacle_left(position, obstacle_position)
        state = "obstacle_left"
        status = 3
    if (np.floor(position[axe]) < path[axe][1]):
        handle_obstacle_right(position, obstacle_position)
        state = "obstacle_right"
        status = 3
    if state == "obstacle_behind":
        k, state = navigate_obstacle(special_step, "Left", k, thymio,obstacle_cooredinates,position,orientation, axe)
    if state == "obstacle_left":
        state = handle_target_left(thymio)
    if state == "obstacle_right":
        state = handle_target_right(thymio)
    if state == "orientation":
        state = handle_target_state(path, k,axe, thymio)
    if state == "end":
        time.sleep(0.2)
        k = 1
        return status
    
def obstacle_extraction(obs_grid):
    ones_coordinates = []
    zeros_coordinates = []

    for row_index, row in enumerate(obs_grid):
        for col_index, value in enumerate(row):
            if value == 1:
                ones_coordinates.append((row_index, col_index))
            elif value == 0:
                zeros_coordinates.append((row_index, col_index))

    # Return a dictionary containing the coordinates for ones and zeros
    occupancy_grid = ones_coordinates
    return occupancy_grid
