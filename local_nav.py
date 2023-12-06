import time
import math
import numpy as np

# xi=0
# yi=0
# xt=0
# yt=2
# xnt=-1
# ynt=2
# xnnt=-2
# ynnt=2
# # initial position : position in front of the obstacle 
# position = [xi,yi]
# # coordinates of the next steps in the global path
# xp=[xt,xnt,xnnt]
# yp=[yt,ynt,ynnt]
# # list of coordinates that we obtain from the global navigation
# path = [xp,yp]
# # the 2 special cases where it is more effective to directly go on this case


def switch_sens(sens):
    if(sens == 1):
        sens = 2
    else:
        sens = 1
    return sens


def switch_axe(axe):
    if(axe == 0):
        axe = 1
    else:
        axe = 0
    return axe

def rotate_robot(angle_degrees, rotation_speed, sens, thymio):
    # Convertir l'angle en radians
    angle_radians = math.radians(angle_degrees)

    # Calculer la distance que chaque roue doit parcourir
    wheel_distance = 5  # Remplacez par la distance réelle entre les roues de votre robot
    wheel_circumference = 2 * math.pi * (wheel_distance / 2)
    distance_to_travel = (angle_radians / (2 * math.pi)) * wheel_circumference
    # Ajuster la vitesse des moteurs pour faire tourner le robot
    if (sens == 1): 
        motor_speed = rotation_speed
        motor_left_target = motor_speed - 5
        motor_right_target = -motor_speed 
    if (sens == 2):
        motor_speed = rotation_speed
        motor_left_target = -motor_speed
        motor_right_target = motor_speed + 3
    # Inverser la vitesse du moteur droit pour tourner

    # Définir les variables des moteurs
    thymio.set_motors_PID(motor_left_target, motor_right_target)
    time.sleep(2)
    # Attendez que le robot ait tourné la distance souhaitée
    time.sleep(distance_to_travel / motor_speed)

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
    
def handle_obstacle_behind(path, special_cases,axe, target):
    #for j in range(len(path)):
    print('case ', path[axe][2])
    print('special case', special_cases[axe][0])
    print('2special case', special_cases[axe][1])
    if path[axe][2] == special_cases[axe][0]:
        target = 1
    elif path[axe][2] == special_cases[axe][1]:
        target = 2
    return target

def handle_obstacle_left(position, obstacle_position):
    print('je suis dans handle left')
    return position[0] > obstacle_position[0]

def handle_obstacle_right(position, obstacle_position):
    return position[0] < obstacle_position[0]

def navigate_obstacle(target, sens, k, thymio,obstacle_cooredinates,position,orientation,axe):
    print(f"target behind obstacle, obstacle {'à droite' if target == 1 else 'à gauche'}")
    if (target == 1):
        print("obstacle à droite")
        sens = 1
    else :
        print("obstacle à gauche") 
        sens = 2
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
    if target == 0:
        print('left', Left)
        if not Left:
            sens = switch_sens(sens)
        print("path est bien derrière")
        rotate_robot(90, 100, switch_sens(sens),thymio)
        forward_robot(100,thymio)
        rotate_robot(90, 100, sens,thymio)
        state = "orientation"
    else:
        k += 1
        state = "orientation"
    return k, state

def handle_target_left(thymio):
    print("target à gauche")
    rotate_robot(90, 100, 2,thymio)
    forward_robot(100,thymio)
    rotate_robot(90, 100, 1,thymio)
    forward_robot(100,thymio)
    state = "orientation"
    return state

def handle_target_right(thymio):
    print("target à droite")
    rotate_robot(90, 100, 1,thymio)
    print("j'ai tourné")
    forward_robot(100,thymio)
    print("j'ai avancé")
    rotate_robot(90, 100, 2,thymio)
    print("j'ai tourné")
    forward_robot(100,thymio)
    print("c'est fini")
    state = "orientation"
    return state

def handle_target_state(path, k, axe, thymio):
    print('k', k)
    if path[axe][k] == path[axe][1]:
        print("je fais rien : ", path[axe][k], " PATH 0 0 : ", path[axe][1])
        state = "end"
    elif path[axe][1] > path[axe][k]:
        print("je tourne à droite", path[axe][1], "PATH 0", path[axe][k])
        rotate_robot(90, 100, 2,thymio)
        state = "end"
    elif path[axe][1] < path[axe][k]:
        print("je tourne à gauche", path[axe][1], "PATH 0", path[axe][k])
        print("je dois tourner")
        rotate_robot(90, 100, 1,thymio)
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
    target = 0
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
    print('obstACLE', obstacle_position)
    print('position', position[axe])
    print('next_step', path)
    if (np.floor(position[axe]) == path[axe][1]):
        target = handle_obstacle_behind(path, special_cases,axe, target)
        state = "obstacle_behind"
        if(target == 1) or (target == 2):
            status = 4
        else:
            status = 3
    if (np.floor(position[axe]) > path[axe][1]):
        print("obstacle position ", obstacle_position)
        print('position', position)
        handle_obstacle_left(position, obstacle_position)
        state = "obstacle_left"
        status = 3
    if (np.floor(position[axe]) < path[axe][1]):
        handle_obstacle_right(position, obstacle_position)
        state = "obstacle_right"
        status = 3
    if state == "obstacle_behind":
        k, state = navigate_obstacle(target, 1, k, thymio,obstacle_cooredinates,position,orientation, axe)
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
