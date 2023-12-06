import numpy as np
import matplotlib.pyplot as plt
import math

# PID Controller parameters
kp = 4.0  # Proportional gain
ki = 0.04  # Integral gain
kd = 0.01  # Derivative gain
robot_width = 0.1  # Width between the two wheels

# Simulation parameters
dt = 0.1  # Time step
# total_time = 10.0  # Total simulation time

# # Generate L-shaped path
# def generate_l_path():
#     path = []
#     path.append((0, 0))

#     # Move forward
#     path.append((2, 0))
#     path.append((2, 4))

#     # Turn right
#     path.append((4, 4))
#     path.append((4, 8))

#     # Move forward
#     path.append((8, 8))

#     return path

# # Simulate movements of the robot based on motor speeds and orientation
# def simulate_movement(position, angle, left_speed, right_speed, dt):
#     # Assuming the robot moves in a straight line between time steps
#     angle += (right_speed - left_speed) * dt / 2.0 # approximate angular velocity
#     dxy = (left_speed + right_speed) / 2.0 * dt # average velocity in x and y directions
#     dx = dxy * math.cos(angle)
#     dy = dxy * math.sin(angle)
#     position[0] += dx
#     position[1] += dy
#     return position, angle

# # Fake PID Controller
# def fake_pid_controller(current_error, previous_error, integral):
#     proportional = kp * current_error
#     integral += ki * current_error
#     derivative = kd * (current_error - previous_error)

#     correction = proportional + integral + derivative
#     return correction, integral, previous_error



# PID Controller for Thymio testing 
# INUT : 
#   target_point 
#   position
#   integral
#   angle
#   previous_error
#   left_speed
#   right_speed
# OUTPU :
#   Left motor Speed
#   RIght motor Speed
#   Error Integral
#   The computed angular error


async def pid_controller(integral, angle, previous_error):
    proportional = kp * angle/2
    integral += ki * angle
    derivative = kd * (angle - previous_error) / dt
    correction = proportional + integral + derivative
    if abs(angle) > 30:
        if (abs(correction) < 300) :
            updated_left_speed = - np.floor(correction)
            updated_right_speed = np.floor(correction)
        else :
            updated_left_speed = -350
            updated_right_speed = 350
        turning = True
    else :
        updated_left_speed = 80 - np.floor(correction/1.3)
        updated_right_speed = 80 + np.floor(correction/1.3)
        turning = False

    return updated_left_speed,updated_right_speed, integral, angle, turning

def normalize_angle(alpha):
    if alpha > 180:
        alpha -= 360
    elif alpha < -180:
        alpha += 360  
    return alpha

# Follow the path using PID control with correction
def follow_path_with_correction(path):  # corresponds to main
    position = np.array([0.0, 0.0]) # initial position
    angle = 0.0
    integral = 0.0
    previous_error = 0.0

    plt.plot(*zip(*path), 'g-', label='Path to Follow')

    # Find the nearest point on the path
    nearest_point_idx = np.argmin(np.sum((position - np.array(path))**2, axis=1))
    if nearest_point_idx != 0:
        target_point = np.array(path[nearest_point_idx]) 
    else :
        target_point = np.array(path[1]) 

    while nearest_point_idx < len(path):
        # If the robot is very close to the target point, move to the next point in the path
        print( " Norm to point is : ", np.linalg.norm(position - target_point))
        if np.linalg.norm(position - target_point) < 0.2:
            nearest_point_idx = (nearest_point_idx + 1)
            target_point = np.array(path[nearest_point_idx])
            print( " TARGET IS REACHED \n")
            print(" New target is : ", target_point)

        #angle_error = math.atan2(target_point[1] - position[1], target_point[0] - position[0]) - angle

        # PID control to adjust motor speeds
        integral, previous_error = pid_controller(target_point, position, integral, angle, previous_error, left_speed, right_speed)

        # PID control to adjust motor speeds For fake testing 
        #correction, integral, previous_error = fake_pid_controller(angle_error, previous_error, integral)

        # Adjust left and right motor speeds for fake simulation
        #left_speed = 3.0 - correction
        #right_speed = 3.0 + correction

        # # Adjust left and right motor speeds for Thymio testing 
        # left_speed = updated_left_speed
        # right_speed = updated_right_speed

        # Simulate robot movement
        # position, angle = simulate_movement(position, angle, left_speed, right_speed, dt)
        # print( " Now at : ", position )
        # print( " And target is : ", target_point)
        # # Visualization (optional)
        # plt.plot(position[0], position[1], 'bo')
        # plt.pause(0.1)

    plt.legend()
    plt.show()
