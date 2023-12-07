import numpy as np
import random
import matplotlib.pyplot as plt
import math


class KalmanFilter(object):
    def __init__(self, dim_x, dim_z, dt=0.1, dim_u=0,):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.A = np.array([[1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  # state transition matrix
        self.Q = np.array([[0.04, 0, 0, 0],
                        [0, 0.04, 0, 0],
                        [0, 0, 6, 0],
                        [0, 0, 0, 6]])  # process uncertainty
        self.H = np.zeros((dim_z, dim_x))  # measurement matrix
        self.R = np.eye(dim_z)  # measurement uncertainty

        self.x_est = np.zeros((dim_x, 1))  # state estimate
        self.P_est = np.eye(dim_x) * 1000  # state covariance estimate
        self.speed_conv_factor = 0.42  # conversion factor for speed

    def predict(self, speed, orientation, x_est_prev, P_est_prev):
        # Prediction through the a priori estimate
        self.x_est = np.dot(self.A, x_est_prev)
        self.P_est = np.dot(np.dot(self.A, P_est_prev), self.A.T)
        self.P_est = self.P_est + self.Q if type(self.Q) != type(None) else self.P_est

        # Update based on the measured speed and orientation
        self.H = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  # Update measurement matrix
        self.R = np.array([[0.25, 0, 0, 0],
                        [0, 0.25, 0, 0],
                        [0, 0, 6, 0],
                        [0, 0, 0, 6]])  # Update measurement uncertainty

        # Convert orientation to a direction vector
        direction = np.array([orientation[0][0],orientation[0][1]])
        speed_array = np.array([speed[0], speed[1]])

        # Measurement vector based on speed and orientation
        # Measurement vector based on speed and orientation
        y = np.dot(self.H, self.x_est)

        # Directly set the relevant elements of y
        y[2] = abs(direction[0]) * speed_array[0] * self.speed_conv_factor
        y[3] = abs(direction[1]) * speed_array[1] * self.speed_conv_factor

        # print("direction : \n", direction)
        # print("Speed Array : \n", speed_array.T)
        # print(" Y VECTOR : \n", y)
        # Innovation / measurement residual
        i = y - np.dot(self.H, self.x_est)

        # Measurement prediction covariance
        S = np.dot(self.H, np.dot(self.P_est, self.H.T)) + self.R

        # Kalman gain
        K = np.dot(self.P_est, np.dot(self.H.T, np.linalg.inv(S)))

        # A posteriori estimate
        self.x_est = self.x_est + np.dot(K, i)
        self.P_est = self.P_est - np.dot(K, np.dot(self.H, self.P_est))

        # Extracting relevant information for the output
        estimated_position = self.x_est[:2].flatten()
        estimated_speed = np.linalg.norm(self.x_est[2:])
        estimated_direction = direction.flatten()

        # print("The new X_Prev is : \n", self.x_est)

        return estimated_position, estimated_speed, estimated_direction, self.x_est, self.P_est


def generate_fake_positions(movement_path, position_noise=0.05):
    fake_positions = [movement_path[0]]

    for i in range(1, len(movement_path)):
        steps_forward = int(movement_path[i][0] - movement_path[i - 1][0])
        steps_left = int(movement_path[i][1] - movement_path[i - 1][1])

        # Move forward
        for _ in range(abs(steps_forward)):
            noise = np.random.normal(0, position_noise, 2)
            fake_positions.append([fake_positions[-1][0] + np.sign(steps_forward) * (0.6 + noise[0]), 
                                fake_positions[-1][1] + noise[1]])

        # Move left
        for _ in range(abs(steps_left)):
            noise = np.random.normal(0, position_noise, 2)
            fake_positions.append([fake_positions[-1][0] + noise[0], 
                                fake_positions[-1][1] - np.sign(steps_left) * (0.6 + noise[1])])

    return fake_positions


def generate_fake_speeds_and_orientations(fake_positions, dt, speed_range=(0, 80), noise_std=0.01, orientation_noise=0.05):
    fake_speeds = []

    for i in range(1, len(fake_positions)):
        delta_x = 2.6 * (fake_positions[i][0] - fake_positions[i - 1][0])
        delta_y = 2.6 * (fake_positions[i][1] - fake_positions[i - 1][1])

        # Calculate speed (speed = distance / time)
        speed_x = delta_x / dt
        speed_y = delta_y / dt

        # Add noise to simulate uncertainties
        speed_x += random.gauss(0, noise_std)
        speed_y += random.gauss(0, noise_std)

        # Determine orientation based on movement direction
        orientation = np.array([delta_x, delta_y])
        orientation /= np.linalg.norm(orientation)

        # Add noise to orientation
        orientation_noise_vector = np.random.normal(0, orientation_noise, 2)
        orientation += orientation_noise_vector
        orientation /= np.linalg.norm(orientation)

        fake_speeds.append([speed_x, speed_y, orientation.tolist()])

    return fake_speeds


# Example of generating a movement path (straight line from [0, 0] to [8, 0])
movement_path = [[0, 0], [5, 0], [5, 10], [10, 10], [15, 10], [20, 10],[20, 15], [20, 20], [30, 20]]

# Example of using the Kalman filter with fake positions, speeds, and orientations
tracker = KalmanFilter(dim_x=4, dim_z=2)
dt = 0.1
qp = 0.04
rp = 0.25 
q_nu = 5
r_nu = 5

fake_positions = generate_fake_positions(movement_path)
fake_speeds = generate_fake_speeds_and_orientations(fake_positions, dt)
# print("Fake positions : \n")
# print(fake_positions)
# print("Fake speeds : \n")
# print(fake_speeds)

initial_condition = [np.array([[0], [0], [0], [0]])]
initial_orientation = np.array([[1], [0]]) 
initial_covariance = [np.ones(4) * 1000]

# Lists to store the estimated positions
x_est = initial_condition
P_est = initial_covariance
estimated_positions = []


def generate_fake_motor_speeds(num_steps, straight_prob=0.7, max_speed=80): 
    motor_speeds = []

    for _ in range(num_steps):
        if random.uniform(0, 1) < straight_prob:
            # Generate similar speeds for straight line movement
            speed = random.uniform(0, max_speed)
            motor_speeds.append((speed, speed))
        else:
            # Generate varying speeds for turning
            left_speed = random.uniform(0, max_speed)
            right_speed = random.uniform(0, max_speed)
            motor_speeds.append((left_speed, right_speed))

    return motor_speeds


def compute_robot_direction(left_wheel_speed, right_wheel_speed, current_direction):
    # Convert the current direction vector to polar coordinates
    current_angle = np.arctan2(current_direction[1], current_direction[0])

    # Compute the linear speed of the robot
    linear_speed = (left_wheel_speed + right_wheel_speed) / 2.0

    # Compute the angular speed of the robot
    angular_speed = (right_wheel_speed - left_wheel_speed) / 2.0

    new_angle = np.arctan2(angular_speed,linear_speed)
    # Update the direction of the robot based on the current angle and angular speed
    #new_angle = current_angle + angular_speed

    # Compute the new direction vector
    new_direction_vector = np.array([np.cos(new_angle), np.sin(new_angle)])

    return new_direction_vector

# Example usage:
num_steps = 50
fake_motor_speeds = generate_fake_motor_speeds(num_steps)

orientations = [[1,0]]
for left_speed, right_speed in fake_motor_speeds:
    final_direction = compute_robot_direction(left_speed, right_speed,current_direction=orientations[-1])
    orientations.append(final_direction)
    print(f"Left Speed = {left_speed}, Right Speed = {right_speed}, Orientation = {final_direction}")

print(np.arctan2(1,0))

# Perform prediction and update for each time step (assuming constant speed and orientation)
for i in range(len(fake_speeds)):
    speed_i = fake_speeds[i][:2]
    orientation_i = fake_speeds[i][2:]
    estimated_position, _, _, new_x_est, new_P_est = tracker.predict(speed=speed_i, orientation=orientation_i, x_est_prev=x_est[-1], P_est_prev=P_est[-1])
    x_est.append(new_x_est)
    P_est.append(new_P_est)
    estimated_positions.append(estimated_position)

# Convert lists to NumPy arrays for easier plotting
fake_positions = np.array(fake_positions)
estimated_positions = np.array(estimated_positions)

#Plot the original movement path and the estimated positions
plt.plot(fake_positions[:, 0], fake_positions[:, 1], label='Original Path', marker='o')
plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], label='Estimated Positions', marker='x')
plt.legend()
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Original Path vs. Estimated Positions')
plt.grid(True)
plt.show()