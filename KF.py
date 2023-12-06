import numpy as np

class KalmanFilter(object):
    def __init__(self, dim_x=4, dim_z=2, dt=0.05, dim_u=0):
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
        self.speed_conv_factor = 0.2  # conversion factor for speed
        

    def compute_x_y_speed(self, left_motor_speed, right_motor_speed, angle,time_difference):
        # Compute the linear speed of the robot
        delta = abs(left_motor_speed - right_motor_speed)
        if delta < 5:
            print("-------------Delta found-------------")
            left_motor_speed = right_motor_speed
        robot_wheel_width = 5
        linear_speed = (left_motor_speed + right_motor_speed) / 2.0
        angular_speed = (left_motor_speed - right_motor_speed) / robot_wheel_width
        delta_angle = angular_speed * time_difference
        angle = angle + delta_angle
        new_direction_vector = np.array([np.cos(round(np.radians(angle))), np.sin(round(np.radians(angle)))])

        x_speed = linear_speed*new_direction_vector[0] * self.speed_conv_factor
        y_speed = linear_speed*new_direction_vector[1] * self.speed_conv_factor
        speed =  [x_speed,y_speed]

        return speed,new_direction_vector,angle 
         

    def predict(self, x_est_prev, P_est_prev, time_diff, speed, direction):
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
        speed_array = np.array([speed[0], speed[1]])

        # Measurement vector based on speed and orientation
        # Measurement vector based on speed and orientation
        y = np.dot(self.H, self.x_est)

        # Directly set the relevant elements of y
        y[2] = abs(direction[0]) * speed_array[0] * self.speed_conv_factor // 10
        y[3] = abs(direction[1]) * speed_array[1] * self.speed_conv_factor// 10

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

        return estimated_position, speed , self.x_est, self.P_est,estimated_direction