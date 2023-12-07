import numpy as np

class KalmanFilter(object):
    def __init__(self, dim_x=4, dim_z=2, dt=0.08, dim_u=0):
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
        self.H = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  # measurement matrix
        self.R = np.array([[0.25, 0, 0, 0],
                        [0, 0.25, 0, 0],
                        [0, 0, 6, 0],
                        [0, 0, 0, 6]])  # measurement uncertainty
        
        self.x_est = np.zeros((dim_x, 1))  # state estimate
        self.P_est = np.eye(dim_x) * 1000  # state covariance estimate

        self.speed_conv_factor = 0.2  # conversion factor for speed
        self.wheel_width = 50 # distance between the two wheels in mm
        self.time_normalizatin_factor = 5.5
        

    def compute_x_y_speed(self, left_motor_speed, right_motor_speed, angle,time_difference):
        # Compute the linear speed of the robot
        linear_speed = (left_motor_speed + right_motor_speed) * self.speed_conv_factor / 2.0
        angular_speed = -(left_motor_speed - right_motor_speed) * self.speed_conv_factor / (self.wheel_width)
        delta_angle = angular_speed * time_difference
        angle = angle + np.degrees(delta_angle)

        # Compute the direction vector of the robot
        rad_angle = np.radians(angle)
        cosine = np.cos(rad_angle)
        if abs(cosine) < 0.15: cosine = 0
        sine = np.sin(rad_angle)
        if abs(sine) < 0.3: sine = 0

        new_direction_vector = np.array([cosine, sine])

        x_speed = linear_speed*new_direction_vector[0]
        y_speed = linear_speed*new_direction_vector[1] 
        speed =  [x_speed,y_speed]

        angle = np.degrees(rad_angle)
        return speed, new_direction_vector, angle 
         

    def predict(self, x_est_prev, P_est_prev, speed, direction, time_difference):
        dt = time_difference/self.time_normalizatin_factor
        # matrix A is dynamic - compute each time taking into account the time difference between each prediction step
        self.A = np.array([[1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  # state transition matrix
        self.x_est = np.dot(self.A, x_est_prev)
        self.P_est = np.dot(np.dot(self.A, P_est_prev), self.A.T)
        self.P_est = self.P_est + self.Q if type(self.Q) != type(None) else self.P_est

        # Convert orientation to a direction vector
        speed_array = np.array([speed[0], speed[1]])

        # Measurement vector based on speed and orientation
        y = np.dot(self.H, self.x_est)

        # Directly set the relevant elements of y
        y[2] = abs(direction[0]) * speed_array[0] * self.speed_conv_factor // 10
        y[3] = abs(direction[1]) * speed_array[1] * self.speed_conv_factor// 10

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
        estimated_direction = direction.flatten()

        return estimated_position, speed , self.x_est, self.P_est, estimated_direction