class KalmanFilter(object):
    def _init_(self, dim_x=4, dim_z=2, dt=0.1, dim_u=0):
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

    def compute_x_y_speed(self, left_motor_speed, right_motor_speed):
        # Compute the linear speed of the robot
        linear_speed = (left_motor_speed + right_motor_speed) / 2.0

        # Compute the angular speed of the robot
        angular_speed = (left_motor_speed - right_motor_speed) / 2.0

        new_angle = np.arctan2(angular_speed,linear_speed)
        new_direction_vector = np.array([np.cos(new_angle), np.sin(new_angle)])
        x_speed = linear_speed*new_direction_vector[0]
        y_speed = linear_speed*new_direction_vector[1]
        speed =  [x_speed,y_speed]
        return speed,new_angle
         

    def predict(self,left_motor_speed, right_motor_speed, orientation, x_est_prev, P_est_prev):
        speed, angle = self.compute_x_y_speed(left_motor_speed, right_motor_speed)
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

        return estimated_position, estimated_speed, angle, self.x_est, self.P_est