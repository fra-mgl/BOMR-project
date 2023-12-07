from tdmclient import aw
import numpy as np
from IPython.display import display


# PID Controller parameters
kp = 2.0  # Proportional gain
ki = 0.04  # Integral gain
kd = 0.01  # Derivative gain
robot_width = 0.1  # Width between the two wheels

# Simulation parameters
# dt = 0.1  # Time step


class Control(object):

    def __init__(self,node,client):
        print("init")
        self.node = node
        self.client = client
    

    async def get_sensors(self):
        await self.node.wait_for_variables({"prox.horizontal","motor.left.speed","motor.right.speed"})
        ls = self.node.v.motor.left.speed
        rs = self.node.v.motor.right.speed
        prox_sensor = list(self.node.v.prox.horizontal)
        return ls, rs, prox_sensor
    

    def set_motors_PID(self, left_speed,right_speed):
        motors = {
            "motor.left.target": [round(left_speed)],
            "motor.right.target": [round(right_speed)],
        }
        aw(self.node.set_variables(motors))

    # --------- LEDs AND SOUND --------- #
    def set_LEDs(self, color):
        if color == "red":
            c = [ 255, 0, 0]
        elif color == "blue":
            c = [ 0, 0, 255]
        elif color == "green":
            c = [ 0, 255, 0]
        else:
            c = [ 0, 0, 0]
        leds = {
            "leds.top": c,
            "leds.bottom.right": c,
            "leds.bottom.left": c
        }
        aw(self.node.set_variables(leds))


# --------- PID CONTROLLER --------- #
def pid_controller(integral, angle, previous_error,dt):
    proportional = kp * angle
    integral += ki * angle
    if dt > 0:
        derivative = kd * (angle - previous_error) / (dt)
    else:
        derivative = 0
    correction = proportional + integral + derivative
    # if angle is big enough, correction is more aggressive (meaning we need to perform a turn)
    if abs(angle) > 30:
        if (abs(correction) < 300) :
            updated_left_speed = - np.floor(correction)
            updated_right_speed = np.floor(correction)
        else :
            # cropping speed
            updated_left_speed = -350
            updated_right_speed = 350
        turning = True
    else :
        updated_left_speed = 80 - np.floor(correction/1.3)
        updated_right_speed = 80 + np.floor(correction/1.3)
        turning = False

    return updated_left_speed,updated_right_speed, integral, angle, turning

def normalize_angle(alpha):
    """
    normalizing the angle between -180 and 180
    """
    if alpha > 180:
        alpha -= 360
    elif alpha < -180:
        alpha += 360  
    return alpha
