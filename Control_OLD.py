from typing import Any
from tdmclient import aw
from tdmclient import ClientAsync
import math
import numpy as np
from IPython.display import display

class Control(object):

    def __init__(self,node,client):
        print("init")
        self.node = node
        self.client = client
        self.speed_conversion = 0.4322 #Value to set with calibration (either function or direct value)
        self.tolerance_radius = 0.5 #Value to determine : how close do we need to be
                                  #to the checkpoint to consider we are at the checkpoint
        self.tolerance_angle = 45  #Value to determine : how close do we need to be
                                  #to the right angle to consider we are in the right direction
        self.kp = 5               #Value to determine : weight for forward speed control S1-15
        self.kangle =0.6             #Value to determine : weight for angle control S1-15
    
    async def sync(self):
        print("in sync")
        self.node = await self.client.wait_for_node()
        print("out sync")
        await self.node.lock()
        return
    
    async def unsync(self):
        await self.node.unlock()
        await self.node.stop()
        
    #Do we need other sensors ?
    #Not quite sure to understand the node.v. part

    async def get_sensors(self):
        await self.node.wait_for_variables({"prox.horizontal","motor.left.speed","motor.right.speed"})
        # for i in range(10):
        #     #print(list(self.node.v.prox.horizontal))
        #     #print(self.node.v.motor.left.speed)
        #     #print(self.node.v.motor.right.speed)
        #     await self.client.sleep(0.2)
        ls = self.node.v.motor.left.speed
        rs = self.node.v.motor.right.speed
        prox_sensor = list(self.node.v.prox.horizontal)
        return ls, rs, prox_sensor
 
    async def following_path(self, pos, angle, checkpoint):
        #We wasync t to follow the path if we are not at the checkpoint
        #x = pos and y = checkpoint
        distance = math.sqrt((pos[1] - checkpoint[1])**2 + (pos[0] - checkpoint[0])**2) 
        await self.client.sleep(0.2)
        print("following path", distance)
        print("checkpoint: ",checkpoint)
        await self.client.sleep(0.2)
        if distance < self.tolerance_radius:
            print("Inside tolerance radius")
            return True
        #If we are not at the checkpoint, we need to move in the right direction
        else:
            vector_to_checkpoint = (checkpoint[0] - pos[0], checkpoint[1] - pos[1])
            #Need to calculate the angle between the vector to the checkpoint and the x axis
            v_axis = (0,1)
            #angle_to_checkpoint = math.degrees(math.atan2((checkpoint[1]-v_axis[1]),(checkpoint[0]-v_axis[0])))
            angle_to_checkpoint = math.degrees(self.angle_between(v_axis,vector_to_checkpoint))
            print("angle to checkpoints:", angle_to_checkpoint)
            print("vector to checkpoint:", vector_to_checkpoint)
            distance_to_checkpoint = math.sqrt(vector_to_checkpoint[0]**2 + vector_to_checkpoint[1]**2)
            delta_angle = angle_to_checkpoint - angle #If angle is in radians, need to convert it to degrees
            print("delta : ", delta_angle)
            #Might have to keep the angle between -180 and 180 ?
            #If we are not in the right direction, we need to turn
            if abs(delta_angle) > self.tolerance_angle:
                print("turn")
                self.set_motors(self.kangle*delta_angle,"turn")
                await self.client.sleep(2)
                print("awake")
                    
            else:
                print("forward")
                self.set_motors(delta_angle,"forward")
            return False
        
    def set_motors(self, correction,state):
        left_fw = 50 + round(correction)
        right_fw = 50+ round(correction)
        if state == "turn":
            print("Turning")
            motors = {
                "motor.left.target": [round(correction)],
                "motor.right.target": [-round(correction)],
            }
        elif state == "forward":
            print("Going forward")
            motors = {
                "motor.left.target": [left_fw],
                "motor.right.target": [right_fw],
            }
        print("left speed:", left_fw)
        print("righ speed:", right_fw)
        aw(self.node.set_variables(motors))

    def set_motors_PID(self, left_speed,right_speed):
        print("SETTING PID")
        motors = {
            "motor.left.target": [round(left_speed)],
            "motor.right.target": [round(right_speed)],
        }
        aw(self.node.set_variables(motors))

    
    
    def angle_between(self,v1, v2):
        dot_product = sum((a * b) for a, b in zip(v1, v2))
        magnitude_v1 = math.sqrt(sum(a**2 for a in v1))
        magnitude_v2 = math.sqrt(sum(b**2 for b in v2))
        
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        
        # Ensure the result is within the valid range for arccosine
        # cos_theta = max(min(cos_theta, 1.0), -1.0)
        
        # Calculate the angle in radians
        angle_rad = math.acos(cos_theta)
        
        # Convert the angle to degrees
        #angle_deg = math.degrees(angle_rad)
        
        return angle_rad

    def printtext(self, string, variable):
        print(string, variable)
        return 