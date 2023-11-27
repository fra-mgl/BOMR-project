from typing import Any
from tdmclient import aw
from tdmclient import ClientAsync
import math

class Control_tymio(object):
    def __init__(self,node,client):
        self.node = node
        self.client = client
        self.speed_conversion = 0 #Value to set with calibration (either function or direct value)
        self.tolerance_radius = 0 #Value to determine : how close do we need to be
                                  #to the checkpoint to consider we are at the checkpoint
        self.tolerance_angle = 0  #Value to determine : how close do we need to be
                                  #to the right angle to consider we are in the right direction
        self.kp = 0               #Value to determine : weight for forward speed control S1-15
        self.kangle = 0           #Value to determine : weight for angle control S1-15
        self.client = ClientAsync()
    
    async def sync(self):
        self.node = await self.client.wait_for_node()
        await self.node.lock()
        return
    async def unsync(self):
        await self.node.unlock()
        await self.node.stop()
        
    #Do we need other sensors ?
    #Not quite sure to understand the node.v. part
    async def get_sensors(self):
        aw(self.node.wait_for_variable({
            "motor.right.speed",
            "motor.left.speed",
            "prox.horizontal",
        }))
        return(self.node.v.motor.right.speed*self.speed_conversion,self.node.v.motor.left.speed*self.speed_conversion,list(self.node.v.prox.horizontal))

    async def following_path(self, pos, angle, checkpoint):
        #We want to follow the path if we are not at the checkpoint
        #x = pos and y = checkpoint
        distance = math.sqrt((pos[1] - checkpoint[1])**2 + (pos[0] - checkpoint[0])**2) 
        if distance < self.tolerance_radius:
            #Update the path in the main ?
            return True
        #If we are not at the checkpoint, we need to move in the right direction
        else:
            vector_to_checkpoint = (checkpoint[0] - pos[0], checkpoint[1] - pos[1])
            #Need to calculate the angle between the vector to the checkpoint and the x axis
            angle_to_checkpoint = math.degrees(angle_between(1,0), vector_to_checkpoint)
            distance_to_checkpoint = math.sqrt(vector_to_checkpoint[0]**2 + vector_to_checkpoint[1]**2)
            delta_angle = angle_to_checkpoint - angle #If angle is in radians, need to convert it to degrees
            #Might have to keep the angle between -180 and 180 ?
            #If we are not in the right direction, we need to turn
            if abs(delta_angle) > self.tolerance_angle:
                self.set_motors(self.kangle*delta_angle,"turn")
            else: #No need to turn we can go forward
                #Might need to edit if we want to slowdown when we are close to the checkpoint
                pass
            return False
        
    
    def set_motors(self, correction,state):
        aw(self.node.wait_for_variable({
                    "motor.right.speed",
                    "motor.left.speed",
                    "prox.horizontal",
                    }))
        if state == "turn":
            motors = {
                "motor.left.target": [self.node.v.motor.left.speed - correction],
                "motor.right.target": [self.node.v.motor.right.speed + correction]
            }
        elif state == "forward":
            #Need to edit if we want to slowdown when we are close to the checkpoint
            pass
        self.node.send_set_variables(motors)
    
    def angle_between(v1, v2):
        dot_product = sum((a * b) for a, b in zip(v1, v2))
        magnitude_v1 = math.sqrt(sum(a**2 for a in v1))
        magnitude_v2 = math.sqrt(sum(b**2 for b in v2))
        
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        
        # Ensure the result is within the valid range for arccosine
        cos_theta = max(min(cos_theta, 1.0), -1.0)
        
        # Calculate the angle in radians
        angle_rad = math.acos(cos_theta)
        
        # Convert the angle to degrees
        #angle_deg = math.degrees(angle_rad)
        
        return angle_rad
