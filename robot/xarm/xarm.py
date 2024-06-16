from xarm.wrapper import XArmAPI
import numpy as np
import time

from robot.utils import create_transform, transform_to_vec
from robot.xarm.gripper import Gripper

HOME_POS = [241.788635, 16.398544, 459.922913, 20.221915, -88.524443, 162.505498]
END_EFFECTOR_TO_IPHONE = [125,0,95,0,-15,0]
GRIPPER_OPEN = 3100
GRIPPER_CLOSE = 1000

class xArm:
    def __init__(self, xarm_ip):
        self.arm = XArmAPI(xarm_ip)
        self.arm.connect()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        
        print('xArm initialized')
        
        self.wrist_to_iphone = create_transform(END_EFFECTOR_TO_IPHONE)
        self.gripper = Gripper()
        self.gripper.move_to_pos(GRIPPER_OPEN)
        
    
    def home(self):
        # TODO: remove hardcoded value
        self.gripper.move_to_pos(GRIPPER_OPEN)
        
        self.arm.set_position(*HOME_POS, speed=100, mvacc=1000, wait=True)
        
        # TODO: clean up gripper movement code
        self.gripper_has_moved = False
        
    
    def move_to_pose(self, relative_action, gripper):
        print("Time at executing action", time.time())
        code, current_pos = self.arm.get_position(is_radian=False)
        if code == 0:
            relative_action[:3] *= 1000 # convert translation from m to mm
            relative_action[0] *= -1
            relative_action[2] *= -1
            
            relative_action[3:] = np.rad2deg(relative_action[3:]) # convert from radians to degrees
            relative_action[3] *= -1
            relative_action[5] *= -1
            print(relative_action)
            
            base_to_end_effector = create_transform(current_pos)
            iphone_to_action = create_transform(relative_action)
            
            full_transform = base_to_end_effector @ self.wrist_to_iphone @ iphone_to_action @ np.linalg.inv(self.wrist_to_iphone)
            new_pos = transform_to_vec(full_transform)
            
            self.arm.set_position(*new_pos, speed=100, mvacc=1000, wait=True)
            
            if gripper < 0.4 and not self.gripper_has_moved:
                self.gripper.move_to_pos(GRIPPER_CLOSE)
                self.gripper_has_moved = True