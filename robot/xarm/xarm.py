from xarm.wrapper import XArmAPI
import numpy as np
import time

from robot.utils import create_transform, transform_to_vec
from robot.xarm.gripper import Gripper

HOME_POS = [241.788635, 16.398544, 459.922913, 20.221915, -88.524443, 162.505498]
HOME_POS2 = [241.606491, 16.850739, 459.888062, -60.617502, -87.021116, -120.940504]
HOME_POS3 = [215.990036, 50.805801, 480.877808, -25.347137, -88.273659, -156.329758]
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
        
        self.positions = []
        
        self.gripper_values = [3100, 2000, 1200, 2000, 1200, 2800, 1200]
        self.count = 0
    
    def follow_path(self, path):
        self.arm.move_arc_lines(path, first_pause_time=0, speed=200, mvacc=1000, wait=True)
        self.arm.set_position(*path[-1], speed=100, mvacc=1000, wait=True)
        
    def home(self):
        # TODO: remove hardcoded value
        self.gripper.move_to_pos(GRIPPER_OPEN)
        
        # self.arm.set_position(*HOME_POS3, speed=100, mvacc=1000, wait=True)
        
        # TODO: clean up gripper movement code
        self.gripper_has_moved = False
        
    def replay_positions(self):
        import pickle as pkl
        with open("positions2.pkl", "rb") as f:
            positions = pkl.load(f)
        print(positions)
        
        path1 = [HOME_POS3] + [list(positions[i][:-1]) for i in range(28)]
        path2 = [list(positions[i][:-1]) for i in range(28, 44)]
        path3 = [list(positions[i][:-1]) for i in range(44, 76)]
        path4 = [list(positions[i][:-1]) for i in range(76, 108)]
        path5 = [list(positions[i][:-1]) for i in range(108, 136)]
        path6 = [list(positions[i][:-1]) for i in range(136, 176)]
        path7 = [list(positions[i][:-1]) for i in range(176, 192)]
        
        self.follow_path(path1)
        self.gripper.move_to_pos(self.gripper_values[1])
        self.follow_path(path2)
        self.gripper.move_to_pos(self.gripper_values[2])
        self.follow_path(path3)
        self.gripper.move_to_pos(self.gripper_values[3])
        self.follow_path(path4)
        self.gripper.move_to_pos(self.gripper_values[4])
        self.follow_path(path5)
        self.gripper.move_to_pos(self.gripper_values[5])
        self.follow_path(path6)
        self.gripper.move_to_pos(self.gripper_values[6])
        self.follow_path(path7)
        
    
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
            
            self.positions.append(np.append(np.array(new_pos), gripper))
            
            self.arm.set_position(*new_pos, speed=100, mvacc=1000, wait=True)
            
            # if gripper < 0.4 and not self.gripper_has_moved:
            #     self.gripper.move_to_pos(GRIPPER_CLOSE)
            #     self.gripper_has_moved = True
            # if self.count in self.gripper_values:
            #     self.gripper.move_to_pos(self.gripper_values[self.count])
            
            self.count += 1