import stretch_body.robot
import numpy as np
import PyKDL
import rospy
#from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import math
import time
import random
import os
from .utils import euler_to_quat, urdf_joint_to_kdl_joint, urdf_pose_to_kdl_frame, urdf_inertial_to_kdl_rbi, kdl_tree_from_urdf_model



 

OVERRIDE_STATES = {}

class HelloRobot:

    def __init__(self, urdf_file = 'stretch_nobase_raised.urdf', gripper_threshold = 7.0, stretch_gripper_max = 40, stretch_gripper_min = 0):
        
        self.STRETCH_GRIPPER_MAX = stretch_gripper_max
        self.STRETCH_GRIPPER_MIN = stretch_gripper_min
        self.urdf_file = urdf_file
        
        self.urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', self.urdf_file) 
        self.GRIPPER_THRESHOLD = gripper_threshold

        #Initializing ROS node
        self.joint_list = ["joint_fake","joint_lift","joint_arm_l3","joint_arm_l2","joint_arm_l1" ,"joint_arm_l0","joint_wrist_yaw","joint_wrist_pitch","joint_wrist_roll"]
        try:
            rospy.init_node('hello_robot_node')
        except:
            print('node already initialized')
        
        
        self.robot = stretch_body.robot.Robot()
        self.robot.startup()

        # Initializing the robot base position
        self.base_x = self.robot.base.status['x']
        self.base_y = self.robot.base.status['y']

    
        time.sleep(2)

        # Constraining the robots movement
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

        # Joint dictionary for Kinematics
        self.setup_kdl()
        self.initialize_home_params()

    def move_to_position(self, lift_pos = 0.5, arm_pos = 0.02, base_trans = 0.0, wrist_yaw = 0.0, wrist_pitch = 0.0, wrist_roll = 0.0, gripper_pos = None):

        self.CURRENT_STATE = self.STRETCH_GRIPPER_MAX if gripper_pos is None \
                             else gripper_pos*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN)+self.STRETCH_GRIPPER_MIN

        self.robot.lift.move_to(lift_pos)
        self.robot.end_of_arm.move_to('stretch_gripper',self.CURRENT_STATE)
        self.robot.push_command()
        
        while self.robot.get_status()['arm']['pos']>arm_pos+0.002 or self.robot.get_status()['arm']['pos']<arm_pos-0.002:
            # print(self.robot.get_status()['arm']['pos'])
            self.robot.arm.move_to(arm_pos)
            self.robot.push_command()

        self.robot.end_of_arm.move_to('wrist_yaw', wrist_yaw)
        PITCH_VAL = wrist_pitch
        self.robot.end_of_arm.move_to('wrist_pitch', PITCH_VAL)
        #NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
        OVERRIDE_STATES['wrist_pitch'] = PITCH_VAL  
        self.robot.end_of_arm.move_to('wrist_roll', wrist_roll)
        self.robot.base.translate_by(base_trans)
        print('moving to position 3')
        self.robot.push_command()
        print('moving to position 4')

    def initialize_home_params(self, home_lift = 0.43, home_arm = 0.02, home_base = 0.0, home_wrist_yaw = 0.0, home_wrist_pitch = 0.0, home_wrist_roll = 0.0, home_gripper = 1):
        self.home_lift = home_lift
        self.home_arm = home_arm
        self.home_wrist_yaw = home_wrist_yaw
        self.home_wrist_pitch = home_wrist_pitch
        self.home_wrist_roll = home_wrist_roll
        self.home_gripper = home_gripper
        self.home_base = home_base

    def home(self):
        self.move_to_position(self.home_lift, self.home_arm, self.home_base, self.home_wrist_yaw, self.home_wrist_pitch, self.home_wrist_roll, self.home_gripper)
        

    def setup_kdl(self):
        self.joints = {'joint_fake':0}
        
        robot_model = URDF.from_xml_file(self.urdf_path)
        kdl_tree = kdl_tree_from_urdf_model(robot_model)
        self.arm_chain = kdl_tree.getChain('base_link', 'link_raised_gripper')
        self.joint_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())

        # Forward kinematics
        self.fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self.arm_chain)
        # Inverse Kinematics
        self.ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self.arm_chain)
        self.ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self.arm_chain, self.fk_p_kdl, self.ik_v_kdl) 


    def updateJoints(self):
        #Update the joint state values in 'self.joints' using hellorobot api calls
        # print('x, y:', self.robot.base.status['x'], self.robot.base.status['y'])

        origin_dist = math.sqrt((self.base_y - self.robot.base.status['y'])**2+(self.base_x - self.robot.base.status['x'])**2)

        # print('orig_dist:', origin_dist)
        # far_dist = math.sqrt((self.far_y - self.robot.base.status['y'])**2+(self.far_x - self.robot.base.status['x'])**2)


        ## commented for debugging
        # print('far dist vals:', far_dist, self.far_to_origin)
        # if(far_dist <= self.far_to_origin):
        ## commented for debugging

        
        self.joints['joint_fake'] = origin_dist
        ## commented for debugging
        # else:
        #     self.joints['joints_fake'] = -1*origin_dist
        ## commented for debugging
        
        self.joints['joint_lift'] = self.robot.lift.status['pos']
        
        armPos = self.robot.arm.status['pos']
        self.joints['joint_arm_l3'] = armPos / 4.0
        self.joints['joint_arm_l2'] = armPos / 4.0
        self.joints['joint_arm_l1'] = armPos / 4.0
        self.joints['joint_arm_l0'] = armPos / 4.0
        
        self.joints['joint_wrist_yaw'] = self.robot.end_of_arm.status['wrist_yaw']['pos']
        self.joints['joint_wrist_roll'] = self.robot.end_of_arm.status['wrist_roll']['pos']
        self.joints['joint_wrist_pitch'] = OVERRIDE_STATES.get('wrist_pitch', self.robot.end_of_arm.status['wrist_pitch']['pos'])

        self.joints['joint_stretch_gripper'] = self.robot.end_of_arm.status['stretch_gripper']['pos']  

    # following function is used to move the robot to a desired joint configuration 
    def move_to_joints(self, joints, gripper):
        # update the robot joints to the new values from 'joints'

        ## the commented code adds a wall on the right side of the robot wrt its starting base position
        #joints['joint_fake'] = self.clamp(joints['joint_fake'], 0.0002, 0.20)


        # print('jt_fk:',joints['joint_fake'])
        # self.base_motion += joints['joint_fake']-self.joints['joint_fake']
        # print('base motion:', self.base_motion)

        self.robot.base.translate_by(joints['joint_fake']-self.joints['joint_fake'], 5)
        self.robot.arm.move_to(joints['joint_arm_l3'] + 
                            joints['joint_arm_l2'] + 
                            joints['joint_arm_l1'] + 
                            joints['joint_arm_l0'])
        
        self.robot.lift.move_to(joints['joint_lift'])
        
        #yaw, pitch, roll limits 
        self.robot.end_of_arm.move_to('wrist_yaw', self.clamp(joints['joint_wrist_yaw'], -0.4, 1.7))
        self.robot.end_of_arm.move_to('wrist_pitch', self.clamp(joints['joint_wrist_pitch'], -0.8, 0.2))
        #NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
        OVERRIDE_STATES['wrist_pitch'] = joints['joint_wrist_pitch']
        self.robot.end_of_arm.move_to('wrist_roll', self.clamp(joints['joint_wrist_roll'], -1.53, 1.53))

        # gripper[0] value ranges from 0 to 1, 0 being closed and 1 being open. Below code maps the gripper value to the range of the gripper joint
        self.CURRENT_STATE  = gripper[0]*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN) + self.STRETCH_GRIPPER_MIN

        self.robot.end_of_arm.move_to('stretch_gripper', self.CURRENT_STATE)
        #code below is to map values below certain threshold to negative values to close the gripper much tighter
        if self.CURRENT_STATE<self.GRIPPER_THRESHOLD:
            self.robot.end_of_arm.move_to('stretch_gripper', -25)

  
        self.robot.push_command()

        #sleeping to make sure all the joints are updated correctly (remove if not necessary)
        time.sleep(.7)



    
    def move_to_pose(self, translation_tensor, rotational_tensor, gripper):
        

        translation = [translation_tensor[0], translation_tensor[1], translation_tensor[2]]
        rotation = rotational_tensor
        
        # move logic
        self.updateJoints()

        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]

        curr_pose = PyKDL.Frame()
        del_pose = PyKDL.Frame()
        self.fk_p_kdl.JntToCart(self.joint_array, curr_pose)



        rot_matrix = R.from_euler('xyz', rotation, degrees=False).as_matrix()


#new code from here
        del_rot = PyKDL.Rotation(PyKDL.Vector(rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0]),
                                  PyKDL.Vector(rot_matrix[0][1], rot_matrix[1][1], rot_matrix[2][1]),
                                  PyKDL.Vector(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]))
        del_trans = PyKDL.Vector(translation[0], translation[1], translation[2])
        del_pose.M = del_rot
        del_pose.p = del_trans
        goal_pose_new = curr_pose*del_pose

        seed_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())
        self.ik_p_kdl.CartToJnt(seed_array, goal_pose_new, self.joint_array)

        ik_joints = {}

        for joint_index in range(self.joint_array.rows()):
            # print(joint_index)
            # print(joint_list[joint_index])
            ik_joints[self.joint_list[joint_index]] = self.joint_array[joint_index]


        # print('ik_joints', ik_joints)
        # test_pose = PyKDL.Frame()
        # self.fk_p_kdl.JntToCart(self.joint_array, test_pose)

        # # print(test_pose.p)
        # # print(test_pose.M.GetRPY())

        self.move_to_joints(ik_joints, gripper)

        
        self.robot.push_command()

        self.updateJoints()
        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]
        



