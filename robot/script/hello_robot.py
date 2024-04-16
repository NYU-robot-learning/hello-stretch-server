import stretch_body.robot
import numpy as np
import PyKDL
from pathlib import Path

# from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import math
import time
import random
import os
from ..utils import kdl_tree_from_urdf_model


pick_place = [38.0, 15, 47]  # 15 looks wrong
pouring = [33, 19, 53]

OVERRIDE_STATES = {}


class HelloRobot:
    def __init__(
        self,
        urdf_file="stretch_nobase_raised.urdf",
        gripper_threshold=0.2*80, # unused
        stretch_gripper_max=170,
        stretch_gripper_min=0,
        stretch_gripper_tight=-35,
        sticky_gripper=False,
        gripper_threshold_post_grasp=0.1*60,
    ):
        self.STRETCH_GRIPPER_MAX = stretch_gripper_max
        self.STRETCH_GRIPPER_MIN = stretch_gripper_min
        self.STRETCH_GRIPPER_TIGHT = stretch_gripper_tight
        self._has_gripped = False
        self._sticky_gripper = sticky_gripper
        self.urdf_file = urdf_file
        self.threshold_count = 0

        self.urdf_path = os.path.join(
            str(Path(__file__).resolve().parent.parent / "urdf" / self.urdf_file)
        )
        self.GRIPPER_THRESHOLD = gripper_threshold
        self.GRIPPER_THRESHOLD_POST_GRASP = gripper_threshold_post_grasp

        self.robot = stretch_body.robot.Robot()
        self.startup()

        # Constraining the robots movement
        
        self.set_home_position()
        self.home()

        # wait until it gets to position
        time.sleep(5)

        print("STARTING")

    def startup(self):
        self.robot.startup()

        self.robot.arm.motor.enable_sync_mode()
        self.robot.base.left_wheel.enable_sync_mode()
        self.robot.base.right_wheel.enable_sync_mode()
        self.robot.lift.motor.enable_sync_mode()

    def move_to_position(
        self,
        lift_pos=0.7,
        arm_pos=0.02,
        base_trans=0.0,
        wrist_yaw=0.0,
        wrist_pitch=0.0,
        wrist_roll=0.0,
        gripper_pos=None,
    ):
        self.CURRENT_STATE = (
            self.STRETCH_GRIPPER_MAX
            if gripper_pos is None
            else gripper_pos * (self.STRETCH_GRIPPER_MAX - self.STRETCH_GRIPPER_MIN)
            + self.STRETCH_GRIPPER_MIN
        )

        self.robot.lift.move_to(lift_pos)
        self.robot.end_of_arm.move_to("stretch_gripper", self.CURRENT_STATE)
        self.robot.push_command()

        while (
            self.robot.get_status()["arm"]["pos"] > arm_pos + 0.002
            or self.robot.get_status()["arm"]["pos"] < arm_pos - 0.002
        ):
            # print(self.robot.get_status()['arm']['pos'])
            self.robot.arm.move_to(arm_pos)
            self.robot.push_command()

        self.robot.end_of_arm.move_to("wrist_yaw", wrist_yaw)
        PITCH_VAL = wrist_pitch
        self.robot.end_of_arm.move_to("wrist_pitch", PITCH_VAL)
        # NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
        OVERRIDE_STATES["wrist_pitch"] = PITCH_VAL
        self.robot.end_of_arm.move_to("wrist_roll", wrist_roll)
        self.robot.base.translate_by(base_trans)
        print("moving to position 3")
        self.robot.push_command()
        print("moving to position 4")

    def set_home_position(
        self,
        lift=0.7,
        arm=0.02,
        base=0.0,
        wrist_yaw=0.0,
        wrist_pitch=0.0,
        wrist_roll=0.0,
        gripper=1.0,
    ):
        self.home_lift = lift
        self.home_arm = arm
        self.home_wrist_yaw = wrist_yaw
        self.home_wrist_pitch = wrist_pitch
        self.home_wrist_roll = wrist_roll
        self.home_gripper = gripper
        self.home_base = base

    def home(self):
        self.not_grasped = True
        self._has_gripped = False

        self.robot.push_command()

        self.threshold_count = 0
        self.move_to_position(
            self.home_lift,
            self.home_arm,
            self.home_base,
            self.home_wrist_yaw,
            self.home_wrist_pitch,
            self.home_wrist_roll,
            self.home_gripper,
        )
    
    def get_gripper(self):
        return self.robot.end_of_arm.status["stretch_gripper"]["pos_pct"]
    
    def move_gripper(self, relative_gripper):
        self.robot.end_of_arm.get_joint("stretch_gripper").move_by(relative_gripper)