import logging
import os
import pickle
import threading
import time

import numpy as np
import PyKDL
import stretch_body.robot
from scipy.spatial.transform import Rotation as R
from stretch_body.dynamixel_XL430 import DynamixelCommError
from urdf_parser_py.urdf import URDF

from .utils import (euler_to_quat, kdl_tree_from_urdf_model,
                    urdf_inertial_to_kdl_rbi, urdf_joint_to_kdl_joint,
                    urdf_pose_to_kdl_frame)

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
lerp = lambda x, a, b: x * (b - a) + a  # linear interp between a and b


class HelloRobot:
    def __init__(
        self,
        urdf_file="stretch_nobase_raised.urdf",
    ):
        self.robot = stretch_body.robot.Robot()
        self.GRIPPER_MAX = 40.0
        self.GRIPPER_MIN = 0.0
        self.GRIPPER_THRESHOLD = 7.0
        self.GRIPPER_TIGHT = -25.0
        self.VELOCITY_DEADZONE = 0.002

        self.urdf_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "urdf", urdf_file
        )
        self.raw_wrist_motors = {}
        for key, motor in self.robot.end_of_arm.motors.items():
            self.raw_wrist_motors[key] = motor.motor

        self.controller_thread = None
        self.stop_event = threading.Event()
        self.target_joints = {"joint_gripper_cmd": 1.0}
        self.states_record = []

        self.ik_joint_list = [
            "joint_fake",
            "joint_lift",
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_arm_l0",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]
        self.joint_limits = {
            "joint_wrist_yaw": (-0.2, 1.7),
            "joint_wrist_pitch": (-0.8, 0.2),
            "joint_wrist_roll": (-1.53, 1.53),
            "joint_arm": (0.0, 1.0),
            "joint_lift": (0.0, 99.0),
        }
        self.velocity_limits = {
            "joint_lift": (-0.1, 0.1),
            "joint_arm": (-0.2, 0.2),
            "joint_base": (-0.1, 0.1),
        }
        self.dynamixel_joints = [
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_gripper_cmd",
        ]
        self.joint_names = [
            *self.dynamixel_joints,
            "joint_base",
            "joint_lift",
            "joint_arm",
        ]
        # Joint dictionary for Kinematics
        # TODO: change gripper to binary control?
        self.state = {"joint_gripper_cmd": 1.0}
        self.started_up = False
        self._update_state()
        self.setup_kdl()

        self.set_home_position()
        self._update_state()

    def startup(self, home=False):
        if self.started_up:
            return
        self.robot.startup()
        for key, motor in self.robot.end_of_arm.motors.items():
            motor.stop()  # control them manually ourselves
        for key, raw_motor in self.raw_wrist_motors.items():
            raw_motor.startup()

        self.robot.arm.motor.disable_sync_mode()
        self.robot.base.left_wheel.disable_sync_mode()
        self.robot.base.right_wheel.disable_sync_mode()
        self.robot.lift.motor.disable_sync_mode()
        self.reset_base()
        self.started_up = True
        if home:
            self.home()

    def stop(self):
        self.robot.stop()
        for key, raw_motor in self.raw_wrist_motors.items():
            raw_motor.stop()
        self.started_up = False

    def set_home_position(
        self,
        lift=0.5,
        arm=0.1,
        base=0.0,
        wrist_yaw=0.0,
        wrist_pitch=0.0,
        wrist_roll=0.0,
        gripper=1.0,
    ):
        self.home_position = {
            "joint_lift": lift,
            "joint_arm": arm,
            "joint_base": base,
            "joint_wrist_yaw": wrist_yaw,
            "joint_wrist_pitch": wrist_pitch,
            "joint_wrist_roll": wrist_roll,
            "joint_gripper_cmd": gripper,
        }

    def home(self):
        self.move_to_joints(self.home_position)

    def get_state(self):
        self._update_state()
        return self.state.copy()

    def get_joints(self, ik=False):
        result = {k: v for k, v in self.get_state().items() if k in self.joint_names}
        if ik:
            result["joint_arm_l0"] = result["joint_arm"] / 4.0
            result["joint_arm_l1"] = result["joint_arm"] / 4.0
            result["joint_arm_l2"] = result["joint_arm"] / 4.0
            result["joint_arm_l3"] = result["joint_arm"] / 4.0
            result["joint_fake"] = result["joint_base"]
            # TODO: double check if these are needed
            del result["joint_arm"]
            del result["joint_base"]
        return result

    def setup_kdl(self):
        robot_model = URDF.from_xml_file(self.urdf_path)
        kdl_tree = kdl_tree_from_urdf_model(robot_model)
        self.arm_chain = kdl_tree.getChain("base_link", "link_raised_gripper")
        self.joint_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())

        # Forward kinematics
        self.fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self.arm_chain)
        # Inverse Kinematics
        self.ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self.arm_chain)
        self.ik_p_kdl = PyKDL.ChainIkSolverPos_NR(
            self.arm_chain, self.fk_p_kdl, self.ik_v_kdl
        )

    def _wrist_move_to_pos(self, motor_name, pos) -> bool:
        motor = self.robot.end_of_arm.motors[motor_name]
        raw_motor = self.raw_wrist_motors[motor_name]
        if motor_name == "stretch_gripper":
            rad = motor.pct_to_world_rad(pos)
        else:
            rad = pos
        ticks = motor.world_rad_to_ticks(rad)
        ticks = clamp(ticks, *motor.params["range_t"])
        success = False
        n_retry = 2

        for i in range(n_retry):
            try:
                raw_motor.go_to_pos(ticks)
                success = True
                break
            except:
                print(f"Failed to move {motor_name} to {pos}")
        return success

    def _get_wrist_pos(self, motor_name):
        n_retry = 2
        for i in range(n_retry):
            try:
                ticks = self.raw_wrist_motors[motor_name].get_pos()
                break
            except DynamixelCommError:
                print("Failed to commmunicate with Dynamixel motor")

        pos = self.robot.end_of_arm.motors[motor_name].ticks_to_world_rad(ticks)
        if motor_name == "stretch_gripper":
            pos = self.robot.end_of_arm.motors[motor_name].world_rad_to_pct(pos)
        return pos

    def _update_state(self):
        # Update the joint state values in 'self.joints' using hellorobot api calls
        self.state["joint_base"] = self.robot.base.status["x"]
        self.state["joint_lift"] = self.robot.lift.status["pos"]
        self.state["joint_arm"] = self.robot.arm.status["pos"]
        self.state["joint_wrist_yaw"] = self._get_wrist_pos("wrist_yaw")
        self.state["joint_wrist_roll"] = self._get_wrist_pos("wrist_roll")
        self.state["joint_wrist_pitch"] = self._get_wrist_pos("wrist_pitch")
        self.state["joint_wrist_gripper"] = self._get_wrist_pos("stretch_gripper")
        self.state["timestamp"] = time.time()

    def _gripper_cmd_to_pos(self, cmd):
        # map 0-1 command to actual gripper position
        gripper_pos = lerp(
            cmd,
            self.GRIPPER_MIN,
            self.GRIPPER_MAX,
        )
        if gripper_pos < self.GRIPPER_THRESHOLD:
            gripper_pos = self.GRIPPER_TIGHT
        return gripper_pos

    # following function is used to move the robot to a desired joint configuration
    def move_to_joints(self, joints):
        joints = self.clamp_to_limits(joints, self.joint_limits)

        self.state["joint_gripper_cmd"] = joints["joint_gripper_cmd"]
        gripper_pos = self._gripper_cmd_to_pos(joints["joint_gripper_cmd"])

        self.robot.base.translate_by(
            x_m=joints["joint_base"] - self.state["joint_base"]
        )
        # TODO: check if we need the blocking set point arm movement behavior from cd1f3d1d
        self.robot.arm.move_to(joints["joint_arm"])
        self.robot.lift.move_to(joints["joint_lift"])

        self.robot.base.push_command()
        self.robot.arm.push_command()
        self.robot.lift.push_command()

        self._wrist_move_to_pos("wrist_yaw", joints["joint_wrist_yaw"])
        self._wrist_move_to_pos("wrist_pitch", joints["joint_wrist_pitch"])
        self._wrist_move_to_pos("wrist_roll", joints["joint_wrist_roll"])
        self._wrist_move_to_pos("stretch_gripper", gripper_pos)
        # sleeping to make sure all the joints are updated correctly (remove if not necessary)
        time.sleep(0.7)
        self.robot.arm.wait_until_at_setpoint()
        self.robot.lift.wait_until_at_setpoint()
        self.robot.base.wait_until_at_setpoint()

        self._update_state()

    # NOTE: x lift (+ down), y base (+ right), z arm (+ in) relative???
    #       x base (+ left), y arm (+ in), z lift (+ up) absolute?
    #       this seems to be caused by this wonky R matrix below:
    #       home pose by forward kinematics:
    #       R = [[   0.0101394,   -0.999648,   -0.024534;
    #                  0.29416,  -0.0204677,    0.955537;
    #                -0.955702,  -0.0169055,    0.293849]
    #       t = [  -0.0470669,   -0.282123,    0.757802]]
    def move_to_pose(
        self, translation_tensor, rotational_tensor, gripper, absolute=False
    ):
        translation = [
            translation_tensor[0],
            translation_tensor[1],
            translation_tensor[2],
        ]
        rotation = rotational_tensor

        # move logic
        joints = self.get_joints(ik=True)

        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = joints[self.ik_joint_list[joint_index]]

        curr_pose = PyKDL.Frame()
        del_pose = PyKDL.Frame()
        self.fk_p_kdl.JntToCart(self.joint_array, curr_pose)

        rot_matrix = R.from_euler("xyz", rotation, degrees=False).as_matrix()

        # new code from here
        del_rot = PyKDL.Rotation(
            PyKDL.Vector(rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0]),
            PyKDL.Vector(rot_matrix[0][1], rot_matrix[1][1], rot_matrix[2][1]),
            PyKDL.Vector(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]),
        )
        del_trans = PyKDL.Vector(translation[0], translation[1], translation[2])
        del_pose.M = del_rot
        del_pose.p = del_trans
        if absolute:
            goal_pose_new = del_pose
            print("Here")
        else:
            goal_pose_new = curr_pose * del_pose

        seed_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())
        # changed seed array to self.joint_array
        # NOTE: i think we should start from self.joint_array?
        self.ik_p_kdl.CartToJnt(seed_array, goal_pose_new, self.joint_array)

        ik_joints = {}

        for joint_index in range(self.joint_array.rows()):
            ik_joints[self.ik_joint_list[joint_index]] = self.joint_array[joint_index]
        ik_joints["joint_gripper_cmd"] = gripper[0]
        ik_joints["joint_base"] = ik_joints["joint_fake"]

        self.move_to_joints(ik_joints)
        joints = self.get_joints(ik=True)
        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = joints[self.ik_joint_list[joint_index]]

    def set_joint_limits(self, joint_limits):
        # joint_limits is a dictionary like so:
        # {
        #     "joint_name": (joint_min, joint_max)
        #     ...
        # }
        self.joint_limits = joint_limits

    def set_velocity_limits(self, velocity_limits):
        self.velocity_limits = velocity_limits

    def clamp_to_limits(self, joints, limits):
        result = {}
        for key in joints:
            if key in limits:
                result[key] = clamp(joints[key], *self.joint_limits[key])
            else:
                result[key] = joints[key]
        return result

    def reset_base(self):
        self._update_state()
        self.robot.base.status["x"] = 0

    def set_velocity(self, joint, command):
        joints = {
            "joint_arm": (self.robot.arm.set_velocity, self.robot.arm),
            "joint_lift": (self.robot.lift.set_velocity, self.robot.lift),
            "joint_base": (self.robot.base.set_translate_velocity, self.robot.base),
        }
        method, joint_object = joints[joint]
        command = clamp(command, *self.velocity_limits[joint])
        if abs(command) < self.VELOCITY_DEADZONE:
            # TODO: maybe manually stop base motor (firmware does not stop it when vel = 0)
            command = 0.0
        method(command)
        joint_object.push_command()

    def start_controller(self):
        if self.controller_thread is None or not self.controller_thread.is_alive():
            self.stop_event.clear()
            self.controller_thread = threading.Thread(target=self.controller_loop)
            self.controller_thread.start()
        logging.info("controller started")

    def stop_controller(self):
        if self.controller_thread and self.controller_thread.is_alive():
            self.stop_event.set()
            self.controller_thread.join()
        logging.info("controller stopped")

    def update_target_joints(self, joints):
        self.target_joints = self.clamp_to_limits(joints, self.joint_limits)

    def controller_loop(self):
        logging.info("controller loop running")
        last_update = time.time()
        while not self.stop_event.is_set():
            if time.time() - last_update < 0.1:
                time.sleep(0.01)
            # logging.info("Controller update")
            last_update = time.time()
            self._update_state()
            self.state["joint_gripper_cmd"] = self.target_joints["joint_gripper_cmd"]
            self.states_record.append(self.state.copy())
            for joint, cmd_position in self.target_joints.items():
                if joint in self.joint_names:
                    if joint in self.dynamixel_joints:
                        # positional control for the dynamixel joints
                        if joint == "joint_gripper_cmd":
                            joint_name = "stretch_gripper"
                            target_position = self._gripper_cmd_to_pos(cmd_position)
                        else:
                            joint_name = joint.strip("joint_")
                            target_position = cmd_position
                        self._wrist_move_to_pos(joint_name, target_position)
                    else:
                        # velocity control for the other joints
                        error = self.state[joint] - cmd_position
                        # logging.info(f"{joint}: {cmd_position:.4f} {error:.4f}")
                        command = -1 * error
                        self.set_velocity(joint, command)

    def save_states(self, fpath):
        with open(fpath, "wb") as f:
            pickle.dump(self.states_record, f)

    def get_joint_names(self):
        return self.joint_names
