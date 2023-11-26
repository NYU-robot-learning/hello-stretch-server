import argparse
import logging
import random
import time
from typing import Optional

import numpy as np
import rospy
from std_msgs.msg import Int64

from .hello_robot import HelloRobot
from .tensor_subscriber import TensorSubscriber

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

PING_TOPIC_NAME = "/run_model_ping"
STATE_TOPIC_NAME = "/run_model_state"


parser = argparse.ArgumentParser()
parser.add_argument("--lift", type=float, default=0.5, help="position of robot lift")
parser.add_argument("--arm", type=str, default=0.02, help="arm position")
parser.add_argument("--base", type=float, default=0.0, help="position of robot base")
parser.add_argument(
    "--yaw", type=float, default=0.0, help="position of robot wrist yaw"
)
parser.add_argument(
    "--pitch", type=float, default=0.0, help="position of robot wrist pitch"
)
parser.add_argument(
    "--roll", type=float, default=0.0, help="position of robot wrist roll"
)
parser.add_argument(
    "--gripper", type=float, default=1.0, help="position of robot gripper"
)

args = parser.parse_args()
params = vars(args)


class Listener:
    GRIPPER_SAFETY_LIMITS = (-1.0, 1.0)
    TRANSLATION_SAFETY_LIMITS = (-0.05, 0.05)

    def __init__(
        self,
        hello_robot: Optional[HelloRobot] = None,
        gripper_safety_limits=GRIPPER_SAFETY_LIMITS,
        translation_safety_limits=TRANSLATION_SAFETY_LIMITS,
        stream_during_motion=True,
    ):
        logging.info("Starting robot listener")
        if hello_robot is None:
            self.hello_robot = HelloRobot()
        else:
            self.hello_robot = hello_robot

        try:
            rospy.init_node("Acting_node")
        except rospy.exceptions.ROSException:
            logging.warning("Node already initialized.")
        self.hello_robot.home()
        self._create_publishers()
        self.tensor_subscriber = TensorSubscriber()
        self.rate = rospy.Rate(5)

        self.gripper_safety_limits = gripper_safety_limits
        self.translation_safety_limits = translation_safety_limits
        self.stream_during_motion = stream_during_motion

    def _create_publishers(self):
        self.ping_publisher = rospy.Publisher(PING_TOPIC_NAME, Int64, queue_size=1)
        self.state_publisher = rospy.Publisher(STATE_TOPIC_NAME, Int64, queue_size=1)

    def _create_and_publish_uid(self):
        self._wait_for_robot_motion()
        self.uid = random.randint(0, 30000)
        self.ping_publisher.publish(Int64(self.uid))

    def _publish_uid(self):
        logging.info(f"Published uid: {self.uid}")
        self.ping_publisher.publish(Int64(self.uid))

    def _wait_for_data(self):
        logging.info("Waiting for the data")
        wait_count = 0
        waiting = True
        while waiting:
            # if wait_count > 10, publish uid again
            if wait_count > 15:
                self._publish_uid()
                wait_count = 0
            if (
                (
                    (self.tensor_subscriber.tr_data_offset == self.uid)
                    and (self.tensor_subscriber.rot_data_offset == self.uid)
                    and (self.tensor_subscriber.gr_data_offset == self.uid)
                )
                or (self.tensor_subscriber.home_data_offset == self.uid)
                or (self.tensor_subscriber.home_params_offset == self.uid)
            ):
                waiting = False

            wait_count += 1
            self.rate.sleep()

    def _wait_for_robot_motion(self):
        if self.stream_during_motion:
            logging.info("Waiting for robot motion, but streaming...")
            time.sleep(8 / 10)
            return
        logging.info("Waiting for robot motion...")
        time.sleep(1.0)

    def _wait_till_ready(self):
        while self.hello_robot.robot.pimu.status["runstop_event"]:
            self.rate.sleep()

    def _execute_action(self):
        self._wait_till_ready()

        if self.tensor_subscriber.home_data_offset == self.uid:
            self.hello_robot.home()
        elif self.tensor_subscriber.home_params_offset == self.uid:
            self.hello_robot.set_home_position(*self.tensor_subscriber.home_params)
        else:
            logging.debug("Received action to execute at ", time.time())
            translation_tensor = np.clip(
                np.array(self.tensor_subscriber.translation_tensor),
                a_min=self.translation_safety_limits[0],
                a_max=self.translation_safety_limits[1],
            )
            translation_tensor = translation_tensor.tolist()
            rotational_tensor = self.tensor_subscriber.rotational_tensor
            gripper_tensor = np.clip(
                np.array(self.tensor_subscriber.gripper_tensor),
                a_min=self.gripper_safety_limits[0],
                a_max=self.gripper_safety_limits[1],
            )
            gripper_tensor = gripper_tensor.tolist()
            self.hello_robot.move_to_pose(
                translation_tensor, rotational_tensor, gripper_tensor
            )
        self.rate.sleep()

        self._wait_till_ready()

    def start(self):
        while True:
            self._create_and_publish_uid()
            self._wait_for_data()
            self._execute_action()


if __name__ == "__main__":
    listner_object = Listener()
    listner_object.start()
