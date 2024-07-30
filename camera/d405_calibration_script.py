import cv2

import numpy as np
import time
import pyrealsense2 as rs
from datetime import datetime
import os

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32
from robot.zmq_utils import ZMQCameraPublisher, ProcessInstantiator
import matplotlib.pyplot as plt
from robot.script.hello_robot import HelloRobot

# from numpy_ros import converts_to_message, to_message

NODE_NAME = "gopro_node"
IMAGE_PUBLISHER_NAME = "/gopro_image"
DEPTH_PUBLISHER_NAME = "/gopro_depth"
SEQ_PUBLISHER_NAME = "/gopro_seq"

D405_COLOR_SIZE = [640, 480]
D405_DEPTH_SIZE = [640, 480]
RESIZED_IMAGE = (256, 256)
RESIZED_DEPTH = (256, 192)
D405_FPS = 15


realsense_ctx = rs.context()
connected_devices = {}

for i in range(len(realsense_ctx.devices)):
    camera_name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
    camera_serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    connected_devices[camera_name] = camera_serial


# @converts_to_message(Float32MultiArray))
def convert_numpy_array_to_float32_multi_array(matrix):
    # Create a Float64MultiArray object
    data_to_send = Float32MultiArray()

    # Set the layout parameters
    data_to_send.layout.dim.append(MultiArrayDimension())
    data_to_send.layout.dim[0].label = "rows"
    data_to_send.layout.dim[0].size = len(matrix)
    data_to_send.layout.dim[0].stride = len(matrix) * len(matrix[0])

    data_to_send.layout.dim.append(MultiArrayDimension())
    data_to_send.layout.dim[1].label = "columns"
    data_to_send.layout.dim[1].size = len(matrix[0])
    data_to_send.layout.dim[1].stride = len(matrix[0])

    # Flatten the matrix into a list
    data_to_send.data = matrix.flatten().tolist()

    return data_to_send


def setup_realsense_camera(serial_number, color_size, depth_size, fps):
    """
    Returns a Realsense camera pipeline used for accessing D435i & D405's video streams
    """
    pipeline = rs.pipeline()
    config = rs.config()

    if serial_number:
        config.enable_device(serial_number)

    config.enable_stream(
        rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps
    )
    config.enable_stream(
        rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps
    )

    profile = pipeline.start(config)
    return pipeline


# class D405ImagePublisher(ProcessInstantiator):
class D405:
    def __init__(self):
        try:
            d405_serial = connected_devices["Intel RealSense D405"]
        except KeyError:
            raise SystemError("Unable to find Realsense D405...")

        self.pipeline_d405 = setup_realsense_camera(
            serial_number=d405_serial,
            color_size=D405_COLOR_SIZE,
            depth_size=D405_DEPTH_SIZE,
            fps=D405_FPS,
        )

        self.hello_robot = HelloRobot()

    def stream(self):
        curr_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        folder_path = f"calibration_images/d405/{curr_time}"
        os.makedirs(folder_path, exist_ok=True)
        
        count = 0
        input("Press Enter")
        while True:
            instruction = input("Instruction:")

            frames_d405 = self.pipeline_d405.wait_for_frames()
            color_frame_d405 = frames_d405.get_color_frame()
            # depth_frame_d405 = frames_d405.get_depth_frame()
            image = np.asanyarray(color_frame_d405.get_data())
            
            # save image to folder labeled as {count}.jpg, including current est time in folder name
            
            cv2.imwrite(f"{folder_path}/{count:04d}.jpg", image)
            
            if instruction == "r":
                self.hello_robot.robot.base.translate_by(-0.1)
                self.hello_robot.robot.push_command()
            elif instruction == "l":
                self.hello_robot.robot.base.translate_by(0.1)
                self.hello_robot.robot.push_command()
            elif instruction == "f":
                self.hello_robot.robot.arm.move_by(0.05)
                self.hello_robot.robot.push_command()
            elif instruction == "b":
                self.hello_robot.robot.arm.move_by(-0.25)
                self.hello_robot.robot.push_command()
            elif instruction == "h":
                self.hello_robot.robot.home()
            elif instruction == "q":
                self.hello_robot.robot.stop()
                exit()
            # Stopping the camera
            # if cv2.waitKey(1) == 27:
            #     break
            # time.sleep(1 / D405_FPS)
            count += 1
            print(count)

        # cv2.destroyAllWindows()


if __name__ == "__main__":
    print("connected")
    camera_publisher = D405ImagePublisher("localhost", 32922)
    # print('calling publisher')
    camera_publisher.stream()
    # print('publisher end')