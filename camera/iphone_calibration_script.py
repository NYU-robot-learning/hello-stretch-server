from camera.demo import R3DApp
import cv2
import time
from robot.zmq_utils import *
from robot.script.hello_robot import HelloRobot
import random
from torchvision import transforms
import os
from datetime import datetime
import json

BASE_DIR = "/home/hello-robot/code/hello-stretch-server/gripper_data"
MAX_OPEN = 170
    
class R3DCamera(ProcessInstantiator):
    def __init__(self):
        super().__init__()
        self._start_camera()

        self.count = 0

        self.hello_robot = HelloRobot()

        self.resize = transforms.Compose([
            transforms.ToTensor(),
            # transforms.Resize((256, 256)),
        ])

        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.save_dir = os.path.join(BASE_DIR, now)
        os.makedirs(self.save_dir, exist_ok=True)

    # start the Record3D streaming
    def _start_camera(self):
        self.app = R3DApp()
        while self.app.stream_stopped:
            try:
                self.app.connect_to_device(dev_idx=0)
            except RuntimeError as e:
                print(e)
                print(
                    "Retrying to connect to device with id {idx}, make sure the device is connected and id is correct...".format(
                        idx=0
                    )
                )
                time.sleep(2)

    # get the RGB images and pose from Record3D
    def get_image(self):
        image = None
        while image is None:
            image, _, _ = self.app.start_process_image()
        
            return image
        
    def capture_image(self):
        image = self.get_image()

        image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
        image = (self.resize(image.copy()).permute(1,2,0).numpy() * 255).astype(np.uint8)

        return image

    def stream(self):
        curr_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        folder_path = f"calibration_images/iphone/{curr_time}"
        os.makedirs(folder_path, exist_ok=True)
        
        count = 0
        input("Press Enter")
        while True:
            if self.app.stream_stopped:
                try:
                    self.app.connect_to_device(dev_idx=0)
                except RuntimeError as e:
                    print(e)
                    print(
                        "Retrying to connect to device with id {idx}, make sure the device is connected and id is correct...".format(
                            idx=0
                        )
                    )
                    time.sleep(2)
            else:
                # Capture and save both image and gripper state after instruction is entered

                instruction = input("Instruction:")

                image = self.capture_image()
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