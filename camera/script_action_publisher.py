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
            transforms.Resize((256, 256)),
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
        
    def capture_image_gripper(self):
        image = self.get_image()

        image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
        image = (self.resize(image.copy()).permute(1,2,0).numpy() * 255).astype(np.uint8)

        image_path = os.path.join(self.save_dir, f"{self.count:04d}.jpg")
        cv2.imwrite(image_path, image)

        gripper = self.hello_robot.get_gripper()

        json_file_path = os.path.join(self.save_dir, "gripper_data.json")
    
        # Check if the JSON file exists and read it, if not create a new list
        if os.path.exists(json_file_path):
            with open(json_file_path, 'r') as file:
                gripper_data = json.load(file)
        else:
            gripper_data = []
        
        # Append new gripper data with image reference
        gripper_data.append({"image": f"{self.count:04d}.jpg", "gripper": gripper})
        
        # Write updated data back to JSON
        with open(json_file_path, 'w') as file:
            json.dump(gripper_data, file, indent=4)

        self.count += 1

        return gripper

    def stream(self):
        count = 0
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

                gripper = self.capture_image_gripper()

                if instruction == "c":
                    gripper_action = random.uniform(-30, -10)
                    self.hello_robot.move_gripper(gripper_action)
                    
                    print("Current_Gripper:", gripper + gripper_action)
                elif instruction == "o":
                    gripper_action = random.uniform(10, 30)
                    self.hello_robot.move_gripper(gripper_action)

                    print("Current_Gripper:", gripper + gripper_action)
                elif instruction == "h":
                    gripper_action = MAX_OPEN - gripper
                    self.hello_robot.move_gripper(gripper_action)

                    print("Current_Gripper:", gripper + gripper_action)
                elif instruction == "q":
                    self.hello_robot.robot.stop()
                    break

                time.sleep(1/20)