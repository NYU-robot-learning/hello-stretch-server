from camera.demo import R3DApp
import cv2
import time
from robot.zmq_utils import *
from robot.teleop.hello_robot import HelloRobot
from utils.gripper_net import GripperNet
import torch
from torchvision import transforms


from scipy.spatial.transform import Rotation as R
    
class R3DCameraPublisher(ProcessInstantiator):
    def __init__(self):
        super().__init__()
        self.timer = FrequencyTimer(20)

        self._start_camera()

        self.count = 0

        self.hello_robot = HelloRobot()

        self.pose_initialized = False

        self._init_gripper_model()


    def _init_gripper_model(self):
        model = GripperNet()
        model.to(torch.device("cpu"))
        model.load_state_dict(torch.load("/home/hello-robot/hello-stretch-server/gripper_model.pth", map_location="cpu"))
        model.eval()

        self.gripper_model = model
        self.resize = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((224, 224)),
        ])

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
    def get_image_and_pose(self):
        image = None
        while image is None:
            image, depth, pose = self.app.start_process_image()
        
            return image, pose
    
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
                self.timer.start_loop()

                image, pose = self.get_image_and_pose()
                
                if count % 20 == 0:
                    image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
                    image = self.resize(image.copy())

                    gripper = self.gripper_model(image.unsqueeze(0)).item()
                
                translation_tensor = -1 * pose[4:]

                rotational_tensor = R.from_quat(pose[:4]).as_euler('xyz', degrees=False)
                rotational_tensor[4] += 15/180*np.pi
                rotational_tensor[5] += 90/180*np.pi

                self.hello_robot.move_to_pose(translation_tensor, rotational_tensor, gripper)
                
                self.timer.end_loop()
