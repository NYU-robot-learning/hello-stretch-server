from camera.demo import R3DApp
import cv2
import time
from robot.zmq_utils import *
import os
import numpy as np
from quaternion import as_rotation_matrix, quaternion

from scipy.spatial.transform import Rotation as R

import matplotlib
import matplotlib.pyplot as plt
import random
import time



matplotlib.use("TkAgg")

# Initialize the figure and axis
fig, ax = plt.subplots()
ax.set_xlim(-1, 3)  # Set x-axis limits
ax.set_ylim(-1, 8)  # Set y-axis limits
ax.set_aspect('equal', adjustable='box')

# Initialize lists to store x and y data points
x_data, y_data = [], []

# Set up the line and scatter plot
line, = ax.plot([], [], lw=2, color='blue')  # Line connecting points
# scat = ax.scatter([], [], c='red')  # Scatter plot for points

# Update function for manual calling
def manual_update(x, y):
    # Get new data point

    # Append new data point to the lists
    x_data.append(x)
    y_data.append(y)

    # Update line and scatter plot
    line.set_data(x_data, y_data)

    # Redraw the plot with the new data
    fig.canvas.draw()
    fig.canvas.flush_events()

# Display the plot
plt.ion()  # Turn on interactive mode
plt.show()

    
class R3DCameraPublisher(ProcessInstantiator):
    def __init__(self, host, port, use_depth):
        super().__init__()
        self.host = host
        self.port = port
        self.use_depth = use_depth
        self.rgb_publisher = ZMQCameraPublisher(
            host = self.host, 
            port = self.port
        )
        
        self._seq = 0
        self.timer = FrequencyTimer(30)

        # self.mask1 = np.load("new_stick_mask_60.npy")
        # self.roi = np.load("old_stick_pos.npy")
        # self.mask3 = np.load("old_stick_pos2.npy")

        self._start_camera()

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

    # get the RGB and depth images from the Record3D
    def get_rgb_depth_images(self):
        image = None
        while image is None:
            image, depth, pose = self.app.start_process_image()
            image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
            image = cv2.resize(image, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
        if self.use_depth: 
            depth = np.ascontiguousarray(np.rot90(depth, -1)).astype(np.float64)  
            return image, depth, pose
        else:
            return image, pose

    def create_transform(self, pose):
        qx, qy, qz, qw, px, py, pz = pose
        transform = np.eye(4)
        transform[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        transform[:3, -1] = [px, py, pz]
        return transform

    def transform_to_vec(self, transform):
        x,y,z = transform[:3,3]
        rx,ry,rz = R.from_matrix(transform[:3,:3]).as_euler('xyz', degrees=False)
        return [x,y,z,rx,ry,rz]
    
    # get RGB images at 50Hz and publish them to the ZMQ port
    def stream(self):
        count = 0
        init_pose = None
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
                if self.use_depth:
                    wrist_image, wrist_depth, pose = self.get_rgb_depth_images()
                    self.rgb_publisher.pub_image_and_depth(wrist_image, wrist_depth, time.time())
                else:
                    wrist_image, pose = self.get_rgb_depth_images()
                    self.rgb_publisher.pub_rgb_image(wrist_image, time.time())
                
                
                if count % 20 == 0:
                    extrinsic_matrix = self.create_transform(pose)

                    if init_pose is None:
                        init_pose = np.copy(extrinsic_matrix)

                    relative_pose = np.linalg.inv(init_pose) @ extrinsic_matrix
                    x, y, z, rx, ry, rz = self.transform_to_vec(relative_pose)
                    # update_plot(-y, -z)
                    manual_update(-z, -y)
                    
                count += 1
                
                    
                self.timer.end_loop()

                if "DISPLAY" in os.environ:
                    cv2.imshow("iPhone", wrist_image)
            
                if cv2.waitKey(1) == 27:
                    break
        
        cv2.destroyAllWindows()