import cv2
import rospy

import numpy as np
import time
from imgcat import imgcat
import pyrealsense2 as rs

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32

# from numpy_ros import converts_to_message, to_message

NODE_NAME = "gopro_node"
IMAGE_PUBLISHER_NAME = "/gopro_image"
DEPTH_PUBLISHER_NAME = "/gopro_depth"
SEQ_PUBLISHER_NAME = "/gopro_seq"

D405_COLOR_SIZE = [640, 480]
D405_DEPTH_SIZE = [640, 480]
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


class D405ImagePublisher(object):
    def __init__(self):
        # Initializing ROS node
        try:
            rospy.init_node(NODE_NAME)
        except rospy.exceptions.ROSException as e:
            print(e)
            print("ROS node already initialized")
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(
            IMAGE_PUBLISHER_NAME, Image, queue_size=1
        )
        self.depth_publisher = rospy.Publisher(
            DEPTH_PUBLISHER_NAME, Float32MultiArray, queue_size=1
        )
        self.seq_publisher = rospy.Publisher(SEQ_PUBLISHER_NAME, Int32, queue_size=1)
        self._seq = 0

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

    def publish_image_from_camera(self):
        rate = rospy.Rate(D405_FPS)
        while True:

            frames_d405 = self.pipeline_d405.wait_for_frames()
            image = frames_d405.get_color_frame()
            depth = frames_d405.get_depth_frame()

            # image, depth, pose = self.app.start_process_image()
            # image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
            image = cv2.resize(image, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
            depth = np.ascontiguousarray(depth).astype(np.float64)

            cv2.imshow("D405", image)
            cv2.imshow("D405 Depth", depth)

            # Creating a CvBridge and publishing the data to the rostopic
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            depth_data = convert_numpy_array_to_float32_multi_array(depth)
            self.image_publisher.publish(self.image_message)
            self.depth_publisher.publish(depth_data)
            self.seq_publisher.publish(Int32(self._seq))
            self._seq += 1

            # Stopping the camera
            if cv2.waitKey(1) == 27:
                break
            if self.app.stream_stopped:
                print("breaking")
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("connected")
    camera_publisher = D405ImagePublisher()
    # print('calling publisher')
    camera_publisher.publish_image_from_camera()
    # print('publisher end')
