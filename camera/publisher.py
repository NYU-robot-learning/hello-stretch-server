import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# from numpy_ros import converts_to_message, to_message
import sys
from .demo import R3DApp

NODE_NAME = "gopro_node"
IMAGE_PUBLISHER_NAME = "/gopro_image"
DEPTH_PUBLISHER_NAME = "/gopro_depth"


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


class ImagePublisher(object):
    def __init__(self, app):
        # Initializing camera
        self.app = app
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

    def publish_image_from_camera(self):
        rate = rospy.Rate(28)
        while True:
            image, depth, pose = self.app.start_process_image()
            # print("Depth shape: ", depth.shape, image.shape)
            image = np.moveaxis(image, [0], [1])[..., ::-1, ::-1]
            depth = np.ascontiguousarray(np.rot90(depth, -1)).astype(np.float64)
            # print("Depth shape 2: ", depth.shape, image.shape)
            # plot image
            # cv2.imshow('image', image)
            # Creating a CvBridge and publishing the data to the rostopic
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            depth_data = convert_numpy_array_to_float32_multi_array(depth)
            self.image_publisher.publish(self.image_message)
            self.depth_publisher.publish(depth_data)

            # Stopping the camera
            if cv2.waitKey(1) == 27:
                break
            if self.app.stream_stopped:
                print("breaking")
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    app = R3DApp()
    app.connect_to_device(dev_idx=0)
    print("connected")
    camera_publisher = ImagePublisher(app)
    # print('calling publisher')
    camera_publisher.publish_image_from_camera()
    # print('publisher end')
