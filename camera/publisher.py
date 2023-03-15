import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray,MultiArrayDimension

import sys
from .demo import DemoApp 

NODE_NAME = 'gopro_node'
IMAGE_PUBLISHER_NAME = '/gopro_image'
DEPTH_PUBLISHER_NAME = '/gopro_depth'
CAM_MATRIX_PUBLISHER_NAME = '/iphone_k_matrix'

def convert_2d_array_to_multi_array(matrix, data_type=Float32MultiArray):
	  # Create a Float64MultiArray object
    data_to_send = data_type()

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


class ImagePublisher (object):
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
        self.image_publisher = rospy.Publisher(IMAGE_PUBLISHER_NAME, Image, queue_size = 1)
        self.depth_publisher = rospy.Publisher(DEPTH_PUBLISHER_NAME, Float32MultiArray, queue_size = 1)
        self.intrinsic_publisher = rospy.Publisher(CAM_MATRIX_PUBLISHER_NAME, numpy_msg(Floats), queue_size = 1)

    def publish_image_from_camera(self):
        rate = rospy.Rate(28)
        while True:

            image, depth, pose, intrinsics_data = self.app.start_process_image()
            #print("Depth shape: ", depth.shape, image.shape)
            image = np.moveaxis(image, [0], [1])[...,::-1,::-1]
            depth = np.moveaxis(depth, [0], [1])[...,::-1,::-1].astype(np.float64)
            #print("Depth shape 2: ", depth.shape, image.shape)
            #plot image
            #cv2.imshow('image', image)
            # Creating a CvBridge and publishing the data to the rostopic
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            depth_data = convert_2d_array_to_multi_array(depth, data_type=Float32MultiArray)

            self.image_publisher.publish(self.image_message)
            self.depth_publisher.publish(depth_data)
            self.intrinsic_publisher.publish(intrinsics_data)

            # Stopping the camera
            if cv2.waitKey(1) == 27:
                break
            if self.app.stream_stopped:
                print("breaking")
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    app = DemoApp()
    app.connect_to_device(dev_idx=0)
    print('connected')
    camera_publisher = ImagePublisher(app)
    #print('calling publisher')
    camera_publisher.publish_image_from_camera()
    #print('publisher end')
