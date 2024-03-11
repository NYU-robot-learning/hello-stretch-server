from camera.d405_publisher import D405ImagePublisher
from robot import Listner, HelloRobot
import cv2
from multiprocessing import Process, Value
import signal
import sys
import time


def camera_process():
    camera_publisher = D405ImagePublisher()
    camera_publisher.publish_image_from_camera()


def robot_process(hello_robot):
    print("robot process started")
    listner = Listner(hello_robot)
    listner.start()


if __name__ == "__main__":
    t2 = Process(target=robot_process, args=(None,))
    t2.start()

    def signal_handler(sig, frame):
        t2.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    camera_process()
