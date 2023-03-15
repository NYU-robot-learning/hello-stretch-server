from camera import DemoApp, ImagePublisher
from robot import Listner, HelloRobot
import cv2
from multiprocessing import Process, Value
import time


def camera_process(app):
    camera_publisher = ImagePublisher(app)
    camera_publisher.publish_image_from_camera()

def robot_process(hello_robot):
    print("robot process started")
    listner = Listner(hello_robot)
    listner.start()

def stream_manager():
    app = DemoApp()
    while app.stream_stopped:
        try:
            app.connect_to_device(dev_idx=0)
        except RuntimeError as e:
            print(e)
            print("Retrying to connect to device with id {idx}, make sure the device is connected and id is correct...".format(idx=0))
            time.sleep(2)

    try:
        camera_process(app)
    except cv2.error as e:
        print(e)
        print("The device was connected but the stream didn't start, trying to reconnect...")
        time.sleep(2)
        stream_manager()

    while not app.stream_stopped:
        time.sleep(2)



    stream_manager()


if __name__ == '__main__':
    t2 = Process(target=robot_process, args=(None, ))
    t2.start()
    stream_manager()