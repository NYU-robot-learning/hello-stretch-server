from camera import R3DApp, ImagePublisher

if __name__ == "__main__":
    app = R3DApp()
    app.connect_to_device(dev_idx=0)
    print("connected")
    camera_publisher = ImagePublisher(app)
    camera_publisher.publish_image_from_camera()
