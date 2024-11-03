import stretch_body.robot

def calibrate():
    r = stretch_body.robot.Robot()
    r.startup()
    r.end_of_arm.motors['stretch_gripper'].home()

if __name__ == "__main__":
    calibrate()