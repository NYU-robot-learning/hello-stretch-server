
from hello_robot import HelloRobot
import pickle
from normalized_velocity_control import zero_vel

class OpenLoop:
    def __init__(self):
        self.run_for = 1
        self.actions = pickle.load(open('/home/hello-robot/pose_actions.pkl', 'rb'))
        self.hello_robot = HelloRobot()
        self.step_n = 0

    def _process_instruction(self, instruction):
        global schedule
        if instruction.lower() == "h":
            self.hello_robot.home()
            self.step_n = 0

        elif instruction.lower() == "r":
            h = input("Enter height:")
            self.h = float(h)
        elif instruction.lower() == "q":
            self.hello_robot.controller.stop()
        elif len(instruction) == 0:
            action = self.actions[self.step_n]
            self.hello_robot.move_to_pose(action[0], action[1], action[2])
            self.step_n += 1
        elif instruction.isdigit():
            for i in range(int(instruction)):
                action = self.actions[self.step_n]
                self.hello_robot.move_to_pose(action[0], action[1], action[2])
                self.step_n += 1
            self.hello_robot.controller.set_command(zero_vel)
        else:
            print("Invalid instruction")

    def run(self):
        self.hello_robot.home()
        while True:
            instruction = input("Enter instruction:")
            if instruction.lower() == "q":
                instruction = self._process_instruction(instruction)
                break
            instruction = self._process_instruction(instruction)

if __name__ == "__main__":
    openloop = OpenLoop()
    openloop.run()