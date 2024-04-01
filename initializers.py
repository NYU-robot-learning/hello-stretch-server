from multiprocessing import Process
from robot.zmq_utils import *
import hydra


class Start_Server(ProcessInstantiator):
    """
    Returns all the teleoperation processes. Start the list of processes 
    to run the teleop.
    """
    def __init__(self, configs):
        super().__init__()
        self.configs=configs
      
        self._init_camera_process()
        self._init_robot_process()
        
    #Function to start the components
    def _start_component(self, configs):
        # print(configs)
        # assert False
        component = hydra.utils.instantiate(configs)
        component.stream()

    #Function to start camera process
    def _init_camera_process(self):
        self.processes.append(Process(
            target = self._start_component,
            args = (self.configs.camera, )
        ))

    def _init_robot_process(self):
        self.processes.append(Process(
            target = self._start_component,
            args = (self.configs.controller, )
        ))