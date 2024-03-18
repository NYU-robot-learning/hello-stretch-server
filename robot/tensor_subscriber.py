import numpy as np
from .zmq_utils import *

class TensorSubscriber(object):
    def __init__(self,configs):
        # create the ZMQ subscriber for robot action, home, and home params
        self.robot_action_subscriber = ZMQKeypointSubscriber(
            host=configs['host'],
            port=configs['action_port'],
            topic="robot_action",
        )
        self.home_subscriber = ZMQKeypointSubscriber(
            host=configs['host'],
            port=configs['action_port'],
            topic="home",
        )
        self.home_params_subscriber = ZMQKeypointSubscriber(
            host=configs['host'],
            port=configs['action_port'],
            topic="params",
        )
        # create the ZMQ response socket for flag
        self.flag_socket = create_response_socket(host=configs['host'], port=configs['flag_port'])