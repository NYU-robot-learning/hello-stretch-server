<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>


<!-- PROJECT LOGO -->
<br />
<div align="center">
  

  <h3 align="center">Hello Stretch Server</h3>

  <p align="center">
    Code to start the camera stream publisher and robot controller. The code is useful for record3d based camera streaming and controller but can be adapted for other use cases.
  </p>
</div>


<!-- ABOUT THE PROJECT -->
## Coformer/ Custom Server
Added pose publisher in `camera/publisher.py`; gripper threshold is set on line 32 of `hello_robot.py`. Uncomment lines 279-286 in that file to enable gripper thresholding, which should be done in most tasks.
This is because Coformer needs pose data.

Whenever you need to tune the gripper threshold upon deploying a new policy, you need to ssh into the robot, go into the robot/hello_robot.py, and change it manually.

- gripper max open is 1, min close is 0. It closes tight if the policy predicts gripper as anything below your thresh value.
- Make sure you put in the code: {thresh_value} * 47.


## [From main branch] Instruction for Installation and Running
First clone the repository to your hello robot. Using requirements.txt, you can install the required packages.

To run the server, follow the following steps:
* Make sure your robot is joint-calibrated by running 
  ```sh
  stretch_robot_home.py
  ```
* Once calibrated, run ```roscore``` in an independent terminal
* Then, in a new terminal, cd to the hello-stretch-server directory and run 
  ```sh
  python3 start_server.py
  ```