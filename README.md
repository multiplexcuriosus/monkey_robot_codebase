# Instructions for setup and usage of control interface for monkey robot created during my bachelor's thesis

## Overview
- Bachelor's thesis:  "Assembly and Programming of a Robot Monkey to Study Imitation Learning in Marmosets" (see main branch)
- Existing documentation: The appendix of the above mentioned thesis already contains a lot of the necessary instructions for the setup of the control interface. As is hinted at in section C.2.4 of the appendix of the thesis, it was not clear at the time of writing, whether the moveit config package created during the thesis would be usable on foreign machines. It turned out it is not. For this reason, this ReadMe contains a detailed instruction on how to generate the moveit config package used during the thesis. For completeness, most of the other setup steps are described as well.


## High level instructions

1. Use the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash a Ubuntu 20.04 Server Distribution to the Raspberry Pi (RPP). The image used during the thesis can be found [here](https://old-releases.ubuntu.com/releases/20.04/)


## Necessary installs
- Rospy message converter 

install with: 

```
sudo apt install ros-noetic-rospy-message-converter
```



## Creation of a working moveit config pkg 
These instructions assume you have setup a catkin workspace.
1. Download your valid URDF file. For the remainder of this tutorial I will use the URDF file name "monkey_robot.urdf".

Make sure that your URDF contains the following lines below ```<robot name="monkey_robot">```:
```xml
    <link name="world" />
    <joint name="world_to_base_link=" type="fixed">
    	<parent link="world"/>
        <child link="base_link"/>
    </joint>
  ```
  
2. Launch the moveit setup assistant with ``` roslaunch moveit_setup_assistant setup_assistant.launch```. Note that this will only work if:
- you have installed and setup moveit according to this [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
- you have sourced your workspace with ```source devel/setup.bash```
3. In the setup assistant, select "Create New Moveit Configuration Package"
4. Click on "Browse" and select your URDF file 
5. Click on "Load Files"
6. In **Self-Collision**: Set "Sampling Density to max" and click "Generate Collision Matrix"
7. In **Virtual Joints**: Create an entry according to the following picture:

![virtual_joints_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/d13b2786-2f2d-4cb3-a9d1-381b475a8d9a)

8. In **Planning groups**: Create entries according to the following picture:
 
![planning_groups_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/bfb11fae-02b0-4fc5-a143-46e951aad534)

9. In **Robot Poses**: Create entries according to the following pictures:
![Screenshot from 2023-06-08 13-42-08](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/229191b2-0624-47ec-8534-6de6b541d00c)
![Screenshot from 2023-06-08 13-43-14](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/11759217-dc7c-43cf-b7c4-5f5431a7c3ce)
![Screenshot from 2023-06-08 13-44-13](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/43201653-b480-4239-a40e-ad5f28059e59)

10. In **End Effectors**: Create entries according to the following picture:
![eef_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/67d41e35-fa33-42ad-b33a-849618700e0c)

11. In **Author Information**, type in a name and a valid email, otherwise we can't save the config 
12. In **Configuration Files**, specify the name of the package and place it in the source folder of your catkin workspace, e.g ```/home/<user>/ws_moveit/src/<name-of-your-moveit-config>```
13. Click "Generate Package" 
14. Now we need to build the moveit config package. To do that, open your catkin ws in a terminal and run:
 ```
 source devel/setup.bash
 catkin build <name-of-your-moveit-config>
 ``` 
16. In order to test the freshly generated config package, run (in the same terminal as before):

```
roslaunch <name-of-your-moveit-config> demo.launch
``` 

Now you should see a window popping up containing the simulation environment Rviz. It should look like this:
![Screenshot from 2023-06-08 13-59-16](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/fbc231dd-3a55-407f-bff5-a6328b919ae9)



## Modifications of standard Rviz setup
In order to have a efficient workflow and make use of all the tools and ideas conceived during the thesis, the following modifications have to be made inside of Rviz and saved.
1. In the "Displays" panel, click on "Add". In the then appearing panel (named "Rviz"), order the visualizations "By display type", select "MarkerArray" and click "Ok". 

<img src="https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/c7b48752-9687-4233-907a-78f788a528e0" width="400">

2. In the MotionPlanning Panel, tick the tickbox labeled "Approximate IK solutions". Without that enabled, we cannot manipulate the end effectors of the robots arms. 
3. In the Displays Panel, go to the MotionPlanning/Planned Path rider and uncheck the tickbox "Show Robot Visual". This will deactivate the animation of the planned robot arm trajectory, which can get annoying. 

4. Press "CTRL+S" to save the current Rviz config

## Modifications of standard kinematics config
In order to have a efficient workflow and make use of all the tools and ideas conceived during the thesis, a few modifications have to be made to the **Kinematics.yaml** file, which can be found in the "config" folder of your moveit-config-pkg. The file should look as follows:
```yaml
monkey_left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
monkey_right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
monkey_head:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
```
List of changes made:
- added line ```position_only_ik: True``` to each planning group
- changed "kinematics_solver_search_resolution" to 0.01 for each planning group


## Quick test ##
If you did everything right up until here, you should be able to drag around the hands of the robot quite freely around in space (of course only inside the space which is reachable by the robot and permitted by his joint limits). If you can't drag around the hands, it might help to untick and retick the "Approximate IK solutions" box. This setting seems to deactivate itself sometimes. 


## Download and build the monkey_interface ##
1. From this repo, download the folder "monkey_interface" and place it in the "src" folder of your catkin ws
2. Run ```catkin build monkey_interface``` to build the package.
3. If you haven't done so already, run ```caktin build ``` in the src folder, to build all packages. This will take about 10min (if you never build them before).

## Setup the monkey_interface ##
1. Open your catkin ws in two different terminals and source it in both.
2. In the first terminal run ```roslaunch <name-of-your-moveit-config> demo.launch```. Rviz should open. 
3. In the second terminal run ```rosrun monkey_interface monkey_interface.py```
4. Arrange all windwos such that you have Rviz on the left side of your screen and the second terminal on the right side (having multiple screens helps).

## Use the monkey_interface ##
Through the shell you can select one of the following actions, which I will call "modes":
```
[1]  Display the (hard coded) single pose goal
[2]  Display the (hard coded) trajectory
[3]  Collect waypoints for a cartesian path
[4]  Load and edit waypoints from a specific json file 
[5]  Exit
```

Now follows a detailed explanation of the modes:

[1]: In the python script you can hard code a pose goal (pose: position + orientation) and running mode 1 will:
- Display a (TODO) blue sphere in Rviz at the coordinate of the pose goal.
- Plan a trajectory for the selectecd planning group. This will obviously only work for the arms, not for the head.
- Execute the trajectory. This will make the arm of the robot in the simulation go to the pose goal. The real robot will only execute this movement too if the joint_control_listener.py script is running on the raspberry pi (RPP) and the ROS environment variables have been setup correctly.
[2]: Same thing as 1 but for multiple poses
[3]: Once you have selected mode 3, you are directly prompted to move the eef of your chosen planning group to the first waypoint in Rviz. Recall that the way we create movements for the arms of the robot, is that we specify a list of poses (called waypoints by me) and compute a path for a specific eef to go through those waypoints. So mode 3 will let you collect waypoints for as long as you like.
 
**Important** Whenever you have moved the eef of the robot to a desired waypoint collection **you must manipulate the rotation of the interactive marker (blue sphere)**. This ensures that the last interactive marker position recorded by the monkey_interface node corresponds to the position of the hand. More details can be found in section 3.4 of my thesis. This means you must "rotate" the interactive marker by pulling at one of the three circles (red, green, blue). 

![Screenshot from 2023-06-08 15-01-51](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/6f732623-7db9-4f73-82b8-45f0a5802f8f)

If you don't manipulate the interactive marker, there is a high probabilty that the waypoint you save is outside the reachable space of the robot. This will then make it impossible to compute a trajectory. To prevent this from happening, a IK validty check is done for each newly added waypoint. It checks whether the newly set waypoint is in the reachable space of the robot by setting the new waypoint as a pose goal and trying to compute a trajectory to reach it. 

Once your done collecting waypoints, simply write "no" when the program asks you if you want to add another waypoint. If desired, you can now save your waypoint collection into a json file whose name you must input (this will only work if the rospy-message-converter was installed). 

Now you can plan a trajectory and if the planning was successfull, execute the trajectory. Again, note that it will only run on the physical robot if the listener script is running on the RPP. 

In case you didn't save the waypoint collection before the trajectory execution but have changed your mind, you get a last prompt which offers to save the waypoints.

[4]: In this mode, you can load an existing trajectory from memory. More specifically, the trajectory has to be a ROS PoseArray converted to a json file by the rospy-message-converter, and the file must be located in your catkin workspace. If you specified a name of a valid json file, it is loaded and the contained waypoints displayed in Rviz as spheres. Now you can either:
- plan a trajectory through the loaded waypoints and then decide if you want to execute it
- append new waypoints -> switch to mode 3
- exit

The following diagram describes the control flow of the monkey_interface.py script:

![control_flow_all](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/c0ea0a91-2c30-446e-bc22-1dbd28313763)









