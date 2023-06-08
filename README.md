# robot_monkey_thesis
Assembly and Programming of a Robot Monkey to Study Imitation Learning in Marmosets

## Necessary installs
- Rospy message converter 

install with: 

```
sudo apt install ros-noetic-rospy-message-converter
```


## Necessary modifications of standard Rviz setup



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



