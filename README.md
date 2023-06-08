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
7. In **Define Virtual Joints**: Create an entry according to the following picture:
![virtual_joints_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/d13b2786-2f2d-4cb3-a9d1-381b475a8d9a)
8. In **Planning groups**: Create an entry according to the following picture:

![planning_groups_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/bfb11fae-02b0-4fc5-a143-46e951aad534)

