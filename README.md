# robot_monkey_thesis
Assembly and Programming of a Robot Monkey to Study Imitation Learning in Marmosets

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

![Screenshot from 2023-06-08 14-08-26](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/32950c11-2125-4e70-afb8-e526500a1aea  | width=200)



