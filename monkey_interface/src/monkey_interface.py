#!/usr/bin/env python3

# Description =============================================================================================
'''
This script was written during and for the bachelor thesis "Assembly and Programming of a
Robot Monkey to Study Imitation Learning in Marmosets". 

It is intended to be used with the robot monkey assembled during the mentioned thesis. 
Once all setup steps as described on the ReadMe on the source repository and in the appendix of the thesis have been completed, 
this script can control the mentioned robot monkey or a different robot. 

This script interfaces the moveit move_group_python_interface, whose class reference can be found here:
https://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
(Note: At time of writing docs.ros.org is unreachable)

It was derived from the move_group_python_interface_tutorial which can be found here:
https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py


Author: Jaú Gretler
E-Mail: gretleja@ethz.ch
Last changed: 28.6.23

'''

# imports ==================================================================================================
import sys
import os
import copy
import pathlib
import rospy #pyright: ignore
import colorsys
import moveit_commander #pyright: ignore
import moveit_msgs.msg #pyright: ignore
from visualization_msgs.msg import InteractiveMarkerFeedback#pyright: ignore
import geometry_msgs #pyright: ignore
from  geometry_msgs.msg import Pose #pyright: ignore
from  geometry_msgs.msg import PoseArray #pyright: ignore
from visualization_msgs.msg import Marker,MarkerArray #pyright: ignore
from math import pi, dist, fabs, cos
from std_msgs.msg import String #pyright: ignore
from moveit_commander.conversions import pose_to_list #pyright: ignore
import json
import yaml #pyright: ignore
from rospy_message_converter import json_message_converter #pyright: ignore
from std_msgs.msg import String #pyright: ignore

try:
    from math import pi, TWO_PI, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    TWO_PI = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# imports done =============================================================================================

# Convenience functions ====================================================================================
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

# Convenience functions done ================================================================================

# MoveGroupInterface class ===============================================================================================================================
# This class serves as an interface for the moveit move_group node.
# This code was adapted from: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
class MoveGroupInterface(object): 
    "MoveGroupInterface"

    def __init__(self):
        super(MoveGroupInterface, self).__init__()

        # Setup markerArray (for visualization of poses)
        self.markerArray = MarkerArray()
        self.mtestArray = MarkerArray()
        self.wp_m_id_counter = 2**10 # To disambiguate markers, each one gets an ID. For duplicate markers in MarkerArray only one marker is displayed
        self.im_m_id_counter = 0
        self.markerArrayPub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100) # Instantiate publisher for markerArray topic
        self.lrimpa_max_size = 3 # Max size of last recorded IM pose array
        self.lrimp_count = 0 # Counter for last recorded IM pose
        self.imp_buffer = []

        # Setup moveit_commander and dependent variables (robot, scene)
        moveit_commander.roscpp_initialize(sys.argv)
        # This node will appear in the ROS network under the name "move_group_interface". 
        # The anonymous flag renders the existence of multiple move_group nodes possible, which would otherwise cause a crash.
        rospy.init_node("move_group_interface", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Query planning group from user
        target_planning_group_name = "monkey_left_arm" # Or input("Please specify planning group: ")
        
        # Setup move_group handle
        move_group = moveit_commander.MoveGroupCommander(target_planning_group_name)
        move_group.set_planning_time(0.2) # This sets a time limit for the planning of trajectories
        move_group.set_max_velocity_scaling_factor(1.0) # Scaler for execution of trajectories
        move_group.set_max_acceleration_scaling_factor(1.0)
        move_group.set_goal_position_tolerance(0.001) # Set position tolerance

        # Get eef link of target planning group
        eef_link = move_group.get_end_effector_link()
        print("End effector link: %s" % eef_link)

        # Get initial pose of eef
        self.eef_def_pose = move_group.get_current_pose().pose

        # Create a "DisplayTrajectory" ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Create a subscriber to the feedback of the IM in Rviz
        rospy.Subscriber("rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", InteractiveMarkerFeedback, self.intMarkerCallBack)
        
        # Variable to store last recorded position of IM in Rviz (received from Subscriber) 
        self.last_rec_im_pose = None
        # Array to store waypoints collected in Rviz gui 
        self.wpoints = []

        # Indicator for whether we are appending to a loaded json pose array
        self.appending = False
                
        # Array to store loaded waypoints from json file
        self.loaded_json_wpoints = None
        # For rainbow coloring collected wpoints in marker creator this index is necessary
        self.loaded_json_wpoint_index = 0

        # Save some variables as member variables
        self.move_group = move_group
        self.robot = robot
        self.eef_link = eef_link


    # Callback function for interactive marker subscriber. Saves the last published IM pose and creates a marker for it.
    # The last <self.lrimpa_max_size> IM are saved, stored in a buffer and displayed as a visual aid to locate the last published IM pose
    def intMarkerCallBack(self,data):
        #print("len markers before callback body: ",len(self.markerArray.markers))
        #print("Entered int marker")
        # Extract pose
        p = data.pose
        # Create marker for it
        self.createMarker(p,"im_trace")
        # Append to pose buffer
        self.imp_buffer.append(p)
        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        if(self.lrimp_count > self.lrimpa_max_size):
            # Pop from markerArray
            self.mtestArray.markers.pop(0)
            # Pop from IM pose buffer
            self.imp_buffer.pop(0)
            # Renumber the marker IDs
            id = 0
            for m in self.mtestArray.markers:
                m.id = id
                id += 1
        # Publish the MarkerArray
        #self.markerArrayPub.publish(self.markerArray)
        self.markerArrayPub.publish(self.mtestArray)
        # Get idx of last recorded IM pose
        last_imp_idx = len(self.imp_buffer) - 1
        # Save last recorded IM pose
        self.last_rec_im_pose = self.imp_buffer[last_imp_idx]

        #print("len markers after callback body: ",len(self.markerArray.markers))

    # Set joint goal for eef of move_group, execute trajectory, check if target and final pose of eef are within tolerance [Adapter from tutorial]
    def go_to_joint_goal(self):
        # Instantiate joint_state object
        joint_goal = self.move_group.get_current_joint_values()
        # Set joint states manually. ATTENTION: Must be in accordance to joint limits in URDF, otherwise an error will occur
        joint_goal[0] = 0 
        joint_goal[1] = 0 
        # Go to joint go state
        self.move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop() 
        # Check if current and target joint state are within tolerance
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    # Create a (visual) marker for a pose 
    def createMarker(self,marker_pose,_ns):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp    = rospy.get_rostime()
        m.ns = _ns
        if _ns == "im_trace":
            m.id = self.im_m_id_counter
            self.im_m_id_counter += 1
        else:
            m.id = self.wp_m_id_counter
            self.wp_m_id_counter += 1
        m.type = 2 # sphere
        m.action = 0
        m.pose.position = copy.deepcopy(marker_pose.position)
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.005
        m.scale.y = 0.005
        m.scale.z = 0.005
        #color
        if _ns == "appended_waypoints":
            #orange
            r = 1.0
            g = 0.5
            b = 0.0
        elif _ns =="im_trace":
            #pink
            r = 1.0
            g = 0.0
            b = 1.0
        elif _ns == "collected_waypoint" or _ns == "hard_coded_waypoint": 
            # light blue
            r = 0.0
            g = 1.0
            b = 1.0
        elif _ns == "loaded_wpoint":
            if self.loaded_json_wpoints:
                r,g,b = colorsys.hsv_to_rgb(0.4+self.loaded_json_wpoint_index/100.0*2.5, 1.0, 1.0)
                self.loaded_json_wpoint_index += 1
        #print(self.m_id_counter)
        ## If the namespace of the input points identifies it als waypoint, color it in HSV color mode with the number of existing markers determining the hue
        #if _ns == "waypoint":
        #    r,g,b = colorsys.hsv_to_rgb(0.01+self.m_id_counter/360.0/4.0, 1.0, 1.0)
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0


        if _ns == "im_trace":
            self.mtestArray.markers.append(m)
        else:
            self.markerArray.markers.append(m)

        # To keep track of the buffer size for the IM poses we must count how many we created
        if _ns == "im_trace":
            self.lrimp_count += 1

    # Set pose goal for eef of move_group, execute trajectory, check if target and final pose of eef are within tolerance [Adapter from tutorial]
    def go_to_pose_goal(self,tpose):
        # Extract target pose position
        pos = tpose.position
        pg_pos_arr = [pos.x,pos.y,pos.z]
        # Set pose goal
        self.move_group.set_position_target(pg_pos_arr,self.eef_link)
        # Move to pose goal, save success status. `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True) 
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop() 
        self.move_group.clear_pose_targets()
        # For testing  tolerance
        current_pose_check = self.move_group.get_current_pose().pose 
        return all_close(tpose, current_pose_check, 0.01)

    # Returns boolean indicating if there is valid trajectory from the eef def pos to a certain wpoint
    def isValid(self,wpoint):
        if not wpoint:
            print("No wpoint recorded. Have you moved the IM?")
            return None

        # Get pos of wpoint
        pos = wpoint.position
        # Create pos array
        pos_arr = [pos.x,pos.y,pos.z]
        # Set wpoint as pose goal
        self.move_group.set_position_target(pos_arr,self.eef_link)
        # Get a potential trajectory
        plan = self.move_group.plan()
        # The first element of the robot trajectory indicates whether the trajectory is valid
        return plan[0]


# Utility class mainly providing often used motion planning functionalities such as:
# - Planning, displaying and executing a single pose goal
# - Collecting waypoints in gui and computing a trajectory T for the eef of one planning group to go through
# - Saving, loading and editing of T
# This helper class could be integrated into the moveGroupInteface, this would however decrease code readability.
class Utils:
    def __init__(self,_interface) -> None:
        # A handle to the moveGroupInteface is necessary for some actions. 
        self.iface = _interface

    # Let user collect an arbitrary number of waypoints in gui, return the collected waypoints
    def collect_wpoints_in_gui(self,initial_wp_count):
        wcounter = initial_wp_count # For the shell interaction a waypoint counter is needed. An initial_count != 0 means that we will append new poses to a poseArray loaded form a json file
        wp_collecting_intent_status = " "         
        wp_ns = "collected_waypoint"
        col_poses = PoseArray() # Create empty poseArray
        # If initial_wp_count is greater 1 this means we are not populating an empty poseArray, but instead appending new poses to an existing poseArray
        if initial_wp_count > 1:
            self.appending = True 
            col_poses = self.iface.loaded_json_wpoints
            wp_ns = "appended_waypoints"
        # Collect as many poses as the user wants
        while wp_collecting_intent_status != "no": 
            wpoint_valid = False
            # Try to get a valid wpoint
            while not wpoint_valid:
                print("Move EEF to waypoint ",wcounter)
                input("To save waypoint press [Enter]")
                # Get the last recorded pose of the interactive marker
                if self.iface.last_rec_im_pose:
                    w = self.iface.last_rec_im_pose 
                else:
                    print("No IM pose has been recorded yet. Have you moved the IM?")
                    return None

                # Check if this pose is valid 
                wpoint_valid = self.iface.isValid(w)
                if wpoint_valid:
                    break
                else:
                    try_again_dec =input("Invalid waypoint (outside robot range). Do you want to try again? [Enter | no]")
                    if try_again_dec == "no":
                        return None


            col_poses.poses.append(w)
            # Append collected wpoint to wpoints stored in iface
            self.iface.wpoints.append(w)
            # Create marker for this waypoint
            self.iface.createMarker(w,wp_ns)
            wcounter += 1
            # Publish markers
            self.iface.markerArrayPub.publish(self.iface.markerArray)
            # Query continuation of waypoint collection
            wp_collecting_intent_status = input("Do you want to add another waypoint? [ Enter | no]")
        
        return col_poses
    
    # Query saving of poseArray from user, if desired query name of json file to save poseArray to. The file name without .json
    def querySave(self,pa):       
        # Query saving from user
        ans = input("Do you want to save these waypoints? [yes | Enter]")
        if ans == "yes":
            # Query target json file name (without .json ending)
            desired_filename = input("Please specify an (unused) filename without file type: ")
            # Consctruct path name
            path_to_current_dir = str(pathlib.Path().resolve()) # The path gets saved in the moveit workspace top folder
            path_name = path_to_current_dir + "/" + desired_filename + '.json' 
            # Convert json poseArray to json
            json_pose_array = json_message_converter.convert_ros_message_to_json(pa)
            # Dump json data into json file
            with open(path_name, 'w+') as f:
                json.dump(json_pose_array, f)
    
    # Create three hardcoded waypoints
    def pdExampleWaypoints(self):
        # Create waypoints based on current pose of end effector
        pa = PoseArray()
        current_pose = self.iface.move_group.get_current_pose().pose
        w0 = copy.deepcopy(current_pose)
        w1 = copy.deepcopy(w0)
        w1.position.z += 0.05
        w1.position.y += 0.06
        w1.position.x -= 0.02
        w2 = copy.deepcopy(w0)
        w2.position.z += 0.09
        w2.position.y += 0.07
        w2.position.x -= 0.0
        w3 = copy.deepcopy(w0)
        w3.position.z += 0.12
        w3.position.y += 0.07
        w3.position.x -= 0.02
        pa.poses = [w1,w2,w3]
        # Add markers for the created waypoints
        for w in pa.poses:
            self.iface.createMarker(w,'hard_coded_waypoint')
        # Publish the markerArray containing waypoint markers
        self.iface.markerArrayPub.publish(self.iface.markerArray)
        # Query execution
        self.queryCPP(pa)
        
    # Plan to a hardcoded pose goal, display the plan and query its execution
    def pdExamplePoseGoal(self):
        # Get default pose of current end effector
        def_pose = self.iface.eef_def_pose
        # Create marker for default pose
        self.iface.createMarker(def_pose,"hard_coded_waypoint")
        # Create target pose
        pose_goal = copy.deepcopy(def_pose)
        pose_goal.position.z += 0.05
        pose_goal.position.y += 0.07
        # Create marker for target pose
        self.iface.createMarker(pose_goal,"hard_coded_waypoint")
        # Publish marker array containing def and target pose
        self.iface.markerArrayPub.publish(self.iface.markerArray)
        # Query planning and execution of single pose goal
        execute_pose_goal = input("Should the single pose goal be planned and executed? [yes | Enter]")
        if execute_pose_goal == "yes":
            self.iface.go_to_pose_goal(pose_goal)

    # Query if user wants to plan to a cartesian path. If so, plan it. Then, query for execution. If planning failed, exit.
    def queryCPP(self,col_pa):
        # Query planning of cartesian path
        execute_cart_path_goal_dec = input("Do you want to plan a cart. path from the waypoints? [yes | Enter]")
        if execute_cart_path_goal_dec == "yes":
            eef_step_size = 1.0 # 1m -> no interpolation, cartesian path will have as many points as pose vector in iface 
            # Plan a path
            (plan, suc_frac) = self.iface.move_group.compute_cartesian_path(col_pa.poses, float(eef_step_size), 0.0) # last argument: jump_threshold -> not used
            # Inform user of success fraction
            print("Sucess fraction: ",suc_frac)
            if suc_frac == 1.0:
                # Display the plan
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.iface.robot.get_current_state()
                display_trajectory.trajectory.append(plan)
                # Publish the plan
                self.iface.display_trajectory_publisher.publish(display_trajectory)
                # Query decision to execute cart. plan
                exec_dec = input("Do you want to execute the cart. plan? [yes | Enter]")
                if exec_dec == "yes":
                    self.iface.move_group.execute(plan, wait=True) # Waits until feedback from execution is received
                # Query save
                self.querySave(col_pa)
            else:
                print("Planning failed")            

    # Query the user for the name of json file containing a poseArray, then query saving of cart. path, then query execution of cart. path
    def loadWaypointsFromJSON(self):
        file_name = input("Please input name of json file to load: ")
        path_to_current_dir = str(pathlib.Path().resolve()) # The path gets loaded from the moveit workspace top folder
        path_name = path_to_current_dir + "/" + file_name + '.json' 
        with open(path_name, 'rb') as f:
                self.appending = True
                # Get poseArray data from json object
                jsonOjbect = json.load(f)
                loaded_pose_array = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect) 
                # Save loaded poseArray in interface
                self.iface.loaded_json_wpoints = loaded_pose_array 
                # Create marker for all poses in poseArray
                for pose in loaded_pose_array.poses:
                    self.iface.createMarker(pose,'loaded_wpoint')
                self.iface.markerArrayPub.publish(self.iface.markerArray)
                print("Waypoints were loaded and are being displayed")
                # Query if user wishes to continue
                cont_ans = input("Do you wish to continue? [yes | Enter]")
                if cont_ans == "yes":
                    # Query adding of new waypoints
                    question = "Do you want to add new waypoints to the file "+file_name+" ? [yes | Enter]"
                    edit_ans = input(question)
                    if edit_ans == "yes":
                        # If no waypoints are loaded, inform user and exit
                        if self.iface.loaded_json_wpoints == None:
                            print("No waypoints were loaded!")
                        else:
                            # If a poseArray was loaded, give these waypoints to the collect_wpoints_in_gui method, 
                            # which will append new waypoints and return a poseArray with all old poses + all new poses
                            existing_wpoints = self.iface.loaded_json_wpoints
                            # The coutner counting the existing waypoints in the waypoints array is incremented by one, 
                            #since the counter refers to the index refers to the next waypoint to be added
                            n = len(existing_wpoints.poses) + 1
                            extended_waypoints = self.collect_wpoints_in_gui(n)
                            if extended_waypoints:
                                # Query saving of waypoints
                                self.querySave(extended_waypoints)
                                # Query decision to plan car. path
                                self.queryCPP(extended_waypoints)
                            else:
                                print("No wpoints addded.")
                                return None
                    else:
                        # Query decision to plan car. path
                        self.queryCPP(loaded_pose_array)

    # Query a valid 'mode' from the user, where mode is one of the scripts functionalities: 
    # {single pose goal, hardcoded trajectory,  collecting waypoints,loading and editing waypoints, exit}
    def queryValidMode(self):
        print("")
        print("What do you want to do? : ")
        print("[1]  Display the (hard coded) single pose goal")
        print("[2]  Display the (hard coded) trajectory")
        print("[3]  Collect waypoints for a cartesian path")
        print("[4]  Load and edit waypoints from a specific json file ")
        print("[5]  Exit")
        print("")

        mode = -1
        while True:
            mode_in = input("Select mode [1 | 2 | 3 | 4 | 5]: ")
            try:
                mode = int(mode_in)
                if mode not in (1,2,3,4,5):
                    print("Unknown mode")
                else:
                    break
            except ValueError:
                print("Invalid input")
        return mode

    # Clear terminal and greet user
    def greet(self):
        os.system('clear') # Clear terminal 
        print("")
        print("----------------------------------------------------------")
        print("Monkey Interface")
        print("----------------------------------------------------------")



def main():
    try:
        # Create interface to Moveit:RobotCommander (move-group python interface)   
        interface = MoveGroupInterface()

        # Create helper class
        helper = Utils(interface)

        # Greet user
        helper.greet()

        # Query user for a valid mode
        mode = helper.queryValidMode()

        if mode == 1: # Plan display execute hard coded pose goal

            # Plan and display trajectory to hard coded pose goal, query user for execution
            helper.pdExamplePoseGoal() 
        elif mode == 2: # Plan display hard coded trajectory, query for execution
            # Plan and display trajectory based on hardcoded waypoints
            helper.pdExampleWaypoints()

        elif mode == 3: # Let user collect waypoints in gui, query save and finally query execution

            # Collect waypoints in gui
            collected_pose_array = helper.collect_wpoints_in_gui(1) 
            if collected_pose_array:
                # Ask user if he wants to save
                helper.querySave(collected_pose_array)
                    
                # Make sure that all user set waypoints are being displayed
                interface.markerArrayPub.publish(interface.markerArray)

                # Query decision to plan car. path
                helper.queryCPP(collected_pose_array)
            
        elif mode == 4: # Load waypoints from json, potentially edit them

            # Query json file name, load poseArray, query appending new poses     
            helper.loadWaypointsFromJSON()

        elif mode == 5: # Exit
            pass

        else:
            print("Unrecognized mode")

        # Before exiting
        print("Exiting")

        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
