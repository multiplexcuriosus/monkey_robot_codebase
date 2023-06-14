#!/usr/bin/env python3

# Description =============================================================================================
'''
This script was written during and for the bachelor thesis "Assembly and Programming of a
Robot Monkey to Study Imitation Learning in Marmosets". 

It is intended to be used with the robot monkey assembled during the mentioned thesis. 
Once all setup steps as described on the ReadMe on the source repository and in the appendix of the thesis have been completed, 
this script can be run on e.g a Raspberry Pi to control the mentioned robot monkey or a different robot. 

This script launches a ROS node subscribed to the 'joint_states' topic which continuously listens to incoming target joint state data.
It then uses the newest data in 'joint_states' to update the servo motors duty cycles accordingly. 

The servo motor installed in the robot monkey during the thesis is the KST A12-T, whose rotational range can be covered by setting the duty cycle of a PWM signal to [4.5,10.5] %. 
The rotational range of the mechanical joint is 180° 
However, effects such as friction and varying tension make it such that the servos can often not pull hard enough at the threads to achieve the full ROM.
Thus, each Joint object in this script has its own so called predefined states, namely {min,middle,max,def}, which contain the duty cycle for a particular joint (and servo) for a particular state.
The states are not based on the duty cycle range based on the data sheet, but rather on the duty cycles which, when applied to the servos, 
lead to them e.g reaching their lower/upper rotational limit as permitted by the mentioned effects. Similarly, 
the middle and def value are the duty cycles for two other useful states, to which the user can then to refer to on a higher level. 
The 'def' state refers to the duty cycle the servo must have, such that the joint corresponding to the servo is at an angle such that the planning group of which the joint is part of (e.g left arm) is in its resting position.

To reduce jitter, the PiGPIOFactory from gpiozero is used. 
As a consequence, the final value written to the servo motor has to be in [-1,1].
To make use of the predefined states (especially min,max) mapping functions were created which take the following things into account:
- potentially inversed attachment of threads to servo wheel
- different angle ranges of joints (comes from URDF, which in turn comes from the URDF exporter from SolidWorks)

Author: Jaú Gretler
E-Mail: gretleja@ethz.ch
Last changed: 13.6.23

'''


# Imports ==================================================================================================
import rospy
import sys
from gpiozero import Servo, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from numpy import interp
factory = PiGPIOFactory() #joris approach
# Imports done =============================================================================================



# Joint class ==============================================================================================

# Class to store the duty cycles of the predefined states and who performs the mapping from duty cycle to [-1,1]

class joint:

    def __init__(self,_name,_motor_pin,_default_dc = 7.5,_min_dc = 4.5,_middle_dc = 7.5,_max_dc = 10.5) -> None:
        # Init name, motor pin
        self.name = _name
        self.motor_pin = _motor_pin

        # Map min,middle,max,default value from full duty cycle range [4.5,10.5] to [-1,1] (necessary for PiGPIOFactory())
        self.min_val = interp(_min_dc,[4.5,10.5],[-1,1])
        self.middle_val = interp(_middle_dc,[4.5,10.5],[-1,1])
        self.max_val = interp(_max_dc,[4.5,10.5],[-1,1])
        self.default_val = interp(_default_dc,[4.5,10.5],[-1,1])
        
        # Init motor (by using the factory pattern from the gpiozero lib we can significantly reduce the jitter)
        self.servo = Servo(self.motor_pin, initial_value=self.default_val, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory) 
    
    # Set joint servo to one of 4 pre defined states {0,0.5,1,def}
    def setState(self,_target_state):
        # Value to be written to servo
        next_val = 0
        if(_target_state == "def"):
            next_val =self.default_val
        elif(_target_state == "0"):
            next_val =self.min_val
        elif(_target_state == "0.5"):
            self.servo.value = self.middle_val
        elif(_target_state == "1"):
            next_val =self.max_val
        else:
            rospy.loginfo("Unknown target state")
            return
        
        # Log which value was set to which motor
        next_val_disp = '%.3f'% next_val
        rospy.loginfo("Setting %s to [itp %s]",self.name,next_val_disp)
        # Write value to motor
        self.servo.value = next_val
    
    # Sweep through predefined states
    def sweep(self):
        delay = 1.0
        self.setState("0")
        rospy.sleep(delay)
        self.setState("0.5")
        rospy.sleep(delay)
        self.setState("1")
        rospy.sleep(delay)
        self.setState("def")
        rospy.sleep(delay)

    


class Body:
    def __init__(self) -> None:

        self.joints = {}
        
        #init joints [name,pin,default,min,middle,max]

        #left arm

        LW_joint = joint("LW",3) #board 5
        
        LSH_joint = joint("LSH",4,4,4,6,7) #board 7
        LEB_joint = joint("LEB",17,10.5,10.5,8,6.75) #board 11
        LSL_joint = joint("LSL",27,10.5,10.5,7.5,4.5,) #board 13
        #LH_joint = joint("LH",22,0,0,0,0) #board 15
        LSF_joint = joint("LSF",10,6.25,6.25,7.5,10.5,) #board 19

        #right arm
        RW_joint = joint("RW",11) #board 23
        RSH_joint = joint("RSH",5,4.5,4.5,7,8) #board 29
        REB_joint = joint("REB",6,10,10,8,7) #board 31
        RSL_joint = joint("RSL",13,10,10,8,5.75) #board 33
        #RH_joint = joint("RH",2) #board 3
        
        RSF_joint = joint("RSF",26,6,6,7.5,10.5) #board 37
        

        #head
        NH_joint = joint("NH",9,8,6.25,8,10.5) #board 21
        NF_joint = joint("NF",19,10.5,10.5,7.5,4.5) #board 35

        # put joints in dict
        #left arm

        self.joints["LW"] = LW_joint
        
        self.joints["LSH"] = LSH_joint
        self.joints["LEB"] = LEB_joint
        self.joints["LSL"] = LSL_joint
        #self.joints["LH"] = LH_joint
        self.joints["LSF"] = LSF_joint
        
        #right arm
        self.joints["RW"] = RW_joint
        self.joints["RSH"] = RSH_joint
        self.joints["REB"] = REB_joint
        self.joints["RSL"] = RSL_joint
        #self.joints["RH"] = RH_joint
        
        self.joints["RSF"] = RSF_joint
        
        #head
        self.joints["NH"] = NH_joint
        self.joints["NF"] = NF_joint
    
        
    def allToDef(self):
        # set all servos to default position
        for k, j in self.joints.items():
            j.setState("def")
        rospy.sleep(1.0)







if __name__ == '__main__':

    rospy.init_node("test_servo_node") 
    rospy.loginfo("test servo node has started")

    mode = sys.argv[1]

    #create body objects which manages joints
    body = Body()

    #switch between different modes
    if mode == "single" or mode== "raw": #mode to set individual joint states
        target_joint = sys.argv[2]
        target_state = sys.argv[3] # {def,0,0.5,1}
        rospy.loginfo("Target joint: %s",target_joint)
        rospy.loginfo("Target state: %s",target_state)

        #set target state
        j = body.joints[target_joint]
        if mode == "single":
            j.setState(target_state)
        elif mode =="raw":
            j.servo.value = interp(float(target_state),[4.5,10.5],[-1,1])
        rospy.sleep(1.0)

    elif mode == "demo": #demo which iterates over all joints and all positions
        limb = sys.argv[2] #{L,R,N,all}
        rospy.loginfo("Starting Demo")
        for k, j in body.joints.items():
            if limb == "all" or j.name[0] == limb:
                j.sweep()

    
    elif mode =="wave": #demo which lets the left arm wave
        body.joints["LSL"].setState("0.5")
        rospy.sleep(1.0)
        body.joints["LSH"].setState("1")
        rospy.sleep(1.0)
        
        for i in range(3):
            body.joints["LEB"].setState("0")
            rospy.sleep(1.0)
            body.joints["LEB"].setState("1")
            rospy.sleep(1.0)

        rospy.sleep(1.0)
        body.allToDef()

    elif mode=="alldef": #set all joints to their default position
        body.allToDef()

    elif mode == "joris":
        target_joint = sys.argv[2]
        target_state = sys.argv[3] # [-1,1]
        J = body.joints[target_joint]
        J.servo.value = float(target_state)
        rospy.loginfo("Setting %s to %s",target_joint, target_state)

        
    else:
        rospy.loginfo("Invalid mode (first argument)")


    print ("Everything's cleaned up")





    