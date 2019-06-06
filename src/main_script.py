#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8
from edo_core_msgs.msg import JointInit
import rospkg
import time
import json
from os.path import join
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list


try:
    import RPi.GPIO as GPIO
except ImportError:
    gpio_available = False
else:
    gpio_available = True

class MoveEdo(object):

    PIN_LED_INDUSTRIEL = 2 # OUT
    PIN_BUTTON_INDUSTRIEL= 3 #IN

    PIN_LED_MILITAIRE = 4
    PIN_BUTTON_MILITAIRE = 15

    #PIN_LED_SEXUEL = None
    #PIN_BUTTON_SEXUEL= None

    #PIN_LED_JUDICIAIRE = None
    #PIN_BUTTON_JUDICIAIRE = None

    #PIN_LED_SOCIAL = None
    #PIN_BUTTON_SOCIAL = None

    #PIN_LED_VOITURE = None
    #PIN_BUTTON_VOITURE = None


  def __init__(self):
        super(MoveEdo, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('sub_test', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the edo robot, so we set the group's name to "edo".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "edo"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

   
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
 

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        if gpio_available:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.PIN_BUTTON_INDUSTRIEL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.PIN_BUTTON_MILITAIRE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.PIN_LED_INDUSTRIEL, GPIO.OUT)
            GPIO.setup(self.PIN_LED_MILITAIRE, GPIO.OUT)
        else:
            rospy.logwarn("Node hasn't found the GPIO, buttons will not work")
        rospy.loginfo("Buttons node is started!")

    
  
  def robot_move(self,str):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        pub=rospy.Publisher("/gripper_state",Bool,queue_size=1)
        move_group = self.move_group
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        rospack = rospkg.RosPack()

        with open(join(rospack.get_path("cs_avenir"), "config/move.json")) as f:
            poses = json.load(f)
        
        cpt = 0

        gripper_state=poses[str]["gripper_open"]
        for move in poses[str]["joints"]:

            #rospy.loginfo(gripper_state[cpt])
            

            
            for i in range(6):
                joint_goal[i]=move[i]
                if gripper_state[cpt]==3:
                    pub.publish(0) # close gripper
                    time.sleep( 1 ) # wait 6*1secondes

                else:
                    pub.publish(gripper_state[cpt])

            
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
            move_group.go(joint_goal, wait=True)
            cpt+=1
        
       
        # Calling ``stop()`` ensures that there is no residual movement
            move_group.stop()
        


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    action = ["sexuel","industriel","judiciaire","social","voiture","militaire"]
    robot = MoveEdo()

    if data.data == None:
        pass
    else:
        print "lancer action "+ action[data.data]
        robot.robot_move(action[data.data])


def listen():

    rospy.init_node('sub_test', anonymous=True)
    machine_init = rospy.Publisher('machine_init', JointInit, queue_size=10, latch=True)
    # first param: mode 3 - manage collision thresold
    # second param: axes mask (63 affect all axes)
    # third param: current threshold  
    imsg = JointInit()        
    imsg.mode = 3
    imsg.joints_mask = 127
    imsg.reduction_factor = 100.0
    # publish init to remove collision 
    machine_init.publish(imsg)
    rospy.Subscriber("cs_avenir/buttons", UInt8, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listen()