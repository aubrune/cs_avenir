#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Bool
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
from moveit_commander.conversions import pose_to_list



class MoveEdo(object):
  """MoveEdoTutorial"""
  def __init__(self):
        super(MoveEdo, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cs_avenir_controller', anonymous=True)
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
        # the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        #print "============ Planning frame: %s" % planning_frame

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        #print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print "============ Printing robot state"
        #print robot.get_current_state()
        #print "============"
 

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.group_names = group_names


  
  def robot_move(self,str):
        # publisher that allows you know if there is a move in progress
        pub_move_in_progress= rospy.Publisher("/move_in_progress", Bool ,queue_size=1)

        # publisher that allows you to close or open the gripper
        pub_gripper_state = rospy.Publisher("/gripper_state", Bool ,queue_size=1)

        move_group = self.move_group

        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        
        #read the json file where the moves of the robot are saved
        rospack = rospkg.RosPack()
        with open(join(rospack.get_path("cs_avenir"), "config/move.json")) as f:
            poses = json.load(f)
        

        #to read the table of the gripper state
        cpt = 0

        gripper_state=poses[str]["gripper_open"]
        for move in poses[str]["joints"]:

            
            
            for i in range(6):
                joint_goal[i]=move[i]

                # to wait some second to alow the visitor to read the text on the card
                if gripper_state[cpt]==3:
                    pub_gripper_state.publish(0) # close gripper
                    time.sleep( 1 ) # wait 6*1 secondes

                else:
                    # publish if the gripper must be close or open 
                    pub_gripper_state.publish(gripper_state[cpt])

           
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            move_group.go(joint_goal, wait=True)
            cpt+=1
        
       
            # Calling ``stop()`` ensures that there is no residual movement
            move_group.stop()

        #########################################
        #########################################
        # publish false when the move is finished
        pub_move_in_progress.pubish(0)
        #########################################
        #########################################
        


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

    # refere to the buttons
    action = ["hotel","militaire","sexuel","judiciaire","social","voiture"]
    eDO = MoveEdo()

    if data.data == None:
        pass
    else:
        print "lancer action "+ action[data.data]
        eDO.robot_move(action[data.data])




def listen():

    #########################################
    #########################################
    # Subscribe to know if there is a curent move
    #rospy.Subscriber("/move_in_progress", Bool, callback_move)
    # if no current move :
    #  execute the move
    # else pass 
    #########################################
    #########################################

    rospy.init_node('cs_avenir_controller', anonymous=True)

    # to calibrate the collision system
    machine_init = rospy.Publisher('machine_init', JointInit, queue_size=10, latch=True)
    # first param: mode 3 - manage collision thresold
    # second param: axes mask (63 affect all axes)
    # third param: current threshold  
    imsg = JointInit()        
    imsg.mode = 3
    imsg.joints_mask = 127 # all but the gripper
    imsg.reduction_factor = 100.0
    # publish init to remove collision 
    machine_init.publish(imsg)

    # listen if a button is pushed and get the number 
    rospy.Subscriber("cs_avenir/buttons", UInt8, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    listen()