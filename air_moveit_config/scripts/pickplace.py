#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit_commander.roscpp_initializer import roscpp_initialize
from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, TwistStamped
import time
import rospy


roscpp_initialize("pickplace")
rospy.loginfo("Z<OERJGHSDFKJSDFJKASDJKLFASJKLDFJKLASDFJKLKASJDFJKLASDLF")
# [pickAndPlaceTut1]
# Specify robot parameters
arm = "air"
eef = "gripper"
# [pickAndPlaceTut1]

# [pickAndPlaceTut2]
# Specify object parameters
object_name = "grasp_object"
object_radius = 0.02

# Start with a clear planning scene
psi = PlanningSceneInterface(synchronous=True)
psi.remove_world_object()

# [initCollisionObject]
# Grasp object properties
objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.orientation.w = 1.0
objectPose.pose.position.x = 0.55
objectPose.pose.position.y = -0.25
objectPose.pose.position.z = 0.82
# [initCollisionObject]

# Add the grasp object to the planning scene
psi.add_cylinder(object_name, objectPose, height=0.2, radius=0.01)
# [pickAndPlaceTut2]

# [pickAndPlaceTut3]
# Create a task
task = core.Task("PandaPickPipelineExample")
task.enableIntrospection()
# [pickAndPlaceTut3]

# [pickAndPlaceTut4]
# Start with the current state
task.add(stages.CurrentState("current"))

# [initAndConfigConnect]
# Create a planner instance that is used to connect
# the current state to the grasp approach pose
pipeline = core.PipelinePlanner()
pipeline.planner = "RRTConnect"
planners = [(arm, pipeline)]

# Connect the two stages
task.add(stages.Connect("connect1", planners))
# [initAndConfigConnect]
# [pickAndPlaceTut4]
  
# [initAndConfigGenerateGraspPose]
# The grasp generator spawns a set of possible grasp poses around the object
grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
grasp_generator.angle_delta = 0.3
grasp_generator.pregrasp = "open"
grasp_generator.grasp = "closed"
grasp_generator.setMonitoredStage(task["current"])  # Generate solutions for all initial states
# [initAndConfigGenerateGraspPose]
# [pickAndPlaceTut5]

# [pickAndPlaceTut6]
# [initAndConfigSimpleGrasp]
# SimpleGrasp container encapsulates IK calculation of arm pose as well as finger closing
simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")
# Set frame for IK calculation in the center between the fingers
ik_frame = PoseStamped()
ik_frame.header.frame_id = "qbhand_base_link"
ik_frame.pose.position.y = 0.04
ik_frame.pose.position.z = 0.13
ik_frame.pose.orientation.w= 1
ik_frame.pose.orientation.x=-0.5646338
ik_frame.pose.orientation.y=-0.5646338
ik_frame.pose.orientation.z=0.5646338
simpleGrasp.setIKFrame(ik_frame)
# [initAndConfigSimpleGrasp]
# [pickAndPlaceTut6]

# [pickAndPlaceTut7]
# [initAndConfigPick]
# Pick container comprises approaching, grasping (using SimpleGrasp stage), and lifting of object
pick = stages.Pick(simpleGrasp, "Pick")
pick.eef = eef
pick.object = object_name

# Twist to approach the object
approach = TwistStamped()
approach.header.frame_id = "world"
approach.twist.linear.z = -1.0
pick.setApproachMotion(approach, 0.00, 0.25) # (approach, 0.03, 0.1)

# Twist to lift the object
lift = TwistStamped()
lift.header.frame_id = "qbhand_base_link"
lift.twist.linear.z = - 1.0
pick.setLiftMotion(lift, 0.00, 0.25) # (lift, 0.03, 0.1)
# [pickAndPlaceTut7]

# [pickAndPlaceTut8]
# Add the pick stage to the task's stage hierarchy
task.add(pick)
# [initAndConfigPick]
# [pickAndPlaceTut8]

# [pickAndPlaceTut9]
# Connect the Pick stage with the following Place stage

if task.plan():
    try:
        task.publish(task.solutions[0])
    except:
        print("Can't publish task")    


# avoid ClassLoader warning
del pipeline
del planners
# [pickAndPlaceTut13]

# Prevent the program from exiting, giving you the opportunity to inspect solutions in rviz
time.sleep(120)
