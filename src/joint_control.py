#!/usr/bin/env python3.8
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Int32
## END_SUB_TUTORIAL
def go_to_joint_state(data):
    a=40#data.data
    b=60
    c=30
    d=40
    e=50
    f=40
    moveit_commander.roscpp_initialize(sys.argv)   
    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = float(a)*(3.1428/180)
    joint_goal[1] = float(b)*(3.1428/180)
    joint_goal[2] = float(c)*(3.1428/180)
    joint_goal[3] = float(d)*(3.1428/180)
    joint_goal[4] = float(e)*(3.1428/180)
    joint_goal[5] = float(f)*(3.1428/180)
    move_group.set_joint_value_target(joint_goal)

    #plan2 = move_group.plan()
    #move_group.go(wait=True)
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    #move_group.execute(joint_goal, wait=True)
    current_joints = move_group.get_current_joint_values()
    return True
def service(self):
      rospy.init_node('joint_control1')
      srv=rospy.Service('/joint_control1',Int32,go_to_joint_state)
      rospy.spin()
if __name__=="__main__":
  service()
