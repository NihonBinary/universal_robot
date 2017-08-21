#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# このサンプルは、以下のURLから参照可能です。
# http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
#
# このコードの実行時にエラーが生じる場合には、以下のパッチをcore.plに適用する必要があります。
# https://launchpadlibrarian.net/263969718/patch.txt
#
#

import sys
import copy
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



# moveit_commanderとrospyを初期化
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#　RobotCommandarオブジェクトをインスタンス化
# RobotCommandar：ロボット全体へのインターフェイス
robot = moveit_commander.RobotCommander()

#　PlanningSceneInterfaceオブジェクトをインスタンス化
#　PlanningSceneInterface:　ロボットの周辺環境へのインターフェイス
scene = moveit_commander.PlanningSceneInterface()

#　MoveGroupCommandarオブジェクトをインスタンス化
#　MoveGroupCommander： ジョイントグループへのインターフェイス
# アームの動作のプランと実行時に使用する
group = moveit_commander.MoveGroupCommander("manipulator")

# DisplayTrajectoryパブリッシャーを作成
#　RVIZへの可視化に使用する
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

# Cartesian Paths
print "*** Cartesian Paths ***"

waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.x = waypoints[0].orientation.x
wpose.orientation.y = waypoints[0].orientation.y
wpose.orientation.z = waypoints[0].orientation.z
wpose.orientation.w = waypoints[0].orientation.w
wpose.position.x = waypoints[0].position.x + 0.20
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.20
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y -= 0.20
waypoints.append(copy.deepcopy(wpose))

(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)

print "============ Go with plan3..."
group.execute(plan3)
#group.go(wait=True)

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)

#collision_object = moveit_msgs.msg.CollisionObject()

moveit_commander.roscpp_shutdown()







