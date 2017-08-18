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

#　RVIZの起動を待ちます（ただ待つだけ）
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

#
# 基本情報の表示
#
print "Basic Information"
print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End Effector link: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"

# ゴールポーズへの計画を作成します
print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
pose_target.orientation.w = 0.0
pose_target.position.x = -0.120
pose_target.position.y = -0.431
pose_target.position.z = 0.146
group.set_pose_target(pose_target)
print "plan1 = group.plan()"
plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

#　RVIZへplan1の可視化を行います
print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

print "============ Waiting while plan1 is visualized (again)..."
rospy.sleep(5)

# 計画・実行を行い、実際にロボットを動かします
#print "============ Planning and Executing plan1..."
#group.go(wait=True)

# すでに計画済みの場合は、execute() で実行のみが可能です
#print "============ Executing plan1..."
#group.execute(plan1)


#
# 関節座標空間での移動
#
print "*** Planning to a joint-space goal ***"

#　既存のポーズターゲットなどをクリア
group.clear_pose_targets()

#　現在の関節位置を取得
group_variable_values = group.get_current_joint_values()
print "============ Joint values: ", group_variable_values

group_variable_values[0] = 1.0
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()

print "============ Waiting while RVIZ displays plan2..."
rospy.sleep(5)

# 計画・実行を行い、実際にロボットを動かします
#print "============ Planning and Executing plan2..."
#group.go(wait=True)

# すでに計画済みの場合は、execute() で実行のみが可能です
#print "============ Executing plan2..."
#group.execute(plan2)

#
# デカルト座標系での移動
#
print "*** Cartesian Paths ***"

waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))

(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)

# execute() で実行が可能です
#print "============ Executing plan3..."
#group.execute(plan3)

# オブジェクトの追加やアタッチ／デタッチ
collision_object = moveit_msgs.msg.CollisionObject()

#　終了時には、コマンダーをシャットダウンする
moveit_commander.roscpp_shutdown()







