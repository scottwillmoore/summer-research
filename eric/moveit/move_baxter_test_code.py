#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import pi

def main():
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    state_topic = ['state:=/robot/state']
    moveit_commander.roscpp_initialize(state_topic)

    rospy.init_node('get_info', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "left_arm"
    left_arm = moveit_commander.MoveGroupCommander(group_name)

    both_arms = moveit_commander.MoveGroupCommander('both_arms')

    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', 
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = left_arm.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = left_arm.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    print "============ Printing group pose"
    print "=== left arm pose"
    print left_arm.get_current_pose()
    #print "=== both arms pose"
    #print both_arms.get_current_pose()

    print "============ Printing joint values"
    joint_goal = left_arm.get_current_joint_values()
    print "current joint values are: %s" % joint_goal
    print "both arms: %s" % both_arms.get_current_joint_values()

    # Move left arm by specifying joint values
    print "============ Moving arm" 
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = pi/2
    joint_goal[4] = pi/2
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    left_arm.go(joint_goal, wait=True)

    left_arm.stop()

    # Create new MoveGroupCommander for right arm
    right_arm = moveit_commander.MoveGroupCommander('right_arm')

    # Move right arm by specifying pose values
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    right_arm.set_pose_target(pose_goal)

    plan = right_arm.go(wait=True)

    right_arm.stop()

    right_arm.clear_pose_targets()

    rospy.spin()
 
if __name__ == '__main__':
    main()
