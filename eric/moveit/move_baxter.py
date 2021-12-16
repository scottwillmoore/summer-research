#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import pi

class MoveBaxter(object):
    def __init__(self):

        # setup
        # initialise commander with remapped topics
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # instantiate rospy node
        rospy.init_node('move_baxter', anonymous=True)

        # instantiate moveit_commander objects
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.both_arms = moveit_commander.MoveGroupCommander('both_arms')
        self.left_hand = moveit_commander.MoveGroupCommander('left_hand')
        self.right_hand = moveit_commander.MoveGroupCommander('right_hand')

        # publisher for publishing trajectories to Rviz
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

    def display_robot_info(self):
        print("============ Robot groups: %s" % self.robot.get_group_names())

        # print("============ Robot state: ")
        # print(self.robot.get_current_state())
        # print("")

    def display_group_info(self, group):
        print("============ Group name: %s" % group.get_name())

        print("============ Group reference frame: %s" % group.get_planning_frame())

        print("============ Group pose reference frame: %s" % group.get_pose_reference_frame())

        print("============ Group end effector: %s" % group.get_end_effector_link())

        print("============ Group pose:")
        print(group.get_current_pose())
        print("")

        print("============ Group joint values:")
        print(group.get_current_joint_values())
        print("")
    
    def go_to_joint_state(self, group, joint_goal):
        print("Attempting to move arm ...")
        # joint_goal should be vector of length 7 for 7 DoF arm
        
        # move to specified joint goal
        group.go(joint_goal, wait=True)

        # halt any residual movements
        group.stop()

        print("Finished attempt")

    def go_to_pose_goal(self, group, pose_goal):
        print("Attempting to move arm ...")
        # pose_goal = geometry_msgs.msg.Pose()

        group.set_goal_tolerance(0.01)
        group.set_planner_id('ESTkConfigDefault')
        group.allow_replanning(True)
        group.set_num_planning_attempts(3)

        # specify pose goal for group
        group.set_pose_target(pose_goal)

        # move to specified pose goal
        group.go(wait=False)

        # halt any residual movements
        group.stop()

        # clear pose goal for group
        group.clear_pose_targets()

        print("Finished attempt")

    def both_arms_pose_goal(self, left_pose_goal, right_pose_goal, left_end_effector_link, right_end_effector_link):
        print("Attempting to move arm ...")
        # pose_goal = geometry_msgs.msg.Pose()

        self.both_arms.set_goal_tolerance(0.01)
        self.both_arms.set_planner_id('ESTkConfigDefault')
        self.both_arms.allow_replanning(True)
        self.both_arms.set_num_planning_attempts(3)

        # specify pose goal for group
        self.both_arms.set_pose_target(left_pose_goal, left_end_effector_link)
        self.both_arms.set_pose_target(right_pose_goal, right_end_effector_link)

        # move to specified pose goal
        self.both_arms.go(wait=False)

        # halt any residual movements
        self.both_arms.stop()

        # clear pose goal for group
        self.both_arms.clear_pose_targets()

        print("Finished attempt")

    def get_pose_goal():
        return

def main():
    try:
        baxter = MoveBaxter()

        baxter.display_robot_info()

        # while True:
        #     print(baxter.left_arm.get_current_pose())

        print(baxter.both_arms.get_joints())

        baxter.display_group_info(baxter.left_arm)

        print(baxter.both_arms.get_end_effector_link())

        # set joint goal
        # joint_goal = baxter.left_arm.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -pi/4
        # joint_goal[2] = 0
        # joint_goal[3] = pi/2
        # joint_goal[4] = pi/2
        # joint_goal[5] = pi/3
        # joint_goal[6] = 0

        # move by specifying joint values
        # baxter.go_to_joint_state(baxter.right_arm, joint_goal)

        right_pose_goal = geometry_msgs.msg.Pose()
        right_pose_goal.position.x = 0.83#0.7#0.4
        right_pose_goal.position.y = -0.55#-0.12#0.12
        right_pose_goal.position.z = 0.18#-0.11#0.4
        right_pose_goal.orientation.x = 0.67
        right_pose_goal.orientation.y = 0.48
        right_pose_goal.orientation.z = 0.55
        right_pose_goal.orientation.w = 0.13

        # baxter.go_to_pose_goal(baxter.right_arm, right_pose_goal)

        # set pose goal
        left_pose_goal = geometry_msgs.msg.Pose()
        left_pose_goal.position.x = 0.7#0.7#0.4
        left_pose_goal.position.y = -0.12#-0.12#0.12
        left_pose_goal.position.z = -0.11#-0.11#0.4
        left_pose_goal.orientation.x = 0.984479463478
        left_pose_goal.orientation.y = 0.175397977365
        left_pose_goal.orientation.z = 0.00106214234588
        left_pose_goal.orientation.w = 0.00588280380328

        # move by specifying pose values
        # baxter.go_to_pose_goal(baxter.left_arm, left_pose_goal)


        baxter.both_arms_pose_goal(left_pose_goal, right_pose_goal, "left_gripper", "right_gripper")


        moveit_commander.os._exit(0)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
 
if __name__ == '__main__':
    main()
