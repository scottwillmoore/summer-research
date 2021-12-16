#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/  planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectroy.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    // initialise
    ros::init(argc, argv, "move_baxter");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(20.0);

    // define movegroup and planning scene
    moveit::planning_interface::MoveGroup group("right_arm");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // print infor
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("End effector: %s", group.getEndEffectorLink().c_str());

    // create pose goal
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    group.setPoseTarget(target_pose1);

    // visualise plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualising plan 1 (pose goal) %s", success?"":"FAILED");
    sleep(5.0)

    // move to pose goal
    group.move()
}