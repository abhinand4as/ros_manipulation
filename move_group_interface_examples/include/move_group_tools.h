#ifndef MOVE_GROUP_TOOLS_H
#define MOVE_GROUP_TOOLS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_group_tools
{
    static const std::string PLANNING_GROUP = "panda_arm";

    class MoveGroupTools
    {
    public:
        MoveGroupTools();

        void goToEefPose(const geometry_msgs::Pose& eef_pose);
        // void goToJointPose();
        // void cartesianWaypoints();
        // void addCollisionObjects();
        void planAndExecuteGoal();
        // void getRobotInfo();
        // void publishLabelHelper();

    private: 
        ros::NodeHandle nh_;

        moveit::planning_interface::MoveGroupInterface* move_group;
        moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
        const robot_state::JointModelGroup* joint_model_group;

        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        Eigen::Isometry3d text_pose;

        moveit_msgs::CollisionObject collision_object;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    };
}



#endif