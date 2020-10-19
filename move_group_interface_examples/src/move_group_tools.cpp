#include <move_group_tools.h>

namespace rvt = rviz_visual_tools;

namespace move_group_tools
{
    MoveGroupTools::MoveGroupTools() {

        move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface;

        joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("panda_link0"));

        text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools_->publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();

    }
    void MoveGroupTools::goToEefPose(const geometry_msgs::Pose& eef_pose) {

        move_group->setStartState(*move_group->getCurrentState());
        move_group->setPoseTarget(eef_pose);

        planAndExecuteGoal();
        
    }

    void MoveGroupTools::planAndExecuteGoal() {

        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
        ros::Duration(1.0).sleep();

        move_group->execute(my_plan);
    }
}
