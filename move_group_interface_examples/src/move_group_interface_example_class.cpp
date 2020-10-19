#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;

namespace move_group_interface_example
{
    static const std::string PLANNING_GROUP = "panda_arm";  

    class SimpleTasks
    {
    public: 

        SimpleTasks() {

            move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
            planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface;

            joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("panda_link0"));

            text_pose = Eigen::Isometry3d::Identity();
            text_pose.translation().z() = 1.0;
            visual_tools_->publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
            visual_tools_->trigger();


            // goToEefPose();
            // ros::Duration(1.0).sleep();
            // move_group->setStartStateToCurrentState();
            // move_group->setNamedTarget("ready");
            // planAndExecuteGoal();

            // // move_group->setMaxAccelerationScalingFactor(0.1);
            // move_group->setMaxVelocityScalingFactor(0.1);
            // goToEefPose();
            // goToJointPose();
            // cartesianWaypoints();
            // addCollisionObjects();  
            // getRobotInfo();
            // ROS_INFO_STREAM("Reference Frame: " << move_group->getPoseReferenceFrame());
            move_group->allowReplanning(true);
            move_group->setNumPlanningAttempts(10);

            positionTarget();
            

          
        }


        ~SimpleTasks() {
    
            delete move_group;
            delete planning_scene_interface_;
        }

        void positionTarget() {
            
            move_group->setStartState(*move_group->getCurrentState());

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "panda_link8";
            pose.pose.orientation.w = 1.0;
            pose.pose.position.z = 0.1;

            // move_group->clearPoseTargets();
            // move_group->setPoseTarget(pose);
            move_group->setJointValueTarget(pose);
            // move_group->setPlannerId("RRTConnectkConfigDefault");
            // move_group->setNumPlanningAttempts(4);
            // move_group->setPlanningTime(20);
            // move_group->setGoalPositionTolerance(1e-3); // meters
            // move_group->setGoalOrientationTolerance(1e-2);
            visual_tools_->setBaseFrame("panda_link8");
            visual_tools_->publishAxis(pose.pose);
            // visual_tools_->publishArrow(pose, rvt::GREEN);
            visual_tools_->trigger();
            planAndExecuteGoal();
            
        }
        void goToEefPose() {

            move_group->setStartState(*move_group->getCurrentState());
            geometry_msgs::Pose another_pose;
            another_pose.orientation.w = 1.0;
            another_pose.position.x = 0.5;
            another_pose.position.y = 0.3;
            another_pose.position.z = 0.5;
            move_group->setPoseTarget(another_pose);

            planAndExecuteGoal();
        }

        void goToJointPose() {

            moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    
            // Next get the current set of joint values for the group.
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
            joint_group_positions[0] = 2.0;  // radians
            move_group->setJointValueTarget(joint_group_positions);

            move_group->setStartState(*move_group->getCurrentState());

            planAndExecuteGoal();

        }
        void cartesianWaypoints() {

            geometry_msgs::Pose start_pose2;
            start_pose2.orientation.w = 1.0;
            start_pose2.position.x = 0.55;
            start_pose2.position.y = -0.05;
            start_pose2.position.z = 0.8;

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(start_pose2);

            geometry_msgs::Pose target_pose3 = start_pose2;

            target_pose3.position.z -= 0.2;
            waypoints.push_back(target_pose3);  // down

            target_pose3.position.y -= 0.2;
            waypoints.push_back(target_pose3);  // right

            target_pose3.position.z += 0.2;
            target_pose3.position.y += 0.2;
            target_pose3.position.x -= 0.2;
            waypoints.push_back(target_pose3);  // up and left

            // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
            // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
            // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
            move_group->setMaxVelocityScalingFactor(0.1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


            // Visualize the plan in RViz
            {
            visual_tools_->deleteAllMarkers();
            visual_tools_->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools_->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < waypoints.size(); ++i)
                visual_tools_->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
            visual_tools_->trigger();
            }
            move_group->execute(trajectory);

        }
        
        void addCollisionObjects() {
            
            collision_object.header.frame_id = move_group->getPlanningFrame();

            collision_object.id = "Box1";

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.4;
            primitive.dimensions[1] = 0.1;
            primitive.dimensions[2] = 0.4;

            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.4;
            box_pose.position.y = -0.2;
            box_pose.position.z = 0.45;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            std::vector<moveit_msgs::CollisionObject> collision_objects;
            collision_objects.push_back(collision_object);

            planning_scene_interface_->addCollisionObjects(collision_objects);

            move_group->setStartState(*move_group->getCurrentState());
            geometry_msgs::Pose target_pose;
            target_pose.orientation.x = 0.923919;
            target_pose.orientation.y = -0.382587;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 0.0;
            target_pose.position.x = 0.3;
            target_pose.position.y = -0.4;
            target_pose.position.z = 0.5;

            move_group->setPoseTarget(target_pose);

            bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Planning Cartesian Waypoint goal : %s", success ? "SUCCESS" : "FAIL");

            move_group->execute(my_plan);

            visual_tools_->deleteAllMarkers();
            move_group->attachObject(collision_object.id);
            publishLabelHelper(text_pose, "Attach Object");
            ros::Duration(1.0).sleep();

            visual_tools_->deleteAllMarkers();
            move_group->detachObject(collision_object.id);
            publishLabelHelper(text_pose, "Detach Object");
            ros::Duration(1.0).sleep();

            visual_tools_->deleteAllMarkers();
            std::vector<std::string> object_ids;
            object_ids.push_back(collision_object.id);
            planning_scene_interface_->removeCollisionObjects(object_ids);
            publishLabelHelper(text_pose, "Remove Collision Object");


        }

        void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label) {

            Eigen::Isometry3d pose_copy = pose;
            pose_copy.translation().x() -= 0.5;
            visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
            visual_tools_->trigger();
        }

        void planAndExecuteGoal() {

            bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
            ros::Duration(1.0).sleep();
            move_group->execute(my_plan);
        }
        void getRobotInfo() {

            ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group->getPlanningFrame().c_str());

            // We can also print the name of the end-effector link for this group.
            ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

            // We can get a list of all the groups in the robot:
            ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
            std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
                        std::ostream_iterator<std::string>(std::cout, ", "));

            //Active Joints
            ROS_INFO("\nActive Joints: ");
            auto active_joints = move_group->getActiveJoints();
            for (auto v : active_joints)
                ROS_INFO_STREAM(v);

            // another method TODO:
            // std::copy(move_group->getActiveJoints().begin(), move_group->getActiveJoints().end(), 
            //             std::ostream_iterator<std::string>(std::cout, "\n"));
            
            //Default Planner ID
            ROS_INFO_STREAM("Default Planner ID: " << move_group->getDefaultPlannerId());

            //Goal Joint Tolerance
            ROS_INFO_STREAM("Goal Joint Tolerance: " << move_group->getGoalJointTolerance());

            //Goal Orientation Tolerance
            ROS_INFO_STREAM("Goal Orientation Tolerance: " << move_group->getGoalOrientationTolerance());

            //Goal Position Tolerance
            ROS_INFO_STREAM("Goal Position Tolerance: " << move_group->getGoalPositionTolerance());

            ROS_INFO_STREAM("Pose Reference Frame : " << move_group->getPoseReferenceFrame());

            ROS_INFO_STREAM("EEF LINK: " << move_group->getEndEffectorLink());

        }

        


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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_example");
  ROS_INFO_STREAM("move_group_interface_example");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  move_group_interface_example::SimpleTasks task1;

  ROS_INFO_STREAM("Shutting down.");
  
  return 0;
}