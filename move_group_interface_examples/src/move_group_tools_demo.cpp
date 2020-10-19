#include <ros/ros.h>
#include <move_group_tools.h>
#include <geometry_msgs/Pose.h>

namespace move_group_tools
{
    class MoveGroupDemo
    {
    public:
        MoveGroupDemo() {

            geometry_msgs::Pose eef_pose;
            eef_pose.orientation.w = 1.0;
            eef_pose.position.x = 0.5;
            eef_pose.position.y = 0.3;
            eef_pose.position.z = 0.4;
            move_group_tools_.goToEefPose(eef_pose);

        }
    
    private: 
        move_group_tools::MoveGroupTools move_group_tools_;

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_demo");

    ros::AsyncSpinner spinner(1);

    spinner.start();

    move_group_tools::MoveGroupDemo mydemo;

    return 0;

}