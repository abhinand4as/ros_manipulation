/*********************************************************************
* work_scene.cpp
* This code includes different method for add collision object in moveit.
* 
*
* 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
* 
* 
*********************************************************************/


#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

void collision_object_mg()
{

/**
 * @brief Method 1: Collision object is added by using move group and planning scene interface.
 * @brief planning_scene_interface.addCollisionObjects()
 */
	
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "root";	

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.4;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.4;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.6;
	box_pose.position.y = -0.4;
	box_pose.position.z =  1.2;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	
	ROS_INFO("Add an object into the world");
	moveit::planning_interface::MoveGroupInterface move_group("arm");
	planning_scene_interface_.addCollisionObjects(collision_objects);
	/* Sleep so we have time to see the object in RViz */
	sleep(2.0);

}

void collision_object_diff(ros::NodeHandle &nh)
{
	
/**
 * @brief Method 2: Collision object is added by using planning scene diff and "/collision_object" topic.
 * 
 */
	
	// Define publisher to update work scene
    ros::Publisher pub_co_ = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
     
    //~ //Wait until workscene is published successfully
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
 
	//Table
	
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "table";	
    collision_objects.header.frame_id = "root";
    
    //Remove Table
	collision_objects.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(collision_objects);
    
	// Define table plane
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 1.6; // x
    table.dimensions[1] = 0.8; // y
    table.dimensions[2] = 0.03; // z
    
    // Define table position
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table.dimensions[0]/2.0 - 0.15;
    table_pose.position.y = table.dimensions[1]/2.0 - 0.08;
    table_pose.position.z =  -table.dimensions[2]/2.0;

    // Define collision objects
    ros::WallDuration(1).sleep();
    collision_objects.primitives.push_back(table);
    collision_objects.primitive_poses.push_back(table_pose);
    collision_objects.operation = collision_objects.ADD;
	
	//Adding objects to the work scene
    ROS_INFO("Adding the all objects to the work scene.");
    pub_co_.publish(collision_objects);
	moveit_msgs::PlanningScene work_scene;
    work_scene.world.collision_objects.push_back(collision_objects);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);

    ros::WallDuration(1).sleep();
    
	
	
}

void collision_object_vec()
{
	
/**
 * @brief Method 3: Collision object is added by using planning scene interface.
 * @brief planning_scene_interface.applyCollisionObjects()
 * 
 */
	
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(1);
	
	// OBJECT

	// Define the object that we will be manipulating
	collision_objects[0].header.frame_id = "world";
	collision_objects[0].id = "object";

	/* Define the primitive and its dimensions. */
	collision_objects[0].primitives.resize(1);	
	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
	collision_objects[0].primitives[0].dimensions.resize(2);
	collision_objects[0].primitives[0].dimensions[0] = 0.18;  //height 
	collision_objects[0].primitives[0].dimensions[1] = 0.03;  //radius

	/* Define the pose of the object. */
	collision_objects[0].primitive_poses.resize(1);
	collision_objects[0].primitive_poses[0].position.x = 0.5;
	collision_objects[0].primitive_poses[0].position.y = 0.5;
	collision_objects[0].primitive_poses[0].position.z = 0.5;
	collision_objects[0].operation = collision_objects[0].ADD;
	
	planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void attatched_object(ros::NodeHandle &nh)
{
	
/**
 * @brief Collision object is added by using planning scene diff and "/collision_object" topic.
 * 
 */
	
	// Define publisher to update work scene
    ros::Publisher pub_co_ = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
     
    //~ //Wait until workscene is published successfully
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define A Cylinder
    shape_msgs::SolidPrimitive coca_can;
    coca_can.type = coca_can.CYLINDER;
    coca_can.dimensions.resize(2);
    coca_can.dimensions[0] = 0.13; // height
    coca_can.dimensions[1] = 0.036; // radius
    // Define coca can position
    geometry_msgs::Pose coca_can_pose;
    coca_can_pose.orientation.w = 1.0;
    coca_can_pose.position.x = 0.7;
    coca_can_pose.position.y = 0.0;
    coca_can_pose.position.z = coca_can.dimensions[0]/2;


    // Define attached objects
    moveit_msgs::AttachedCollisionObject attached_objects;
    attached_objects.link_name = "j2n6s300_end_effector";
    attached_objects.object.header.frame_id = "root";
    attached_objects.object.id = "cylinder";
    attached_objects.object.primitives.push_back(coca_can);
    attached_objects.object.primitive_poses.push_back(coca_can_pose);
    attached_objects.object.operation = attached_objects.object.ADD;


    //~ // Add all objects to environment
    ROS_INFO("Adding the all objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
    work_scene.world.collision_objects.push_back(attached_objects.object);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::WallDuration(1).sleep();
    	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "work_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    //~ // Display debug information in teminal
    //~ if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        //~ ros::console::notifyLoggerLevelsChanged();
    //~ }

	collision_object_diff(nh);
	collision_object_mg();
	collision_object_vec();
	attatched_object(nh);	

    ros::shutdown();
    return 0;
}
