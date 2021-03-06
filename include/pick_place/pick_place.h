#pragma once
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <iostream>

#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/RemoveObject.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

class MovePickPlace
{
public:
    MovePickPlace(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group);
    void spin();
    
private:
    moveit_msgs::MoveItErrorCodes pick(geometry_msgs::Pose pose);
    moveit_msgs::MoveItErrorCodes place(geometry_msgs::Pose pose);
    bool setGenActuator();
    void rmObject();
    void showObject(geometry_msgs::Pose pose);
    void objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg);
    moveit_msgs::MoveItErrorCodes CartesianPath(geometry_msgs::Pose pose, bool second_plan);
    // 行人检测
    void subCallback(const std_msgs::Bool::ConstPtr &msg);
    geometry_msgs::PoseStamped TransformListener(geometry_msgs::PoseStamped pose);
    moveit_msgs::MoveItErrorCodes planMove(geometry_msgs::Pose& pose);

    void setPoses();
    void setPose(const std::string& path, geometry_msgs::Pose& pose);
    // friend void operator >> (const YAML::Node& doc, geometry_msgs::Pose& pose);
    void assignment(const YAML::Node& doc, geometry_msgs::Pose& pose);

    void setConstraints();
    void clearConstraints();

    ros::NodeHandle& nh;
    moveit::planning_interface::MoveGroupInterface& move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ros::ServiceClient list_generator_client;
    ros::ServiceClient set_gen_actuator_client;
    ros::ServiceClient list_actuator_client;
    ros::ServiceClient show_object_client;
    ros::ServiceClient remove_object_client;
    ros::ServiceClient open_gripper_client;
    ros::ServiceClient close_gripper_client;
    ros::Publisher pick_pose_pub;
    ros::Publisher place_pose_pub;
    // 抓取姿态
    ros::Subscriber pose_sub;
    
    // 行人检测
    ros::Subscriber detetor_sub;
    int intent;
    int object;
    int target;
    geometry_msgs::Pose place_pose1;
    geometry_msgs::Pose place_pose2;
    geometry_msgs::Pose place_pose3;
    std::vector<geometry_msgs::Pose> place_poses;
    //
    std::vector<geometry_msgs::Pose> pick_poses;
};