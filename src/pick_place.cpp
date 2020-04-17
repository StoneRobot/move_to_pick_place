#include "pick_place/pick_place.h"
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
#include <iostream>



PickPlace::PickPlace(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group)
:nh{_n},
move_group{group}
{
    ROS_INFO("init");
    remove_object_client = nh.serviceClient<hirop_msgs::RemoveObject>("removeObject");
    show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
    list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
    set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
    list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
    // gripper
    pick_pose_pub = nh.advertise<geometry_msgs::Pose>("pick_pose", 1);
    place_pose_pub = nh.advertise<geometry_msgs::Pose>("place_pose", 1);
    // service
    open_gripper_client = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    close_gripper_client = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
    // 发布姿态
    pose_sub = nh.subscribe("/object_array", 1, &PickPlace::objectCallback, this);
    action_sub = nh.subscribe("/action_data", 1, &PickPlace::actionDataCallback, this);

    detection_client = nh.serviceClient<hirop_msgs::detection>("detection");

    // 
    detetor_sub = nh.subscribe("pedestrian_detection", 1, &PickPlace::subCallback, this);

    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setMaxVelocityScalingFactor(0.8);


    setGenActuator();

    tf2::Quaternion orien;
    orien.setRPY(0, 0, -1.57);

    place_pose1.position.x = 0.418;
    place_pose1.position.y = -0.68;
    place_pose1.position.z = 0.63;
    place_pose1.orientation = tf2::toMsg(orien);


    place_pose2.position.x = 0.418;
    place_pose2.position.y = -0.68;
    place_pose2.position.z = 0.32;
    place_pose2.orientation = tf2::toMsg(orien);

    place_pose3.position.x = 0.78;
    place_pose3.position.y = 0;
    place_pose3.position.z = 0.20;
    orien.setRPY(0, 0, 6.28);
    place_pose3.orientation = tf2::toMsg(orien);

    place_poses.push_back(place_pose1);
    place_poses.push_back(place_pose2);
    place_poses.push_back(place_pose3);
    ROS_INFO_STREAM(place_poses[0] << place_poses[1] << place_poses[2]);
    ROS_INFO("init_over");
}

void PickPlace::openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "left_finger_1_joint";
    posture.joint_names[1] = "right_finger_1_joint";
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.6;
    posture.points[0].positions[1] = -0.6;  
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void PickPlace::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "left_finger_1_joint";
    posture.joint_names[1] = "right_finger_1_joint";
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0;
    posture.points[0].positions[1] = 0;  
    posture.points[0].time_from_start = ros::Duration(0.5);
}

moveit_msgs::MoveItErrorCodes PickPlace::planMove(geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped poseStamep;
    poseStamep.pose = pose;
    poseStamep.header.frame_id = "base_link";
    move_group.setPoseTarget(poseStamep);
    for(int i=0; i<5; ++i)
    {
        bool succeed = (move_group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        if(succeed)
        {
            ROS_INFO_STREAM("planed succeed");
            move_group.move();
            break;
        }
        else
        {
            ROS_INFO_STREAM("planed faild");
        }
    }
}

moveit_msgs::MoveItErrorCodes PickPlace::pick(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose p1;
    p1 = pose;
    p1.position.y *= 0.8;
    // 临近点
    planMove(p1);
    // 
    hirop_msgs::openGripper open_srv;
    this->open_gripper_client.call(open_srv);
    CartesianPath(pose);
    this->close_gripper_client.call(open_srv);
    // move_group.attachObject("object");
    CartesianPath(p1);
}

moveit_msgs::MoveItErrorCodes PickPlace::place(geometry_msgs::Pose pose)
{
    // 临近点
    geometry_msgs::Pose target_finish = pose;
    hirop_msgs::openGripper open_srv;
    target_finish.position.x *= 0.85;
    target_finish.position.y *= 0.85;
    target_finish.position.z *= 1;
    target_finish.orientation.x = 0;
    target_finish.orientation.y = 0.254;
    target_finish.orientation.z = 0;
    target_finish.orientation.w = 0.967;
    CartesianPath(target_finish);
    // 抵达
    CartesianPath(pose);
    this->open_gripper_client.call(open_srv);
    CartesianPath(target_finish);
}

bool PickPlace::setGenActuator()
{
    hirop_msgs::listGenerator list_generator_srv;
    hirop_msgs::listActuator list_actuator_srv;
    hirop_msgs::SetGenActuator set_gen_actuator_srv;
    if(list_generator_client.call(list_generator_srv) && list_actuator_client.call(list_actuator_srv))
    {
        set_gen_actuator_srv.request.generatorName = list_generator_srv.response.generatorList[0];
        set_gen_actuator_srv.request.actuatorName = list_actuator_srv.response.actuatorList[0];
        if(set_gen_actuator_client.call(set_gen_actuator_srv))
            return true;
    }
    return false;
}

void PickPlace::showObject(geometry_msgs::Pose pose)
{
    hirop_msgs::ShowObject srv;
    srv.request.objPose.header.frame_id = "base_link";
    srv.request.objPose.pose.position.x = pose.position.x;
    srv.request.objPose.pose.position.y = pose.position.y;
    srv.request.objPose.pose.position.z = pose.position.z;
    srv.request.objPose.pose.orientation.x = pose.orientation.x;
    srv.request.objPose.pose.orientation.y = pose.orientation.y;
    srv.request.objPose.pose.orientation.z = pose.orientation.z;
    srv.request.objPose.pose.orientation.w = pose.orientation.w;
    if(show_object_client.call(srv))
    {
        ROS_INFO_STREAM("show object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\showObject service ");
    }
}

void PickPlace::rmObject()
{
    hirop_msgs::RemoveObject srv;
    if(remove_object_client.call(srv))
    {
        ROS_INFO_STREAM("remove object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\removeObject service ");
    }
}

void PickPlace::actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    intent = msg->data[0];
    object = msg->data[1];
    target = msg->data[2];
    hirop_msgs::detection det;
    det.request.objectName = "object";
    det.request.detectorName = "detector";
    det.request.detectorType = 1;
    det.request.detectorConfig = "config";
    if(detection_client.call(det))
    {
        ROS_INFO("Identify the successful");
    }
}

moveit_msgs::MoveItErrorCodes PickPlace::CartesianPath(geometry_msgs::Pose pose)
{
    // 直线去抓取object
    // 从现在位置
    move_group.setPoseReferenceFrame("base_link");
    geometry_msgs::Pose target_pose = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose.position.z -= 1.0;
    // ROS_INFO_STREAM("target_pose: " << target_pose);
    waypoints.push_back(target_pose);
    // 到预抓取位置
    geometry_msgs::Pose target_finish = pose;
    // target_finish.position.x *= 0.85;
    // target_finish.position.y *= 0.85;
    // target_finish.position.z *= 1;
    // target_finish.orientation.x = 0;
    // target_finish.orientation.y = 0.254;
    // target_finish.orientation.z = 0;
    // target_finish.orientation.w = 0.967;

    waypoints.push_back(target_finish);
    // ROS_INFO_STREAM("target_finish: " << target_finish);
    // 警告 - 在操作实际硬件时禁用跳转阈值可能会
    // 导致冗余接头的大量不可预知运动，并且可能是一个安全问题
    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.02;
    double fraction = 0;
    int cnt = 0;
    moveit_msgs::MoveItErrorCodes errorCode;

    while (fraction < 1.0 && cnt < 100)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        cnt ++;
    }
    if(fraction != 1.0)
    {
        planMove(pose);
    }
    else
    {
        ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
        move_group.plan(my_plan);
        move_group.move();
    }
}

geometry_msgs::PoseStamped PickPlace::TransformListener(geometry_msgs::PoseStamped pose)
{
    // int ii = msg->objects.size();
    geometry_msgs::PoseStamped returnPose;
    geometry_msgs::PoseStamped * detectPoseFromCamera = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped * base_detectPoseFromCamera = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;
    detectPoseFromCamera[0].header.seq = 1;
    detectPoseFromCamera[0].header.frame_id = pose.header.frame_id;
    detectPoseFromCamera[0].pose = pose.pose;
    for(int cout = 0; cout < 5; cout++)
    {
        try
        {
            listener.transformPose("base_link",detectPoseFromCamera[0], base_detectPoseFromCamera[0]);
        break;
        }
        catch(tf::TransformException &ex)
        {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.5).sleep();
        continue;
        }
    }
    base_detectPoseFromCamera[0].pose.position.z = 0.70;
    base_detectPoseFromCamera[0].pose.orientation.x = 0;
    base_detectPoseFromCamera[0].pose.orientation.y = 0;
    base_detectPoseFromCamera[0].pose.orientation.z = -0.706825;
    base_detectPoseFromCamera[0].pose.orientation.w = 0.707388;

    std::cout << base_detectPoseFromCamera[0].pose.position.x <<" " \
        << base_detectPoseFromCamera[0].pose.position.y <<" "\
        << base_detectPoseFromCamera[0].pose.position.z <<" "\
        << base_detectPoseFromCamera[0].pose.orientation.x <<" "\
        << base_detectPoseFromCamera[0].pose.orientation.y <<" "\
        << base_detectPoseFromCamera[0].pose.orientation.z <<" "\
        << base_detectPoseFromCamera[0].pose.orientation.w << std::endl;
    returnPose = base_detectPoseFromCamera[0];
    delete[] detectPoseFromCamera;
    delete[] base_detectPoseFromCamera;
    return returnPose;
}

void PickPlace::objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    move_group.allowReplanning(true);
    geometry_msgs::Pose pose;
    nh.getParam("intent", intent);
    nh.getParam("target", target);
    // ROS_INFO_STREAM("intent: "<< intent << "target: " << target);
    int i = msg->objects.size();
    static int cnt = 0;
    static int errorCnt = 0;
    cnt++;
    nh.setParam("/cnt", cnt);


    for(int j = 0; j < i; ++j)
    {   
        geometry_msgs::PoseStamped toPickPoseStamp;
        toPickPoseStamp = TransformListener(msg->objects[j].pose);

        ROS_INFO("Press 'enter' to continue");
        std::cin.ignore();
        moveit_msgs::MoveItErrorCodes code;
        pose = toPickPoseStamp.pose;
        rmObject();
        showObject(pose);
        if(intent == 0)
        {
            ROS_INFO_STREAM("intent:" << intent);
            this->CartesianPath(pose);
        }
        else if(intent == 1)
        {
            ROS_INFO_STREAM("intent:" << intent);
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, -1.57);
            pose.orientation = tf2::toMsg(orientation);
        }
        pick(pose);
        ros::WallDuration(1.0).sleep();
        move_group.setNamedTarget("home");
        move_group.move();
        // 测试
        code = place(place_poses[target]);
        if(code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            errorCnt ++;
        }
        nh.setParam("error_cnt", errorCnt);
        move_group.setNamedTarget("home");
        move_group.move();
        nh.setParam("over", true);
    }
}


void PickPlace::subCallback(const std_msgs::Bool::ConstPtr& msg)
{
    static bool flag = false;
	if(flag != msg->data)
	{
        flag = msg->data;
		if(msg->data)
		{
			ROS_INFO("slow down ...");
			move_group.setMaxVelocityScalingFactor(0.2);
		}
		else
		{
			ROS_INFO("normal speed");
			move_group.setMaxVelocityScalingFactor(0.8);
		}
	}
}

