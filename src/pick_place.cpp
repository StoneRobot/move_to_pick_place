#include "pick_place/pick_place.h"

MovePickPlace::MovePickPlace(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group)
:nh{_n},
move_group{group}
{
    ROS_INFO("init");
    remove_object_client = nh.serviceClient<hirop_msgs::RemoveObject>("removeObject");
    show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
    list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
    list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
    set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
    // gripper
    pick_pose_pub = nh.advertise<geometry_msgs::Pose>("pick_pose", 1);
    place_pose_pub = nh.advertise<geometry_msgs::Pose>("place_pose", 1);
    // service
    open_gripper_client = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    close_gripper_client = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
    // 发布姿态
    pose_sub = nh.subscribe("/object_array", 1, &MovePickPlace::objectCallback, this);
    // 
    detetor_sub = nh.subscribe("pedestrian_detection", 1, &MovePickPlace::subCallback, this);

    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);

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

moveit_msgs::MoveItErrorCodes MovePickPlace::planMove(geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped poseStamep;
    moveit_msgs::MoveItErrorCodes code;
    poseStamep.pose = pose;
    poseStamep.header.frame_id = "base_link";
    move_group.setPoseTarget(poseStamep);
    for(int i=0; i<5; ++i)
    {
        code = move_group.plan(my_plan);
        ROS_INFO("plan %d", i);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("to move");
            code = move_group.move();
            break;
        }
    }
    return code;
}

moveit_msgs::MoveItErrorCodes MovePickPlace::pick(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose p1;
    p1 = pose;
    p1.position.y *= 0.8;
    hirop_msgs::openGripper open_srv;
    moveit_msgs::MoveItErrorCodes code;
    // 临近点
    code = planMove(p1);
    ROS_INFO_STREAM(code.val);
    if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        this->open_gripper_client.call(open_srv);
        code = planMove(pose);
        ROS_INFO_STREAM(code.val);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("pick succeed");
            move_group.attachObject("object");
            this->close_gripper_client.call(open_srv);
            CartesianPath(p1, false);
        }
    }    
    return code;
}

moveit_msgs::MoveItErrorCodes MovePickPlace::place(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose target_finish = pose;
    hirop_msgs::openGripper open_srv;
    moveit_msgs::MoveItErrorCodes code;
    target_finish.position.x *= 0.85;
    target_finish.position.y *= 0.85;
    target_finish.position.z *= 1;
    target_finish.orientation.x = 0;
    target_finish.orientation.y = 0.254;
    target_finish.orientation.z = 0;
    target_finish.orientation.w = 0.967;
    
    code = CartesianPath(target_finish, true);
    if(code.val = moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        code = CartesianPath(pose, true);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("place succeed");
            move_group.detachObject("object");
            this->open_gripper_client.call(open_srv);
            CartesianPath(target_finish, true);
        }
    }
    return code;
}

bool MovePickPlace::setGenActuator()
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

void MovePickPlace::showObject(geometry_msgs::Pose pose)
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

void MovePickPlace::rmObject()
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



moveit_msgs::MoveItErrorCodes MovePickPlace::CartesianPath(geometry_msgs::Pose pose, bool second_plan)
{
    // 直线去抓取object
    // 从现在位置
    move_group.setPoseReferenceFrame("base_link");
    geometry_msgs::Pose target_pose = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose.position.z -= 1.0;
    waypoints.push_back(target_pose);
    // 到预抓取位置
    geometry_msgs::Pose target_finish = pose;
    waypoints.push_back(target_finish);
    // 警告 - 在操作实际硬件时禁用跳转阈值可能会
    // 导致冗余接头的大量不可预知运动，并且可能是一个安全问题
    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.02;
    double fraction = 0;
    int cnt = 0;
    moveit_msgs::MoveItErrorCodes code;
    while(fraction < 1.0 && cnt < 100)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        cnt ++;
    }
    if(fraction != 1.0 && second_plan == true)
    {
        code = planMove(pose);
    }
    else
    {
        ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
        code = move_group.plan(my_plan);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            code = move_group.move();
    }
    return code;
}

geometry_msgs::PoseStamped MovePickPlace::TransformListener(geometry_msgs::PoseStamped pose)
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
    base_detectPoseFromCamera[0].pose.position.z = 0.68;
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

void MovePickPlace::objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg)
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
            this->CartesianPath(pose, true);
        }
        else if(intent == 1)
        {
            ROS_INFO_STREAM("intent:" << intent);
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, -1.57);
            pose.orientation = tf2::toMsg(orientation);
        }
        code = pick(pose);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            // ros::WallDuration(1.0).sleep();
            // move_group.setNamedTarget("home");
            // move_group.move();
            code = place(place_poses[target]);
            if(code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                errorCnt ++;
            }
        }
        else
        {
            errorCnt++;
        }
        nh.setParam("error_cnt", errorCnt);
        move_group.setNamedTarget("home");
        move_group.move();
        nh.setParam("over", true);
    }
}


void MovePickPlace::subCallback(const std_msgs::Bool::ConstPtr& msg)
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

