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

    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setMaxVelocityScalingFactor(0.95);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.allowReplanning(true);
    move_group.setPlanningTime(1);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setWorkspace(-0.3, -1, 0, 1, 0.45, 1.25);
    ROS_INFO_STREAM("planning Frame: " << move_group.getPlanningFrame());

    setGenActuator();


    setPoses();
    ROS_INFO_STREAM(place_poses[0] << place_poses[1] << place_poses[2]);
    ROS_INFO("init_over");
}

void MovePickPlace::setPoses()
{
    std::string path;
    nh.getParam("/move_to_pick_place/pose1", path);
    setPose(path, place_pose1);
    nh.getParam("/move_to_pick_place/pose2", path);
    setPose(path, place_pose2);
    nh.getParam("/move_to_pick_place/pose3", path);
    setPose(path, place_pose3);
    place_poses.push_back(place_pose1);
    place_poses.push_back(place_pose2);
    place_poses.push_back(place_pose3);
}

void MovePickPlace::setPose(const std::string& path, geometry_msgs::Pose& pose)
{
    YAML::Node doc;
    doc = YAML::LoadFile(path);
    assignment(doc, pose);
}

void MovePickPlace::assignment(const YAML::Node& node, geometry_msgs::Pose& pose)
{
    pose.position.x = node["position"]["x"].as<float>();
    pose.position.y = node["position"]["y"].as<float>();
    pose.position.z = node["position"]["z"].as<float>();
    pose.orientation.x = node["orientation"]["x"].as<float>();
    pose.orientation.y = node["orientation"]["y"].as<float>();
    pose.orientation.z = node["orientation"]["z"].as<float>();
    pose.orientation.w = node["orientation"]["w"].as<float>();
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
    p1.position.y *= 0.85;
    hirop_msgs::openGripper open_srv;
    hirop_msgs::closeGripper close_srv;
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
            this->close_gripper_client.call(close_srv);
            ros::WallDuration(1.0).sleep();
            planMove(p1);
        }
    }    
    return code;
}

moveit_msgs::MoveItErrorCodes MovePickPlace::place(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose target_finish = pose;
    hirop_msgs::openGripper open_srv;
    moveit_msgs::MoveItErrorCodes code;
    target_finish.position.x *= 0.95;
    target_finish.position.z *= 1;
    target_finish.orientation.w = 1.0;
    
    code = planMove(target_finish);
    if(code.val = moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        code = planMove(pose);
        if(code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("place succeed");
            move_group.detachObject("object");
            this->open_gripper_client.call(open_srv);
            ros::WallDuration(1.0).sleep();
            planMove(target_finish);
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
    double eef_step = 0.03;
    double fraction = 0;
    int cnt = 0;
    moveit_msgs::MoveItErrorCodes code;
    while(fraction < 1.0 && cnt < 100)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        cnt ++;
    }
    if(fraction < 0.7 && second_plan == true)
    {
        code = planMove(pose);
    }
    else
    {
        ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
        my_plan.trajectory_ = trajectory;
        code = move_group.execute(my_plan);
    }
    return code;
}

geometry_msgs::PoseStamped MovePickPlace::TransformListener(geometry_msgs::PoseStamped pose)
{
    // int ii = msg->objects.size();
    pose.pose.position.x *= 0.01;
    pose.pose.position.y *= 0.01;
    pose.pose.position.z *= 0.01;
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
    float x, y;
    nh.getParam("position_x_add", x);
    nh.getParam("position_y_add", y);
    base_detectPoseFromCamera[0].pose.position.x += x;
    base_detectPoseFromCamera[0].pose.position.y += y;

    int seat;
    nh.getParam("/seat", seat);
    if(seat == 0)
        base_detectPoseFromCamera[0].pose.position.z = 0.65;
    else if (seat == 1)
        base_detectPoseFromCamera[0].pose.position.z = 0.38; 
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
    nh.setParam("is_back_home", false);

    geometry_msgs::Pose pose;
    nh.getParam("intent", intent);
    nh.getParam("target", target);
    // ROS_INFO_STREAM("intent: "<< intent << "target: " << target);
    static int cnt = 0;
    static int errorCnt = 0;
    cnt++;
    nh.setParam("/cnt", cnt);
    bool add_collision = true;
    int i = msg->objects.size();
    for(int j = 0; j < i; ++j)
    {   
        nh.getParam("add_collision", add_collision);
        if(add_collision == false)
        {
            system("rosservice call /pcl_clear");
            system("rosservice call /clean_pcl");
            system("rosservice call /look");
            system("rosservice call /add_collision");
        }
        geometry_msgs::PoseStamped toPickPoseStamp;
        moveit_msgs::MoveItErrorCodes code;
        toPickPoseStamp = TransformListener(msg->objects[j].pose);
        pose = toPickPoseStamp.pose;
        rmObject();
        showObject(pose);

        // ROS_INFO("Press 'enter' to continue");
        // std::cin.ignore();
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
        move_group.setWorkspace(-0.3, -0.4, 0, 1, 0.45, 1.25);
        move_group.setNamedTarget("home");
        move_group.move();
        nh.setParam("over", true);
    }
}

// 待扩充的内容
// void MovePickPlace::spin()
// {
//     move_group.allowReplanning(true);
//     move_group.setPlanningTime(1);
//     while(ros::ok())
//     {

//     }
// }


void MovePickPlace::subCallback(const std_msgs::Bool::ConstPtr& msg)
{
    static bool flag = false;
	if(flag != msg->data)
	{
        flag = msg->data;
		if(msg->data)
		{
			ROS_INFO("slow down ...");
			move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setMaxAccelerationScalingFactor(0.1);
		}
		else
		{
			ROS_INFO("normal speed");
			move_group.setMaxVelocityScalingFactor(0.95);
		}
	}
}

