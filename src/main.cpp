#include "pick_place/pick_place.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    std::string PLANNING_GROUP;
    nh.getParam("/pick_place/group", PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    MovePickPlace p(nh, move_group);
    ros::waitForShutdown();
    return 0;
}