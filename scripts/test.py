#! /usr/bin/env python
#coding=utf-8

import rospy
from hirop_msgs.msg import ObjectArray
from hirop_msgs.msg import ObjectInfo

rospy.init_node("test_pickplace")
pub = rospy.Publisher('object_array', ObjectArray , queue_size=10)
rate = rospy.Rate(1)

def setPose(x, y, z, qx, qy, qz, qw):
    pose = ObjectArray()
    object = ObjectInfo()
    object.pose.header.frame_id = "base_link"
    object.pose.pose.position.x = x
    object.pose.pose.position.y = y
    object.pose.pose.position.z = z
    object.pose.pose.orientation.x = qx
    object.pose.pose.orientation.y = qy
    object.pose.pose.orientation.z = qz
    object.pose.pose.orientation.w = qw
    pose.objects.append(object)
    return pose

poses = []
pose = ObjectArray()
pose = setPose(0.418, -0.65, 0.66, 0, 0, -0.706825, 0.707388)
poses.append(pose)
pose = setPose(0.418, -0.65, 0.38, 0, 0, -0.706825, 0.707388)
poses.append(pose)
pose = setPose(0.85, 0, 0.25, 0, 0, 0, 1)
poses.append(pose)

rospy.set_param("/intent", 1)
rospy.set_param("/target", 2)
print(pose)
cnt = 0
error_cnt = 0
while not rospy.is_shutdown():
    flag = rospy.get_param("/over", False)
    if flag ==  True:
        pub.publish(poses[0])
        rospy.loginfo("pub pose")
        cnt = rospy.get_param("/cnt")
        error_cnt = rospy.get_param("/error_cnt")
        print("cnt: ", cnt, "error_cnt: ", error_cnt)
        rospy.set_param("/over", False)