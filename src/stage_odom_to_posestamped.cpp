#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
std::string global_frame = "/map", base_link_frame = "base_link", odom_frame = "odom";

void pose_callback(nav_msgs::Odometry::ConstPtr  msg){

    static tf::TransformBroadcaster tfb;
    static tf::TransformListener tl;

    tf::StampedTransform map_to_base_link;
    tf::StampedTransform odom_to_base_link;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position = msg->pose.pose.position;
    pose.pose.pose.orientation = msg->pose.pose.orientation;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = msg->header.frame_id;
    pub.publish(pose);

    /* Map to base_link */
    tf::Quaternion q;
    map_to_base_link.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    map_to_base_link.setRotation(q);

    /* Odom to base_link */
    tl.waitForTransform(odom_frame, base_link_frame, ros::Time(0), ros::Duration(3.0));
    tl.lookupTransform(odom_frame, base_link_frame, ros::Time(0), odom_to_base_link);

    /* Compute */
    tf::Transform t =   map_to_base_link * odom_to_base_link.inverse();

    /* Map to odom */
    tf::StampedTransform map_to_odom;
    map_to_odom.setOrigin(t.getOrigin());
    map_to_odom.setRotation(t.getRotation());
    map_to_odom.frame_id_ = global_frame;
    map_to_odom.child_frame_id_ = odom_frame;
    map_to_odom.stamp_ = msg->header.stamp;
    tfb.sendTransform(map_to_odom);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "stage_odom_to_posestamped") ;
    ros::NodeHandle n, nh("~");

    nh.getParam("global_frame", global_frame);
    nh.getParam("odom_frame", odom_frame);
    nh.getParam("base_link_frame", base_link_frame);

    ros::Subscriber sub = n.subscribe("base_pose_ground_truth",  1, pose_callback);
    pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);



    ros::Publisher pub2 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("kk", 1);
    geometry_msgs::PoseWithCovarianceStamped kk;
    kk.header.frame_id = "/map";
    pub2.publish(kk);

    ros::spin();

    return 0;

}
