
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
#include <pcl/point_types.h>
#include <fstream>

std::vector<ros::Subscriber> subs;
std::vector<ros::Publisher> pubs;
std::vector<bool> done;

void pose_callback(nav_msgs::Odometry::ConstPtr  msg, int i){
    if (!done[i]) {
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.pose.pose.position = msg->pose.pose.position;
        pose.pose.pose.orientation = msg->pose.pose.orientation;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pubs[i].publish(pose);
        done[i] = true;
        fprintf(stderr, "\n<!-- robot_%d -->\n", i);
        fprintf(stderr, "<arg name=\"initial_pose_x\" value=\"%f\" />\n", pose.pose.pose.position.x);
        fprintf(stderr, "<arg name=\"initial_pose_y\" value=\"%f\" />\n", pose.pose.pose.position.y);
        fprintf(stderr, "<arg name=\"initial_pose_a\" value=\"%f\" />\n", pose.pose.pose.orientation.z);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stage_initial_pose") ;
    ros::NodeHandle n, nh("~");

    int number_of_robots = 3;

    nh.getParam("number_of_robots",number_of_robots);

    for (int i=0; i< number_of_robots; i++) {
        subs.push_back(n.subscribe<nav_msgs::Odometry>("/robot_" + std::to_string(i) + "/base_pose_ground_truth", 1000, boost::bind(pose_callback, _1, i)));
        pubs.push_back(n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_" + std::to_string(i) + "/initialpose", 100));
        done.push_back(false);
    }


    ros::Rate rate(5);

    while (ros::ok()) {
        ros::spinOnce();

        bool all_done = true;
        for (int i=0; i< number_of_robots; i++) {
            all_done &= done[i];
        }
        rate.sleep();

        if (all_done){
            ros::shutdown();
        }
    }
    pubs.clear();
    subs.clear();
    fprintf(stderr, "Done setting initial positions.\n");
    return 0;

}
