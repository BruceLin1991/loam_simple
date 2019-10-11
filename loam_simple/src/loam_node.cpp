#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include"scanRegistration.h"
#include"laserOdometry.h"
#include"laserMapping.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "loam_simple");
    ScanRegistration scanRegis;
    LaserOdometry laserOdom(&scanRegis);
    LaserMapping laserMapper(&laserOdom);


    while (ros::ok()) {
        ros::spinOnce();//need it to subscribe ros topic
        usleep(1000*50);
    }

    //cout<<"main.cpp: closing main node"<<endl;
    laserOdom.closeThread();
    usleep(1000*200);
    laserMapper.closeThread();
    usleep(1000*200);
    return 0;
}
