#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> 

using namespace std;

bool got_kinect_image = false; 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
bool isStartShoot = false;

void startShootingCB(std_msgs::UInt8 suc)
{
    isStartShoot= true;
}


void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    if(isStartShoot)
    {
        if (!got_kinect_image)
        { 
            ROS_INFO("got new selected kinect image");
            pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
            ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
            got_kinect_image = true;
        }
    } 
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "terrain_shooting"); 
    ros::NodeHandle nh;
    ros::Subscriber snapshot_subscriber = nh.subscribe("Shooting_command",1,startShootingCB);//接受开始拍摄指令
    ros::Subscriber pointcloud_subscriber = nh.subscribe("/camera/depth_registered/points", 1, kinectCB);
    ros::Publisher  shooting_done_publisher = nh.advertise<std_msgs::UInt8>("shooting_done",1);

    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("got snapshot; saving to file terrain_shooting.pcd");
    pcl::io::savePCDFile("/home/jiangjingwen/project_code/src/pcd_file/terrain_shooting.pcd", *pclKinect_clr_ptr, true);

    std_msgs::UInt8 ShootingDone;
    ShootingDone.data=1;
    shooting_done_publisher.publish(ShootingDone);//通知拍摄成功
    
    return 0;
}
