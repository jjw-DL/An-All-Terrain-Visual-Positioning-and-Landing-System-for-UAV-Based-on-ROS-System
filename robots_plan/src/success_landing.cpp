//ros相关头文件
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

//moveit相关头文件
#include <moveit/move_group_interface/move_group.h>



class success_landing
{

public:
    success_landing()
    {
        planning_done = nh.subscribe("arm_4_success", 100, &success_landing::success_cb, this);
    }

  
     void success_cb(const std_msgs::UInt8& arm_4_have_success)
     {
           std::cout<<"planning have done,the aircraft can land safely."<<std::endl;
     }



protected:
    ros::NodeHandle nh;
    ros::Subscriber planning_done;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "success_landing");
  
    success_landing handler;

    ros::spin();

    return 0;
}
