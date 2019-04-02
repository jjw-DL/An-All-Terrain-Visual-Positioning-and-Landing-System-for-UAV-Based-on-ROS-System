#include <ros/ros.h>
#include "std_msgs/String.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
   
    sub_1 = n_.subscribe("arm_1_motor_degree", 1000, &SubscribeAndPublish::callback, this);
  
    sub_2 = n_.subscribe("arm_2_motor_degree", 1000, &SubscribeAndPublish::callback, this);
   
    sub_3 = n_.subscribe("arm_3_motor_degree", 1000, &SubscribeAndPublish::callback, this);

    sub_4 = n_.subscribe("arm_4_motor_degree", 1000, &SubscribeAndPublish::callback, this);

   
    pub_ = n_.advertise<std_msgs::String>("degree",1000);

    count=1;
    degree.data="";
 
  }
 
  void callback(const std_msgs::String::ConstPtr& msg)
  {
       degree.data+=msg->data;

       if ((count%4)==0)
       {
         pub_.publish(degree);
       }
       else
       {
         ++count;
       }
  }
 
private:
  ros::NodeHandle n_; 

  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  ros::Subscriber sub_3;
  ros::Subscriber sub_4;

  ros::Publisher pub_;

  std_msgs::String degree;
  int count;
 
};//End of class SubscribeAndPublish
 
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Send_Degree");
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
 
  ros::spin();
 
  return 0;
}
