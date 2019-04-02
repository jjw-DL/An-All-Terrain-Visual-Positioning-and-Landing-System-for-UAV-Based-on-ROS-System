#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#define PI 3.1415926

using namespace std;

class armTrajectoryFollower
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
        std::string action_name_;
        
        ros::Publisher joint_pub;
	sensor_msgs::JointState joint_state;
 
        ros::Publisher joint_motor_pub; //声明一个舵机关节发布者
        ros::Publisher trigger;//声明一个触发器

	std::string degree_topic;
	std::string success_topic;

    public:
        //构造函数
        armTrajectoryFollower(std::string name,std::string degree_topic,std::string success_topic):as_(nh_,name,boost::bind(&armTrajectoryFollower::goalCB,this,_1),false),action_name_(name)
        {      
            joint_motor_pub = nh_.advertise<std_msgs::String>(degree_topic, 1000);  //初始化舵机关节发布者
            joint_pub = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);
            trigger = nh_.advertise<std_msgs::UInt8>(success_topic, 1);
            as_.registerPreemptCallback(boost::bind(&armTrajectoryFollower::preemptCB, this));
            as_.start();
        }
        //析构函数
        ~armTrajectoryFollower(void){}

        
        //设置关节名字
        void setJointStateName(std::vector<std::string> joint_names)
        {
            joint_state.name.resize(joint_names.size());
            joint_state.name.assign(joint_names.begin(), joint_names.end());
            vector<string>::iterator iter;
            for (iter=joint_state.name.begin();iter!=joint_state.name.end();iter++)
            {
                cout<<*iter<<',';
            }
            cout<<endl;
        }
        //设置关节位置
        void setJointStatePosition(std::vector<double> joint_posi)
        {
            joint_state.position.resize(joint_posi.size());
	    joint_state.position.assign(joint_posi.begin(), joint_posi.end());
            vector<double>::iterator iter;
            for (iter=joint_state.position.begin();iter!=joint_state.position.end();iter++)
            {
                cout<<*iter<<',';
            }
            cout<<endl;
	}

        //发布关节状态
        void publishJointState()
        {
	    joint_state.header.stamp = ros::Time::now();
	    joint_pub.publish(joint_state);
	}
 
        //发布舵机关节
        void publishMotorState(std::vector<double> joint_posi)
        {
            std_msgs::String msg;
            std::stringstream ss;
            vector<double>::iterator iter;
            for(iter=joint_posi.begin()+1;iter!=joint_posi.end();iter++)
            {
                  ss<<(int)(90+(*iter)*180/PI)<<',';
            }
            msg.data=ss.str();
            ROS_INFO("PUBLISH THE FINALL POSITIONS: %s",msg.data.c_str());
            joint_motor_pub.publish(msg);
            std_msgs::UInt8 trigger_1;
            trigger_1.data = 1;
            trigger.publish(trigger_1);
        }
        //回调函数
        void goalCB(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal)
        {
            ROS_INFO("%s:: Action goal recieved.",action_name_.c_str());

            std::vector<std::string> joint_names=(*goal).trajectory.joint_names;
            setJointStateName(joint_names);

            vector<trajectory_msgs::JointTrajectoryPoint> points = (*goal).trajectory.points;
            std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator points_it;

            ros::Rate rate(10);
            for (points_it = points.begin(); points_it != points.end(); points_it++)
            {
		   rate.sleep();
		   setJointStatePosition((*points_it).positions);
		   publishJointState();
                   if(points_it == (points.end()-1))
                   {
                       publishMotorState((*points_it).positions);
                   } 
            }

            if (as_.isActive())
               as_.setSucceeded();
        }

        void preemptCB()
	    {
		    ROS_INFO("%s: Preempted", action_name_.c_str());
		    if (as_.isActive())
                    {
			  as_.setPreempted();
		    }
	    }

};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robot_controller");

    armTrajectoryFollower ArmTrajectoryFollower_1("arm_1/follow_joint_trajectory","arm_1_motor_degree","arm_1_success");
    armTrajectoryFollower ArmTrajectoryFollower_2("arm_2/follow_joint_trajectory","arm_2_motor_degree","arm_2_success");
    armTrajectoryFollower ArmTrajectoryFollower_3("arm_3/follow_joint_trajectory","arm_3_motor_degree","arm_3_success");
    armTrajectoryFollower ArmTrajectoryFollower_4("arm_4/follow_joint_trajectory","arm_4_motor_degree","arm_4_success");

    ROS_INFO("robot action server is running ...");

    ros::spin();

    return 0;
}
