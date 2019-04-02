//ros相关头文件
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
//moveit相关头文件
#include <moveit/move_group_interface/move_group.h>



class arm4_planning
{
public:
    arm4_planning()
    {
        height4_sub =nh.subscribe("position_4",100,&arm4_planning::height4_cb, this);
        arm4_sub =nh.subscribe("arm_3_success", 100, &arm4_planning::arm_4_cb, this);
    }

    void height4_cb(std_msgs::Float32 pos)
     {
         height_4=pos.data;
     }
  
     void arm_4_cb(const std_msgs::UInt8& arm_3_have_success)
     {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        // 连接move_group节点中的机械臂实例组
        moveit::planning_interface::MoveGroup group("arm_4");
        group.moveit::planning_interface::MoveGroup::allowReplanning (true); 	
        //设置终端姿态
        geometry_msgs::Pose target_pose;
        target_pose.orientation.x= 0;
        target_pose.orientation.y = 0;
        target_pose.orientation.z = 0;
        target_pose.orientation.w = 1;
        //设置终端位置
        target_pose.position.x = -0.185;
        target_pose.position.y = 0;
        target_pose.position.z = height_4;
        //设置终端位姿
        group.setPoseTarget(target_pose);
        //进行运动规划
        moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success = group.plan(my_plan);
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        //执行规划轨迹
        if(success)
            group.execute(my_plan);
     }

protected:
    ros::NodeHandle nh;

    ros::Subscriber height4_sub;
    ros::Subscriber arm4_sub;
 
    float height_4;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm4_planning");
  
    arm4_planning handler;

    ros::spin();

    return 0;
}
