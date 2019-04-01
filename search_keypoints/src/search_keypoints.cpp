#include <ros/ros.h> //generic C++ stuff
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <vector>
#include<algorithm>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//存储加载点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//存储滤波后点云
bool have_load_pcl_file =false; 

void shootDoneCB(std_msgs::UInt8 suc)//加载点云
{   
    string fname ="/home/jiangjingwen/project_code/src/pcd_file/terrain_shooting.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pcl_clr_ptr) == -1) 
    {
        ROS_ERROR ("Couldn't read file \n");
    }
    std::cout << "Loaded "<< pcl_clr_ptr->width * pcl_clr_ptr->height<< " data points from file "<<fname<<std::endl;
    have_load_pcl_file =true;
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "search_keypoints"); 
    ros::NodeHandle nh; 
    //查看拍摄是否完成
    ros::Subscriber SubShootDone =nh.subscribe<std_msgs::UInt8> ("shooting_done",1,shootDoneCB);
    //输出点云
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    //发布四个对应高度
    ros::Publisher pubHeight1 = nh.advertise<std_msgs::Float32> ("position_1",1);
    ros::Publisher pubHeight2 = nh.advertise<std_msgs::Float32> ("position_2",1);
    ros::Publisher pubHeight3 = nh.advertise<std_msgs::Float32> ("position_3",1);
    ros::Publisher pubHeight4 = nh.advertise<std_msgs::Float32> ("position_4",1);
    //发布降落信号
    ros::Publisher pubPlanSuc= nh.advertise<std_msgs::UInt8> ("start_landing",1);
    //4个高度变量
    float height1=0.0;
    float height2=0.0;
    float height3=0.0;
    float height4=0.0;
    float height1_ave,height2_ave,height3_ave,height4_ave;//4个平均高度变量
    int count1=0;
    int count2=0;
    int count3=0;
    int count4=0;

    while(!have_load_pcl_file)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    } 

    if(have_load_pcl_file)
    {
        //体素滤波
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (pcl_clr_ptr);
        sor.setLeafSize (0.005f, 0.005f, 0.005f);
        sor.filter (*cloud_filtered);
        
        //进行关键点的查找
        std::cout<<"search the keypoints:"<<endl;

        for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
        { 
            if(!pcl_isfinite(cloud_filtered->points[i].x))
            { 
                continue;
            }

            if(cloud_filtered->points[i].y >-0.188 && cloud_filtered->points[i].y <-0.182)
            { 
                if(cloud_filtered->points[i].x >-0.003 && cloud_filtered->points[i].x <0.003)
                {
                    std::cout << "    " << cloud_filtered->points[i].x
                            << " "    << cloud_filtered->points[i].y
                            << " "    << cloud_filtered->points[i].z 
                            << " "    << "arm1_position" << std::endl;
                    ++count1;
                    height1 +=cloud_filtered->points[i].z;
                }
            }

            if(cloud_filtered->points[i].y >-0.003 && cloud_filtered->points[i].y <0.003)
            { 
                if(cloud_filtered->points[i].x >0.182 && cloud_filtered->points[i].x <0.188)
                {
                    std::cout << "    " << cloud_filtered->points[i].x
                            << " "    << cloud_filtered->points[i].y
                            << " "    << cloud_filtered->points[i].z 
                            << " "    << "arm2_position" << std::endl;
                    ++count2;
                    height2 +=cloud_filtered->points[i].z;
                }
            }



            if(cloud_filtered->points[i].y >0.182 && cloud_filtered->points[i].y <0.188)
            { 
                if(cloud_filtered->points[i].x >-0.003 && cloud_filtered->points[i].x <0.003)
                {
                    std::cout << "    " << cloud_filtered->points[i].x
                            << " "    << cloud_filtered->points[i].y
                            << " "    << cloud_filtered->points[i].z 
                            << " "    << "arm3_position" << std::endl;
                    ++count3;
                    height3 +=cloud_filtered->points[i].z;
                }
            }


            if(cloud_filtered->points[i].y >-0.003 && cloud_filtered->points[i].y <0.003)
            { 
                if(cloud_filtered->points[i].x >-0.188 && cloud_filtered->points[i].x <-0.182)
                {
                    std::cout << "    " << cloud_filtered->points[i].x
                            << " "    << cloud_filtered->points[i].y
                            << " "    << cloud_filtered->points[i].z 
                            << " "    << "arm4_position" << std::endl;
                    ++count4;
                    height4 +=cloud_filtered->points[i].z;
                }
            }

        }
        
        height1_ave=  height1/count1;
        cout<<"arm_1 平均高度 ："<<height1_ave<<endl;
        height2_ave=  height2/count2;
        cout<<"arm_2 平均高度 ："<<height2_ave<<endl;
        height3_ave=  height3/count3;
        cout<<"arm_3 平均高度 ："<<height3_ave<<endl;
        height4_ave=  height4/count4;
        cout<<"arm_4 平均高度 ："<<height4_ave<<endl;

        vector<float>  height_ave;
        height_ave.push_back(height1_ave);
        height_ave.push_back(height2_ave);
        height_ave.push_back(height3_ave);
        height_ave.push_back(height4_ave);
        //排序
        sort(height_ave.begin(), height_ave.end());
        //找到标准高度
        float std_height=height_ave[1];
        cout<<"标准高度："<<std_height<<endl;
        //计算实际位置
        std_msgs::Float32 ave_pos1,ave_pos2,ave_pos3,ave_pos4;
        ave_pos1.data=height1_ave-std_height+0.37;
        cout<<"arm_1 实际规划位置 ："<<ave_pos1.data<<endl;
        ave_pos2.data=height2_ave-std_height+0.37;
        cout<<"arm_2 实际规划位置 ："<<ave_pos2.data<<endl;
        ave_pos3.data=height3_ave-std_height+0.37;
        cout<<"arm_3 实际规划位置 ："<<ave_pos3.data<<endl;
        ave_pos4.data=height4_ave-std_height+0.37;
        cout<<"arm_4 实际规划位置 ："<<ave_pos4.data<<endl;
        //发布位置数据
        pubHeight1.publish(ave_pos1);
        pubHeight2.publish(ave_pos2);
        pubHeight3.publish(ave_pos3);
        pubHeight4.publish(ave_pos4);


        sensor_msgs::PointCloud2 ros_cloud;  
        pcl::toROSMsg(*pcl_clr_ptr, ros_cloud); 
        ros_cloud.header.frame_id = "base_link";
        cout << "view in rviz; choose: topic= /output; and fixed frame= base_link" << endl;
        pubCloud.publish(ros_cloud);



        std_msgs::UInt8 PlanDone;
        PlanDone.data=1;
        pubPlanSuc.publish(PlanDone);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    return 0;
}

    
