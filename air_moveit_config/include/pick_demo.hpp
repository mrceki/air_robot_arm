#ifndef PICK_DEMO_HPP
#define PICK_DEMO_HPP

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>



class air_pick
{
    public:

    air_pick(ros::NodeHandle& nh);
    ~air_pick(){delete move_group;}

    void planningSceneWorldCallback(const moveit_msgs::PlanningScene::ConstPtr& msg);
    void pick();
    std_msgs::Float32MultiArray m_xyz;
   

    ros::NodeHandle m_NodeHandle;
    ros::Subscriber m_cord_sub;
    const std::string PLANNING_GROUP_ARM = "air";
    moveit::planning_interface::MoveGroupInterface* move_group;

    float m_x;
    float m_y;
    float m_z;

    private:

};

#endif // PICK_DEMO_HPP