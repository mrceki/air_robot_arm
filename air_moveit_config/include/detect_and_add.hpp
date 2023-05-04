#ifndef DETECT_AND_ADD_HPP
#define DETECT_AND_ADD_HPP

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/PointCloud2.h>
class air_object
{
    public:

    air_object(ros::NodeHandle& nh);

    void yolo_callback(const vision_msgs::Detection2DArrayConstPtr& d2d_arr_msg);
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void radius_estimator_callback(const std_msgs::Float32ConstPtr& radius);
    void sphereCallback(const std_msgs::Float32MultiArray::ConstPtr& xyz);
    std_msgs::Float32MultiArray m_xyz;

    ros::NodeHandle m_NodeHandle;
    ros::Publisher  m_coordinate_pub;
    ros::Subscriber m_coordinate_sub;
    ros::Subscriber m_yolo_sub;
    ros::Subscriber m_pcl_sub;
    ros::Subscriber m_sub;
    ros::Subscriber m_estimator_sub;

    float m_x;
    float m_y;
    float m_z;
    
    int m_center_x;
    int m_center_y;
    int m_center_x_buffer;
    int m_center_y_buffer;
    float m_radius;
    private:

};

#endif // DETECT_AND_ADD_HPP