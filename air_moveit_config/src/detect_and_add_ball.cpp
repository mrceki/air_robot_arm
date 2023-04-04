#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "../include/detect_and_add.hpp"

air_object::air_object(ros::NodeHandle& nh)
  : m_NodeHandle(nh) 
{
  m_center_x = 0; 
  m_center_y = 0;
  m_center_y_buffer = 0;
  m_center_x_buffer = 0;
  m_yolo_sub = m_NodeHandle.subscribe("/yolov7/yolov7", 1000, &air_object::yolo_callback, this);
  m_pcl_sub = m_NodeHandle.subscribe("/camera/depth/color/points",1000, &air_object::pcl_callback, this);  // Subscribe to the cylinder dimensions topic
  m_coordinate_pub = nh.advertise<std_msgs::Float32MultiArray>("/coordinates_of_object", 1000);
  ROS_INFO("INITIALIZING COMPLETED!!!!!!!!");
  }

void air_object::yolo_callback(const vision_msgs::Detection2DArrayConstPtr& d2d_arr_msg)
{
  if(d2d_arr_msg->detections.empty())
  {
    ROS_WARN("NO OBJECT DETECTED!!!");
    return;
  }
  vision_msgs::Detection2DArray d2dArr = *d2d_arr_msg;
  for(auto detection : d2dArr.detections){
    if(detection.results[0].id == 32){ //id of sports_ball in coco
      ROS_INFO("Orange Detected..."); 
      m_center_x = (int) detection.bbox.center.x;
      m_center_y = (int) detection.bbox.center.y;
      m_center_x_buffer = m_center_x;
      m_center_y_buffer = m_center_y; 
    }
  }

    ROS_INFO("The center of detected object is (%d, %d) !!", m_center_x, m_center_y); 
}


void air_object::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{ 
  // Convert the PointCloud2 message to PCL data type
  if(m_center_x == 0 && m_center_y == 0){
    ROS_ERROR("PCL ERROR");
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  
  if(cloud.empty())
  {
    return;
  }
  // Get the XYZ data for a specific pixel
  int r = m_center_y; // specify the row of the pixel you are interested in
  int c = m_center_x; // specify the column of the pixel you are interested in

  pcl::PointXYZ p = cloud.at(c, r);
  m_x = p.x;
  m_y = p.y;
  m_z = p.z;

  if(m_x == 0 && m_y == 0 && m_z == 0){
    ROS_ERROR("OBJECT DETECTED BUT CANNOT ACCESS DEPTH!!!!!!!!");
    return;
  }

  ROS_INFO("The XYZ data for pixel at (%d, %d) is: (%f, %f, %f)", c, r, m_x, m_y, m_z);

  m_xyz.data.resize(3); 
  m_xyz.data[0] = m_x;
  m_xyz.data[1] = m_y;
  m_xyz.data[2] = m_z;
  m_coordinate_pub.publish(m_xyz);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_add_cylinder_collision_object_demo");
  ros::NodeHandle nh;
  air_object obj(nh);
  
  ros::spin();  

  return 0;
}   
