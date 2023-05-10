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
  m_coordinate_pub = nh.advertise<vision_msgs::Detection3DArray>("/coordinates_of_object", 1000);
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
  vision_msgs::Detection3DArray coordinate_msg;
  for(auto detection : d2dArr.detections){
    for(auto result : detection.results){
      /* auto res = std::find(
        m_RegisteredObjIDs.begin(),
        m_RegisteredObjIDs.end(),
        detection.results.at(0).id
      );
      if(res != m_RegisteredObjIDs.end())
      {
        const auto resVal = *res;
        
      } */
      if(detection.results[0].id == 47 || detection.results[0].id == 49 || detection.results[0].id == 80 ){ //id of sports_ball in coco
        ROS_INFO("Object Detected...");
        int center_x = (int) detection.bbox.center.x;
        int center_y = (int) detection.bbox.center.y;

        // Create a message for the detected object's coordinates
        vision_msgs::Detection3D detection_msg;
        detection_msg.results.resize(detection.results.size());
        for(size_t i = 0; i < detection.results.size(); ++i) {
          pcl::PointXYZ p = air_object::getPointXYZ(center_x, center_y);
          detection_msg.results[i].id = detection.results[i].id;
          detection_msg.results[i].pose.pose.position.x = p.x;
          detection_msg.results[i].pose.pose.position.y = p.y;
          detection_msg.results[i].pose.pose.position.z = p.z;
        }
        coordinate_msg.detections.push_back(detection_msg);
      }
    }
  }

  if(!coordinate_msg.detections.empty()){
    // Publish the message for all the detected objects' coordinates
    ROS_INFO_STREAM("Detected objects: " << coordinate_msg.detections.size());
    m_coordinate_pub.publish(coordinate_msg);
  }
}

pcl::PointXYZ air_object::getPointXYZ(int x, int y)
{

  // Convert the PointCloud2 message to PCL data type
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*&m_pcl_msg, cloud);
  
  if(cloud.empty())
  {
    return pcl::PointXYZ(0, 0, 0);
  }
  
  // Get the XYZ data for a specific pixel
  pcl::PointXYZ p = cloud.at(x, y);
  
  return p;
}



void air_object::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{ 
  m_pcl_msg = *msg; // Store the PointCloud2 message to be used later
  
  ROS_INFO("PointCloud2 received");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_add_cylinder_collision_object_demo");
  ros::NodeHandle nh;
  air_object obj(nh);
  
  ros::spin();  

  return 0;
}   
