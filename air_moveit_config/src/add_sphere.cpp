#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include "../include/detect_and_add.hpp"



air_object::air_object(ros::NodeHandle& nh)
  : m_NodeHandle(nh) 
{
  ros::Subscriber sub;
  m_estimator_sub = m_NodeHandle.subscribe("/radius_estimator", 1000, &air_object::radius_estimator_callback, this);
  m_coordinate_sub = m_NodeHandle.subscribe("/coordinates_of_object", 1000, &air_object::sphereCallback, this);
}

void air_object::radius_estimator_callback(const std_msgs::Float32MultiArrayConstPtr& radius)
{
  m_radius = radius->data;
}



// Callback function for the subscriber
void air_object::sphereCallback(const vision_msgs::Detection3DArrayConstPtr& detections)
{ 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // int id_counter = 1;
  int detection_counter = 1;
  for (const auto& detection : detections->detections)
  { 
    for (const auto& result : detection.results)
    {
      float x = result.pose.pose.position.x;
      float y = result.pose.pose.position.y;
      float z = result.pose.pose.position.z;
      std::string id = std::to_string(result.id);

      ROS_INFO("Object added to planning scene: ID: %s  X: %f  Y: %f  Z: %f", id.c_str(), x, y, z);
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      float radius;
      //if(m_radius.size() >= detection_counter -1)
      radius = m_radius[detection_counter-1] * 0.01;
      primitive.dimensions[0] = radius;
      ROS_INFO("Radius: %f", radius);

      geometry_msgs::Pose pose;
      float cam_degree = 30; // Angle between robot and camera
      pose.position.x = z + (((radius/2) / sin(((90-cam_degree) * M_PI)/180))*2);
      pose.position.y = -x; 
      pose.position.z = -y + (radius/4); //hotfix for collision

      // Orientation fix (to do: cam_degree)
      pose.orientation.w = 0.9659258;
      pose.orientation.y = -0.258819;

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "camera_link";

      // if (detection_counter > 1)
      // {
      // id += "_" + std::to_string(detection_counter);
      // }
    
      detection_counter++;
      
      collision_object.id = id;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);
    }
    // id_counter++;
  }
  if (!collision_objects.empty()){
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("COLLISION PUBLISHED");
  }
  else{
    ROS_INFO("COLLISION EMPTY");
    return;
  }
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_add_sphere_collision_object_demo");
  ros::NodeHandle nh;
  air_object object(nh);

  ros::spin();
  return 0;
}