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

void air_object::radius_estimator_callback(const std_msgs::Float32ConstPtr& radius)
{
  m_radius = radius->data*0.01;
}



// Callback function for the subscriber
void air_object::sphereCallback(const std_msgs::Float32MultiArray::ConstPtr& xyz)
{ 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Get the sphere dimensions from the message
  float x = xyz->data[0];
  float y = xyz->data[1];
  float z = xyz->data[2];
  ROS_INFO("Object added to planning scene: X: %f  Y: %f  Z: %f",x,y,z);
  // Create the sphere shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[0] = m_radius;

  // Specify the pose of the sphere
  geometry_msgs::Pose pose;
  // To fix data transformations -> x = z, y = -x, z = -y
  float cam_degree = 30; // Angle between robot and camera
  pose.position.x = z + (((m_radius/2) / sin(((90-cam_degree) * M_PI)/180))*2);
  pose.position.y = -x; 
  pose.position.z = -y + (m_radius/4); //hotfix for collision

  // Orientation fix (to do: cam_degree)
  pose.orientation.w = 0.9659258;
  pose.orientation.y = -0.258819;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "camera_link";
  collision_object.id = "sphere";

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Add the sphere to the planning scene
  planning_scene_interface.applyCollisionObjects(collision_objects);
  
  // Unsubscribe after receiving the first message
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_add_sphere_collision_object_demo");
  ros::NodeHandle nh;
  air_object object(nh);

  ros::spin();
  return 0;
}