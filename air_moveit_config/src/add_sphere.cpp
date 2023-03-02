#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

ros::Subscriber sub;


// Callback function for the subscriber
void sphereCallback(const std_msgs::Float32MultiArray::ConstPtr xyz)
{ 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Get the sphere dimensions from the message
  float radius = 0.04; // Radius of sphere
  float x = xyz->data[0];
  float y = xyz->data[1];
  float z = xyz->data[2];
  ROS_INFO("Object added to planning scene: X: %f  Y: %f  Z: %f",x,y,z);
  // Create the sphere shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[0] = radius;

  // Specify the pose of the sphere
  geometry_msgs::Pose pose;
  // To fix data transformations -> x = z, y = -x, z = -y
  float cam_degree = 30; // Angle between robot and camera
  pose.position.x = z + (((radius/2) / sin(((90-cam_degree) * M_PI)/180))*2);
  pose.position.y = -x; 
  pose.position.z = -y + radius; //hotfix for collision

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
  ros::NodeHandle node_handle;
  // Subscribe to the sphere dimensions topic
  sub = node_handle.subscribe("/coordinates_of_object", 10, sphereCallback);
  ros::spin();
  return 0;
}