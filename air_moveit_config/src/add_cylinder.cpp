#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

ros::Subscriber sub;

// Callback function for the subscriber
void cylinderCallback(const geometry_msgs::Vector3::ConstPtr& dimensions)
{
  // Get the cylinder dimensions from the message
  float radius = dimensions->x;
  float height = dimensions->y;
  float x = dimensions->z;

  // Create the cylinder shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = height;
  primitive.dimensions[1] = radius;

  // Specify the pose of the cylinder
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = 0.0;
  pose.position.z = 0.5;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "cylinder";

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Add the cylinder to the planning scene
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Unsubscribe after receiving the first message
  sub.shutdown();
}

int x_offset, y_offset, z_offset;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Check if the XYZ fields have been set
  if (x_offset == -1)
  {
    for (int i = 0; i < msg->fields.size(); i++)
    {
      if (msg->fields[i].name == "x")
      {
        x_offset = i;
      }
      else if (msg->fields[i].name == "y")
      {
        y_offset = i;
      }
      else if (msg->fields[i].name == "z")
      {
        z_offset = i;
      }
    }
  }
  
  // Convert the PointCloud2 message to PCL data type
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // Get the XYZ data for a specific pixel
  int r = 100; // specify the row of the pixel you are interested in
  int c = 200; // specify the column of the pixel you are interested in

  pcl::PointXYZ p = cloud.at(c, r);
  float x = p.x;
  float y = p.y;
  float z = p.z;

  ROS_INFO("The XYZ data for pixel at (%d, %d) is: (%f, %f, %f)", r, c, x, y, z);
  return x, y, z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_add_cylinder_collision_object_demo");
  ros::NodeHandle node_handle;

  // Subscribe to the cylinder dimensions topic
  sub = node_handle.subscribe("cylinder_dimensions", 10, cylinderCallback);

  ros::spin();

  return 0;
}