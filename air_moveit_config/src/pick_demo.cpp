
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "../include/pick_demo.hpp"

air_pick::air_pick(ros::NodeHandle& nh)
  : m_NodeHandle(nh) 
{
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
  ros::Duration(4.0).sleep();
  m_cord_sub = m_NodeHandle.subscribe<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1000, &air_pick::planningSceneWorldCallback,this);
  
  ROS_INFO("Initializing completed.");
  }


void air_pick::pick(){

  move_group->setPlanningTime(45.0);
  geometry_msgs::Pose pick_pose, place_pose;
  geometry_msgs::PoseStamped current_pose;

  ROS_INFO("Object Pose: x= %f, y= %f, z= %f ",m_x,m_y,m_z);

  // Pick Pose
  current_pose = move_group->getCurrentPose("tool0");
  pick_pose.position.x = m_x;
  pick_pose.position.y = m_y;
  pick_pose.position.z = m_z + 0.05;
  pick_pose.orientation = current_pose.pose.orientation;

  // Place Pose
  place_pose.position.x = m_x;
  place_pose.position.y = m_y + 0,05;
  place_pose.position.z = m_z + 0.1;
  move_group->setPoseTarget(pick_pose);

  // Plan and execute the pick motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode result = move_group->plan(plan);
  
  if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    move_group->execute(plan);
  }
  else
  {
    ROS_ERROR("Failed to plan pick motion");
  }
  
  ros::shutdown;
}


// Callback function for the subscriber
void air_pick::planningSceneWorldCallback(const moveit_msgs::PlanningScene::ConstPtr& msg)
{ 
  ROS_INFO("CALLBACK");
  // Get coordinates of object in world
  if (msg->world.collision_objects.size() > 0) {
      geometry_msgs::Point position = msg->world.collision_objects[0].pose.position;
      m_x = position.x;
      m_y = position.y;
      m_z = position.z;
      ROS_INFO("Received position: (%f, %f, %f)", position.x, position.y, position.z);
  }
  else{
      ROS_INFO("No collision object detected in planning scene!");
  }

  // Kill collision updater node
  //system("rosnode kill air_collision_publisher");

  pick(); 
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;
  air_pick obj(nh);
  ros::spinOnce();
  return 0;
}