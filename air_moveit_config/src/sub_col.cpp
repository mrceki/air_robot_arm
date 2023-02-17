#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/Point.h>

void planningSceneWorldCallback(const moveit_msgs::PlanningScene::ConstPtr& msg)
{
    if (msg->world.collision_objects.size() > 0) {
    geometry_msgs::Point position = msg->world.collision_objects[0].pose.position;
    ROS_INFO("Received position: (%f, %f, %f)", position.x, position.y, position.z);
    }
    else{
        ROS_INFO("No collision object detected in planning scene!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_scene_world_subscriber");
    ros::NodeHandle nh;

    // Subscribe to the PlanningSceneWorld topic
    ros::Subscriber sub = nh.subscribe<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 10, planningSceneWorldCallback);

    ros::spin();

    return 0;
}