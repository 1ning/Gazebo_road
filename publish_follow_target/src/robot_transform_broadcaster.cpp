#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>

// The name of the robot model to publish (as given in gazebo)
std::string robot_name;
// The reference frame for the robot
std::string robot_frame;

void callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  static tf2_ros::TransformBroadcaster br;

  ros::Time current_time = ros::Time::now();
  geometry_msgs::TransformStamped transformStamped;

  int index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), robot_name));
  if (index >= 0 && index < msg->name.size())
  {
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = robot_frame;
    transformStamped.transform.translation.x = msg->pose[index].position.x;
    transformStamped.transform.translation.y = msg->pose[index].position.y;
    transformStamped.transform.translation.z = msg->pose[index].position.z;
    transformStamped.transform.rotation.x = msg->pose[index].orientation.x;
    transformStamped.transform.rotation.y = msg->pose[index].orientation.y;
    transformStamped.transform.rotation.z = msg->pose[index].orientation.z;
    transformStamped.transform.rotation.w = msg->pose[index].orientation.w;

    //ROS_INFO("Publishing Transform %s to %s: (%f, %f, %f) (%f, %f, %f, %f)",
    //    transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str(),
    //    transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z,
    //    transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

    br.sendTransform(transformStamped);
  }
}

int main(int argc, char **argv)
{
  robot_name = "mrobot";
  robot_frame = "base_link";

  ros::init(argc, argv, "robot_transform_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 1, callback);

  ros::spin();
  return 0;
};
