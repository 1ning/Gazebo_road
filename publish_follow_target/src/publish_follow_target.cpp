#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <people_msgs/PersonStamped.h>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Name of the robot (poses are published relative to this)
std::string robot_name;

// Name of the target to follow (as identified in ModelStates message)
std::string target_name;
// Have the old_position, old_velocity and last_time been updated by the callback for this target?
bool has_previous_state;
// Old position of the target (used to determine velocity)
geometry_msgs::Point old_position;
// Old velocity of the target (used if delta_time is zero)
geometry_msgs::Point old_velocity;
// Timestamp of the last message
ros::Time last_time;

// A number close enough to zero used for floating point equality comparisons
const double epsilon = 0.000000005;

// The publisher for the output
ros::Publisher pub;

// Callback when ModelStates are published from gazebo
// Finds the target actor's details and computes the velocity compared to previous position
// and publishes the results as the target to follow
// If anything fails, no output will be published
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();
  geometry_msgs::Point velocity;

  int index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), target_name));
  if (index >= 0 && index < msg->name.size()) {
    if (has_previous_state) {
      ros::Duration duration = current_time - last_time;
      double delta_time = duration.toSec();
      if (delta_time > epsilon) {
        velocity.x = (msg->pose[index].position.x - old_position.x) / delta_time;
        velocity.y = (msg->pose[index].position.y - old_position.y) / delta_time;
        velocity.z = (msg->pose[index].position.z - old_position.z) / delta_time;
      } else {
        // If delta_time is effectively zero, calculations would overflow, so use most recent velocity instead
        velocity.x = old_velocity.x;
        velocity.y = old_velocity.y;
        velocity.z = old_velocity.z;
      }

      //ROS_INFO("dt %f: %s is at (%f, %f, %f) with velocity (%f, %f, %f)", delta_time,  msg->name[index].c_str(),
      //    msg->pose[index].position.x, msg->pose[index].position.y, msg->pose[index].position.z,
      //    velocity.x, velocity.y, velocity.z);

      // Create and publish the message
      people_msgs::PersonStamped person;
      person.header.stamp = ros::Time::now();
      person.header.frame_id = "crosswalk";
      person.person.name = msg->name[index];
      person.person.position.x = msg->pose[index].position.x;
      person.person.position.y = msg->pose[index].position.y;
      person.person.position.z = msg->pose[index].position.z;
      person.person.velocity.x = velocity.x;
      person.person.velocity.y = velocity.y;
      person.person.velocity.z = velocity.z;
      pub.publish(person);
    }

    // Save state (in world coordinates) for next call
    old_position.x = msg->pose[index].position.x;
    old_position.y = msg->pose[index].position.y;
    old_position.z = msg->pose[index].position.z;
    old_velocity.x = velocity.x;
    old_velocity.y = velocity.y;
    old_velocity.z = velocity.z;
    last_time = current_time;
    has_previous_state = true;
  }
}

// Used to change the target - update the name and reset the other global variables
void SetTarget(std::string name)
{
  target_name = name;
  has_previous_state = false;
  old_position.x = 0;
  old_position.y = 0;
  old_position.z = 0;
  old_velocity.x = 0;
  old_velocity.y = 0;
  old_velocity.z = 0;
}


int main(int argc, char** argv)
{
  // Hard coded parameters used by the module (TODO Convert to params in launch file)
  SetTarget("actor_1");
  robot_name = "mrobot";
  std::string topic_name = "/gazebo/model_states";
  std::string output_topic = "/follow";

  // Set up ROS
  ros::init(argc, argv, "publish_follow_target");
  ros::NodeHandle node;

  // Create the publisher
  pub = node.advertise<people_msgs::PersonStamped>("/follow", 1);
  ROS_INFO("Publishing output on %s", output_topic.c_str());

  // Create the subscriber
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 1, callback);
  ROS_INFO("Listening on %s for messages about %s", topic_name.c_str(), target_name.c_str());

  // Loop
  ros::spin();
  return 0;
}
