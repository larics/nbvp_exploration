#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

std::string child_frame_id;
std::string parent_frame_id;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame_id;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  tf2::Quaternion q;
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  double theta = atan2(2 * (qw * qz + qx * qy),
     qw * qw + qx * qx - qy * qy - qz * qz);
  q.setRPY(0, 0, theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_broadcaster");
  ros::NodeHandle private_node("~");    
   if (!private_node.hasParam("/red/tf_node/child_frame_id"))
     {
       if (argc != 2){ROS_ERROR("need child and parent name as argument"); return -1;};
       child_frame_id = argv[1];
       parent_frame_id = argv[1];
     }
     else
     {
       private_node.getParam("/red/tf_node/child_frame_id", child_frame_id);
       private_node.getParam("/red/tf_node/parent_frame_id", parent_frame_id);
      // || ! private_node.hasParam("tf_node/parent_frame_id"

     }

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("odometry", 10, &odometryCallback);

  ros::spin();
  return 0;
};
