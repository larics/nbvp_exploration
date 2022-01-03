#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/volume_srv.h>
#include <nbvplanner/rrt.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ROS_INFO("Started testing");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  ros::Rate rate(1);


  while (ros::ok()) {
    // Call sample path with cubes
    std_srvs::Empty testSrv;
      if (ros::service::call("test", testSrv)) {
        
      }
      else {
        ROS_WARN("Could not call empty service."); 
      }   
    rate.sleep();
  }
};

