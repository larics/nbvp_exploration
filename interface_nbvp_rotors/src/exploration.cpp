/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

bool current_goal_reached = true;

void pointReachedCallback(std_msgs::Bool msg)
{
  if (msg.data)
  {
    ROS_INFO("Current goal point is reached!");
    current_goal_reached = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectoryPoint
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ros::Publisher goals_pub = nh.advertise < geometry_msgs::PoseArray > ("nbvp/goals", 1);
  ros::Subscriber point_reached_sub = nh.subscribe("nbvp/point_reached", 1, &pointReachedCallback);
  ROS_INFO("Started exploration");

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

  double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }
  // Get parent frame id
  std::string parent_frame_id = " ";
  nh.param<std::string>("exploration/parent_frame_id", parent_frame_id, "map"); 

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  for (double i = 0; i <= 0.5; i = i + 0.1) {
    nh.param<double>("exploration/wp_x", trajectory_point.position_W.x(), 0.0);
    nh.param<double>("exploration/wp_y", trajectory_point.position_W.y(), 0.0);
    nh.param<double>("exploration/wp_z", trajectory_point.position_W.z(), 1.0);
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), M_PI * i);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(trajectory_point_msg);
    ros::Duration(1.0).sleep();
  }
  trajectory_point.position_W.x() -= 2.0;
  trajectory_point.position_W.y() += 0.5;
  // trajectory_point.position_W.z() -= 0.25;
  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();
  n_seq++;
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  samples_array.points.push_back(trajectory_point_msg);
  trajectory_pub.publish(trajectory_point_msg);
  ros::Duration(1.0).sleep();
  
  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  ros::Rate rate(1);
  nbvplanner::volume_srv volumeSrv;
  while (ros::ok()) {
    ros::spinOnce();
    if (current_goal_reached)
    {
      current_goal_reached = false;
      ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
      ROS_INFO("Iteration started at  %10.5fs",  (ros::Time::now()).toSec());
      nbvplanner::nbvp_srv planSrv;
      planSrv.request.header.stamp = ros::Time::now();
      planSrv.request.header.seq = iteration;
      planSrv.request.header.frame_id = parent_frame_id;
      if (ros::service::call("nbvplanner", planSrv)) {
        n_seq++;
        if (planSrv.response.path.size() == 0) {
          ros::Duration(1.0).sleep();
        }
        else {
          geometry_msgs::PoseArray goals;
          goals.header.seq = n_seq;
          goals.header.stamp = ros::Time::now();
          goals.header.frame_id = parent_frame_id;
          for (int i = 0; i < planSrv.response.path.size(); i++) {
            goals.poses.push_back(planSrv.response.path[i]);
          }
          goals_pub.publish(goals);
          // while (!current_goal_reached)
          // {
          //   ros::spinOnce();
          //   ros::Duration(dt).sleep();
          // }
          // current_goal_reached = false;
          // std::cout << "Current goal reached!" << std::endl;
        }
      } else {
        ROS_WARN_THROTTLE(1, "Planner not reachable");
        ros::Duration(1.0).sleep();
      }
    }
    //Call service for calculating and logging volume
    volumeSrv.request.header.stamp = ros::Time::now();
    volumeSrv.request.header.seq = iteration;
    volumeSrv.request.header.frame_id = parent_frame_id;
    if(ros::service::call("volume_service", volumeSrv)){
      ROS_INFO("Volume updated.");
    }
    iteration++;
    rate.sleep();
  }
};

