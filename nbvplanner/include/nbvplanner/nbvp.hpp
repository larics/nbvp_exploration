/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
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

#ifndef NBVP_HPP_
#define NBVP_HPP_

#include <fstream>
#include <eigen3/Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <nbvplanner/nbvp.h>

#include <iostream>
#include <ios>

#include <string.h>

// Convenience macro to get the absolute yaw difference
#define ANGABS(x) (fmod(fabs(x),2.0*M_PI)<M_PI?fmod(fabs(x),2.0*M_PI):2.0*M_PI-fmod(fabs(x),2.0*M_PI))

using namespace Eigen;
using namespace std;

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner(const ros::NodeHandle& nh,
                                                const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{

  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  // Set up the topics and services
  volumeService_ = nh_.advertiseService("volume_service",
                                         &nbvInspection::nbvPlanner<stateVec>::volumeCallback,
                                         this);
  params_.inspectionPath_ = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  plannerService_ = nh_.advertiseService("nbvplanner",
                                         &nbvInspection::nbvPlanner<stateVec>::plannerCallback,
                                         this);
  testService_ = nh_.advertiseService("test", &nbvInspection::nbvPlanner<stateVec>::testCallback, this);
  posClient_ = nh_.subscribe("pose", 10, &nbvInspection::nbvPlanner<stateVec>::posCallback, this);
  odomClient_ = nh_.subscribe("odometry", 10, &nbvInspection::nbvPlanner<stateVec>::odomCallback, this);

  pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 1,
                                  &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf,
                                  this);
  pointcloud_sub_cam_up_ = nh_.subscribe(
      "pointcloud_throttled_up", 1,
      &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamUp, this);
  pointcloud_sub_cam_down_ = nh_.subscribe(
      "pointcloud_throttled_down", 1,
      &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamDown, this);
  volumesPub_ = nh_.advertise<std_msgs::Float64MultiArray>("octomap_volume", 1);
  compTimesPub_ = nh_.advertise<std_msgs::Float64MultiArray>("comp_times", 1);

  if (!setParams()) {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }

  // Initialize the tree instance.
  tree_ = new RrtTree(manager_);
  tree_->setParams(params_);
  peerPosClient1_ = nh_.subscribe("peer_pose_1", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg1, tree_);
  peerPosClient2_ = nh_.subscribe("peer_pose_2", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg2, tree_);
  peerPosClient3_ = nh_.subscribe("peer_pose_3", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg3, tree_);
  // Not yet ready. Needs a position message first.
  ready_ = false;
}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{
  if (manager_) {
    delete manager_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::posCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tree_->setStateFromPoseMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::odomCallback(
    const nav_msgs::Odometry& pose)
{
  tree_->setStateFromOdometryMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::volumeCallback(nbvplanner::volume_srv::Request& req,
                                                         nbvplanner::volume_srv::Response& res)
{
  double totalVolume = (params_.maxX_ - params_.minX_) * 
                       (params_.maxY_ - params_.minY_) * 
                       (1.0 + params_.maxZ_ - params_.minZ_); 

  //Function for displaying & logging change in free/occupied/undiscovered volume
  double volumes[2];
  manager_->calculateVolume(volumes);
  
  // Prepare data to publish
  double freeVolume = *(volumes);
  double occupiedVolume = *(volumes + 1);
  double unmappedVolume = totalVolume - (occupiedVolume + freeVolume);
  double timeNow = ros::Time::now().toSec();
  std_msgs::Float64MultiArray allVolumes;
  allVolumes.data.resize(5);
  allVolumes.data[0] = occupiedVolume;
  allVolumes.data[1] = freeVolume;
  allVolumes.data[2] = totalVolume;
  allVolumes.data[3] = unmappedVolume;
  allVolumes.data[4] = timeNow;
  volumesPub_.publish(allVolumes);
  return true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::plannerCallback(nbvplanner::nbvp_srv::Request& req,
                                                          nbvplanner::nbvp_srv::Response& res)
{
  ros::Time computationStartTime = ros::Time::now();
  // Check that planner is ready to compute path.
  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }
  if (!ready_) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
    return true;
  }
  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
    return true;
  }
  res.path.clear();

  ROS_WARN_STREAM("History size: " << tree_->getHistorySize());
  // Function for resolving dead ends
  if(resolveDeadEnd && ros::ok()){
    ROS_WARN("Resolving dead end...");
    res.path = tree_->getReturnEdge(req.header.frame_id);
    ROS_WARN_STREAM("History dead end size: " << tree_->getHistoryDeadEndSize());
    if(tree_->getHistoryDeadEndSize() == 0){
        ROS_INFO("Successfully returned to the best node. Continue exploration...");
        resolveDeadEnd = false;
    }
    return true;
  }

  // Clear old tree and reinitialize.
  tree_->clear();
  tree_->initialize();
  vector_t path;
  // Iterate the tree construction method.
  int loopCount = 0;
  double computationTime1, computationTime2;
  ros::Time computationStartTime1 = ros::Time::now();
  while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && ros::ok()) {
    if (tree_->getCounter() > params_.cuttoffIterations_){  
      double totalVolume = (params_.maxX_ - params_.minX_) * 
                            (params_.maxY_ - params_.minY_) * 
                            (params_.maxZ_ - params_.minZ_);
      if(params_.resolveDeadEnd_ && 
        !manager_->checkExploredVolumeShare(totalVolume, 0.98) && 
        tree_->getBestGainValue() < params_.threshold_gain_){
        ROS_ERROR("Gain is small and more than 30% unexplored - dead end detected");
        res.path = tree_->getReturnEdge((req.header.frame_id));
        resolveDeadEnd = true;
        return true;
      } else {
          ROS_WARN("Exploration completed, shutting down node");
          // ros::shutdown();
          // system("tmux kill-session -t single_kopter");
          return true;
        }
    }
    if (loopCount > 100000 * (tree_->getCounter() + 1)) {
      ROS_INFO_THROTTLE(1, "Exceeding maximum failed iterations, return to previous point!");
      res.path = tree_->getPathBackToPrevious(req.header.frame_id);
      return true;
    }
    tree_->iterate(1);
    loopCount++;
  }
  computationTime1 = (ros::Time::now() - computationStartTime1).toSec();
  ROS_INFO("While loop lasted %6.15fs", computationTime1);

  // Resolve dead end
  ros::Time computationStartTime2 = ros::Time::now();
  if(params_.resolveDeadEnd_ && tree_->getBestGainValue() < params_.threshold_gain_) {
    double totalVolume = (params_.maxX_ - params_.minX_) * 
                        (params_.maxY_ - params_.minY_) * 
                        (1+ params_.maxZ_ - params_.minZ_); 
    if(!manager_->checkExploredVolumeShare(totalVolume, 0.98)){
      ROS_ERROR("Gain less than threshold, exploration not completed - dead end detected!");
      res.path = tree_->getReturnEdge((req.header.frame_id));
      resolveDeadEnd = true;
      return true;
    }
    ROS_WARN("Gain less than 1e-1 but almost all explored!");
  }
  // computationTime2 = (ros::Time::now() - computationStartTime2).toSec();
  // ROS_INFO("Function for gain lasted %6.15fs", computationTime2);
  // Extract the best edge.
  res.path = tree_->getBestPathNodes(req.header.frame_id);

  tree_->memorizeBestBranch();
  double computationTime;
  double timeNow = ros::Time::now().toSec();
  computationTime = (ros::Time::now() - computationStartTime).toSec();
  ROS_INFO("Path computation lasted %6.15fs", computationTime);

  // Publish computation times
  std_msgs::Float64MultiArray allTimes;
  allTimes.data.resize(3);
  allTimes.data[0] = computationTime1;
  allTimes.data[1] = computationTime2;
  allTimes.data[2] = computationTime;
  compTimesPub_.publish(allTimes);
  return true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::testCallback(std_srvs::EmptyRequest& req,
                                                          std_srvs::EmptyResponse& res)
{
  ros::Time computationStartTime = ros::Time::now();
  // Check that planner is ready to compute path.
  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }
  if (!ready_) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
    return true;
  }
  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
    return true;
  }

  // Clear old tree and reinitialize.
  tree_->clear();
  tree_->initialize();

  nbvInspection::RrtTree * rrtTree = new nbvInspection::RrtTree(manager_); 
  nbvInspection::RrtTree::StateVec startState;
  nbvInspection::RrtTree::StateVec endState;

  startState[0] = -1;
  startState[1] = 0;
  startState[2] = 0.7;

  endState[0] = -2;
  endState[1] = 0;
  endState[2] = 0.7;
  rrtTree->setParams(params_);
  double information_gain = rrtTree->samplePathWithCubes(startState, endState, "mavros/world");
  std::cout << "Information gain: " << information_gain << std::endl;
  return true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 0.25;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
             (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
             (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = {15.0};
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
             (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = {90.0};
  if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
             (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = {60.0};
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
             (ns + "/system/camera/vertical").c_str());
  }
  params_.laserPitch_ = {10.0};
  if (!ros::param::get(ns + "/system/laser/pitch", params_.laserPitch_)) {
    ROS_WARN("No laser pitch specified. Looking for %s. Default is 10.0deg.",
             (ns + "/system/laser/pitch").c_str());
  }
  params_.laserHorizontal_ = {360.0};
  if (!ros::param::get(ns + "/system/laser/horizontal", params_.laserHorizontal_)) {
    ROS_WARN("No laser horizontal opening specified. Looking for %s. Default is 360deg.",
             (ns + "/system/laser/horizontal").c_str());
  }
  params_.laserVertical_ = {30.0};
  if (!ros::param::get(ns + "/system/laser/vertical", params_.laserVertical_)) {
    ROS_WARN("No laser vertical opening specified. Looking for %s. Default is 30deg.",
             (ns + "/system/laser/vertical").c_str());
  }
  if(params_.camPitch_.size() != params_.camHorizontal_.size() ||params_.camPitch_.size() != params_.camVertical_.size() ){
    ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
    params_.camPitch_.clear();
    params_.camPitch_ = {15.0};
    params_.camHorizontal_.clear();
    params_.camHorizontal_ = {90.0};
    params_.camVertical_.clear();
    params_.camVertical_ = {60.0};
  }

  if(params_.laserPitch_.size() != params_.laserHorizontal_.size() ||params_.laserPitch_.size() != params_.laserVertical_.size() ){
    ROS_WARN("Specified laser fields of view unclear: Not all parameter vectors have same length! Setting to default.");
    params_.laserPitch_.clear();
    params_.laserPitch_ = {7.56};
    params_.laserHorizontal_.clear();
    params_.laserHorizontal_ = {360.0};
    params_.laserVertical_.clear();
    params_.laserVertical_ = {30.0};
  }
  
  params_.igProbabilistic_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_)) {
    ROS_WARN(
        "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
        (ns + "/nbvp/gain/probabilistic").c_str());
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/unmapped").c_str());
  }
  params_.igArea_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_)) {
    ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/area").c_str());
  }
  params_.degressiveCoeff_ = 0.25;
  if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_)) {
    ROS_WARN(
        "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
        (ns + "/nbvp/gain/degressive_coeff").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/tree/extension_range").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_)) {
    ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
             (ns + "/nbvp/tree/initial_iterations").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/nbvp/dt", params_.dt_)) {
    ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
             (ns + "/nbvp/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/gain/range").c_str());
  }
  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
    ret = false;
  }
  params_.softBounds_ = false;
  if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_)) {
    ROS_WARN(
        "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
        (ns + "/bbx/softBounds").c_str());
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
             (ns + "/system/bbx/z").c_str());
  }
  params_.cuttoffIterations_ = 200;
  if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_)) {
    ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
             (ns + "/nbvp/tree/cuttoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/zero").c_str());
  }
  params_.threshold_gain_ = 0.1;
  if (!ros::param::get(ns + "/nbvp/gain/threshold", params_.threshold_gain_)) {
    ROS_WARN("No threshold gain value specified. Looking for %s. Default is 0.1.",
             (ns + "/nbvp/gain/threshold").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN(
        "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
        (ns + "/system/bbx/overshoot").c_str());
  }
  params_.gainVisualization_ = false;
  if (!ros::param::get(ns + "/nbvp/gain/visualization", params_.gainVisualization_)) {
    ROS_WARN("Gain visualization is off by default. Turn on with %s: true", (ns + "/nbvp/gain/visualization").c_str());
  }
  params_.resolveDeadEnd_ = false;
  if (!ros::param::get(ns + "/nbvp/tree/resolve_dead_end", params_.resolveDeadEnd_)) {
    ROS_WARN("Return to best node is off by default. Turn on with %s: true", (ns + "/nbvp/tree/resolve_dead_end").c_str());
  }
  params_.log_throttle_ = 0.5;
  if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_)) {
    ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
             (ns + "/nbvp/log/throttle").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
             (ns + "/tf_frame").c_str());
  }
  params_.pcl_throttle_ = 0.333;
  if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.333.",
        (ns + "/pcl_throttle").c_str());
  }
  params_.inspection_throttle_ = 0.25;
  if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the inspection view insertion specified. Looking for %s. Default is 0.1.",
        (ns + "/inspection_throttle").c_str());
  }
  params_.exact_root_ = true;
  if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_)) {
    ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
             (ns + "/nbvp/tree/exact_root").c_str());
  }
  return ret;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamUp(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamDown(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}
#endif // NBVP_HPP_
