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

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <multiagent_collision_check/multiagent_collision_checker.h>
#include <nbvplanner/rrt.h>
#include <nbvplanner/tree.hpp>

nbvInspection::RrtTree::RrtTree()
    : nbvInspection::TreeBase<StateVec>::TreeBase()
{
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvp/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ros::package::getPath("nbvplanner") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager)
{
  mesh_ = mesh;
  manager_ = manager;
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvp/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ros::package::getPath("nbvplanner") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::~RrtTree()
{
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
}

void nbvInspection::RrtTree::setStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setStateFromOdometryMsg(
    const nav_msgs::Odometry& pose)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setPeerStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  geometry_msgs::Pose poseTransformed;
  tf::poseTFToMsg(transform * poseTF, poseTransformed);
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[n_peer] > params_.inspection_throttle_) {
    inspectionThrottleTime_[n_peer] += params_.inspection_throttle_;
    if (mesh_) {
      mesh_->setPeerPose(poseTransformed, n_peer);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, n_peer);
    }
  }
}

void nbvInspection::RrtTree::iterate(int iterations)
{
// In this function a new configuration is sampled and added to the tree.
  StateVec newState;

// Sample over a sphere with the radius of the maximum diagonal of the exploration
// space. Throw away samples outside the sampling region it exiting is not allowed
// by the corresponding parameter. This method is to not bias the tree towards the
// center of the exploration space.
  double radius = sqrt(
      SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
      + SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound = false;
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
      if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    solutionFound = true;
  }

// Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
      nearest);
  kd_res_free(nearest);

// Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                            newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
  if (volumetric_mapping::OctomapManager::CellStatus::kFree
      == manager_->getLineStatusBoundingBox(
          origin, direction + origin + direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)
      && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {
    // Sample the new orientation
    //newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);

    //MAV orientation towards next point .. atan2(dy,dx)
    // newState[3] = atan2(direction[1], direction[0]);

    // Create new node and insert into tree
    nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    /*
    newNode->gain_ = newParent->gain_
        + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);
    */
    if(!params_.updateDegressiveCoeff_){
      degressiveCoeff_ = params_.degressiveCoeff_;
    }

    // Total gain is sum of all cubes along the rrt tree
    newNode->gain_ = newParent->gain_
        + samplePathWithCubes(newNode->state_, newParent->state_, params_.navigationFrame_) 
        * exp(-params_.degressiveCoeff_ * newNode->distance_);
    // std::cout << "Gain: " << newNode->gain_ << std::endl;
    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);
    // Display new node
    publishNode(newNode);

    // Update best IG and node if applicable
    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }
}

void nbvInspection::RrtTree::initialize()
{
// This function is to initialize the tree, including insertion of remainder of previous best branch.
  g_ID_ = 0;
// Remove last segment from segment list (multi agent only)
  int i;
  for (i = 0; i < agentNames_.size(); i++) {
    if (agentNames_[i].compare(params_.navigationFrame_) == 0) {
      break;
    }
  }
  if (i < agentNames_.size()) {
    segments_[i]->clear();
  }
// Initialize kd-tree with root node and prepare log file
  kdTree_ = kd_create(3);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                   std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }
  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(),
             rootNode_);
  iterationCount_++;

// Insert all nodes of the remainder of the previous best branch, checking for collisions and
// recomputing the gain.
  for (typename std::vector<StateVec>::reverse_iterator iter = bestBranchMemory_.rbegin();
      iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
        nearest);
    kd_res_free(nearest);

    // Check for collision
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
        == manager_->getLineStatusBoundingBox(
            origin, direction + origin + direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)
        && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_,
                                      segments_)) {
      // Create new node and insert into tree
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      /*
      newNode->gain_ = newParent->gain_
          + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);
          */
      if(!params_.updateDegressiveCoeff_){
        degressiveCoeff_ = params_.degressiveCoeff_;
      }
      newNode->gain_ = newParent->gain_
        + samplePathWithCubes(newNode->state_, newParent->state_, params_.navigationFrame_)
        * exp(-params_.degressiveCoeff_ * newNode->distance_);
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // Display new node
      publishNode(newNode);

      // Update best IG and node if applicable
      //
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

// Publish visualization of total exploration area
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getBestEdge(std::string targetFrame)
{
// This function returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  nbvInspection::Node<StateVec> * current = bestNode_;
  ROS_INFO("Best Gain: %4.15f", bestNode_->gain_);
  publishBestNode();
  //Resursive method for finding next point for UAV to move towards 
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      current = current->parent_;
    }
    publishCurrentNode(current);
    ret = samplePath(current->parent_->state_, current->state_, targetFrame);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getBestPathNodes(std::string targetFrame)
{
// This function returns the nodes of the best branch
  std::vector<geometry_msgs::Pose> ret;
  branchHistory_.clear();
  nbvInspection::Node<StateVec> * current = bestNode_;
  ROS_INFO("Best Gain: %4.15f", bestNode_->gain_);
  publishBestNode();
  //Resursive method for collecting all nodes on the best branch
  geometry_msgs::Pose node_pose;
  node_pose.position.x = current->state_[0];
  node_pose.position.y = current->state_[1];
  node_pose.position.z = current->state_[2];
  ret.push_back(node_pose);
  // Exact root is the best node
  exact_root_ = current->state_;
  ROS_INFO("Tu sam");   
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      node_pose.position.x = current->parent_->state_[0];
      node_pose.position.y = current->parent_->state_[1];
      node_pose.position.z = current->parent_->state_[2];
      ret.push_back(node_pose);
      publishCurrentNode(current->parent_);
      branchHistory_.push_back(current->parent_->state_);
      current = current->parent_;
    }

    // Save into history from back to front, to kepp returning to the closest node from bestNode
    if (branchHistory_.size() != 0){
      for (std::list<StateVec>::reverse_iterator it = branchHistory_.rbegin(); it != branchHistory_.rend(); it++){
        history_.push(*it);
      }
    }
    // Reverse vector
    std::reverse(ret.begin(), ret.end());
    // ret = samplePath(current->parent_->state_, current->state_, targetFrame);  
  }
  return ret;
}

//Function for returning to origin
std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getReturnEdge(std::string targetFrame)
{
  if(callOnce){
    callOnce = setGoal();  //false
  }
  bool foundShortest = false;
  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    ROS_INFO("History empty!");
    exact_root_ = goal_;
    return ret;
  }
  foundShortest = findShortestPath(goal_);
  if(boolFirstSeen_){
    shortest_ = firstSeen_;
  } else {
    ROS_INFO("Shortest not found, returning to previous point!");
    shortest_ = history_.top();
    }
  publishReturnNode(shortest_);
  ret = samplePath(root_, shortest_, targetFrame);
  history_.pop();
  if(history_.empty()){
    exact_root_ = goal_;
  }
  return ret;
}

//Recursive function returns first node from origin that is collision free
//Nodes from first found to MAV root are removed
bool nbvInspection::RrtTree::findShortestPath(StateVec goal){
  //Find shortest path to origin
  bool found;
  boolFirstSeen_ = false;
  if (history_.empty()) {
    return false;
  }

  StateVec newState;
  StateVec state = history_.top();
  history_.pop();

  //Recursion
  found = findShortestPath(goal_);

  Eigen::Vector3d origin(root_[0], root_[1], root_[2]);
  Eigen::Vector3d direction(state[0] - origin[0], state[1] - origin[1],
                              state[2] - origin[2]);
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
    == manager_->getLineStatusBoundingBox(
        origin, direction + origin + direction.normalized() * params_.dOvershoot_,
        params_.boundingBox_)) {
      if(!boolFirstSeen_){
        firstSeen_ = state;
        boolFirstSeen_ = true;
        found = true;
        history_.push(state);
      }
    }
    //remove remaining nodes from stack
    if(!found){
      history_.push(state);
    }
  return found;
}

bool nbvInspection::RrtTree::setGoal(){
  if (history_.empty()) {
    return true;
  }
  StateVec state = history_.top();
  history_.pop();
  bool foundLast = setGoal();

  if(foundLast){
    goal_ = state;
    ROS_INFO("Goal: X = %f, Y = %f, Z = %f, Yaw = %f",
              goal_[0], goal_[1], goal_[2], goal_[3]);
  }
  history_.push(state);
  return false;
}


int nbvInspection::RrtTree::getHistorySize(){
  return history_.size();
}

void nbvInspection::RrtTree::updateDegressiveCoeff(){
  double *dV;
  //Calculate volume derivation
  //dV[0] = dVfree ; dV[1] = dVoccupied
  dV = manager_->getDerivation();
  double totalVolume = (params_.maxX_ - params_.minX_) * 
                       (params_.maxY_ - params_.minY_) * 
                       (1.0 + params_.maxZ_ - params_.minZ_); 

/*

... TO DO ...

*/

  degressiveCoeff_ = params_.degressiveCoeff_;
  ROS_INFO("[STATUS]Degressive Coeff.: %f", degressiveCoeff_);

/*
    if(params_.updateDegressiveCoeff){
      std::string logFilePath = ros::package::getPath("nbvplanner") + "/Degressive Coeff/";
      system(("mkdir -p " + logFilePath).c_str());
      logFilePath += "/";

      //setting path..
      std::ofstream fileDegCoeff((logFilePath + "degressiveCoeff.txt").c_str(), std::ios::app | std::ios::out);
      std::ofstream fileStep((logFilePath + "step.txt").c_str(), std::ios::app | std::ios::out);
      //writing data..
      fileDegCoeff << degressiveCoeff_ << "\n";
      fileStep << iterationCount_ << "\n";
    }
*/
}

void nbvInspection::RrtTree::visualizeGain(Eigen::Vector3d vec)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = v_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = v_ID_;
  v_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = vec[0];
  p.pose.position.y = vec[1];
  p.pose.position.z = vec[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 1.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 0.1;
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 1.0;
  p.color.g = 0.9;
  p.color.b = 0.3;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::visualizeGainRed(Eigen::Vector3d vec)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = vr_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = vr_ID_;
  vr_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = vec[0];
  p.pose.position.y = vec[1];
  p.pose.position.z = vec[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 1.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 0.1;
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 1.0;
  p.color.g = 0.0;
  p.color.b = 0.0;
  p.color.a = 0.6;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::visualizeCuboid(StateVec start, StateVec end)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = v_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = v_ID_;
  v_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  // Parent
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  // Current - parent
  Eigen::Vector3f dir(start[0] - end[0],
                      start[1] - end[1],
                      start[2] - end[2]);
  // The center of the cube is in the middle
  p.pose.position.x = end[0] + dir[0]/2;
  p.pose.position.y = end[1] + dir[1]/2;
  p.pose.position.z = end[2] + dir[2]/2;
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 5;
  p.scale.z = 5;
  p.color.r = 1.0;
  p.color.g = 0.4;
  p.color.b = 0.8;
  p.color.a = 0.5;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

double nbvInspection::RrtTree::gain(StateVec state)
{
// This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  Eigen::Vector3d pom;
  Eigen::Vector3d pom2;
  Eigen::Vector4d rootPom;
  rootPom[0] = root_[0];
  rootPom[1] = root_[1];
  rootPom[2] = root_[2] - 0.7; //sensor offset error correction
  rootPom[3] = root_[3];
  double rangeSq = pow(params_.gainRange_, 2.0);
// Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
      vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
        vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
          vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        //Calculating angle between two vectors with origin in root_
        //Shifting both vectors in same origin ( root_ )
        //MAV is located in root_ when new nodes are being calculated
        int i;
        for(i = 0; i < sizeof(vec)/sizeof(vec[0]); i++){
          pom2[i] = vec[i] - rootPom[i];
          pom[i] = pom2[i];
        }
        pom[2] = rootPom[2];

        double minRange = sqrt(pow(pom2[0],2) + pow(pom2[1],2) + pow(pom2[2],2));
        if (minRange < 1.0){
          continue;
        }
        //Math equations for angle between two vectors in 3D space
        //Angle between vector and its orthogonal projection on xy-plane
        double num = pom2[0]*pom[0] + pom2[1]*pom[1] + pom2[2]*pom[2];
        double denum = sqrt( pow(pom2[0],2) + pow(pom2[1],2) + pow(pom2[2],2) ) 
                      * sqrt( pow(pom[0],2) + pow(pom[1],2) + pow(pom[2],2) );
        double normalAngle = acos(num/denum) * 180 / M_PI;
        double x2 = pom[0] * cos(-rootPom[3]) - pom[1] * sin(-rootPom[3]);
        double y2 = pom[0] * sin(-rootPom[3]) + pom[1] * cos(-rootPom[3]);

        //Recommended to set pitch for VLP-16 in model config to see the floor
        //default pitch is set to 10 deg
          bool aboveZ;
          bool pitchForward;
          //Checking whether RRT is in first or fourth quadrant with root as new local origin
          //Laser points towards x axis and has a pitch of 10.0 degrees by default
          if ( ((x2 > rootPom[0]) && (y2 > rootPom[1]) ) || ((x2 > rootPom[0]) && (y2 < rootPom[1])) ) {
            pitchForward = true;
          } else { 
              pitchForward = false;
            }
          //Checking if dot is above/below xy-plane
          if (vec[2] > rootPom[2]){
            aboveZ = true;
          } else {
              aboveZ = false;
            }
            //Calculating new angles with pitch included
            if(pitchForward && aboveZ) {
              if (normalAngle > (params_.laserVertical_[0]/2 - params_.laserPitch_[0]) ) {
                continue;
              }
            } else if (pitchForward && !aboveZ){
                if(normalAngle > (params_.laserVertical_[0]/2 + params_.laserPitch_[0]) ) {
                  continue;
                }
            } else if (!pitchForward && aboveZ){
                if(normalAngle > (params_.laserVertical_[0]/2 + params_.laserPitch_[0]) ) {
                  continue;
                }
            } else if (!pitchForward && !aboveZ) {
                if(normalAngle > (params_.laserVertical_[0]/2 - params_.laserPitch_[0]) ) {
                  continue;
                }
              }

        // Check cell status and add to the gain considering the corresponding factor.
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
            vec, &probability);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igUnmapped_;
            // ETHZCommentTODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        } else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igOccupied_;
            // ETHZcomment:TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        } else {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igFree_;
            // ETHZcomment:TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        }
        //Gain visualization
        if(params_.gainVisualization_){
          if(node == volumetric_mapping::OctomapManager::CellStatus::kUnknown &&
            volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)){
            visualizeGainRed(vec);
          } else {
            visualizeGain(vec);
          }
        }
      }
    }
  }
// Scale with volume
  gain *= pow(disc, 3.0);

//Area exploration
// Check the gain added by inspectable surface
/*  if (mesh_) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));
    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, state[3]);
    transform.setRotation(quaternion);
    gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
  }
  //-------------redundant--------
  */
  return gain;
}

// double nbvInspection::RrtTree::gainCube(StateVec state, double distance, double a)
// {
//   ros::Time computationStartTime1 = ros::Time::now();
//   // This function computes the gain inside the cuboid with the origin in the state
//   double gain = 0.0;
//   const double resolution = manager_->getResolution();
//   Eigen::Vector3d origin(state[0], state[1], state[2]);
//   Eigen::Vector3d vec;
//   int unknownNum = 0;
//   // All points in 3D space
//   int allNum = 0;
//   int allNum = (a / resolution) * (a / resolution) * (a / resolution);
  
//   static double multipliers[4][8] = {
//     {1, 0, 0, -1, -1, 0, 0, 1},
//     {0, 1, -1, 0, 0, -1, 1, 0},
//     {0, 1, 1, 0, 0, -1, -1, 0},
//     {1, 0, 0, 1, -1, 0, 0, -1}
//   };
//   // Calculate the number of unknown cells inside a box around the node
//   // Gain is propotional to number of the unknown cells inside a box
//   // a is a side length of a cube
//   // Iterate over all nodes within the allowed distance
//   // for (vec[0] = std::max(state[0], params_.minX_);
//   //     vec[0] < std::min(state[0] + (a/2), params_.maxX_); vec[0] += disc) {
//   //   for (vec[1] = std::max(state[1], params_.minY_);
//   //       vec[1] < std::min(state[1] + (a/2), params_.maxY_); vec[1] += disc) {
//   //     for (vec[2] = std::max(state[2], params_.minZ_);
//   //         vec[2] < std::min(state[2] + (a/2), params_.maxZ_); vec[2] += disc) {
//   //       allNum++;
//   //       // Check cell status and add to the gain considering the corresponding factor.
//   //       double probability;
//   //       volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
//   //           vec, &probability);

//   //       if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
//   //         unknownNum++;
//   //         //Gain visualization
//   //         if(params_.gainVisualization_){
//   //           visualizeGainRed(vec);
//   //         }
//   //       }
//   //     }
//   //   }
//   // } 

//   // Go through all octants; find the sum of unknown cells
//   for (uint i = 0; i < 8; i++)
//   {
//     unknownNum += castUnknown(state, a, distance, 1.0, 1.0, 0.0, multipliers[0][i],
//                 multipliers[1][i], multipliers[2][i], multipliers[3][i]);
//   }
//   // std::cout << "Value of unknown: " << unknownNum << std::endl;

//   // Scale with volume
//   // gain *= pow(disc, 3.0);
  
//   gain = (double)unknownNum / (double)allNum;
//   double computationTime;
//   computationTime = (ros::Time::now() - computationStartTime1).toSec();
//   // ROS_INFO("Gain cube method lasted %6.15fs", computationTime);
//   return gain;
// }

double nbvInspection::RrtTree::gainCuboid(StateVec state, double distance, double a)
{
  // Computes the number of unknown cells inside the cuboid with the origin in the state
  double gain = 0.0;
  const double resolution = manager_->getResolution();
  int unknownNum = 0;
  // All points in 3D space
  double allNum = (distance / resolution) * (a / resolution) * (a / resolution);

  static double multipliers[4][8] = {
    {1, 0, 0, -1, -1, 0, 0, 1},
    {0, 1, -1, 0, 0, -1, 1, 0},
    {0, 1, 1, 0, 0, -1, -1, 0},
    {1, 0, 0, 1, -1, 0, 0, -1}
  };

  // Go through all octants; find the sum of unknown cells
  for (uint i = 0; i < 8; i++)
  {
    unknownNum += castUnknown(state, a, distance, 1.0, 1.0, 0.0, multipliers[0][i],
                multipliers[1][i], multipliers[2][i], multipliers[3][i]);
  }

  double volume = unknownNum * pow(resolution, 3.0); 
  gain = volume;
  // gain = (double)unknownNum / (double)allNum;
  return gain;
}

int nbvInspection::RrtTree::castUnknown(StateVec state, double a, double distance, double row, 
  double start_slope, double end_slope, double xx, double xy, double yx, double yy)
{
  // Returns number of unknown in one octant
  int unknownNum = 0;
  const double resolution = manager_->getResolution();
  if (start_slope < end_slope) {
    return 0;
  }

  double next_start_slope = start_slope;
  for (double z = -a / 2; z <= a / 2; z += resolution){
    for (double i = row; i <= a / 2; i += resolution) {
      bool blocked = false;
      for (double dx = -i, dy = -i; dx <= 0; dx += resolution) {
        // Slide l_slope and r_slope in one row, left goes through the first point
        // the right one 45 degrees down
        double l_slope = (dx - 0.5) / (dy + 0.5);
        double r_slope = (dx + 0.5) / (dy - 0.5);
        if (start_slope < r_slope) {
          continue;
        } else if (end_slope > l_slope) {
            break;
        }
        double sax = dx * xx + dy * xy;
        double say = dx * yx + dy * yy;
        if (sax < 0 || say < 0)
        {
          continue;
        }
        // Global position
        double ax = state[0] + sax;
        double ay = state[1] + say;
        // Check if the calculated point is out of the map bounding box given in yaml
        if (ax >= params_.maxX_ || ax < params_.minX_ ||
          ay >= params_.maxY_ || ay < params_.minY_ ||
          z >= params_.maxZ_ || z < params_.minZ_) {
          continue;
        }

        Eigen::Vector3d vec(ax, ay, z);
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
          vec, &probability);
        // If the point is inside the cuboid
        if (std::abs(sax) < distance / 2 && std::abs(say) < a / 2) {
          // Mark the unknown cell
          if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          unknownNum++;
            //Gain visualization
            if(params_.gainVisualization_){
              visualizeGainRed(vec);
            }
          }
        }
        if (blocked) {
          if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
            next_start_slope = r_slope;
            continue;
          } else {
            blocked = false;
            start_slope = next_start_slope;
            }
        } else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
            blocked = true;
            next_start_slope = r_slope;
            castUnknown(state, a, distance, i + 1, start_slope, l_slope, xx, xy, yx, yy);
          }
      }
      if (blocked) {
          break;
      }
    }
  }
  return unknownNum;
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getPathBackToPrevious(
    std::string targetFrame)
{
  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  
  // If root is in history_
  if (history_.size() == 1) {
  return ret;
  }
  
  // Return to the previous node; Original = return to root
  geometry_msgs::Pose node_pose;
  node_pose.position.x = history_.top()[0];
  node_pose.position.y = history_.top()[1];
  node_pose.position.z = history_.top()[2];
  ret.push_back(node_pose);
  history_.pop();
  return ret;
}

void nbvInspection::RrtTree::memorizeBestBranch()
{
  bestBranchMemory_.clear();
  Node<StateVec> * current = bestNode_;
  while (current->parent_ && current->parent_->parent_) {
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void nbvInspection::RrtTree::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;

  kd_free(kdTree_);
}

void nbvInspection::RrtTree::publishNode(Node<StateVec> * node)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ / 20.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(20.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (!node->parent_)
    return;

  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

void nbvInspection::RrtTree::publishBestNode()
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = bestNode_->state_[0];
  p.pose.position.y = bestNode_->state_[1];
  p.pose.position.z = bestNode_->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, bestNode_->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(bestNode_->gain_ / 10.0, 0.5);
  p.scale.y = 0.5;
  p.scale.z = 0.5;
  p.color.r = 1.0;
  p.color.g = 1.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(20.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::publishCurrentNode(Node<StateVec> * node)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ / 10.0, 0.5);
  p.scale.y = 0.5;
  p.scale.z = 0.5;
  p.color.r = 0.0;
  p.color.g = 1.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(20.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::publishReturnNode(StateVec node)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = ret_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = ret_ID_;
  ret_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node[0];
  p.pose.position.y = node[1];
  p.pose.position.z = node[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 0.5;
  p.scale.y = 0.5;
  p.scale.z = 0.5;
  p.color.r = 1.0;
  p.color.g = 0.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(50.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::samplePath(StateVec start, StateVec end,
                                                                    std::string targetFrame)
{
  std::vector<geometry_msgs::Pose> ret;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return ret;
  }
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                         params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
  assert(disc > 0.0);
  for (double it = 0.0; it <= 1.0; it += disc) {
    tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1],
                       (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI)
      yaw -= 2.0 * M_PI;
    if (yaw < -M_PI)
      yaw += 2.0 * M_PI;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    origin = transform * origin;
    quat = transform * quat;
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);
    if (params_.log_) {
      filePath_ << poseTF.getOrigin().x() << ",";
      filePath_ << poseTF.getOrigin().y() << ",";
      filePath_ << poseTF.getOrigin().z() << ",";
      filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
    }
  }
  return ret;
}

double nbvInspection::RrtTree::samplePathWithCubes(StateVec start, StateVec end,
                                                    std::string targetFrame)
{
  double gain = 0;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return gain;
  }
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);

  // If start and end point are the same
  if (distance.norm() == 0){
    return gain;
  }
  
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  tf::Vector3 origin(cos(start[3]) * (start[0] + distance[0]/2), 
                      sin(start[3]) * (start[1] + distance[1]/2),
                      start[2] + distance[2]/2);
  origin = transform * origin;
  StateVec state;
  state[0] = origin[0];
  state[1] = origin[1];
  state[2] = origin[2];
  gain += gainCuboid(state, distance.norm(), params_.gainRange_);
  // Visualize cuboid on the path segment
  // visualizeCuboid(start, end);
  return gain;
}

#endif
