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
}

nbvInspection::RrtTree::RrtTree(volumetric_mapping::OctomapManager * manager)
{
  manager_ = manager;
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
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
          params_.boundingBox_)) {
    // Create new node and insert into tree
    nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    // Total gain is sum of all cubes along the rrt tree
    newNode->gain_ = newParent->gain_
        + samplePathWithCubes(newNode->state_, newParent->state_, params_.navigationFrame_) 
        * exp(-params_.degressiveCoeff_ * newNode->distance_);
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
            params_.boundingBox_)) {
      // Create new node and insert into tree
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);

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
  }
  return ret;
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getReturnEdge(std::string targetFrame)
{
  if(callOnce){
    // Fill histroyDeadEnd
    StateVec return_node = getReturnNode();
    callOnce = setGoal();  //false
  }
  bool foundShortest = false;
  std::vector<geometry_msgs::Pose> ret;
  if (historyDeadEnd_.empty()) {
    ROS_INFO("History dead end empty!");
    callOnce = false;
    exact_root_ = goal_;
    return ret;
  }
  foundShortest = findShortestPath();
  if(boolFirstSeen_){
    shortest_ = firstSeen_;
  } else {
    ROS_INFO("Shortest not found, returning to previous point!");
    shortest_ = historyDeadEnd_.top();
  }
  publishReturnNodePom(shortest_);
  historyDeadEnd_.pop();
  if(historyDeadEnd_.empty()){
    exact_root_ = goal_;
  }
 
  // Collect all nodes from root_ to the best node to return to  
  geometry_msgs::Pose pose;
  pose.position.x = shortest_.x();
  pose.position.y = shortest_.y();
  pose.position.z = shortest_.z();
  ret.push_back(pose);
  return ret;
}

// Recursive function returns first node from origin that is collision free
// Nodes from first found to MAV root are removed
bool nbvInspection::RrtTree::findShortestPath(){
  // Find shortest path to origin
  bool found;
  boolFirstSeen_ = false;
  if (historyDeadEnd_.empty()) {
    return false;
  }

  StateVec state = historyDeadEnd_.top();
  historyDeadEnd_.pop();

  // Recursion
  found = findShortestPath();

  Eigen::Vector3d origin(root_[0], root_[1], root_[2]);
  Eigen::Vector3d direction(state[0] - origin[0], state[1] - origin[1],
                              state[2] - origin[2]);
  if (volumetric_mapping::OctomapManager::CellStatus::kFree
  == manager_->getLineStatusBoundingBox(
      origin, direction + origin + direction.normalized() * params_.dOvershoot_,
      params_.boundingBox_)) {
    if(!boolFirstSeen_){
      firstSeen_ = state;
      boolFirstSeen_ = true;
      found = true;
      historyDeadEnd_.push(state);
    }
  }
  // Remove remaining nodes from stack
  if(!found){
    historyDeadEnd_.push(state);
  }
  return found;
}

bool nbvInspection::RrtTree::setGoal(){
  if (historyDeadEnd_.empty()) {
    return true;
  }
  StateVec state = historyDeadEnd_.top();
  historyDeadEnd_.pop();
  bool foundLast = setGoal();

  if(foundLast){
    goal_ = state;
    ROS_INFO("Goal: X = %f, Y = %f, Z = %f, Yaw = %f",
              goal_[0], goal_[1], goal_[2], goal_[3]);
  }
  historyDeadEnd_.push(state);
  return false;
}

nbvInspection::RrtTree::StateVec nbvInspection::RrtTree::getReturnNode(){
  // [1,2,3,4,5,6]
  std::stack<StateVec> history_local = history_;

  double gain_max = params_.zero_gain_;
  std::vector<StateVec> history_vector;
  StateVec state_to_return;
  if (history_local.empty()) {
    ROS_WARN("History local is empty.");
    return state_to_return;
  }
  // Find the best node from history using shadowcasting in each node from history
  int history_size = history_local.size();
  int index_gain_max = 0;
  for (int i = 0; i < history_size; i++) {
    StateVec state = history_local.top();
    history_local.pop();
    // [6,5,4,3,2,1]
    history_vector.push_back(state);
    double gain = gainCuboid(state, params_.gainRange_, params_.gainRange_);
    if (gain > gain_max){
      gain_max = gain;
      index_gain_max = i;
      state_to_return = state;
    }  
  }
  bool find_best_node_in_vector = false;
  // Stack looks like [1,2,3..], the nearest is on the top
  for (int i = history_size - 1; i >= 0; i--) {
    if (i == index_gain_max) find_best_node_in_vector = true;
    if (find_best_node_in_vector) {
      historyDeadEnd_.push(history_vector[i]);
    }
  }
  goal_ = state_to_return;
  ROS_INFO("State to return: X = %f, Y = %f, Z = %f",
          goal_[0], goal_[1], goal_[2]);
  publishReturnNode(goal_);
  ROS_INFO_STREAM("Size of historyDeadEnd_: " << historyDeadEnd_.size());
  ROS_INFO_STREAM("Size of history_vector: " << history_vector.size());
  return state_to_return;
}

int nbvInspection::RrtTree::getHistorySize(){
  return history_.size();
}

int nbvInspection::RrtTree::getHistoryDeadEndSize(){
  return historyDeadEnd_.size();
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

void nbvInspection::RrtTree::visualizeCenter(Eigen::Vector3d vec)
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
  p.scale.x = 0.5;
  p.scale.y = 0.5;
  p.scale.z = 0.5;
  p.color.r = 0.0;
  p.color.g = 0.0;
  p.color.b = 1.0;
  p.color.a = 1.0;
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
  p.scale.x = params_.gainRange_;
  p.scale.y = params_.gainRange_;
  p.scale.z = params_.gainRange_;
  p.color.r = 1.0;
  p.color.g = 0.4;
  p.color.b = 0.8;
  p.color.a = 0.5;
  p.lifetime = ros::Duration(5.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

double nbvInspection::RrtTree::gainCuboid(StateVec state, double distance, double gain_range)
{
  // Computes the number of unknown cells inside the cuboid with the origin in the state
  const double resolution = manager_->getResolution();
  int unknownNum = 0;

  static double multipliers[4][8] = {
    {1, 0, 0, -1, -1, 0, 0, 1},
    {0, 1, -1, 0, 0, -1, 1, 0},
    {0, 1, 1, 0, 0, -1, -1, 0},
    {1, 0, 0, 1, -1, 0, 0, -1}
  };

  // Go through all octants; do shadowcasting; find the number of unknown cells 
  for (uint i = 0; i < 8; i++)
  {
    unknownNum += castUnknown(state, gain_range, distance, resolution, 1.0, 0.0, multipliers[0][i],
                multipliers[1][i], multipliers[2][i], multipliers[3][i]);
  }

  // return unknown volume
  return unknownNum * pow(resolution, 3.0); 
}

int nbvInspection::RrtTree::castUnknown(StateVec state, double gain_range, double distance, double row, 
  double start_slope, double end_slope, double xx, double xy, double yx, double yy)
{
  // Returns number of unknown in one octant
  int unknownNum = 0;
  const double resolution = manager_->getResolution();
  if (start_slope < end_slope) {
    return 0;
  }
  double next_start_slope = start_slope;
 
  for (double z = - gain_range / 2; z <= gain_range / 2; z += resolution){
    for (double j = row; j <= gain_range; j += resolution) {
      bool blocked = false;
      for (double dx = -j - resolution, dy = -j; dx <= 0; dx += resolution) {
        // Translate dx , dy coordinates into map coordinates
        double X = state[0] + dx * xx + dy * xy;
        double Y = state[1] + dx * yx + dy * yy;
        // Slide l_slope and r_slope in one row, left goes through the first point
        // the right one 45 degrees down
        // l_slope and r_slope store the slopes of the left and right
        // extremities of the square we're considering:
        double l_slope = (dx - 0.5) / (dy + 0.5);
        double r_slope = (dx + 0.5) / (dy - 0.5);
        if (start_slope < r_slope) {
          continue;
        } else if (end_slope > l_slope) {
            break;
        }
        else {
          // Else our beam is touching this square, consider it
          // Check if the calculated point is out of the map bounding box given in yaml
          if (X >= params_.maxX_ || X < params_.minX_ ||
            Y >= params_.maxY_ || Y < params_.minY_ ||
            z >= params_.maxZ_ || z < params_.minZ_) {
            continue;
          }
          
          // Get cell probability (status)
          Eigen::Vector3d vec(X, Y, z);
          double probability;
          volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
            vec, &probability);
          // Check if the point is inside the circle with radius 

          // Check if the point is inside the square
          if (std::abs(dx) < gain_range / 2  && std::abs(dy) < gain_range / 2) {
            // Consider it, but check if it is unknown
            // If the point is inside the cuboid
            // Mark the unknown cell
            if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
              unknownNum++;
              // Gain visualization
              if(params_.gainVisualization_) visualizeGainRed(vec);
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
          } else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied && j < gain_range) {
              // This is a blocking square, start a child scan
              blocked = true;
              castUnknown(state, gain_range, distance, j + 1, start_slope, l_slope, xx, xy, yx, yy);
              next_start_slope = r_slope;
            }
        }
      }
      // Row is scanned; do next row unless last square was blocked
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
  // p.scale.x = std::max(node->gain_ / 20.0, 0.05);
  p.scale.x = 0.1;
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
  // p.scale.x = std::max(bestNode_->gain_ / 10.0, 0.5);
   p.scale.x = 0.5;
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
  // p.scale.x = std::max(node->gain_ / 10.0, 0.5);
  p.scale.x = 0.5;
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
  p.lifetime = ros::Duration(100.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::publishReturnNodePom(StateVec node)
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
  p.color.g = 0.5;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(100.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
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
  double delta_x = end[0] - start[0];
  double delta_y = end[1] - start[1];
  double alpha = atan2(delta_y, delta_x);
  
  // Origin is the center between start and end point
  tf::Vector3 origin(start[0] + distance.norm() / 2 * cos(alpha), 
                    start[1] + distance.norm() / 2 * sin(alpha),
                    start[2]);

  Eigen::Vector3d center(origin[0], origin[1], origin[2]);
  // visualizeCenter(center);
  origin = transform * origin;
  StateVec state;
  state[0] = origin[0];
  state[1] = origin[1];
  state[2] = origin[2];
  gain += gainCuboid(state, distance.norm(), params_.gainRange_);
  return gain;
}

#endif
