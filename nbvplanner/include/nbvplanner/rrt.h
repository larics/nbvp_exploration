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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/tree.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;
  
  RrtTree();
  RrtTree(volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer);
  virtual void initialize();
  virtual void iterate(int iterations);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
  virtual std::vector<geometry_msgs::Pose> getBestPathNodes(std::string targetFrame);
  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  int castUnknown(StateVec state, double a, double distance, double row,
    double start_slope, double end_slope, double xx, double xy, double yx, double yy);
  void publishNode(Node<StateVec> * node);
  void publishBestNode();
  void publishCurrentNode(Node<StateVec> * node);
  void publishReturnNode(StateVec node);
  bool checkIfVisited(StateVec state);
  void visualizeGain(Eigen::Vector3d vec);
  void visualizeGainRed(Eigen::Vector3d vec);
  void visualizeCenter(Eigen::Vector3d vec);
  void visualizeCuboid(StateVec start, StateVec end);
  virtual std::vector<geometry_msgs::Pose> getReturnEdge(std::string targetFrame);
  virtual void updateDegressiveCoeff();
  bool findShortestPath(StateVec goal);
  bool setGoal();
  virtual int getHistorySize();
  double gain(StateVec state);
  double gainCube(StateVec start,  double distance, double a);
  double gainCuboid(StateVec start,  double distance, double a);
  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end,
                                              std::string targetFrame);
  double samplePathWithCubes(StateVec start, StateVec end,
                                              std::string targetFrame);
 protected:
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::list<StateVec> branchHistory_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int v_ID_= 0;
  int vr_ID_= 0;
  int ret_ID_ = 0;
  StateVec shortest_;
  StateVec goal_;
  StateVec firstSeen_;
  bool boolFirstSeen_;
  bool callOnce = true;
  double degressiveCoeff_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;
};
}

#endif
