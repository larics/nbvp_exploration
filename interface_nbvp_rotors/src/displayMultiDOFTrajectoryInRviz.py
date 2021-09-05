#!/usr/bin/env python
import sys, os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty

class displayMultiDOFTrajectoryInRviz():

	def __init__(self):
		# Subscribes to trajectory and stores it in message below
		self.trajectorySub = rospy.Subscriber(
			"/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, 
			self.trajectoryCallback)
		self.trajectory = MultiDOFJointTrajectory()

		# Clear path
		rospy.Subscriber("clearVisualizationPath", Empty, self.clearPathCallback)

		# Loop rate determines display rate of path in rviz
		self.loopRate = rospy.get_param("loop_rate", 30)
		self.appendFlag = rospy.get_param("append", False)
		

		# Path is sent to Rviz for display
		self.pathPub = rospy.Publisher("path", Path, queue_size=1)
		self.path = Path()
		for i in range(2):
			tempPoseStamped = PoseStamped()
			self.path.poses.append(tempPoseStamped)
		# In addition, current position of robot can be displayed, in that case
		# subscriber to position feedback will be needed
		#self.pointPub = rospy.Publisher("position", PointStamped, queue_size=1)


	def run(self):
		r = rospy.Rate(self.loopRate)
		while not rospy.is_shutdown():
			#pt = PointStamped()
			#pt.header.stamp = rospy.Time.now()
			#pt.header.frame_id = "world"
			#self.pointPub.publish(pt)

			self.path.header.stamp = rospy.Time.now()
			self.path.header.frame_id = "mavros/world"
			self.pathPub.publish(self.path)

			r.sleep()

	def trajectoryCallback(self, data):
		self.trajectory.points.append(data)
		# print len(self.trajectory.points)

		if self.appendFlag == False:
			# print "Append Flag False"
			self.path.poses = []

		for i in range(len(self.trajectory.points)):
			tempPoseStamped = PoseStamped()
			tempPoseStamped.pose.position.x = self.trajectory.points[i].transforms[0].translation.x
			tempPoseStamped.pose.position.y = self.trajectory.points[i].transforms[0].translation.y
			tempPoseStamped.pose.position.z = self.trajectory.points[i].transforms[0].translation.z
			tempPoseStamped.pose.orientation.w = 1.0
			tempPoseStamped.header.stamp = rospy.Time.now()
			tempPoseStamped.header.frame_id = "mavros/world"
			self.path.poses.append(tempPoseStamped)

	def clearPathCallback(self, msg):
		self.path.poses = []

if __name__ == "__main__":
	rospy.init_node('displayMultiDofTrajectoryInRviz')
	a = displayMultiDOFTrajectoryInRviz()
	a.run()
