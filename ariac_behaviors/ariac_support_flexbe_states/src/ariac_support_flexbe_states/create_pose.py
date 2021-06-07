#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Delft University of Technology nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Gerard Harkema

import rospy
import rostopic
import inspect

import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from tf.transformations import *


from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped
from flexbe_core.proxy import ProxySubscriberCached
from osrf_gear.msg import LogicalCameraImage, Model


'''

Created on March 21, 2021

@author: Gerard Harkema
@author-updated: Noah Smit

'''

class CreatePoseState(EventState):
	'''
	Creats a pose
	-- xyz			Point		Position
	-- rpy			Vector3		Twist
	#> pose			Pose		Output of the pose

	<= continue 				if the pose of the part has been succesfully obtained
	<= failed 				otherwise

	'''



	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CreatePoseState, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['pose'], input_keys = ['pose' , 'rpy'])


		#rospy.logerr(self._pose)
Point(13)

from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped
		def posetr(self, point):
			p = PoseStamped()
    		p.pose.position.x = self.point.x
    		p.pose.position.y = self.point.y
			p.pose.position.z = self.point.z
			q = quaternion_from_euler(0,0,0)
			p.pose.orientation = Quaternion(*q)
    		return p


import rospy
import rostopic
import inspect

import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from tf.transformations import *


from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped
from flexbe_core.proxy import ProxySubscriberCached
from osrf_gear.msg import LogicalCameraImage, Model
from std_msgs import Header
	PoseStamped(Header(0,rospy.Time.now(),'world'),Pose(Point(0,0,0), Quaternion(0,0,0,0)))



	def execute(self, userdata):
		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = rospy.Time.now()
		pose_stamped.header.frame_id = 'world'

		pose2 = Pose()
		pose2.position.x


		pose_stamped.pose.position.x = userdata.x
		pose_stamped.pose.position.y = userdata.y
		pose_stamped.pose.position.z = userdata.z
		q = quaternion_from_euler(userdata.rpy[0], userdata.rpy[1], userdata.rpy[2])
		defined_pose.orientation = Quaternion(*q)

		pose_stamped.pose = defined_pose

		userdata.pose = pose_stamped
		return 'continue'

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		pass # Nothing to do



	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		self._start_time = rospy.Time.now()

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do
