#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.add_offset_to_pose_state import AddOffsetToPoseState
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
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
from osrf_gear.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Wed Jun 02 2021
@author: Rick Verschuuren
'''
class iteration_position_placesSM(Behavior):
	'''
	iteration for changing the drop positions of the bins.
	'''


	def __init__(self):
		super(iteration_position_placesSM, self).__init__()
		self.name = 'iteration_position_places'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		xyz = [0.15, 0.15, 0.5]
		# x:335 y:625, x:246 y:624
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['positie_xyz', 'bin'], output_keys=['positie_xyz'])
		_state_machine.userdata.iterator = 1
		_state_machine.userdata.P1 = 1
		_state_machine.userdata.P2 = 2
		_state_machine.userdata.P3 = 3
		_state_machine.userdata.P4 = 4
		_state_machine.userdata.P5 = 5
		_state_machine.userdata.P6 = 6
		_state_machine.userdata.one_up = 1
		_state_machine.userdata.positie_xyz = []
		_state_machine.userdata.offset_p3 = ['0' '-0.15' '0' '0' '0' '0' '0']
		_state_machine.userdata.offset_p4 = ['-0.3' '0.30' '0' '0' '0' '0' '0']
		_state_machine.userdata.offset_p5 = ['0' '-0.15' '0' '0' '0' '0' '0']
		_state_machine.userdata.offset_p2 = ['0' '-0.15' '0' '0' '0' '0' '0']
		_state_machine.userdata.offset_p6 = ['0' '-0.15' '0' '0' '0' '0' '0']
		_state_machine.userdata.reset_iterator = -5
		_state_machine.userdata.reset_positie = ['0.3' '0.30' '0' '0' '0' '0' '0']
		_state_machine.userdata.bin = ''
		_state_machine.userdata.camera_bin_frame = ''
		_state_machine.userdata.offset_p1 = PoseStamped(Header(0,rospy.Time.now(),'world'),Pose(Point(0.15, 0.15, 0.5), Quaternion(0,0,0,0)))
		_state_machine.userdata.bin_frame = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:14 y:632
			OperatableStateMachine.add('Lookup camera_bin_frame',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='bin_configuration', index_title='bin', column_title='camera_bin_frame'),
										transitions={'found': 'Lookup camera_bin_frame_2', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_bin_frame'})

			# x:329 y:130
			OperatableStateMachine.add('Add_offset_p2',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p2', 'output_pose': 'positie_xyz'})

			# x:524 y:133
			OperatableStateMachine.add('Add_offset_p3',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p3', 'output_pose': 'positie_xyz'})

			# x:724 y:133
			OperatableStateMachine.add('Add_offset_p4',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p4', 'output_pose': 'positie_xyz'})

			# x:905 y:134
			OperatableStateMachine.add('Add_offset_p5',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p5', 'output_pose': 'positie_xyz'})

			# x:1091 y:133
			OperatableStateMachine.add('Add_offset_p6',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p6'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p6', 'output_pose': 'positie_xyz'})

			# x:21 y:436
			OperatableStateMachine.add('Get Bin pose',
										GetObjectPoseState(),
										transitions={'continue': 'If p1 is available ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ref_frame': 'camera_bin_frame', 'frame': 'bin_frame', 'pose': 'positie_xyz'})

			# x:131 y:47
			OperatableStateMachine.add('If p1 is available ',
										EqualState(),
										transitions={'true': 'Add_offset_p1', 'false': 'If p2 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P1', 'value_b': 'iterator'})

			# x:318 y:50
			OperatableStateMachine.add('If p2 is available',
										EqualState(),
										transitions={'true': 'Add_offset_p2', 'false': 'If p3 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P2', 'value_b': 'iterator'})

			# x:517 y:49
			OperatableStateMachine.add('If p3 is available',
										EqualState(),
										transitions={'true': 'Add_offset_p3', 'false': 'If p4 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P3', 'value_b': 'iterator'})

			# x:713 y:48
			OperatableStateMachine.add('If p4 is available',
										EqualState(),
										transitions={'true': 'Add_offset_p4', 'false': 'If p5 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P4', 'value_b': 'iterator'})

			# x:896 y:48
			OperatableStateMachine.add('If p5 is available',
										EqualState(),
										transitions={'true': 'Add_offset_p5', 'false': 'If p6 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P5', 'value_b': 'iterator'})

			# x:1079 y:48
			OperatableStateMachine.add('If p6 is available',
										EqualState(),
										transitions={'true': 'Add_offset_p6', 'false': 'Reset iterator '},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P6', 'value_b': 'iterator'})

			# x:12 y:529
			OperatableStateMachine.add('Lookup camera_bin_frame_2',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='bin_configuration', index_title='bin', column_title='bin_frame'),
										transitions={'found': 'Get Bin pose', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'bin_frame'})

			# x:1296 y:93
			OperatableStateMachine.add('Reset iterator ',
										AddNumericState(),
										transitions={'done': 'Reset positie'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'reset_iterator', 'result': 'iterator'})

			# x:1253 y:296
			OperatableStateMachine.add('Reset positie',
										AddOffsetToPoseState(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'reset_positie', 'output_pose': 'positie_xyz'})

			# x:138 y:217
			OperatableStateMachine.add('Update iterator p1',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:314 y:214
			OperatableStateMachine.add('Update iterator p2',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:511 y:215
			OperatableStateMachine.add('Update iterator p3',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:710 y:230
			OperatableStateMachine.add('Update iterator p4',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:889 y:256
			OperatableStateMachine.add('Update iterator p5',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:1075 y:274
			OperatableStateMachine.add('Update iterator p6',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:141 y:120
			OperatableStateMachine.add('Add_offset_p1',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p1', 'output_pose': 'positie_xyz'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
