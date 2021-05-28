#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.detect_first_part_camera_ariac_state import DetectFirstPartCameraAriacState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 25 2021
@author: Rick Verschuuren
'''
class pick_part_from_conveyorSM(Behavior):
	'''
	pick's a part form athe conveyor
	'''


	def __init__(self):
		super(pick_part_from_conveyorSM, self).__init__()
		self.name = 'pick_part_from_conveyor'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:749 y:343, x:554 y:290
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot_namespace'], output_keys=['part'])
		_state_machine.userdata.detected_part = ''
		_state_machine.userdata.robot_namespace = ''
		_state_machine.userdata.camera_topic = ''
		_state_machine.userdata.beam_topic = ''
		_state_machine.userdata.laser_topic = ''
		_state_machine.userdata.proximity_topic = ''
		_state_machine.userdata.camera_frame = ''
		_state_machine.userdata.laser_frame = ''
		_state_machine.userdata.proximity_frame = ''
		_state_machine.userdata.index_value = ''
		_state_machine.userdata.part_list = [gasket_part, piston_rod_part, gear_part]
		_state_machine.userdata.gasket_part = gasket_part
		_state_machine.userdata.pose_detected_part = ''
		_state_machine.userdata.ref_frame = 'world'
		_state_machine.userdata.config_name_home = 'home'
		_state_machine.userdata.config_name_pregrasp_conveyor = 'pregraspconveyor'
		_state_machine.userdata.move_group_R1 = 'robot1'
		_state_machine.userdata.action_topic_namespace = '/ariac/arm1'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = 'robot1'
		_state_machine.userdata.part = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 topic',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title='logical_camera_4', column_title='camera_topic'),
										transitions={'found': 'Lookup break_beam_1 topic', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'camera_topic'})

			# x:274 y:48
			OperatableStateMachine.add('Lookup break_beam_1 topic',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title='break_beam_1', column_title='beam_topic'),
										transitions={'found': 'Lookup logical_camera_4 frame', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'beam_topic'})

			# x:488 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 frame',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title='logical_camera_4', column_title='camera_frame'),
										transitions={'found': 'MoveR1Home', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'camera_frame'})

			# x:736 y:49
			OperatableStateMachine.add('MoveR1Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Detect part', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group_R1', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:946 y:53
			OperatableStateMachine.add('Detect part',
										DetectFirstPartCameraAriacState(part_list=part_list, time_out=0.5),
										transitions={'continue': 'finished', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'detected_part', 'pose': 'pose_detected_part'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
