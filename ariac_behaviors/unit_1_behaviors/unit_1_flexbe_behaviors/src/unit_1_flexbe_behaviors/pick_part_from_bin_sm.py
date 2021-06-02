#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.detect_part_camera_ariac_state import DetectPartCameraAriacState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.equal_state import EqualState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
from ariac_support_flexbe_states.text_to_float_state import TextToFloatState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 25 2021
@author: docent
'''
class pick_part_from_binSM(Behavior):
	'''
	pick's a specific part form a it's bin
	'''


	def __init__(self):
		super(pick_part_from_binSM, self).__init__()
		self.name = 'pick_part_from_bin'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = '/ariac_unit1_tables'
		# x:30 y:365, x:437 y:159
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'robot_namespace', 'agv', 'agv2'])
		_state_machine.userdata.part = ''
		_state_machine.userdata.robot_namespace = ''
		_state_machine.userdata.move_group = 'manipulator'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.joint_values = []
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.config_name_home = 'bin3PreDropPickR1'
		_state_machine.userdata.action_topic_namespace = '/ariac/arm1'
		_state_machine.userdata.index = 0
		_state_machine.userdata.index_value = 'bin4'
		_state_machine.userdata.camera_frame = ''
		_state_machine.userdata.camera_topic = ''
		_state_machine.userdata.camera_ref_frame = 'arm1_linear_arm_actuator'
		_state_machine.userdata.part_rotation = 0
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.gripper_service = ''
		_state_machine.userdata.home = 'homeR1'
		_state_machine.userdata.agv = ''
		_state_machine.userdata.agv2 = 2
		_state_machine.userdata.check = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:237 y:54
			OperatableStateMachine.add('Material Location',
										GetMaterialLocationsState(),
										transitions={'continue': 'FindBinFromList'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'material_locations': 'material_locations'})

			# x:763 y:9
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'FindBinPosition', 'false': 'FindBin2'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'check', 'value_b': 'agv2'})

			# x:691 y:531
			OperatableStateMachine.add('Compute P',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveToPick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:865 y:530
			OperatableStateMachine.add('DetectPart(NL)',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'Compute P', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'camera_ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:979 y:102
			OperatableStateMachine.add('FindBin2',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='robot2_config'),
										transitions={'found': 'LookUpNameSpace', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'position'})

			# x:427 y:22
			OperatableStateMachine.add('FindBinFromList',
										GetItemFromListState(),
										transitions={'done': 'Message Bin', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'material_locations', 'index': 'index', 'item': 'bin'})

			# x:994 y:19
			OperatableStateMachine.add('FindBinPosition',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='robot1_config'),
										transitions={'found': 'LookUpNameSpace', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'position'})

			# x:30 y:426
			OperatableStateMachine.add('Found!',
										MessageState(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'material_locations'})

			# x:1088 y:415
			OperatableStateMachine.add('LookUpCameraTopic',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='camera_topic'),
										transitions={'found': 'LookUpOffset', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_topic'})

			# x:377 y:606
			OperatableStateMachine.add('LookUpGripperService',
										LookupFromTableState(parameter_name=table, table_name='robots', index_title='robot', column_title='service'),
										transitions={'found': 'ActivateGripper', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'agv', 'column_value': 'gripper_service'})

			# x:1109 y:204
			OperatableStateMachine.add('LookUpNameSpace',
										LookupFromTableState(parameter_name=table, table_name='robots', index_title='robot', column_title='namespace'),
										transitions={'found': 'Move', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'agv', 'column_value': 'action_topic_namespace'})

			# x:1080 y:476
			OperatableStateMachine.add('LookUpOffset',
										LookupFromTableState(parameter_name=table, table_name='parts_bin', index_title='part', column_title='height'),
										transitions={'found': 'texttofloat', 'not_found': 'LookUpOffset'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'part', 'column_value': 'offset_text'})

			# x:1093 y:348
			OperatableStateMachine.add('LookupCameraFrame',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='camera_frame'),
										transitions={'found': 'LookUpCameraTopic', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_frame'})

			# x:593 y:24
			OperatableStateMachine.add('Message Bin',
										MessageState(),
										transitions={'continue': 'CheckEqual'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'bin'})

			# x:1096 y:286
			OperatableStateMachine.add('Move',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'LookupCameraFrame', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'position', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_namespace', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:108 y:500
			OperatableStateMachine.add('Move Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Found!', 'planning_failed': 'WaitRetry3', 'control_failed': 'WaitRetry3', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'position', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_namespace', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:491 y:531
			OperatableStateMachine.add('MoveToPick',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'LookUpGripperService', 'planning_failed': 'WaitRetry2', 'control_failed': 'WaitRetry2'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:260 y:630
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Move Home'},
										autonomy={'done': Autonomy.Off})

			# x:1234 y:291
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=1),
										transitions={'done': 'Move'},
										autonomy={'done': Autonomy.Off})

			# x:541 y:626
			OperatableStateMachine.add('WaitRetry2',
										WaitState(wait_time=1),
										transitions={'done': 'MoveToPick'},
										autonomy={'done': Autonomy.Off})

			# x:106 y:579
			OperatableStateMachine.add('WaitRetry3',
										WaitState(wait_time=1),
										transitions={'done': 'Move Home'},
										autonomy={'done': Autonomy.Off})

			# x:1056 y:535
			OperatableStateMachine.add('texttofloat',
										TextToFloatState(),
										transitions={'done': 'DetectPart(NL)'},
										autonomy={'done': Autonomy.Off},
										remapping={'text_value': 'offset_text', 'float_value': 'offset'})

			# x:265 y:518
			OperatableStateMachine.add('ActivateGripper',
										VacuumGripperControlState(enable=True),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'service_name': 'gripper_service'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
