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
from ariac_flexbe_states.end_assignment_state import EndAssignment
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
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
		table = '/ariac_tables_unit1'
		# x:30 y:365, x:437 y:159
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'robot_namespace'])
		_state_machine.userdata.part = 'gasket_part'
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
		_state_machine.userdata.part_height = 0.035
		_state_machine.userdata.part_rotation = 0
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.gripper_service = '/ariac/arm1/gripper/control'
		_state_machine.userdata.home = 'homeR1'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:9 y:55
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'Material Location'},
										autonomy={'continue': Autonomy.Off})

			# x:691 y:531
			OperatableStateMachine.add('Compute P',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveToPick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'part_height', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:750 y:399
			OperatableStateMachine.add('DetectPart(NL)',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'Compute P', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'camera_ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:30 y:117
			OperatableStateMachine.add('End',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:427 y:22
			OperatableStateMachine.add('FindBinFromList',
										GetItemFromListState(),
										transitions={'done': 'Message Bin', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'material_locations', 'index': 'index', 'item': 'bin'})

			# x:785 y:24
			OperatableStateMachine.add('FindBinPosition',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='robot1_config'),
										transitions={'found': 'Move', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'column_value'})

			# x:30 y:426
			OperatableStateMachine.add('Found!',
										MessageState(),
										transitions={'continue': 'End'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'material_locations'})

			# x:785 y:279
			OperatableStateMachine.add('LookUpCameraTopic',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='camera_topic'),
										transitions={'found': 'DetectPart(NL)', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_topic'})

			# x:791 y:180
			OperatableStateMachine.add('LookupCameraFrame',
										LookupFromTableState(parameter_name=table, table_name='bin_configuration', index_title='bin', column_title='camera_frame'),
										transitions={'found': 'LookUpCameraTopic', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_frame'})

			# x:237 y:54
			OperatableStateMachine.add('Material Location',
										GetMaterialLocationsState(),
										transitions={'continue': 'FindBinFromList'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'material_locations': 'material_locations'})

			# x:593 y:24
			OperatableStateMachine.add('Message Bin',
										MessageState(),
										transitions={'continue': 'FindBinPosition'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'bin'})

			# x:786 y:110
			OperatableStateMachine.add('Move',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'LookupCameraFrame', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'column_value', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_namespace', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:108 y:500
			OperatableStateMachine.add('Move Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Found!', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'column_value', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_namespace', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:491 y:531
			OperatableStateMachine.add('MoveToPick',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'ActivateGripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:265 y:518
			OperatableStateMachine.add('ActivateGripper',
										VacuumGripperControlState(enable=True),
										transitions={'continue': 'Move Home', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'service_name': 'gripper_service'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
