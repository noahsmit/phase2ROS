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
from ariac_flexbe_states.detect_first_part_camera_ariac_state import DetectFirstPartCameraAriacState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
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
		# x:46 y:601, x:554 y:290
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
		_state_machine.userdata.part_list = ['gasket_part', 'piston_rod_part', 'gear_part']
		_state_machine.userdata.pose_detected_part = ''
		_state_machine.userdata.ref_frame = 'world'
		_state_machine.userdata.config_name_homeR1 = 'homeR1'
		_state_machine.userdata.config_name_pregrasp_conveyor = ''
		_state_machine.userdata.move_group_R1 = 'manipulator'
		_state_machine.userdata.action_topic_namespace_R1 = '/ariac/arm1'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name_R1 = ''
		_state_machine.userdata.part = ''
		_state_machine.userdata.config_name_homeR2 = 'homeR2'
		_state_machine.userdata.robot_name_R2 = ''
		_state_machine.userdata.move_group_R2 = 'manipulator'
		_state_machine.userdata.action_topic_namespace_R2 = '/ariac/arm2'
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.joint_values_pick_part = []
		_state_machine.userdata.height_detected_part = ''
		_state_machine.userdata.joint_names1 = []
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.gripper_service = ''
		_state_machine.userdata.index_camera_4 = 'logical_camera_4'
		_state_machine.userdata.index_beam = 'break_beam_1'
		_state_machine.userdata.index_detected_part = 'gasket_part'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 topic',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='conveyor_configuration', index_title='logical_camera', column_title='camera_topic'),
										transitions={'found': 'Lookup break_beam_1 topic', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_camera_4', 'column_value': 'camera_topic'})

			# x:1156 y:582
			OperatableStateMachine.add('ComputeGraspConveyor',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'Activate gripper', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'pose_detected_part', 'offset': 'height_detected_part', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names1'})

			# x:1088 y:78
			OperatableStateMachine.add('Detect part',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'piston_rod_part', 'gear_part'], time_out=0.5),
										transitions={'continue': 'Lookup robot move_group', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'detected_part', 'pose': 'pose_detected_part'})

			# x:274 y:48
			OperatableStateMachine.add('Lookup break_beam_1 topic',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='conveyor_configuration', index_title='break_beam', column_title='beam_topic'),
										transitions={'found': 'Lookup logical_camera_4 frame', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_beam', 'column_value': 'beam_topic'})

			# x:1150 y:442
			OperatableStateMachine.add('Lookup gripper service',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='part_robot_keuze', index_title='detected_part', column_title='gripper_service'),
										transitions={'found': 'MoveR1PreGraspConveyor', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_detected_part', 'column_value': 'gripper_service'})

			# x:488 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 frame',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='conveyor_configuration', index_title='logical_camera', column_title='camera_frame'),
										transitions={'found': 'MoveR1Home', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_camera_4', 'column_value': 'camera_frame'})

			# x:1150 y:380
			OperatableStateMachine.add('Lookup part_height',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='part_robot_keuze', index_title='detected_part', column_title='part_height'),
										transitions={'found': 'Lookup gripper service', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_detected_part', 'column_value': 'height_detected_part'})

			# x:1105 y:315
			OperatableStateMachine.add('Lookup robot PreGraspPositie',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='part_robot_keuze', index_title='detected_part', column_title='pregrasp_conveyor'),
										transitions={'found': 'Lookup part_height', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_detected_part', 'column_value': 'config_name_pregrasp_conveyor'})

			# x:1101 y:243
			OperatableStateMachine.add('Lookup robot action topic ns',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='part_robot_keuze', index_title='detected_part', column_title='action_topic_namespace'),
										transitions={'found': 'Lookup robot PreGraspPositie', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_detected_part', 'column_value': 'action_topic_namespace'})

			# x:1091 y:168
			OperatableStateMachine.add('Lookup robot move_group',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='part_robot_keuze', index_title='detected_part', column_title='move_group'),
										transitions={'found': 'Lookup robot action topic ns', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_detected_part', 'column_value': 'move_group'})

			# x:736 y:49
			OperatableStateMachine.add('MoveR1Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'MoveR2Home', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_homeR1', 'move_group': 'move_group_R1', 'action_topic_namespace': 'action_topic_namespace_R1', 'action_topic': 'action_topic', 'robot_name': 'robot_name_R1', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:572 y:601
			OperatableStateMachine.add('MoveR1Home_2',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_homeR1', 'move_group': 'move_group_R1', 'action_topic_namespace': 'action_topic_namespace_R1', 'action_topic': 'action_topic', 'robot_name': 'robot_name_R1', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1145 y:509
			OperatableStateMachine.add('MoveR1PreGraspConveyor',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'ComputeGraspConveyor', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_pregrasp_conveyor', 'move_group': 'move_group_R1', 'action_topic_namespace': 'action_topic_namespace_R1', 'action_topic': 'action_topic', 'robot_name': 'robot_name_R1', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:910 y:52
			OperatableStateMachine.add('MoveR2Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Detect part', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_homeR2', 'move_group': 'move_group_R2', 'action_topic_namespace': 'action_topic_namespace_R2', 'action_topic': 'action_topic', 'robot_name': 'robot_name_R2', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:839 y:664
			OperatableStateMachine.add('Pick part from conveyor',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'MoveR1Home_2', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names1'})

			# x:1028 y:646
			OperatableStateMachine.add('Activate gripper',
										VacuumGripperControlState(enable=True),
										transitions={'continue': 'Pick part from conveyor', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'service_name': 'gripper_service'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
