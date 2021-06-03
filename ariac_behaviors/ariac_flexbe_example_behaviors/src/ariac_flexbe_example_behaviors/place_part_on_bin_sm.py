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
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_support_flexbe_states.create_pose import CreatePoseState
from ariac_support_flexbe_states.equal_state import EqualState
from unit_2_flexbe_behaviors.iteration_position_places_sm import iteration_position_placesSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 31 2021
@author: Rick Verschuuren
'''
class place_part_on_binSM(Behavior):
	'''
	places the part it picked up from the conveyor in an empty bin or in a bin with the same type of products.
	'''


	def __init__(self):
		super(place_part_on_binSM, self).__init__()
		self.name = 'place_part_on_bin'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(iteration_position_placesSM, 'iteration_position_places')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:877 y:519, x:578 y:161
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['index_bin'])
		_state_machine.userdata.index_bin = ''
		_state_machine.userdata.comparison = []
		_state_machine.userdata.positie_xyz = []
		_state_machine.userdata.place_pose = []
		_state_machine.userdata.bin = ''
		_state_machine.userdata.move_group = 'manipulator'
		_state_machine.userdata.offset = 0
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.action_topic_namespace = ''
		_state_machine.userdata.tool_link = 'ee_link'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:481 y:55
			OperatableStateMachine.add('Check of de beginpositie al opgevraagt is',
										EqualState(),
										transitions={'true': 'Lookup eerste drop positie', 'false': 'iteration_position_places'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'positie_xyz'})

			# x:740 y:270
			OperatableStateMachine.add('Compute place bin',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'tool_link': 'tool_link', 'pose': 'place_pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:511 y:255
			OperatableStateMachine.add('Generate pose',
										CreatePoseState(xyz=[0,0,0], rpy=[0,0,0]),
										transitions={'continue': 'Compute place bin', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'place_pose'})

			# x:742 y:130
			OperatableStateMachine.add('Lookup eerste drop positie',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='bin_configuration', index_title='bin', column_title='start_positie_bin'),
										transitions={'found': 'Generate pose', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'positie_xyz'})

			# x:254 y:128
			OperatableStateMachine.add('iteration_position_places',
										self.use_behavior(iteration_position_placesSM, 'iteration_position_places'),
										transitions={'finished': 'Generate pose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'positie_xyz': 'positie_xyz'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
