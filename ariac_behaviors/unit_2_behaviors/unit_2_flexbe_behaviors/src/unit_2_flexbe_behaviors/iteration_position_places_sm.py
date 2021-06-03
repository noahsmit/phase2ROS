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
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

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
		# x:335 y:625, x:246 y:624
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['positie_xyz'], output_keys=['positie_xyz'])
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

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:110 y:48
			OperatableStateMachine.add('If p1 is available ',
										EqualState(),
										transitions={'true': 'Update iterator p1', 'false': 'If p2 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P1', 'value_b': 'iterator'})

			# x:527 y:131
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

			# x:1250 y:95
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

			# x:108 y:121
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

			# x:329 y:130
			OperatableStateMachine.add('Add_offset_p2',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p2', 'output_pose': 'positie_xyz'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
