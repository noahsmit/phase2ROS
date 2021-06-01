#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 01 2021
@author: Rick Verschuuren
'''
class check_for_already_used_drop_binSM(Behavior):
	'''
	checks if there are bins witch are not empty 
	'''


	def __init__(self):
		super(check_for_already_used_drop_binSM, self).__init__()
		self.name = 'check_for_already_used_drop_bin'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:411 y:637, x:1207 y:97
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['gasket_part', 'gear_part', 'piston_rod_part'])
		_state_machine.userdata.comparison = 1
		_state_machine.userdata.gasket_part = ''
		_state_machine.userdata.gear_part = ''
		_state_machine.userdata.piston_rod_part = ''
		_state_machine.userdata.gasket_p1 = ''
		_state_machine.userdata.gasket_p2 = ''
		_state_machine.userdata.gasket_p3 = ''
		_state_machine.userdata.gasket_p4 = ''
		_state_machine.userdata.gasket_p5 = ''
		_state_machine.userdata.gasket_p6 = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:174 y:59
			OperatableStateMachine.add('If gasket_part',
										EqualState(),
										transitions={'true': 'If gasket_p1', 'false': 'If gear_part'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_part'})

			# x:25 y:242
			OperatableStateMachine.add('If gasket_p2',
										EqualState(),
										transitions={'true': 'If gasket_p3', 'false': 'maak p2 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p2'})

			# x:26 y:315
			OperatableStateMachine.add('If gasket_p3',
										EqualState(),
										transitions={'true': 'If gasket_p4', 'false': 'maak p3 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p3'})

			# x:27 y:386
			OperatableStateMachine.add('If gasket_p4',
										EqualState(),
										transitions={'true': 'If gasket_p5', 'false': 'maak p4 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p4'})

			# x:27 y:455
			OperatableStateMachine.add('If gasket_p5',
										EqualState(),
										transitions={'true': 'If gasket_p6', 'false': 'maak p5 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p5'})

			# x:27 y:528
			OperatableStateMachine.add('If gasket_p6',
										EqualState(),
										transitions={'true': 'finished', 'false': 'maak p6 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p6'})

			# x:574 y:59
			OperatableStateMachine.add('If gear_part',
										EqualState(),
										transitions={'true': 'If gear_part', 'false': 'If piston_rod_part'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gear_part'})

			# x:974 y:59
			OperatableStateMachine.add('If piston_rod_part',
										EqualState(),
										transitions={'true': 'If piston_rod_part', 'false': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'piston_rod_part'})

			# x:238 y:165
			OperatableStateMachine.add('maak p1 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p1', 'result': 'gasket_p1'})

			# x:237 y:243
			OperatableStateMachine.add('maak p2 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p2', 'result': 'gasket_p2'})

			# x:235 y:315
			OperatableStateMachine.add('maak p3 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p3', 'result': 'gasket_p3'})

			# x:233 y:388
			OperatableStateMachine.add('maak p4 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p4', 'result': 'gasket_p4'})

			# x:237 y:458
			OperatableStateMachine.add('maak p5 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p5', 'result': 'gasket_p5'})

			# x:237 y:529
			OperatableStateMachine.add('maak p6 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p6', 'result': 'gasket_p6'})

			# x:24 y:167
			OperatableStateMachine.add('If gasket_p1',
										EqualState(),
										transitions={'true': 'If gasket_p2', 'false': 'maak p1 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'gasket_p1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
