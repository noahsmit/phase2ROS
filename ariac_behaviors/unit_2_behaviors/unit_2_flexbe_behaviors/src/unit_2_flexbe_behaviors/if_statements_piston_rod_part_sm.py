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
class If_statements_piston_rod_partSM(Behavior):
	'''
	looks for empty spots on the bin
	'''


	def __init__(self):
		super(If_statements_piston_rod_partSM, self).__init__()
		self.name = 'If_statements_piston_rod_part'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.comparison = 1
		_state_machine.userdata.piston_p1 = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:121 y:59
			OperatableStateMachine.add('If piston_p1',
										EqualState(),
										transitions={'true': 'finished', 'false': 'Maak P1 bezet'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'piston_p1'})

			# x:325 y:59
			OperatableStateMachine.add('Maak P1 bezet',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'comparison', 'value_b': 'piston_p1', 'result': 'piston_p1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
