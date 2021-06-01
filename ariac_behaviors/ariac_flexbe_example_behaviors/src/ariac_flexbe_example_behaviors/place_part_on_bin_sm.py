#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
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

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1068 y:230, x:344 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['index_bin'])
		_state_machine.userdata.index_bin = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:131 y:29
			OperatableStateMachine.add('Lookup PreDropForBin',
										LookupFromTableState(parameter_name='/ariac_tables_unit1', table_name='bin_configuration', index_title='bin', column_title='robot_config'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin', 'column_value': 'column_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
