#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.detect_break_beam_state import DetectBreakBeamState
from ariac_flexbe_states.end_assignment_state import EndAssignment
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.set_conveyorbelt_power_state import SetConveyorbeltPowerState
from ariac_flexbe_states.start_assignment_state import StartAssignment
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 26 2021
@author: Rick Verschuuren
'''
class transport_conveyor_to_pick_locationSM(Behavior):
	'''
	Flexbe behavior for switching on the conveyor and stop it when the breakbeam has detected a part.
	'''


	def __init__(self):
		super(transport_conveyor_to_pick_locationSM, self).__init__()
		self.name = 'transport_conveyor_to_pick_location'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		speed_conveyor = 100
		speed_conveyor_off = 0
		# x:1262 y:56, x:745 y:264
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed_conveyor = speed_conveyor
		_state_machine.userdata.object_detected = ''
		_state_machine.userdata.beam_topic = ''
		_state_machine.userdata.index_value = 'break_beam'
		_state_machine.userdata.speed_conveyor_off = speed_conveyor_off
		_state_machine.userdata.index_beam = 'break_beam_1'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:50
			OperatableStateMachine.add('start',
										StartAssignment(),
										transitions={'continue': 'Lookup break_beam_1_change topic'},
										autonomy={'continue': Autonomy.Off})

			# x:474 y:56
			OperatableStateMachine.add('Conveyor on',
										SetConveyorbeltPowerState(),
										transitions={'continue': 'Part detected ', 'fail': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'power': 'speed_conveyor'})

			# x:212 y:54
			OperatableStateMachine.add('Lookup break_beam_1_change topic',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title='break_beam', column_title='beam_topic'),
										transitions={'found': 'Conveyor on', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_beam', 'column_value': 'beam_topic'})

			# x:684 y:57
			OperatableStateMachine.add('Part detected ',
										DetectBreakBeamState(),
										transitions={'continue': 'Conveyor off', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'beam_topic', 'object_detected': 'object_detected'})

			# x:1096 y:54
			OperatableStateMachine.add('end',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:903 y:54
			OperatableStateMachine.add('Conveyor off',
										SetConveyorbeltPowerState(),
										transitions={'continue': 'end', 'fail': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'power': 'speed_conveyor_off'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
