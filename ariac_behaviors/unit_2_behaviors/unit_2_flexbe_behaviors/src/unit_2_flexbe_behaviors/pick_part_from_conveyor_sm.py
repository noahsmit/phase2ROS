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
		# x:749 y:343, x:390 y:299
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot_namespace'], output_keys=['part'])
		_state_machine.userdata.part = ''
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
		_state_machine.userdata.pose = ''
		_state_machine.userdata.ref_frame = 'world'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 topic',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title=logical_camera_4, column_title=camera_topic),
										transitions={'found': 'Lookup break_beam_1 topic', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'camera_topic'})

			# x:488 y:49
			OperatableStateMachine.add('Lookup logical_camera_4 frame',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title=logical_camera_4, column_title=camera_frame),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'camera_frame'})

			# x:274 y:48
			OperatableStateMachine.add('Lookup break_beam_1 topic',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='conveyor_configuration', index_title=logical_camera_4, column_title=camera_topic),
										transitions={'found': 'Lookup logical_camera_4 frame', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_value', 'column_value': 'beam_topic'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
