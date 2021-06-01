#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.setBinPartType import setBinPartType
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 01 2021
@author: Rick Verschuuren
'''
class lege_bin_zoeken_V2SM(Behavior):
	'''
	zoekt naar de volle bins om vandaaruit de lege bins te vinden.
	'''


	def __init__(self):
		super(lege_bin_zoeken_V2SM, self).__init__()
		self.name = 'lege_bin_zoeken_V2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:67 y:438, x:514 y:277
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.gasket = 'gasket_part'
		_state_machine.userdata.gear = 'gear_part'
		_state_machine.userdata.piston_rod = 'piston_rod_part'
		_state_machine.userdata.bin = ''
		_state_machine.userdata.binPartType = ['empty', 'empty', 'empty', 'empty', 'empty', 'empty']
		_state_machine.userdata.zero = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:153 y:54
			OperatableStateMachine.add('Zoek naar volle bin voor gasket_part',
										GetMaterialLocationsState(),
										transitions={'continue': 'Zoek uit de list de bijbehorende bin'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'gasket', 'material_locations': 'material_locations'})

			# x:946 y:377
			OperatableStateMachine.add('Bin nr toevoegen aan list met volle bins_2',
										setBinPartType(),
										transitions={'continue': 'Zoek naar volle bin voor piston_rod_part', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'bin': 'bin', 'part': 'gear', 'binPartType': 'binPartType'})

			# x:89 y:588
			OperatableStateMachine.add('Bin nr toevoegen aan list met volle bins_3',
										setBinPartType(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'bin': 'bin', 'part': 'piston_rod', 'binPartType': 'binPartType'})

			# x:979 y:236
			OperatableStateMachine.add('Zoek naar volle bin voor gear_part',
										GetMaterialLocationsState(),
										transitions={'continue': 'Zoek uit de list de bijbehorende bin2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'gear', 'material_locations': 'material_locations'})

			# x:616 y:587
			OperatableStateMachine.add('Zoek naar volle bin voor piston_rod_part',
										GetMaterialLocationsState(),
										transitions={'continue': 'Zoek uit de list de bijbehorende bin_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'piston_rod', 'material_locations': 'material_locations'})

			# x:393 y:55
			OperatableStateMachine.add('Zoek uit de list de bijbehorende bin',
										GetItemFromListState(),
										transitions={'done': 'Bin nr toevoegen aan list met volle bins', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'binPartType', 'index': 'zero', 'item': 'bin'})

			# x:971 y:310
			OperatableStateMachine.add('Zoek uit de list de bijbehorende bin2',
										GetItemFromListState(),
										transitions={'done': 'Bin nr toevoegen aan list met volle bins_2', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'binPartType', 'index': 'zero', 'item': 'bin'})

			# x:367 y:588
			OperatableStateMachine.add('Zoek uit de list de bijbehorende bin_2',
										GetItemFromListState(),
										transitions={'done': 'Bin nr toevoegen aan list met volle bins_3', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'binPartType', 'index': 'zero', 'item': 'bin'})

			# x:614 y:55
			OperatableStateMachine.add('Bin nr toevoegen aan list met volle bins',
										setBinPartType(),
										transitions={'continue': 'Zoek naar volle bin voor gear_part', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'bin': 'bin', 'part': 'gasket', 'binPartType': 'binPartType'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
