#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.detect_first_part_camera_ariac_state import DetectFirstPartCameraAriacState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 31 2021
@author: Rick Verschuuren
'''
class lege_bins_zoekenSM(Behavior):
	'''
	checkt alle bins om te kijken of er producten liggen.
	'''


	def __init__(self):
		super(lege_bins_zoekenSM, self).__init__()
		self.name = 'lege_bins_zoeken'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:665 y:341, x:553 y:344
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['output_bin_nr'])
		_state_machine.userdata.ref_frame = 'world'
		_state_machine.userdata.part = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.camera_topic_2 = '/ariac/logical_camera_2'
		_state_machine.userdata.camera_frame_1 = 'logical_camera_1_frame'
		_state_machine.userdata.comparison = ''
		_state_machine.userdata.camera_topic_1 = '/ariac/logical_camera_1'
		_state_machine.userdata.camera_topic_3 = '/ariac/logical_camera_3'
		_state_machine.userdata.camera_topic_4 = '/ariac/logical_camera_4'
		_state_machine.userdata.camera_topic_5 = '/ariac/logical_camera_5'
		_state_machine.userdata.camera_topic_6 = '/ariac/logical_camera_6'
		_state_machine.userdata.camera_frame_2 = 'logical_camera_2_frame'
		_state_machine.userdata.camera_frame_3 = 'logical_camera_3_frame'
		_state_machine.userdata.camera_frame_4 = 'logical_camera_4_frame'
		_state_machine.userdata.camera_frame_5 = 'logical_camera_5_frame'
		_state_machine.userdata.camera_frame_6 = 'logical_camera_6_frame'
		_state_machine.userdata.index_bin1 = 'bin1'
		_state_machine.userdata.index_bin2 = 'bin2'
		_state_machine.userdata.index_bin3 = 'bin3'
		_state_machine.userdata.index_bin4 = 'bin4'
		_state_machine.userdata.index_bin5 = 'bin5'
		_state_machine.userdata.index_bin6 = 'bin6'
		_state_machine.userdata.output_bin_nr = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:144 y:44
			OperatableStateMachine.add('Detect parts in bin1',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin1Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_1', 'camera_frame': 'camera_frame_1', 'part': 'part', 'pose': 'pose'})

			# x:822 y:47
			OperatableStateMachine.add('Bin2Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 2', 'false': 'Detect parts in bin3'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})

			# x:1087 y:188
			OperatableStateMachine.add('Bin3Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 3', 'false': 'Detect parts in bin4'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})

			# x:1101 y:448
			OperatableStateMachine.add('Bin4Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 4', 'false': 'Detect parts in bin5'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})

			# x:686 y:577
			OperatableStateMachine.add('Bin5Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 5', 'false': 'Detect parts in bin6'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})

			# x:156 y:578
			OperatableStateMachine.add('Bin6Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 6', 'false': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})

			# x:621 y:46
			OperatableStateMachine.add('Detect parts in bin2',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin2Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_2', 'camera_frame': 'camera_frame_2', 'part': 'part', 'pose': 'pose'})

			# x:1062 y:107
			OperatableStateMachine.add('Detect parts in bin3',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin3Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_3', 'camera_frame': 'camera_frame_3', 'part': 'part', 'pose': 'pose'})

			# x:1080 y:372
			OperatableStateMachine.add('Detect parts in bin4',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin4Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_4', 'camera_frame': 'camera_frame_4', 'part': 'part', 'pose': 'pose'})

			# x:869 y:577
			OperatableStateMachine.add('Detect parts in bin5',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin5Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_5', 'camera_frame': 'camera_frame_5', 'part': 'part', 'pose': 'pose'})

			# x:348 y:577
			OperatableStateMachine.add('Detect parts in bin6',
										DetectFirstPartCameraAriacState(part_list=['gasket_part', 'gear_part', 'piston_rod_part'], time_out=0.5),
										transitions={'continue': 'Bin6Controle', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic_6', 'camera_frame': 'camera_frame_6', 'part': 'part', 'pose': 'pose'})

			# x:226 y:124
			OperatableStateMachine.add('Lookup name of bin 1',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin1', 'column_value': 'output_bin_nr'})

			# x:719 y:116
			OperatableStateMachine.add('Lookup name of bin 2',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin2', 'column_value': 'output_bin_nr'})

			# x:934 y:180
			OperatableStateMachine.add('Lookup name of bin 3',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin3', 'column_value': 'output_bin_nr'})

			# x:944 y:415
			OperatableStateMachine.add('Lookup name of bin 4',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin4', 'column_value': 'output_bin_nr'})

			# x:794 y:515
			OperatableStateMachine.add('Lookup name of bin 5',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin5', 'column_value': 'output_bin_nr'})

			# x:254 y:512
			OperatableStateMachine.add('Lookup name of bin 6',
										LookupFromTableState(parameter_name='/ariac_tables_unit2', table_name='bin_names', index_title='bin', column_title='bin_name'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'index_bin6', 'column_value': 'output_bin_nr'})

			# x:374 y:46
			OperatableStateMachine.add('Bin1Controle',
										EqualState(),
										transitions={'true': 'Lookup name of bin 1', 'false': 'Detect parts in bin2'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'part', 'value_b': 'comparison'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
