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
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.create_pose import CreatePoseState
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
		offset_p1 = [-0.15,-0.15,0.03]
		offset_p2 = [0.15,0,0]
		rpy = [0,0,0]
		# x:335 y:625, x:246 y:624
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['positie_xyz', 'bin', 'iterator'], output_keys=['positie_xyz', 'iterator'])
		_state_machine.userdata.iterator = 1
		_state_machine.userdata.P1 = 1
		_state_machine.userdata.P2 = 2
		_state_machine.userdata.P3 = 3
		_state_machine.userdata.P4 = 4
		_state_machine.userdata.P5 = 5
		_state_machine.userdata.P6 = 6
		_state_machine.userdata.one_up = 1
		_state_machine.userdata.positie_xyz = []
		_state_machine.userdata.offset_p3 = [0,-0.15,0,0,0,0]
		_state_machine.userdata.offset_p4 = [-0.3,0.30,0,0,0,0]
		_state_machine.userdata.offset_p5 = [0,-0.15,0,0,0,0]
		_state_machine.userdata.offset_p2 = []
		_state_machine.userdata.offset_p6 = [0,-0.15,0,0,0,0]
		_state_machine.userdata.reset_iterator = -5
		_state_machine.userdata.reset_positie = [0.3,0.30,0,0,0,0]
		_state_machine.userdata.bin = ''
		_state_machine.userdata.camera_bin_frame = ''
		_state_machine.userdata.offset_p1 = []
		_state_machine.userdata.bin_frame = ''
		_state_machine.userdata.ref_frame = 'world'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:544
			OperatableStateMachine.add('Lookup camera frame',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='bin_configuration', index_title='bin', column_title='camera_bin_frame'),
										transitions={'found': 'Lookup bin frame', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'bin_frame'})

			# x:322 y:219
			OperatableStateMachine.add('Add_offset_p2',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p2', 'output_pose': 'positie_xyz'})

			# x:524 y:133
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

			# x:131 y:122
			OperatableStateMachine.add('Create offset p1',
										CreatePoseState(xyz=offset_p1, rpy=rpy),
										transitions={'continue': 'Add_offset_p1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'offset_p1'})

			# x:311 y:121
			OperatableStateMachine.add('Create offset p1_2',
										CreatePoseState(xyz=offset_p2, rpy=rpy),
										transitions={'continue': 'Add_offset_p2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'offset_p2'})

			# x:39 y:374
			OperatableStateMachine.add('GetBinPose',
										GetObjectPoseState(),
										transitions={'continue': 'If p1 is available ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'frame': 'bin_frame', 'pose': 'positie_xyz'})

			# x:131 y:47
			OperatableStateMachine.add('If p1 is available ',
										EqualState(),
										transitions={'true': 'Create offset p1', 'false': 'If p2 is available'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'P1', 'value_b': 'iterator'})

			# x:318 y:50
			OperatableStateMachine.add('If p2 is available',
										EqualState(),
										transitions={'true': 'Create offset p1_2', 'false': 'If p3 is available'},
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

			# x:33 y:465
			OperatableStateMachine.add('Lookup bin frame',
										LookupFromTableState(parameter_name='/ariac_unit2_tables', table_name='bin_configuration', index_title='bin', column_title='bin_frame'),
										transitions={'found': 'GetBinPose', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'bin_frame'})

			# x:1296 y:93
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

			# x:135 y:300
			OperatableStateMachine.add('Update iterator p1',
										AddNumericState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'iterator', 'value_b': 'one_up', 'result': 'iterator'})

			# x:308 y:295
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

			# x:145 y:228
			OperatableStateMachine.add('Add_offset_p1',
										AddOffsetToPoseState(),
										transitions={'continue': 'Update iterator p1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'positie_xyz', 'offset_pose': 'offset_p1', 'output_pose': 'positie_xyz'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
