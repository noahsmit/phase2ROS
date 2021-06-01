#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from flexbe_core import EventState, Logger

class getEmptyBin(EventState):
	'''
	State for know witch bin is empty

	># binPartType 		string[] 	array for all content in bins 
	#> bin 			string		name of empty bin
	#> bin_frame 		string		name frame of empty bin

	'''

	def __init__(self):
		super(getEmptyBin,self).__init__(input_keys = ['binPartType'],outcomes = ['continue', 'failed'], output_keys = ['bin','bin_frame'])


	def execute(self, userdata):
		Logger.logwarn('execute start')
		try :
			for i in range(len(userdata.binPartType)):
				Logger.logwarn('Loop started')
				if userdata.binPartType[i] == 'empty' or  userdata.binPartType[i] == 'Empty':
					userdata.bin = 'bin'+ str(i+1)
					userdata.bin_frame = 'bin'+ str(i+1) + '_frame'
					Logger.logwarn('Empty bin : %s' % str(bin))
					return 'continue'
		except :
			Logger.logwarn("An exception occurred")
			return 'failed'

	def on_enter(self, userdata):
		Logger.logwarn('on_enter start')
		self._binPartType = ['','']
		self._binPartType = userdata.binPartType
		pass

	def on_exit(self, userdata):
		pass

	def on_start(self):
		pass


	def on_stop(self):
		pass
