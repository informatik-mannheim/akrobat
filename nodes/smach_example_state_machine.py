#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy
from akrobat.msg import movement


############################################################
# define state NeutralRG
class NeutralRG(smach.State):
	def callback(self, data):
		self.buttons=data.buttons
		self.axes=data.axes
	
#initiation of the state
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.buttons=[0,0,0,0,0,0,0,0,0,0,0]
		self.axes=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.bounce_control=0#bounce control of the button to prohibit very fast cycling through the states
	
		rospy.Subscriber("joy", Joy, self.callback)
#execution of the state
	def execute(self, userdata):
		rospy.loginfo('Executing state NeutralRG')
        
		#TODO: Add reset to state machine instead of just putting the code here.
        
		backPublisher = rospy.Publisher('/movements', movement,queue_size=10)

		msg = movement()
		msg.commands = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		msg.walking_mode = 'reset'
		msg.macro = 'feet'

		backPublisher.publish(msg)
        
	#sleep functions while no action is taken or to prevent bouncing of the button
		while self.buttons[5]==1 and self.bounce_control==0:
			time.sleep(0.05)
		while self.axes[0] == 0.0 and self.axes[1] == 0.0 and self.buttons[5]==0:
			time.sleep(0.05)
			self.bounce_control=1
	#if the Moveprofile button is pressed
		if self.buttons[5]==1:
			self.bounce_control=0
			return 'outcome1'
	#if Joystick is not in neutral position
		elif self.axes[0] != 0.0 or self.axes[1] != 0.0:
			return 'outcome2'

	 
	 

# define state NeutralTG
class NeutralTG(smach.State):
	def callback(self, data):
		#rospy.loginfo(rospy.get_caller_id()+"i heard %s", data.axes[1])
		self.buttons=data.buttons
		self.axes=data.axes
		#rospy.loginfo(rospy.get_caller_id()+"i heard %s", self.x_axes)

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.buttons=[0,0,0,0,0,0,0,0,0,0,0]
		self.axes=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.bounce_control=0
	
		rospy.Subscriber("joy", Joy, self.callback)

	def execute(self, userdata):
		rospy.loginfo('Executing state NeutralTG')
        
		#TODO: Add reset to state machine instead of just putting the code here.
        
		backPublisher = rospy.Publisher('/movements', movement, queue_size=10)

		msg = movement()
		msg.commands = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		msg.walking_mode = 'reset'
		msg.macro = 'feet'

		backPublisher.publish(msg)
		#sleep functions while no action is taken or to prevent bouncing of the button
		while self.buttons[5]==1 and self.bounce_control==0:
			time.sleep(0.05)
		while self.axes[0] == 0.0 and self.axes[1] == 0.0 and self.buttons[5]==0:
			time.sleep(0.05)
			self.bounce_control=1
			#if the Moveprofile button is pressed
			if self.buttons[5]==1:
				self.bounce_control=0
				return 'outcome1'
			#if Joystick is not in neutral position
			elif self.axes[0] != 0.0 or self.axes[1] != 0.0:
				return 'outcome2'


# define state NeutralWG
class NeutralWG(smach.State):
	def callback(self, data):
		#rospy.loginfo(rospy.get_caller_id()+"i heard %s", data.axes[1])
		self.buttons=data.buttons
		self.axes=data.axes
		#rospy.loginfo(rospy.get_caller_id()+"i heard %s", self.x_axes)

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.buttons=[0,0,0,0,0,0,0,0,0,0,0]
		self.axes=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.bounce_control=0
	
		rospy.Subscriber("joy", Joy, self.callback)

	def execute(self, userdata):
		rospy.loginfo('Executing state NeutralWG')
		time.sleep(2)

		#TODO: Add reset to state machine instead of just putting the code here.
        
		backPublisher = rospy.Publisher('/movements', movement,queue_size=10)

		msg = movement()
		msg.commands = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		msg.walking_mode = 'reset'
		msg.macro = 'feet'

		backPublisher.publish(msg)
		#sleep functions while no action is taken or to prevent bouncing of the button
		while self.buttons[5]==1 and self.bounce_control==0:
			time.sleep(0.05)
		while self.axes[0] == 0.0 and self.axes[1] == 0.0 and self.buttons[5]==0:
			time.sleep(0.05)
			self.bounce_control=1
			#if the Moveprofile button is pressed
			if self.buttons[5]==1:
				self.bounce_control=0
				return 'outcome1'
			#if Joystick is not in neutral position
			elif self.axes[0] != 0.0 or self.axes[1] != 0.0:
				return 'outcome2'



############################################################
# define state RG
class RG(smach.State):
	def callback(self, data):
		self.axes=data.axes

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
		rospy.Subscriber("joy", Joy, self.callback)

	def execute(self, userdata):
		rospy.loginfo('Executing state RG')
		#if Joystick not neutral
		while self.axes[0] != 0.0 or self.axes[1] != 0.0: 
			time.sleep(0.05)
			return 'outcome1'
       

# define state TG
class TG(smach.State):
	def callback(self, data):
		self.axes=data.axes

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
		rospy.Subscriber("joy", Joy, self.callback)

	def execute(self, userdata):
		rospy.loginfo('Executing state TG')
		#if Joystick not neutral
		while self.axes[0] != 0.0 or self.axes[1] != 0.0: 
			time.sleep(0.05)
			return 'outcome1'


  # define state WG
class WG(smach.State):
	def callback(self, data):
		self.axes=data.axes

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
		rospy.Subscriber("joy", Joy, self.callback)

	def execute(self, userdata):
		rospy.loginfo('Executing state WG')
		#if Joystick not neutral
		while self.axes[0] != 0.0 or self.axes[1] != 0.0: 
			time.sleep(0.05)
			return 'outcome1'  


############################################################
class StopRG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
        

	def execute(self, userdata):
		rospy.loginfo('Executing state StopRG')
		return 'outcome1'

class StopTG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
        

	def execute(self, userdata):
		rospy.loginfo('Executing state StopTG')
		return 'outcome1'

class StopWG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])


	def execute(self, userdata):
		rospy.loginfo('Executing state StopWG')
		return 'outcome1'




############################################################
class RunRG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
       

	def execute(self, userdata):
		rospy.loginfo('Executing state RunRG')
		return 'outcome1'

class RunTG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
       

	def execute(self, userdata):
		rospy.loginfo('Executing state RunTG')
		return 'outcome1'

class RunWG(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
        

	def execute(self, userdata):
		rospy.loginfo('Executing state RunWG')
		return 'outcome1'
############################################################


#if __name__ == '__main__':

def main():
	rospy.init_node('smach_example_state_machine')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['outcome4'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()


	# Open the container
	with sm:
        # Add states to the container
############################################################
		smach.StateMachine.add('NeutralRG', NeutralRG(), 
                               transitions={'outcome1':'NeutralTG','outcome2':'RunRG'})
		smach.StateMachine.add('NeutralTG', NeutralTG(), 
                               transitions={'outcome1':'NeutralWG','outcome2':'RunTG'})
		smach.StateMachine.add('NeutralWG', NeutralWG(), 
                               transitions={'outcome1':'NeutralRG','outcome2':'RunWG'})
############################################################
		smach.StateMachine.add('RG', RG(), 
                               transitions={'outcome1':'StopRG'})
		smach.StateMachine.add('TG', TG(), 
                               transitions={'outcome1':'StopTG'})
		smach.StateMachine.add('WG', WG(), 
                               transitions={'outcome1':'StopWG'})
############################################################
		smach.StateMachine.add('StopRG', StopRG(), 
                               transitions={'outcome1':'NeutralRG'})
		smach.StateMachine.add('StopTG', StopTG(), 
                               transitions={'outcome1':'NeutralTG'})
		smach.StateMachine.add('StopWG', StopWG(), 
                               transitions={'outcome1':'NeutralWG'})
############################################################
		smach.StateMachine.add('RunRG', RunRG(), 
                               transitions={'outcome1':'RG'})
		smach.StateMachine.add('RunTG', RunTG(), 
                               transitions={'outcome1':'TG'})
		smach.StateMachine.add('RunWG', RunWG(), 
                               transitions={'outcome1':'WG'})


    # Execute SMACH plan
	outcome = sm.execute()

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()

# Create and start the introspection server
#    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
#    sis.start()

# Execute the state machine
#    outcome = sm.execute()

# Wait for ctrl-c to stop the application
#    rospy.spin()
#    sis.stop()


