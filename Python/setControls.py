#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                             Thesis: setControls.py                          ║
╠═════════════════════════════════════ By ════════════════════════════════════╣
║                               Michiels Kilian                               ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                           DOUBLE AVAILABLE CONTROLS                         ║
║                                                                             ║
║                       arrows = direction of the double                      ║
║                                                                             ║
║                            r = retract kickstand                            ║
║                             d = deploy kickstand                            ║
║                                                                             ║
║                                a = pole up                                  ║
║                               q = pole down                                 ║
║                                                                             ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                   IMPORTS                                   ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
import rospy
import os
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, Float32
from Tkinter import *
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                  CONSTANTS                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
imageTitle = 'ROS Image from iPad :)'
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                  VARIABLES                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
pub_controls = rospy.Publisher('/Double_1/controls', Twist, queue_size=10)
pub_kickstand = rospy.Publisher('/Double_1/desiredkickStandState', Int32, queue_size=10)
pub_poleHeight = rospy.Publisher('/Double_1/desiredPoleHeightPercentage', Float32, queue_size=10)

poleHeights = {
	'actualPoleHeight'	: None,
	'desiredPoleHeight'	: None
}

'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                   CLASSES                                   ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                  FUNCTIONS                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''

'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                    PROGRAM                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''

def leftKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                      Left key pressed                     |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	msg = Twist(Vector3(0,0,0),Vector3(0,0,1))
	control_talker(msg)

def rightKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                     Right key pressed                     |
    -------------------------------------------------------------
	''' + bcolors.ENDC)
	msg = Twist(Vector3(0,0,0),Vector3(0,0,-1))
	control_talker(msg)

def upKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                       Up key pressed                      |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	msg = Twist(Vector3(1,0,0),Vector3(0,0,0))
	control_talker(msg)

def downKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                      Down key pressed                     |
    -------------------------------------------------------------
	''' + bcolors.ENDC)
	msg = Twist(Vector3(-1,0,0),Vector3(0,0,0))
	control_talker(msg)

def stopKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                      Stop key pressed                     |
    -------------------------------------------------------------
	''' + bcolors.ENDC)
	msg = Twist(Vector3(0,0,0),Vector3(0,0,0))
	control_talker(msg)

def kickstandDeployKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                   Deploy kickstand pressed                |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	msg = 1
	kickstand_talker(msg)

def kickstandRetractKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                   Retract kickstand pressed               |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	msg = 2
	kickstand_talker(msg)

def updatePoleHeight(ros_data):
	update_UI()
	poleHeights['actualPoleHeight'] = ros_data.data
	#print('actualPoleHeight was changed to: {}').format(poleHeights['actualPoleHeight'])

def poleHeightUpKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                      Pole up pressed                      |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	adjustpoleHeight(1)

def poleHeightDownKey(event):
	update_UI()
	print(bcolors.OKBLUE + '''
    -------------------------------------------------------------
    |                     Pole down pressed                     |
    -------------------------------------------------------------
    ''' + bcolors.ENDC)
	adjustpoleHeight(0)

def adjustpoleHeight(direction):
	if(direction == 1):
		poleHeights['desiredPoleHeight'] = poleHeights['actualPoleHeight'] + 0.02
		if(poleHeights['desiredPoleHeight'] >= 1):
			poleHeights['desiredPoleHeight'] = 1
	elif(direction == 0):
		poleHeights['desiredPoleHeight'] = poleHeights['actualPoleHeight'] - 0.02
		if(poleHeights['desiredPoleHeight'] <= 0):
			poleHeights['desiredPoleHeight'] = 0
	poleHeight_talker(poleHeights['desiredPoleHeight'])

def control_talker(msg):
	pub_controls.publish(msg)

def kickstand_talker(msg):
	pub_kickstand.publish(msg)

def poleHeight_talker(msg):
	pub_poleHeight.publish(msg)

def update_UI():
	os.system("clear")
	print('''
    ---------------------- Double Controls: ---------------------
    |   Arrows : Direction of the Double.                       |
    -------------------------------------------------------------
    |   r = Retract kickstand                                   |
    |   d = Deploy kickstand                                    |
    -------------------------------------------------------------
    |   a = Pole up                                             |
    |   q = Pole down                                           |
    -------------------------------------------------------------
    ''')

#------------------------------------ MAIN ---------------------------------#

def main(args):
	update_UI()
	try:
		print('Initializing controls...')

		sub_poleHeight = rospy.Subscriber('/Double_1/actualPoleHeightPercentage', Float32, updatePoleHeight, queue_size = 10)

		rospy.init_node('controls', anonymous=True)

		print('Starting TKinter...')

		main = Tk()
		frame = Frame(main, width=100, height=100)
		main.bind('<Left>', leftKey)
		main.bind('<Right>', rightKey)
		main.bind('<Down>', downKey)
		main.bind('<Up>', upKey)
		main.bind('<space>', stopKey)
		main.bind('d', kickstandDeployKey)
		main.bind('r', kickstandRetractKey)

		main.bind('a', poleHeightUpKey)
		main.bind('q', poleHeightDownKey)

		frame.pack()

		print('You can now control the Double!')
		main.mainloop()
		print('End of program.')

	except rospy.ROSInterruptException:
		pass

'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                    PROGRAM                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
if __name__ == '__main__':
    main(sys.argv)
