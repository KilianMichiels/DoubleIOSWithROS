# -*- coding: utf-8 -*-

'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                            Thesis: getDoubleData.py                         ║
╠═════════════════════════════════════ By ════════════════════════════════════╣
║                               Michiels Kilian                               ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                             DOUBLE AVAILABLE DATA                           ║
║                                                                             ║
║                     /Double_1/actualPoleHeightPercentage                    ║
║                     /Double_1/actualkickStandState                          ║
║                     /Double_1/batteryFullyCharged                           ║
║                     /Double_1/batteryPercentage                             ║
║                     /Double_1/camera                                        ║
║                     /Double_1/firmwareVersion                               ║
║                     /Double_1/location                                      ║
║                     /Double_1/motionDetection                               ║
║                     /Double_1/serial                                        ║
║                     /Double_1/status                                        ║
╚═════════════════════════════════════════════════════════════════════════════╝

'''
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                   IMPORTS                                   ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
import sys
import rospy
import numpy as np
import os
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, Float32, Bool, String
from sensor_msgs.msg import NavSatFix, CompressedImage
import cv2
import datetime
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
status_message = {
        'status'                     : None,
        'service'                    : None
        }
location_message = {
        'longitude'                  : None,
        'latitude'                   : None,
        'altitude'                   : None,
        'status'                     : status_message,
        }
doubleParameters = {
        'actualPoleHeightPercentage' : None,
        'actualkickStandState'       : None,
        'batteryFullyCharged'        : None,
        'batteryPercentage'          : None,
        'firmwareVersion'            : None,
        'location'                   : location_message,
        'motionDetection'            : None,
        'serial'                     : None,
        'status'                     : None
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

def getActualPoleHeightPercentage():
    print('Starting: actualPoleHeightPercentage')
    return rospy.Subscriber('/Double_1/actualPoleHeightPercentage', Float32, updateActualPoleHeightPercentage, queue_size = 1)

def getActualkickStandState():
    print('Starting: actualkickStandState')
    return rospy.Subscriber('/Double_1/actualkickStandState', Int32, updateActualkickStandState, queue_size = 1)

def getBatteryFullyCharged():
    print('Starting: batteryFullyCharged')
    return rospy.Subscriber('/Double_1/batteryFullyCharged', Bool, updateBatteryFullyCharged, queue_size = 1)

def getBatteryPercentage():
    print('Starting: batteryPercentage')
    return rospy.Subscriber('/Double_1/batteryPercentage', Float32, updateBatteryPercentage, queue_size = 1)

def getFirmwareVersion():
    print('Starting: firmwareVersion')
    return rospy.Subscriber('/Double_1/firmwareVersion', String, updateFirmwareVersion, queue_size = 1)

def getLocation():
    print('Starting: location')
    return rospy.Subscriber('/Double_1/location', NavSatFix, updateLocation, queue_size = 1)

def getMotionDetection():
    print('Starting: motionDetection')
    return rospy.Subscriber('/Double_1/motionDetection', Bool, updateMotionDetection, queue_size = 1)

def getSerial():
    print('Starting: serial')
    return rospy.Subscriber('/Double_1/serial', String, updateSerial, queue_size = 1)

def getStatus():
    print('Starting: status')
    return rospy.Subscriber('/Double_1/status', Bool, updateStatus, queue_size = 1)

def getCamera():
    print('Starting: camera')
    return rospy.Subscriber('/Double_1/camera', CompressedImage, updateCamera, queue_size = 1)

#------------------------------------------------------------------------------#

def updateActualPoleHeightPercentage(ros_data):
    doubleParameters['actualPoleHeightPercentage'] = ros_data.data
    updateConsole()

def updateActualkickStandState(ros_data):
    doubleParameters['actualkickStandState'] = ros_data.data
    updateConsole()

def updateBatteryFullyCharged(ros_data):
    doubleParameters['batteryFullyCharged'] = ros_data.data
    updateConsole()

def updateBatteryPercentage(ros_data):
    doubleParameters['batteryPercentage'] = ros_data.data
    updateConsole()

def updateFirmwareVersion(ros_data):
    doubleParameters['firmwareVersion'] = ros_data.data
    updateConsole()

def updateLocation(ros_data):
    status_message['status'] = ros_data.status.status
    status_message['service'] = ros_data.status.service

    location_message['longitude'] = ros_data.longitude
    location_message['latitude'] = ros_data.latitude
    location_message['altitude'] = ros_data.altitude
    location_message['status'] = status_message

    doubleParameters['location'] = location_message
    updateConsole()

def updateMotionDetection(ros_data):
    doubleParameters['motionDetection'] = ros_data.data
    updateConsole()

def updateSerial(ros_data):
    doubleParameters['serial'] = ros_data.data
    updateConsole()

def updateStatus(ros_data):
    doubleParameters['status'] = ros_data.data
    updateConsole()

def updateCamera(ros_data):
    time = datetime.datetime.now().time()
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, 1)
    #image_np = cv2.flip(image_np,1)
    cv2.putText(image_np, str(time.hour) + ':' + str(time.minute) + ':' + str(time.second), (20, 300), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 255, 255))
    cv2.imshow(imageTitle, image_np)
    cv2.waitKey(20)

#------------------------------------------------------------------------------#

def updateConsole():
    os.system("clear")
    if(doubleParameters['status'] == True):
        print(bcolors.OKGREEN + '\n    ------------------------- CONNECTED -------------------------' + bcolors.ENDC)
    else:
        print(bcolors.FAIL + '\n    ----------------------- NOT CONNECTED -----------------------' + bcolors.ENDC)
    print('''
    --------------------- Double Parameters: --------------------
    |   status:\t\t\t\t{}\t\t\t|
    -------------------------------------------------------------
    |   serial:\t\t\t\t{}\t\t|
    -------------------------------------------------------------
    |   firmwareVersion:\t\t{}\t\t\t|
    -------------------------------------------------------------
    |   batteryFullyCharged:\t\t{}\t\t\t|
    -------------------------------------------------------------
    |   batteryPercentage:\t\t{}\t\t|
    -------------------------------------------------------------
    |   motionDetection:\t\t{}\t\t\t|
    -------------------------------------------------------------
    |   actualPoleHeightPercentage:\t{}\t\t|
    -------------------------------------------------------------
    |   actualkickStandState:\t\t{}\t\t\t|
    -------------------------------------------------------------
    |   location:\t\t\t\t\t\t|
    |   \t\t\t Longitude:\t{}\t|
    |   \t\t\t Latitude:\t{}\t|
    -------------------------------------------------------------
    '''.format(doubleParameters['status'],
    doubleParameters['serial'],
    doubleParameters['firmwareVersion'],
    doubleParameters['batteryFullyCharged'],
    doubleParameters['batteryPercentage'],
    doubleParameters['motionDetection'],
    doubleParameters['actualPoleHeightPercentage'],
    doubleParameters['actualkickStandState'],
    doubleParameters['location']['longitude'],
    doubleParameters['location']['latitude']
    ))

def main(args):
    print('Initializing data receiver...')
    #-------------------------------------------------#

    status = getStatus()
    serial = getSerial()
    firmwareVersion = getFirmwareVersion()
    batteryFullyCharged = getBatteryFullyCharged()
    batteryPercentage = getBatteryPercentage()
    motionDetection = getMotionDetection()
    actualPoleHeightPercentage = getActualPoleHeightPercentage()
    actualkickStandState = getActualkickStandState()
    location = getLocation()
    camera = getCamera()

    #-------------------------------------------------#

    rospy.init_node('doubleDataReceiver', anonymous=True)

    try:
        print("Running...")
        rospy.spin()

	#-------------------------------------------------#
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
        cv2.destroyAllWindows()
'''
╔═════════════════════════════════════════════════════════════════════════════╗
║                                    PROGRAM                                  ║
╚═════════════════════════════════════════════════════════════════════════════╝
'''
if __name__ == '__main__':
    main(sys.argv)
