//
//  Constants.h
//  ROSApplication
//
//  Created by Kilian Michiels on 12/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//

#ifndef Constants_h

#define Constants_h

///////////////////// FOR TESTING PURPOSES ONLY! /////////////////////
/**/                                                              /**/
/**/    #define TESTING_VARIABLES               ((BOOL) YES)      /**/
/**/                                                              /**/
//////////////////////////////////////////////////////////////////////

// PMI-WIFI-ZEE:
#define IP_ROS_ROBOT_Home       ((NSString*) @"ws://192.168.30.100:9090")


// iVisitor:
#define IP_ROS_ROBOT_iGent      ((NSString*) @"ws://10.10.130.35:9090")

// Kot:
#define IP_ROS_ROBOT_Kot        ((NSString*) @"ws://192.168.1.126:9090")

#define PARAMETER_UPDATE_SPEED  ((double) 5.0)

#define CAMERA_UPDATE_SPEED     ((double) 1.0)

#define SPEED                   ((double) 0.2)

#define POLE_TIMEOUT            ((int) 50000000)

#endif /* Constants_h */
