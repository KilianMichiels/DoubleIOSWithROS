//
//  MainROSDoubleControl.h
//  FacialRecognition
//
//  Created by Kilian on 14/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//
/**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**/
/*------------------------------------------------------------------------*/
/*                     ROS MAIN CONTROLLER HEADER FILE                    */
/*------------------------------------------------------------------------*/
/**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**/

#import <Foundation/Foundation.h>

#import "Constants.h"

#import "RBManager.h"
#import "RBMessage.h"
#import "RBPublisher.h"
#import "RBSubscriber.h"

#import "IntMessage.h"
#import "NavSatFixMessage.h"
#import "FloatMessage.h"
#import "StringMessage.h"
#import "CompressedImage.h"

/*
 Delegates from MainROSControl:
 */

@protocol MainROSControlDelegate <NSObject>
@optional

- (void) MainROSControlDidConnect                                  ;
- (void) MainROSControlDidCloseWithCode     :(NSInteger)code reason:(NSString *)reason wasClean:(BOOL)wasClean;
- (void) MainROSControlDidNotConnectWithTimeout;
- (void) MainRosControlDidFailWithError     :(NSError*)error;

- (void) messageUpdate_poleHeightPercentage :(FloatMessage*)   message;
- (void) messageUpdate_kickStandState       :(IntMessage*)   message;
- (void) messageUpdate_Controls             :(TwistMessage*)   message;
- (void) messageUpdate_MotionDetection      :(BoolMessage*)   message;

@end

@interface MainROSDoubleControl : NSObject <RBManagerDelegate>

@property (nonatomic, weak) id<MainROSControlDelegate> delegate;

/*
 Here are all the parameters the Double can have:
 int kickStandState;
 NSString * firmwareVersion;
 float batteryPercentage;
 BOOL batteryIsFullyCharged;
 NSString * serial;
 float poleHeightPercentage;
 
 So for each parameter we have to add a subscriber and/or publisher:
 */

@property RBPublisher * kickStandStatePublisher;
@property RBSubscriber * kickStandStateSubscriber;

@property RBPublisher * firmwareVersionPublisher;

@property RBPublisher * batteryPercentagePublisher;

@property RBPublisher * batteryIsFullyChargedPublisher;

@property RBPublisher * serialPublisher;

@property RBPublisher * poleHeightPercentagePublisher;
@property RBSubscriber * poleHeightPercentageSubscriber;

/*      Other parameters:       */
@property RBPublisher * statusPublisher;

@property RBSubscriber * controlsSubscriber;
@property RBPublisher * controlsPublisher;

@property RBPublisher * locationPublisher;

@property RBSubscriber * motionDetectionSubscriber;
@property RBPublisher * motionDetectionPublisher;

@property RBPublisher * imagePublisher;

/*
 Extra variables for RBManager control:
 */
@property RBManager * manager;


/*!
 Start the Main Control.
 */

- (BOOL)initialiseMainROSDoubleControlManually:(NSString * )ip_adres;
- (BOOL)initialiseMainROSDoubleControl:(int)IP;

/*!
 End the Main Control.
 */
- (BOOL)disconnectMainROSDoubleControl;

/*------------------------------------------------------------------------*/
/*                              ROS PUBLISHERS                            */
/*------------------------------------------------------------------------*/

/*!
 Publish if the iPad is connected to the Double 1.
 */
- (BOOL)initPublishStatus;
- (void)publishStatus:(BOOL)status;

/*!
 Publish the height of the Double 1 pole in percentages.
 */
- (BOOL)initPublishPoleHeightPercentage;
- (void)publishPoleHeight:(float)heightPercentage;

/*!
 Publish the battery percentage of the Double 1.
 */
- (BOOL)initPublishBatteryPercentage;
- (void)publishBatteryPercentage:(float)batteryPercentage;

/*!
 Publish the serial number of the Double 1.
 */
- (BOOL)initPublishSerial;
- (void)publishSerial:(NSString *)serial;

/*!
 Publish the kickstandstate of the Double 1.
 */
- (BOOL)initPublishKickStandState;
- (void)publishKickStandState:(int)kickStandState;

/*!
 Publish if the battery of the Double 1 is fully charged.
 */
- (BOOL)initPublishBatteryIsFullyCharged;
- (void)publishBatteryIsFullyCharged:(BOOL)batteryIsFullyCharged;
/*!
 Publish the firmware version of the Double 1.
 */
- (BOOL)initPublishFirmwareVersion;
- (void)publishFirmwareVersion:(NSString *)firmwareVersion;
/*!
 Publish the Double 1's current location.
 */
- (BOOL)initPublishLocation;
- (void)publishLocation:(CLLocation * )location;
/*!
 Publish motion detection.
 */
- (BOOL)initPublishMotionDetection;
- (void)publishMotionDetection:(BOOL)motionDetection;
/*!
 Publish the cameraview.
 */
- (BOOL)initPublishImage;
- (void)publishImage:(NSString*)image;

/*------------------------------------------------------------------------*/
/*                              ROS SUBSCRIBERS                           */
/*------------------------------------------------------------------------*/

/*!
 Listen to the pole height percentage of the ROS control center.
 */
- (BOOL)subscribeToPoleHeightPercentage;

/*!
 Subscribe to the kickstandstate of the ROS control center.
 */
- (BOOL)subscribeToKickStandState;

/*!
 Listen to controls from ROS control center.
 Possible controls are:
 - Forward
 - Backward
 - Left
 - Right
 */
- (BOOL)subscribeToControls;

/*!
 Listen to the motion detection channel.
 */
- (BOOL)subscribeToMotionDetection;

////////////////////////////////// FOR TESTING ////////////////////////////////
- (void)sendTestObjects;
////////////////////////////////// FOR TESTING ////////////////////////////////

@end
