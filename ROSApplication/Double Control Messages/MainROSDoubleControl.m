//
//  MainROSDoubleControl.m
//  FacialRecognition
//
//  Created by Kilian on 14/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//
/**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**/
/*------------------------------------------------------------------------*/
/*                        ROS MAIN CONTROLLER M FILE                      */
/*------------------------------------------------------------------------*/
/**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**/

#import "MainROSDoubleControl.h"

//// FOR TESTING ////
#include <stdlib.h>
/////////////////////

@implementation MainROSDoubleControl

- (BOOL)initialiseMainROSDoubleControl:(int)IP{
    NSLog(@"\n-------------------------------------------------------------------------\n                MAIN ROS <-> DOUBLE CONTROL STARTUP \n-------------------------------------------------------------------------\n");
    _manager = [[RBManager defaultManager] init];
    _manager.delegate = self;
    
    if(IP == 1){
        @try {
            [_manager connect:IP_ROS_ROBOT_Home];
        } @catch (NSException *exception) {
            return NO;
        }
    }
    else if(IP == 2){
        @try {
            [_manager connect:IP_ROS_ROBOT_iGent];
        } @catch (NSException *exception) {
            return NO;
        }
    }
    else if(IP == 3){
        @try {
            [_manager connect:IP_ROS_ROBOT_Kot];
        } @catch (NSException *exception) {
            return NO;
        }
    }
    return YES;
}

- (BOOL)initialiseMainROSDoubleControlManually:(NSString *)ip_adres{
    
    NSLog(@"\n-------------------------------------------------------------------------\n                MAIN ROS <-> DOUBLE CONTROL STARTUP (MANUALLY)\n-------------------------------------------------------------------------\n");
    _manager = [[RBManager defaultManager] init];
    _manager.delegate = self;
    
    @try {
        [_manager connect:ip_adres];
    } @catch (NSException *exception) {
        return NO;
    }
    
    [self.delegate MainROSControlDidConnect];
    return YES;
}

- (BOOL)disconnectMainROSDoubleControl{
    @try {
        [self cleanup];
        [_manager disconnect];
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not disconnect ROS MAIN CONTROL: %@", exception);
        return NO;
    }
}

-(void)managerDidConnect:(RBManager *)manager{
    // Initialize listeners and/or publishers
    [self initialise];
}

-(void)managerDidTimeout:(RBManager *)manager{
    [self.delegate MainROSControlDidNotConnectWithTimeout];
}

-(void)manager:(RBManager *)manager didCloseWithCode:(NSInteger)code reason:(NSString *)reason wasClean:(BOOL)wasClean{
    [self.delegate MainROSControlDidCloseWithCode:code reason:reason wasClean:wasClean];
}

-(void)manager:(RBManager *)manager didFailWithError:(NSError *)error{
    [self.delegate MainRosControlDidFailWithError:error];
}

/*------------------------------------------------------------------------*/
/*                              ROS PUBLISHERS                            */
/*------------------------------------------------------------------------*/

/*
 Message Type: BoolMessage
 */
- (BOOL)initPublishStatus{
    NSLog(@"Publishing Status");
    
    @try {
        [_statusPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/status";
    
    _statusPublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Bool"];
    
    @try {
        [_statusPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishStatus:(BOOL)status{
    NSNumber * messagedata = [NSNumber numberWithBool:status];
    
    BoolMessage * booldata = [[BoolMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_statusPublisher publish:booldata];
}

/*
 Message Type: FloatMessage
 */
- (BOOL)initPublishPoleHeightPercentage{
    NSLog(@"Publishing PoleHeightPercentage");
    
    @try {
        [_poleHeightPercentagePublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/actualPoleHeightPercentage";
    
    _poleHeightPercentagePublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Float32"];
    
    @try {
        [_poleHeightPercentagePublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishPoleHeight:(float)heightPercentage{
    NSNumber * messagedata = [NSNumber numberWithFloat:heightPercentage];
    
    FloatMessage * floatdata = [[FloatMessage alloc] init];
    
    floatdata.data = messagedata;
    
    [_poleHeightPercentagePublisher publish:floatdata];
}

/*
 Message Type: FloatMessage
 */
- (BOOL)initPublishBatteryPercentage{
    NSLog(@"Publishing BatteryPercentage");
    
    @try {
        [_batteryPercentagePublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/batteryPercentage";
    
    _batteryPercentagePublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Float32"];
    
    @try {
        [_batteryPercentagePublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishBatteryPercentage:(float)batteryPercentage{
    NSNumber * messagedata = [NSNumber numberWithFloat:batteryPercentage];
    
    FloatMessage * booldata = [[FloatMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_batteryPercentagePublisher publish:booldata];
}

/*
 Message Type: StringMessage
 */
- (BOOL)initPublishSerial{
    NSLog(@"Publishing Serial");
    
    @try {
        [_serialPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/serial";
    
    _serialPublisher = [_manager addPublisher:publisher messageType:@"std_msgs/String"];
    
    @try {
        [_serialPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

-(void)publishSerial:(NSString *)serial{
    NSString * messagedata = serial;
    
    StringMessage * booldata = [[StringMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_serialPublisher publish:booldata];
}

/*
 Message Type: IntMessage
 */
- (BOOL)initPublishKickStandState{
    NSLog(@"Publishing KickStandState");
    
    @try {
        [_kickStandStatePublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/actualkickStandState";
    
    _kickStandStatePublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Int32"];
    
    @try {
        [_kickStandStatePublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishKickStandState:(int)kickStandState{
    NSNumber * messagedata = [NSNumber numberWithInt:kickStandState];
    
    IntMessage * booldata = [[IntMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_kickStandStatePublisher publish:booldata];
}

/*
 Message Type: BoolMessage
 */
- (BOOL)initPublishBatteryIsFullyCharged{
    NSLog(@"Publishing BatteryIsFullyCharged");
    
    @try {
        [_batteryIsFullyChargedPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/batteryFullyCharged";
    
    _batteryIsFullyChargedPublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Bool"];
    
    @try {
        [_batteryIsFullyChargedPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishBatteryIsFullyCharged:(BOOL)batteryIsFullyCharged{
    NSNumber * messagedata = [NSNumber numberWithBool:batteryIsFullyCharged];
    
    BoolMessage * booldata = [[BoolMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_batteryIsFullyChargedPublisher publish:booldata];
}

/*
 Message Type: IntMessage
 */
- (BOOL)initPublishFirmwareVersion{
    NSLog(@"Publishing FirmwareVersion");
    
    @try {
        [_firmwareVersionPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/firmwareVersion";
    
    _firmwareVersionPublisher = [_manager addPublisher:publisher messageType:@"std_msgs/String"];
    
    @try {
        [_firmwareVersionPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishFirmwareVersion:(NSString*)firmwareVersion{
    NSString * messagedata = firmwareVersion;
    
    StringMessage * stringdata = [[StringMessage alloc] init];
    
    stringdata.data = messagedata;
    
    [_firmwareVersionPublisher publish:stringdata];
}

/*
 Message Type: NavSatFixMessage
 */
- (BOOL)initPublishLocation{
    NSLog(@"Publishing Location");
    
    @try {
        [_locationPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/location";
    
    _locationPublisher = [_manager addPublisher:publisher messageType:@"sensor_msgs/NavSatFix"];
    
    @try {
        [_locationPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishLocation:(CLLocation *)location{
    NSNumber *latitude = [NSNumber numberWithFloat: location.coordinate.latitude];
    NSNumber *longitude = [NSNumber numberWithFloat: location.coordinate.longitude];
    NSNumber *altitude = [NSNumber numberWithFloat: location.altitude];
    
    NSNumber * status;
    if (location.horizontalAccuracy < 0)
    {
        // No Signal
        status = [NSNumber numberWithInteger:-1];
    }
    else if (location.horizontalAccuracy > 163)
    {
        // Poor Signal
        status = [NSNumber numberWithInteger:0];
    }
    else if (location.horizontalAccuracy > 48)
    {
        // Average Signal
        status = [NSNumber numberWithInteger:1];
    }
    else
    {
        // Full Signal
        status = [NSNumber numberWithInteger:2];
    }
    
    NavSatStatusMessage * statusMessage = [[NavSatStatusMessage alloc] init];
    
    // Check how good the signal is.
    statusMessage.status = status;
    
    // Apple uses all the systems together so...just set to GPS.
    statusMessage.service = [NSNumber numberWithInteger:1];
    
    NavSatFixMessage * navMessage = [[NavSatFixMessage alloc] init];
    
    navMessage.status = statusMessage;
    
    navMessage.latitude = latitude;
    navMessage.longitude = longitude;
    navMessage.altitude = altitude;
    
    NSArray * position_covariance;
    navMessage.position_covariance = position_covariance;
    
    NSNumber * position_covariance_type = [NSNumber numberWithInteger:0];
    navMessage.position_covariance_type = position_covariance_type;
    
    [_locationPublisher publish:navMessage];
}

/*
 Message Type: BoolMessage
 */
- (BOOL)initPublishMotionDetection{
    NSLog(@"Publishing MotionDetection");
    
    @try {
        [_motionDetectionPublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/motionDetection";
    
    _motionDetectionPublisher = [_manager addPublisher:publisher messageType:@"std_msgs/Bool"];
    
    @try {
        [_motionDetectionPublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishMotionDetection:(BOOL)motionDetection{
    NSNumber * messagedata = [NSNumber numberWithBool:motionDetection];
    
    BoolMessage * booldata = [[BoolMessage alloc] init];
    
    booldata.data = messagedata;
    
    [_motionDetectionPublisher publish:booldata];
}

/*
 Message Type: CompressedImage
 */
- (BOOL)initPublishImage{
    NSLog(@"Publishing Image");
    
    @try {
        [_imagePublisher unadvertise];
        NSLog(@"Unadvertised! (Just to be sure)");
    } @catch (NSException *exception) {
        NSLog(@"Could not unadvertise.");
    }
    
    NSString * publisher =  @"/Double_1/camera";
    
    _imagePublisher = [_manager addPublisher:publisher messageType:@"sensor_msgs/CompressedImage"];
    
    @try {
        [_imagePublisher advertise];
        NSLog(@"Advertising!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not advertise.");
        return NO;
    }
}

- (void)publishImage:(NSString*)image{
    
    CompressedImage *imgdata = [[CompressedImage alloc]init];
    imgdata.format = @"jpeg";
    imgdata.data = image;
    
    [_imagePublisher publish:imgdata];
}

/*------------------------------------------------------------------------*/
/*                              ROS SUBSCRIBERS                           */
/*------------------------------------------------------------------------*/

/*
 Message Class: FloatMessage
 */
- (BOOL)subscribeToPoleHeightPercentage{
    NSLog(@"Subscribing PoleHeightPercentage");
    
    @try {
        [_poleHeightPercentageSubscriber unsubscribe];
        NSLog(@"Unsubscribed.");
    } @catch (NSException *exception) {
        NSLog(@"Could not unsubscribe.");
    }
    
    NSString * subscriber = @"/Double_1/desiredPoleHeightPercentage";
    _poleHeightPercentageSubscriber = [_manager addSubscriber:subscriber responseTarget:self selector:@selector(poleHeightMessageUpdate:) messageClass: [BoolMessage class]];
    
    _poleHeightPercentageSubscriber.throttleRate = 100;
    
    @try {
        [_poleHeightPercentageSubscriber subscribe];
        NSLog(@"Subscribed!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not subscribe.");
        return NO;
    }
}

/*
 Message Class: IntMessage
 */
- (BOOL)subscribeToKickStandState{
    NSLog(@"Subscribing KickStandState");
    
    @try {
        [_kickStandStateSubscriber unsubscribe];
        NSLog(@"Unsubscribed.");
    } @catch (NSException *exception) {
        NSLog(@"Could not unsubscribe.");
    }
    
    NSString * subscriber = @"/Double_1/desiredkickStandState";
    _kickStandStateSubscriber = [_manager addSubscriber:subscriber responseTarget:self selector:@selector(KickStandStateMessageUpdate:) messageClass: [IntMessage class]];
    
    _kickStandStateSubscriber.throttleRate = 100;
    
    @try {
        [_kickStandStateSubscriber subscribe];
        NSLog(@"Subscribed!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not subscribe.");
        return NO;
    }
}

/*
 Message Class: TwistMessage
 */
- (BOOL)subscribeToControls{
    NSLog(@"Subscribing Controls");
    
    @try {
        [_controlsSubscriber unsubscribe];
        NSLog(@"Unsubscribed.");
    } @catch (NSException *exception) {
        NSLog(@"Could not unsubscribe.");
    }
    
    NSString * subscriber = @"/Double_1/controls";
    _controlsSubscriber = [_manager addSubscriber:subscriber responseTarget:self selector:@selector(ControlsMessageUpdate:) messageClass: [TwistMessage class]];
    
    _controlsSubscriber.throttleRate = 100;
    
    @try {
        [_controlsSubscriber subscribe];
        NSLog(@"Subscribed!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not subscribe.");
        return NO;
    }
}

/*
 Message Class: BoolMessage
 */
- (BOOL)subscribeToMotionDetection{
    NSLog(@"Subscribing MotionDetection");
    
    @try {
        [_motionDetectionSubscriber unsubscribe];
        NSLog(@"Unsubscribed.");
    } @catch (NSException *exception) {
        NSLog(@"Could not unsubscribe.");
    }
    
    NSString * subscriber = @"/Double_1/motionDetection";
    _motionDetectionSubscriber = [_manager addSubscriber:subscriber responseTarget:self selector:@selector(MotionDetectionMessageUpdate:) messageClass: [BoolMessage class]];
    
    _motionDetectionSubscriber.throttleRate = 100;
    
    @try {
        [_motionDetectionSubscriber subscribe];
        NSLog(@"Subscribed!");
        return YES;
    } @catch (NSException *exception) {
        NSLog(@"Could not subscribe.");
        return NO;
    }
}


/*------------------------------------------------------------------------*/
/*                               ROS DELEGATES                            */
/*------------------------------------------------------------------------*/

/*
 Delegate function.
 Message Class: FloatMessage
 */
- (void)poleHeightMessageUpdate:(FloatMessage *)message{
    NSLog(@"\n--------------------------------------------\npoleHeightMessageUpdate: %@\n--------------------------------------------\n", message.data);
    [self.delegate messageUpdate_poleHeightPercentage:message];
}

/*
 Delegate function.
 Message Class: IntMessage
 */
- (void)KickStandStateMessageUpdate:(IntMessage *)message{
    NSLog(@"\n--------------------------------------------\nKickStandStateMessageUpdate: %@\n--------------------------------------------\n", message.data);
    [self.delegate messageUpdate_kickStandState:message];
}

/*
 Delegate function.
 Message Class: TwistMessage
 */
- (void)ControlsMessageUpdate:(TwistMessage *)message{
    NSLog(@"\n--------------------------------------------\n"
             "ControlsMessageUpdate: \n"
          "Linear x: %@\n"
          "Linear y: %@\n"
          "Linear z: %@\n"
          "Angular x: %@\n"
          "Angular y: %@\n"
          "Angular z: %@"
          "\n--------------------------------------------\n", message.linear.x,message.linear.y,message.linear.z,message.angular.x,message.angular.y,message.angular.z);
    [self.delegate messageUpdate_Controls:message];
}

/*
 Delegate function.
 Message Class: BoolMessage
 */
- (void)MotionDetectionMessageUpdate:(BoolMessage *)message{
    NSLog(@"\n--------------------------------------------\nMotionDetectionMessageUpdate: %@\n--------------------------------------------\n", message.data);
    [self.delegate messageUpdate_MotionDetection:message];
}

/*------------------------------------------------------------------------*/
/*                                ROS CONTROL                             */
/*------------------------------------------------------------------------*/

- (BOOL)initialise
{
    NSLog(@"initializing init");
    if(![self initPublishStatus]){
        return NO;
    }
    if(![self initPublishPoleHeightPercentage]){
        return NO;
    }
    if(![self initPublishBatteryPercentage]){
        return NO;
    }
    if(![self initPublishSerial]){
        return NO;
    }
    if(![self initPublishKickStandState]){
        return NO;
    }
    if(![self initPublishBatteryIsFullyCharged]){
        return NO;
    }
    if(![self initPublishFirmwareVersion]){
        return NO;
    }
    if(![self initPublishLocation]){
        return NO;
    }
    if(![self initPublishMotionDetection]){
        return NO;
    }
    if(![self initPublishImage]){
        return NO;
    }
    if(![self subscribeToPoleHeightPercentage]){
        return NO;
    }
    if(![self subscribeToKickStandState]){
        return NO;
    }
    if(![self subscribeToControls]){
        return NO;
    }
    if(![self subscribeToMotionDetection]){
        return NO;
    }
    [self.delegate MainROSControlDidConnect];
    return YES;
}

- (void)cleanup{
    [_kickStandStatePublisher unadvertise];
    [_firmwareVersionPublisher unadvertise];
    [_batteryPercentagePublisher unadvertise];
    [_batteryIsFullyChargedPublisher unadvertise];
    [_serialPublisher unadvertise];
    [_poleHeightPercentagePublisher unadvertise];
    [_statusPublisher unadvertise];
    [_controlsPublisher unadvertise];
    [_locationPublisher unadvertise];
    [_motionDetectionPublisher unadvertise];
    [_imagePublisher unadvertise];
}

////////////////////////////////// FOR TESTING ////////////////////////////////

- (void)sendTestObjects{
    for(int i = 0 ; i < 5 ; i++){
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(i * 2 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
            [self publishStatus:YES];
            [self publishBatteryPercentage:arc4random_uniform(100)];
            [self publishBatteryIsFullyCharged:NO];
            [self publishPoleHeight:arc4random_uniform(100)];
            [self publishSerial:@"10.203 - 2038"];
            [self publishMotionDetection:YES];
            [self publishFirmwareVersion:@"firmware blabla"];
            [self publishKickStandState:arc4random_uniform(4)];
        });
    }
}

////////////////////////////////// FOR TESTING ////////////////////////////////

@end
