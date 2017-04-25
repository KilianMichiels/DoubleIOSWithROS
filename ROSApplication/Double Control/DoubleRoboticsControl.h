//
//  DoubleRoboticsControl.h
//  Facial Recognition
//
//  Created by Kilian Michiels on 5/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "MainROSDoubleControl.h"
#import <DoubleControlSDK/DoubleControlSDK.h>

@interface DoubleRoboticsControl : NSObject <DRDoubleDelegate>{
    int kickStandState;
    NSString * firmwareVersion;
    float batteryPercentage;
    BOOL batteryIsFullyCharged;
    NSString * serial;
    float poleHeightPercentage;
    BOOL status;
}

@property int kickStandState;
@property NSString * firmwareVersion;
@property float batteryPercentage;
@property BOOL batteryIsFullyCharged;
@property NSString * serial;
@property float poleHeightPercentage;
@property BOOL  status;

@property MainROSDoubleControl * MainROSController;
@property DRDouble * doubleController;

- (void) initializeDRConnection;

- (void) poleUp;

- (void) poleDown;

- (void)poleStop;

- (void)kickstandsRetract;

- (void)kickstandsDeploy;

- (void)moveForward;

- (void)moveBackward;

- (void)moveLeft;

- (void)moveRight;

- (void)stopMovement;

// This method is used to receive the data which we get using post method.
- (void)connection:(NSURLConnection *)connection didReceiveData:(NSData*)data;

// This method receives the error report in case of connection is not made to server.
- (void)connection:(NSURLConnection *)connection didFailWithError:(NSError *)error;

// This method is used to process the data after connection has made successfully.
- (void)connectionDidFinishLoading:(NSURLConnection *)connection;

#pragma mark - DRDoubleDelegate

- (void)doubleDidConnect:(DRDouble *)theDouble;

- (void)doubleDidDisconnect:(DRDouble *)theDouble;

- (void)doubleStatusDidUpdate:(DRDouble *)theDouble;

- (void)doubleDriveShouldUpdate:(DRDouble *)theDouble message:(TwistMessage*)message;

- (void)updateParameters;

- (void) messageUpdate_poleHeightPercentage :(FloatMessage*)   message;
- (void) messageUpdate_kickStandState       :(IntMessage*)   message;
- (void) messageUpdate_Controls             :(NavSatFixMessage*)   message;
- (void) messageUpdate_MotionDetection      :(BoolMessage*)   message;
@end
