//
//  DoubleRoboticsControl.m
//  Facial Recognition
//
//  Created by Kilian Michiels on 5/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//

#import "DoubleRoboticsControl.h"

@implementation DoubleRoboticsControl

@synthesize poleHeightPercentage, kickStandState, batteryPercentage, batteryIsFullyCharged, firmwareVersion, serial, status;

- (void) initializeDRConnection{
    _doubleController = [[DRDouble sharedDouble] init];
    _doubleController.delegate = self;
}

- (void) poleUp{
    [_doubleController poleUp];
    NSLog(@"Pole up action triggered");
}

- (void) poleDown{
    [_doubleController poleDown];
    NSLog(@"Pole down action triggered");
}

- (void)poleStop{
    [_doubleController poleStop];
    NSLog(@"Pole stop action triggered");
}

- (void)kickstandsRetract{
    [_doubleController retractKickstands];
    NSLog(@"Retracting kickstand");
}

- (void)kickstandsDeploy{
    [_doubleController deployKickstands];
    NSLog(@"Deploying kickstand");
}

- (void)doubleDriveShouldUpdate:(DRDouble *)theDouble{
    [self moveForward];
}

- (void)moveForward{
    [_doubleController variableDrive:1.0 turn:0.0];
//    [_doubleController drive:kDRDriveDirectionForward turn:0.0];
    NSLog(@"DRDouble -- Moving forward");
}

- (void)moveBackward{
    [_doubleController variableDrive:-1.0 turn:0.0];
//    [_doubleController drive:kDRDriveDirectionBackward turn:0.0];
    NSLog(@"DRDouble -- Moving backward");
}

- (void)moveLeft{
    [_doubleController variableDrive:0.0 turn:-1.0];
//    [_doubleController drive:0.0 turn:-1.0];
    NSLog(@"DRDouble -- Moving left");
}

- (void)moveRight{
    [_doubleController variableDrive:0.0 turn:1.0];
//    [_doubleController drive:0.0 turn:1.0];
    NSLog(@"DRDouble -- Moving right");
}

- (void)stopMovement{
    [_doubleController variableDrive:0.0 turn:0.0];
//    [_doubleController drive:0.0 turn:0.0];
    NSLog(@"DRDouble -- Stand still");
}

// This method is used to receive the data which we get using post method.
- (void)connection:(NSURLConnection *)connection didReceiveData:(NSData*)data{
    NSLog(@"%@", [NSString stringWithFormat: @"Received data: %@",connection.description]);
}

// This method receives the error report in case of connection is not made to server.
- (void)connection:(NSURLConnection *)connection didFailWithError:(NSError *)error{
    NSLog(@"%@", [NSString stringWithFormat: @"Connection failed: %@",connection.description]);
}

// This method is used to process the data after connection has made successfully.
- (void)connectionDidFinishLoading:(NSURLConnection *)connection{
    NSLog(@"%@", [NSString stringWithFormat: @"Finished Loading: %@",connection.description]);
}

#pragma mark - DRDoubleDelegate

- (void)doubleDidConnect:(DRDouble *)theDouble {
    NSLog(@"Double is Connected");
    status = YES;
}

- (void)doubleDidDisconnect:(DRDouble *)theDouble {
    NSLog(@"Double is Not Connected");
    status = NO;
}

- (void)doubleStatusDidUpdate:(DRDouble *)theDouble {
    NSLog(@"\npoleHeightPercent: %@",[NSString stringWithFormat:@"%d", _doubleController.kickstandState]);
    NSLog(@"\nBattery Percentage: %@", [NSString stringWithFormat:@"%f", _doubleController.batteryPercent]);
    NSLog(@"\nFirmware Version: %@", _doubleController.firmwareVersion);
    NSLog(@"\nSerial Number: %@", _doubleController.serial);
    [self updateParameters];
}

- (void)updateParameters{
    self.poleHeightPercentage = _doubleController.poleHeightPercent;
    self.batteryPercentage = _doubleController.batteryPercent;
    self.firmwareVersion = _doubleController.firmwareVersion;
    self.serial = _doubleController.serial;
    self.batteryIsFullyCharged = _doubleController.batteryIsFullyCharged;
    self.kickStandState = _doubleController.kickstandState;
    
    [_MainROSController publishStatus:self.status];
    [_MainROSController publishPoleHeight:self.poleHeightPercentage];
    [_MainROSController publishKickStandState:self.kickStandState];
    [_MainROSController publishFirmwareVersion:self.firmwareVersion];
    [_MainROSController publishSerial:self.serial];
    [_MainROSController publishBatteryPercentage:self.batteryPercentage];
    [_MainROSController publishBatteryIsFullyCharged:self.batteryIsFullyCharged];
}

@end
