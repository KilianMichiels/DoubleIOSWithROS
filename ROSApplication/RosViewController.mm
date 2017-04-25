//
//  RosViewController.m
//  SocketRocket
//
//  Created by Kilian Michiels on 10/04/17.
//
//

#import "RosViewController.h"
#import "Constants.h"

NSMutableString *ttp;

@interface RosViewController () <DRDoubleDelegate>
@property (strong, nonatomic) NSTimer * parameterUpdateTimer;
@property (strong, nonatomic) NSTimer * updateTimer;
@property (strong, nonatomic) NSString * base64;
@property BOOL alreadyChangingHeight;
@end

@implementation RosViewController

//-------------------------------------------------------------------//
//                               GENERAL                             //
//-------------------------------------------------------------------//

- (void)viewDidLoad {
    [super viewDidLoad];
    
    ///////////////////////////////////////////////////////////////////
    
    [_disconnectButton setAlpha:0.4f];
    [_disconnectButton setEnabled:NO];
    
    ///////////////////////////////////////////////////////////////////
    
    _manager = [[RBManager defaultManager] init];
    ttp = [NSMutableString new];
    _manager.delegate = self;
    
    ///////////////////////////////////////////////////////////////////
    
    _ROSMASTERIP.delegate = self;
    
    ///////////////////////////////////////////////////////////////////
    
    locationManager = [[CLLocationManager alloc] init];
    locationManager.delegate = self;
    locationManager.distanceFilter = kCLDistanceFilterNone;
    locationManager.desiredAccuracy = kCLLocationAccuracyBest;
    
    if ([[[UIDevice currentDevice] systemVersion] floatValue] >= 8.0)
        [self->locationManager requestWhenInUseAuthorization];
    
    [locationManager startUpdatingLocation];
    
    ///////////////////////////////////////////////////////////////////
    if([_connectWithDouble isOn]){
        [DRDouble sharedDouble].delegate = self;
        _alreadyChangingHeight = NO;
        _connectWithDoubleLabel.text = @"With Double";
    }
    else{
        _connectWithDoubleLabel.text = @"Without Double";
    }
    ///////////////////////////////////////////////////////////////////
    
    _MainROSController = [[MainROSDoubleControl alloc]init];
    _MainROSController.delegate = self;
    
    ///////////////////////////////////////////////////////////////////
    
    _doubleSpeed = 0.1;
    
    ///////////////////////////////////////////////////////////////////
    
    _logView.text = @"------------- SESSION Start -------------";
    _logView.text = [_logView.text stringByAppendingString:@"\n\n"];
    _logView.text = [_logView.text stringByAppendingString:@"Good luck... :)"];
    _logView.text = [_logView.text stringByAppendingString:@"\n\n"];
    _logView.text = [_logView.text stringByAppendingString:@"To start: Press connect :)\n"];
}

//-------------------------------------------------------------------//
//                          ROS MAIN CONTROL                         //
//-------------------------------------------------------------------//

- (void)MainROSControlDidConnect{
    NSLog(@"MainROSControlDidConnect -- Connected!");
    
    if(TESTING_VARIABLES){
        
        [self updateLog:@"\nMainROSControlDidConnect -- Connected!"];
        
    }
    
    if([_connectWithDouble isOn]){
        _parameterUpdateTimer = [NSTimer scheduledTimerWithTimeInterval:PARAMETER_UPDATE_SPEED target:self selector:@selector(UpdateParameters) userInfo:nil repeats:YES];
    }
    
    _camera = [[CvVideoCamera alloc] initWithParentView: _imageView];
    _camera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionFront;
    _camera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    _camera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortraitUpsideDown;
    _camera.defaultFPS = 30;
    _camera.grayscaleMode = NO;
    _camera.delegate = self;
    
    [_camera start];
    
    _updateTimer = [NSTimer scheduledTimerWithTimeInterval:CAMERA_UPDATE_SPEED target:self selector:@selector(sendImage) userInfo:nil repeats:YES];
    
    [_connectButton setEnabled:NO];
    [_connectButton setAlpha:0.4f];
    [_disconnectButton setEnabled:YES];
    [_disconnectButton setAlpha:1.0f];
}


//-------------------------------------------------------------------//
//                     ROS MAIN CONTROL DELEGATES                    //
//-------------------------------------------------------------------//

- (void)MainROSControlDidNotConnectWithTimeout{
    [_updateTimer invalidate];
    [_parameterUpdateTimer invalidate];
    
    NSLog(@"MainROSControlDidConnect -- Timeout on connection...");
    
    if(TESTING_VARIABLES){
        [self updateLog:@"\nMainROSControlDidConnect -- Timeout on connection..."];
    }
}

- (void)MainROSControlDidCloseWithCode:(NSInteger)code reason:(NSString *)reason wasClean:(BOOL)wasClean{
    
    [_updateTimer invalidate];
    [_parameterUpdateTimer invalidate];
    
    if(wasClean){
        [self updateLog:[NSString stringWithFormat:@"\nClosed connection (CLEAN) -- code: %ld -- reason: %@", (long)code, reason]];
        NSLog(@"\nClosed connection (CLEAN) -- code: %ld -- reason: %@", (long)code, reason);
        
    }
    else{
        [self updateLog:[NSString stringWithFormat:@"\nClosed connection (UNCLEAN) -- code: %ld -- reason: %@", (long)code, reason]];
        NSLog(@"\nClosed connection (UNCLEAN) -- code: %ld -- reason: %@", (long)code, reason);
        
    }
    
    [_connectButton setAlpha:1.0f];
    [_connectButton setEnabled:YES];
    
    [_disconnectButton setAlpha:0.4f];
    [_disconnectButton setEnabled:NO];
    
}

- (void)MainRosControlDidFailWithError:(NSError *)error{
    
    [_updateTimer invalidate];
    [_parameterUpdateTimer invalidate];
    
    [self updateLog:[NSString stringWithFormat:@"\nFailed -- error: %@",error]];
    NSLog(@"\nFailed -- error: %@", error);
    
    [_connectButton setAlpha:1.0f];
    [_connectButton setEnabled:YES];
    
    [_disconnectButton setAlpha:0.4f];
    [_disconnectButton setEnabled:NO];
    
}


//-------------------------------------------------------------------//
//                            UI: BUTTONS                            //
//-------------------------------------------------------------------//

- (IBAction)ROSMASTEREditingDidBegin:(id)sender {
    if([_ROSMASTERIP.text  isEqual: @""]){
        _ROSMASTERIP.text = @"ws://";
    }
}

- (IBAction)connectButtonPushed:(id)sender {
    
    [self onConnectButton];
    
}

- (IBAction)disconnectButtonPushed:(id)sender {
    
    [self onDisconnectButton];
    
}

-(void)onConnectButton{
    
    if([_IP_choice selectedSegmentIndex] == 0){
        [_MainROSController initialiseMainROSDoubleControlManually:_ROSMASTERIP.text];
    }
    else if ([_IP_choice selectedSegmentIndex] == 1){
        [_MainROSController initialiseMainROSDoubleControl:[_IP_choice selectedSegmentIndex]];
        _ROSMASTERIP.placeholder = IP_ROS_ROBOT_Home;
    }
    else if ([_IP_choice selectedSegmentIndex] == 2){
        [_MainROSController initialiseMainROSDoubleControl:[_IP_choice selectedSegmentIndex]];
        _ROSMASTERIP.placeholder = IP_ROS_ROBOT_iGent;
    }
    else{
        [_MainROSController initialiseMainROSDoubleControl:[_IP_choice selectedSegmentIndex]];
        _ROSMASTERIP.placeholder = IP_ROS_ROBOT_Kot;
    }
}

-(void)onDisconnectButton{
    [_updateTimer invalidate];
    [_parameterUpdateTimer invalidate];
    [_camera stop];
    UIImage * myImage = [UIImage imageNamed: @"Giraffe.jpg"];
    _imageView.image = myImage;
    [_MainROSController disconnectMainROSDoubleControl];
    
}

- (IBAction)clearLogPressed:(id)sender {
    _logView.text = @"";
}

- (IBAction)PoleUpPush:(id)sender {
    if([_connectWithDouble isOn]){
        [[DRDouble sharedDouble] poleUp];
        [[DRDouble sharedDouble] requestStatusUpdate];
        NSLog(@"%f",[[DRDouble sharedDouble] poleHeightPercent]);
    }
}

- (IBAction)PoleDownPush:(id)sender {
    if([_connectWithDouble isOn]){
        [[DRDouble sharedDouble] poleDown];
        [[DRDouble sharedDouble] requestStatusUpdate];
        NSLog(@"%f",[[DRDouble sharedDouble] poleHeightPercent]);
    }
    
}

- (IBAction)EndPoleMove:(id)sender {
    if([_connectWithDouble isOn]){
        [[DRDouble sharedDouble] poleStop];
    }
}

- (IBAction)KickStandSwitchPushed:(id)sender {
    if([_connectWithDouble isOn]){
        if([[DRDouble sharedDouble] kickstandState] == 1){
            [[DRDouble sharedDouble] retractKickstands];
        }
        else{
            [[DRDouble sharedDouble] deployKickstands];
        }
    }
}

- (IBAction)speedSliderDragged:(id)sender {
    _doubleSpeed = 0.2 * [_speedSlider value];
}

- (IBAction)TestButtonPressed:(id)sender {
    [_MainROSController sendTestObjects];
}

- (IBAction)withDoubleValueChanged:(id)sender {
    if([_connectWithDouble isOn]){
        _connectWithDoubleLabel.text = @"With Double";
    }
    else{
        _connectWithDoubleLabel.text = @"Without Double";
    }
}


//-------------------------------------------------------------------//
//                          UI: LOG & MESSAGES                       //
//-------------------------------------------------------------------//

- (void)updateLog:(NSString *) text{
    dispatch_async(dispatch_get_main_queue(), ^{
        _logView.text = [_logView.text stringByAppendingString:text];
        [self scrollTextViewToBottom: _logView];
    });
}

- (void)updateMessageView:(NSString *) text{
    dispatch_async(dispatch_get_main_queue(), ^{
        _messageView.text = [_messageView.text stringByAppendingString:text];
        [self scrollTextViewToBottom: _messageView];
    });
}

-(void)scrollTextViewToBottom:(UITextView *)textView {
    if(textView.text.length > 0 ) {
        NSRange bottom = NSMakeRange(textView.text.length -1, 1);
        [textView scrollRangeToVisible:bottom];
    }
    
}

//-------------------------------------------------------------------//
//                         ROS: MESSAGE UPDATES                      //
//-------------------------------------------------------------------//

-(void)messageUpdate_kickStandState:(IntMessage *)message{
    if([_connectWithDouble isOn]){
        if([message.data intValue] == [[DRDouble sharedDouble] kickstandState]){
            //        [self updateLog:@"\nAlready in the right stand."];
        }
        else if([message.data intValue] == 1){
            NSLog(@"ROS -- Kickstand state was set to: %@", message.data);
            if(TESTING_VARIABLES){
                [self updateLog:[NSString stringWithFormat:@"\nKickstand state was set to: %@", message.data]];
            }
            [self updateLog:@"\nSetting to 1."];
            [[DRDouble sharedDouble] deployKickstands];
        }
        else if([message.data intValue] == 2){
            NSLog(@"ROS -- Kickstand state was set to: %@", message.data);
            if(TESTING_VARIABLES){
                [self updateLog:[NSString stringWithFormat:@"\nKickstand state was set to: %@", message.data]];
            }
            [self updateLog:@"\nSetting to 2."];
            [[DRDouble sharedDouble] retractKickstands];
        }
    }
}

-(void)messageUpdate_poleHeightPercentage:(FloatMessage *)message{
    NSLog(@"ROS -- Pole height was set to: %f", message.data.floatValue);
    
    if(TESTING_VARIABLES){
        [self updateLog:[NSString stringWithFormat:@"\nPole height was set to: %f", message.data.floatValue]];
    }
    
    if([_connectWithDouble isOn]){
        // Update current height.
        [[DRDouble sharedDouble]requestStatusUpdate];
        
        if(!_alreadyChangingHeight){
            _desiredPoleHeight = [message.data floatValue];
            _changePoleHeight = YES;
        }
    }
}

- (void)messageUpdate_Controls:(TwistMessage *)message{
    
    NSLog(@"ROS -- Controls updated");
    _controlMessage = message;
    
}

-(void)messageUpdate_MotionDetection:(BoolMessage *)message{
    
}

//-------------------------------------------------------------------//
//                               DOUBLE                              //
//-------------------------------------------------------------------//

- (void)doubleDriveShouldUpdate:(DRDouble *)theDouble{
    if([_connectWithDouble isOn]){
        _messageView.text = @"--------------------- ROBOT CONTROL ---------------------\n";
        if([_controlMessage.linear.x intValue] == 1){
            if([[DRDouble sharedDouble] kickstandState] == 1){
                [[DRDouble sharedDouble] retractKickstands];
            }
            [self updateMessageView:@"\nMoving forward..."];
            [[DRDouble sharedDouble] variableDrive:_doubleSpeed turn:0.0];
        }
        else if ([_controlMessage.linear.x intValue] == -1){
            if([[DRDouble sharedDouble] kickstandState] == 1){
                [[DRDouble sharedDouble] retractKickstands];
            }
            [self updateMessageView:@"\nMoving backward..."];
            [[DRDouble sharedDouble] variableDrive:-_doubleSpeed turn:0.0];
        }
        else if ([_controlMessage.angular.z intValue] == 1){
            if([[DRDouble sharedDouble] kickstandState] == 1){
                [[DRDouble sharedDouble] retractKickstands];
            }
            [self updateMessageView:@"\nMoving left..."];
            [[DRDouble sharedDouble] variableDrive:0.0 turn:-_doubleSpeed];
        }
        else if ([_controlMessage.angular.z intValue] == -1){
            if([[DRDouble sharedDouble] kickstandState] == 1){
                [[DRDouble sharedDouble] retractKickstands];
            }
            [self updateMessageView:@"\nMoving right..."];
            [[DRDouble sharedDouble] variableDrive:0.0 turn:_doubleSpeed];
            
        }
        else{
            [self updateMessageView:@"\nStand still..."];
            [[DRDouble sharedDouble] variableDrive:0.0 turn:0.0];
        }
    }
    
}

- (void)doubleDidConnect:(DRDouble *)theDouble{
    if([_connectWithDouble isOn]){
        _doubleIsConnected = YES;
    }
}

- (void)UpdateParameters{
    if([_connectWithDouble isOn]){
        [_MainROSController publishStatus:_doubleIsConnected];
        [_MainROSController publishBatteryPercentage:[[DRDouble sharedDouble] batteryPercent]];
        
        if([[DRDouble sharedDouble] batteryIsFullyCharged] == YES){
            // Update battery UI
            _batteryIsFullyChargedLabel.text = @"Fully charged: Yes";
            [_MainROSController publishBatteryIsFullyCharged:YES];
        }
        else{
            _batteryIsFullyChargedLabel.text = @"Fully charged: No";
            [_MainROSController publishBatteryIsFullyCharged:NO];
        }
        
        [_batteryPercentageProgressBar setProgress:[[DRDouble sharedDouble]batteryPercent]];
        [_MainROSController publishPoleHeight:[[DRDouble sharedDouble] poleHeightPercent]];
        _currentPoleHeight.text = [NSString stringWithFormat:@"Current Poleheight: %f",[[DRDouble sharedDouble] poleHeightPercent]];
        [_MainROSController publishSerial:[[DRDouble sharedDouble] serial]];
        [_MainROSController publishMotionDetection:[[DRDouble sharedDouble] batteryPercent]];
        [_MainROSController publishFirmwareVersion:[[DRDouble sharedDouble] firmwareVersion]];
        [_MainROSController publishKickStandState:[[DRDouble sharedDouble] kickstandState]];
    }
}

- (void)doubleStatusDidUpdate:(DRDouble *)theDouble{
    if([_connectWithDouble isOn]){
        _actualPoleHeight = [[DRDouble sharedDouble] poleHeightPercent];
        NSLog(@"Current Pole Height: %f", _actualPoleHeight);
        
        if(_desiredPoleHeight >= _actualPoleHeight - 0.005 && _desiredPoleHeight <= _actualPoleHeight + 0.005){
            NSLog(@"doubleStatusDidUpdate -- Pole Height in range --> _changePoleHeight = NO");
            _changePoleHeight = NO;
            _alreadyChangingHeight = NO;
            [[DRDouble sharedDouble] poleStop];
        }
        
        if(_changePoleHeight == YES){
            
            NSLog(@"doubleStatusDidUpdate -- Pole Height NOT in range");
            
            // Pole has to go up.
            if([[DRDouble sharedDouble] poleHeightPercent] < _desiredPoleHeight){
                
                [[DRDouble sharedDouble] poleUp];
                _alreadyChangingHeight = YES;
                
                // As long as the actual pole height < the desired height --> go up.
                [[DRDouble sharedDouble] requestStatusUpdate];
                
                // ...and send update of the current height:
                [_MainROSController publishPoleHeight:[[DRDouble sharedDouble] poleHeightPercent]];
                _currentPoleHeight.text = [NSString stringWithFormat:@"Current Poleheight: %f",[[DRDouble sharedDouble] poleHeightPercent]];
            }
            else{
                
                [[DRDouble sharedDouble] poleDown];
                _alreadyChangingHeight = YES;
                
                // As long as the actual pole height > the desired height --> go down.
                [[DRDouble sharedDouble] requestStatusUpdate];
                
                // ...and send update of the current height:
                [_MainROSController publishPoleHeight:[[DRDouble sharedDouble] poleHeightPercent]];
                _currentPoleHeight.text = [NSString stringWithFormat:@"Current Poleheight: %f",[[DRDouble sharedDouble] poleHeightPercent]];
            }
            
        }
    }
}

//-------------------------------------------------------------------//
//                             ROS: CAMERA                           //
//-------------------------------------------------------------------//

- (void)processImage:(cv::Mat &)image{
    
    NSCalendar *calendar = [NSCalendar currentCalendar];
    NSDateComponents *components = [calendar components:(NSCalendarUnitHour | NSCalendarUnitMinute | NSCalendarUnitSecond) fromDate:[NSDate date]];
    
    int hour = (int)[components hour];
    int minute = (int)[components minute];
    NSInteger second = [components second];
    
    const char* str = [[NSString stringWithFormat: @"%i:%i:%li", hour, minute, (long)second] cStringUsingEncoding: NSUTF8StringEncoding];
    
    cv::putText(image, str, cv::Point(20, 90), CV_FONT_HERSHEY_DUPLEX, 3, cv::Scalar(0, 0, 255));
}

- (void) setLabel:(cv::Mat&) im label:(const std::string)label point:(const cv::Point &)point
{
    int fontface = CV_FONT_HERSHEY_PLAIN;
    double scale = 1.5;
    int thickness = 1;
    int baseline = 0;
    
    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::rectangle(im, point + cv::Point(0, baseline), point + cv::Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(im, label, point, fontface, scale, cv::Scalar(0, 0, 255), thickness, 8);
}

- (void)sendImage{
    
    dispatch_queue_t backgroundQueue = dispatch_queue_create("com.KM.ROSApplication.imageSender.bgqueue", NULL);
    
    dispatch_async(backgroundQueue, ^(void) {
        
        UIGraphicsBeginImageContext(_cameraView.frame.size);
        
        [_cameraView.layer renderInContext:UIGraphicsGetCurrentContext()];
        UIImage *screenshot = UIGraphicsGetImageFromCurrentImageContext();
        
        CGRect cropRect = CGRectMake(0 ,0 ,_cameraView.frame.size.width,_cameraView.frame.size.height);
        CGImageRef imageRef = CGImageCreateWithImageInRect([screenshot CGImage], cropRect);
        CGImageRelease(imageRef);
        
        UIGraphicsEndImageContext();
        
        NSData * sendImage = UIImageJPEGRepresentation(screenshot, 1.0);
        
        _base64 = [sendImage base64EncodedStringWithOptions:kNilOptions];
        
        // DO something with screenshot.
        
        [_MainROSController publishImage:_base64];
        
    });
}

//-------------------------------------------------------------------//
//                            ROS: LOCATION                          //
//-------------------------------------------------------------------//

- (void)locationManager:(CLLocationManager *)manager didUpdateToLocation:(CLLocation *)newLocation fromLocation:(CLLocation *)oldLocation {
    NSLog(@"OldLocation %f %f", oldLocation.coordinate.latitude, oldLocation.coordinate.longitude);
    NSLog(@"NewLocation %f %f", newLocation.coordinate.latitude, newLocation.coordinate.longitude);
    NSLog(@"Altitude %f", newLocation.altitude);
    
    [_MainROSController publishLocation:newLocation];
}


@end
