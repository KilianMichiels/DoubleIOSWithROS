//
//  RosViewController.h
//  SocketRocket
//
//  Created by Kilian Michiels on 10/04/17.
//
//

#import <UIKit/UIKit.h>
#import "RBManager.h"
#import "RBMessage.h"
#import "PoseMessage.h"
#import "StringMessage.h"
#import "MainROSDoubleControl.h"
#import <DoubleControlSDK/DoubleControlSDK.h>
#import <CoreLocation/CoreLocation.h>
#import <opencv2/videoio/cap_ios.h>

@interface RosViewController : UIViewController <UITextFieldDelegate ,RBManagerDelegate, CLLocationManagerDelegate, MainROSControlDelegate, CvVideoCameraDelegate>{
    
    IBOutlet UITextView *logView;
    CLLocationManager *locationManager;
    
}

// General:
@property MainROSDoubleControl * MainROSController;
@property RBManager * manager;

// Camera with ROS:
@property CvVideoCamera *camera;
@property (strong, nonatomic) IBOutlet UIView *cameraView;
@property (strong, nonatomic) IBOutlet UIImageView *imageView;

// Controls, Connectivity & Log:
@property (strong, nonatomic) IBOutlet UIButton *connectButton;
@property (strong, nonatomic) IBOutlet UIButton *disconnectButton;
@property (strong, nonatomic) IBOutlet UITextView *logView;
@property (strong, nonatomic) IBOutlet UITextView *messageView;
@property (strong, nonatomic) IBOutlet UIButton *clearLogButton;
@property (strong, nonatomic) IBOutlet UISegmentedControl *IP_choice;
@property (strong, nonatomic) IBOutlet UITextField *ROSMASTERIP;
@property (strong, nonatomic) TwistMessage * controlMessage;
@property (strong, nonatomic) IBOutlet UISwitch *connectWithDouble;
@property (strong, nonatomic) IBOutlet UILabel *connectWithDoubleLabel;

// Double paramters:
@property (strong, nonatomic) IBOutlet UILabel *currentPoleHeight;
@property (strong, nonatomic) IBOutlet UIProgressView *batteryPercentageProgressBar;
@property (strong, nonatomic) IBOutlet UILabel *batteryIsFullyChargedLabel;
@property float actualPoleHeight;
@property float desiredPoleHeight;
@property BOOL changePoleHeight;
@property BOOL doubleIsConnected;
@property (weak, nonatomic) IBOutlet UISlider *speedSlider;
@property float doubleSpeed;

// For passing on data
@property (nonatomic, retain) UITextView *textView;

@end
