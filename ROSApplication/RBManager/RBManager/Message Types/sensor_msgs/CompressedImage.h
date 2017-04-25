//
//  CompressedImage.h
//  openCVTestApplication
//
//  Created by Kilian Michiels on 19/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//

#import "RBMessage.h"
@class HeaderMessage;

/*-------------------------------------------------------------*/
/* String format    --->    Specifies the format of the data   */
/*        --> Acceptable values: jpeg, png                     */
/*                                                             */
/* uint8[] data     --->    Compressed image buffer            */
/*-------------------------------------------------------------*/

@interface CompressedImage : RBMessage {
    HeaderMessage * header;
    
    NSString * format;
    NSString * data;
}

@property (nonatomic, strong) HeaderMessage * header;
@property (nonatomic, strong) NSString * format;
@property (nonatomic, strong) NSString * data;

@end
