//
//  CompressedImage.m
//  openCVTestApplication
//
//  Created by Kilian Michiels on 19/04/17.
//  Copyright © 2017 Kilian Michiels. All rights reserved.
//

#import "CompressedImage.h"
#import "HeaderMessage.h"

@implementation CompressedImage
@synthesize header, format, data;

-(void)setDefaults {
    self.header = [[HeaderMessage alloc] init];
}

@end
