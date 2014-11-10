//
//  sample1-3.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/06.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//

#include <core.hpp>
#include <highgui.hpp>
#include <stdio.h>

#define IN_VIDEO_FILE "/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/sample_video_input.avi"
#define OUT_VIDEO_FILE "/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/sample_video_output.avi"

int main(int argc, char *argv[]){
    
    // 1. prepare VideoCapture Object
    cv::VideoCapture cap;                           // ã‚­ãƒ£ãƒ—ãƒãƒ£ç”¨ã®ã‚ªãƒ–ã‚¸ã‚§ã‚¯ãƒˆã‚’ç”¨æ„ã™ã‚‹
    std::string input_index;
    if(argc >= 2){ // capture from video file
        input_index = argv[1];
        cap.open(input_index);                         // ãƒ•ã‚¡ã‚¤ãƒ«ã‹ã‚‰ã®ã‚­ãƒ£ãƒ—ãƒãƒ£ã‚’é–‹å§‹ã™ã‚‹
    }else{ // capture from camera
        cap.open(0);                           // ã‚«ãƒ¡ãƒ©ã‹ã‚‰ã®ã‚­ãƒ£ãƒ—ãƒãƒ£ã‚’é–‹å§‹ã™ã‚‹
    }
    
    // 2. prepare VideoWriter Object
    cv::Mat frame, copy_frame;
    int rec_mode= 0;
    
    cv::namedWindow("video", 1);
    cv::VideoWriter output_video;                           // éŒ²ç”»ç”¨ã®ã‚ªãƒ–ã‚¸ã‚§ã‚¯ãƒˆã‚’ç”¨æ„ã™ã‚‹
    output_video.open(OUT_VIDEO_FILE, CV_FOURCC('W', 'R', 'L', 'E'), 30,cv::Size(1280,720));                           // å‹•ç”»ã®ä¿å­˜ã®ãŸã‚ã®åˆæœŸåŒ–ã‚’è¡Œãªã†
    /* using "MJPG" as the video codec */
    
    if(!cap.isOpened() || !output_video.isOpened()){
        printf("no input video\n");
        return 0;
    }
    else
    {
        bool loop_flag = true;
        while(loop_flag){
            
            // 3. capture frame from VideoCapture
            cap >> frame;                           // ã‚­ãƒ£ãƒ—ãƒãƒ£ã‚’è¡Œãªã†
            if(frame.empty()){
                break;
            }
            
            // 4. save frame
            if(rec_mode){
                output_video << frame;                           // éŒ²ç”»ãƒ¢ãƒ¼ãƒ‰ã§ã‚ã‚Œã°ã‚­ãƒ£ãƒ—ãƒãƒ£ã—ãŸãƒ•ãƒ¬ãƒ¼ãƒ ã‚’ä¿å­˜ã™ã‚‹
                frame.copyTo(copy_frame);                           //
                cv::Size s = frame.size();                           //
                cv::rectangle(copy_frame, cv::Point(0,0), cv::Point(s.width-1,s.height-1), cv::Scalar(0,0,255),4,8,0);                           //
                cv::imshow("video",copy_frame);
            }
            else{
                cv::imshow("video",frame);
            }
            
            // 5. process according to input key
            int k = cvWaitKey(33);
            switch(k){
                case 'q':
                case 'Q':
                    loop_flag = false;
                    break;
                case 'r':
                    if(rec_mode ==0){
                        rec_mode = 1;
                    }else{
                        rec_mode = 0;
                    }
                    break;
            }
        }
    }
    return 0;
}