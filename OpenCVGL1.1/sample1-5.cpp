//
//  main.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/04.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//


//ここ参照
//http://nantekottai.com/2014/04/16/opencv-xcode5-homebrew/

//#include <iostream>
#include <core.hpp>
#include <highgui.hpp>
#include <objdetect.hpp>
#include <imgproc.hpp>
#include <stdio.h>

#define FLAG 0 //0:direct access 1:built-in function

int size_of_mosaic = 0;

int main (int argc, char **argv)
{
    std::string cascadeName = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
    cv::CascadeClassifier cascade;
    if(!cascade.load(cascadeName)){
        printf("ERROR: cascadefile見つからん！\n");
        return -1;
    }
    
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(0);
    cap >> frame;
    
    cv::namedWindow("result",1);
    cv::createTrackbar("size", "result", &size_of_mosaic, 30,0);
    
    double scale = 4.0;
    cv::Mat gray, smallImg(cv::saturate_cast<int>(frame.rows/scale),cv::saturate_cast<int>(frame.cols/scale),CV_8UC1);
    
    for(;;){
        cap >> frame;
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        cv::resize(gray, smallImg, smallImg.size(),0,0,cv::INTER_LINEAR);
        cv::equalizeHist(smallImg, smallImg);
        
        std::vector<cv::Rect> faces;
        cascade.detectMultiScale(smallImg, faces,1.1,3,CV_HAAR_SCALE_IMAGE,cv::Size(20,20));
        
        int i;
        for (i=0; i<faces.size(); i++) {
            
            #if FLAG //use built-in function

            cv::Point center;
            int radius;
            center.x = cv::saturate_cast<int>((faces[i].x + faces[i].width*0.5)*scale);
            center.y = cv::saturate_cast<int>((faces[i].y + faces[i].height*0.5)*scale);
            radius = cv::saturate_cast<int>((faces[i].width + faces[i].height)*0.25*scale);
            if(size_of_mosaic < 1)size_of_mosaic=1;
            cv::Rect roi_rect(center.x-radius,center.y-radius,radius*2,radius*2);
            cv::Mat mosaic = frame(roi_rect);
            cv::Mat tmp = frame(roi_rect);
            cv::resize(mosaic, tmp, cv::Size(radius/size_of_mosaic,radius/size_of_mosaic),0,0);
            cv::resize(tmp, mosaic, cv::Size(radius*2,radius*2),0,0,CV_INTER_NN);
            
            #else
            //目元の辺りに線を引く
            cv::Point center;
            int radius;
            double eye_ratio=0.2;//顔の中心から半径何割のところに線を引くか
            center.x = cv::saturate_cast<int>((faces[i].x + faces[i].width*0.5)*scale);
            center.y = cv::saturate_cast<int>((faces[i].y + faces[i].height*0.5)*scale);
             radius = cv::saturate_cast<int>((faces[i].width + faces[i].height)*0.25*scale);
            cv::line(frame,cv::Point(center.x-radius,center.y-radius*eye_ratio),cv::Point(center.x-radius+radius*2,center.y-radius*eye_ratio) ,cv::Scalar(0),80,8,0);
            
            #endif
        }
        
        cv::imshow("result", frame);
        
        int key = cv::waitKey(10);
        if(key == 'q' || key == 'Q')
            break;
        
        
    }
    
    return 0;
   
}
