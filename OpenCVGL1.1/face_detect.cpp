//
//  face_detect.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/10.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//

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
    
    cv::Mat frame,frame_eye,frame_mouth;
    cv::VideoCapture cap;
    cap.open(0);
    cap >> frame;
    
    cv::namedWindow("whole",1);
    //cv::namedWindow("mouth",1);
    //cv::namedWindow("eye",1);
    
    
    double scale = 4.0;
    cv::Mat gray, smallImg(cv::saturate_cast<int>(frame.rows/scale),cv::saturate_cast<int>(frame.cols/scale),CV_8UC1);
    
    for(;;){
        cap >> frame;
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        cv::resize(gray, smallImg, smallImg.size(),0,0,cv::INTER_LINEAR);
        cv::equalizeHist(smallImg, smallImg);
        
        std::vector<cv::Rect> faces;
        cascade.detectMultiScale(smallImg, faces,1.1,3,CV_HAAR_SCALE_IMAGE,cv::Size(20,20));
        
        //複数顔を検出した場合はそれっぽいやつ一つだけを使う
        int i,face_id;
        face_id=-1;
        for (i=0; i<faces.size(); i++) {
            if(faces[i].width>70){
                face_id=i;
                break;
            }
        }
        if(face_id != -1){
            
            //目元の辺りに線を引く
            cv::Point center;
            int radius;
            center.x = cv::saturate_cast<int>((faces[face_id].x + faces[face_id].width*0.5)*scale);
            center.y = cv::saturate_cast<int>((faces[face_id].y + faces[face_id].height*0.5)*scale);
            radius = cv::saturate_cast<int>((faces[face_id].width + faces[face_id].height)*0.25*scale);
            //cv::rectangle(frame,cv::Point(center.x-radius,center.y-radius),cv::Point(center.x+radius,center.y+radius), cv::Scalar(0,0,255),2,8,0);
            cv::Mat roi(frame, cv::Rect(cv::Point(center.x-radius*0.6,center.y-radius*0.4),cv::Point(center.x-radius*0.15,center.y-radius*0.1)));
            roi.convertTo(frame_eye,frame_eye.type());
            cv::Mat roi2(frame, cv::Rect(cv::Point(center.x-radius*0.7,center.y+radius*0.27),cv::Point(center.x+radius*0.5,center.y+radius*0.9)));
            roi2.convertTo(frame_mouth,frame_mouth.type());
            
            
            cv::Mat roi3(frame, cv::Rect(cv::Point(center.x-radius*0.18,center.y-radius*0.8),cv::Point(center.x-radius*0.18+frame_eye.size().width,center.y-radius*0.8+frame_eye.size().height)));
            frame_eye.copyTo(roi3);
            
            //cv::GaussianBlur(frame, frame,cv::Size(9,9),15,15);

            
        }else{
            frame_eye.create(cv::Size(10,10),CV_8UC1);
            frame_mouth.create(cv::Size(10,10),CV_8UC1);
        }
        
        cv::imshow("whole", frame);
        //cv::imshow("mouth", frame_mouth);
        //cv::imshow("eye", frame_eye);
        
        int key = cv::waitKey(10);
        if(key == 'q' || key == 'Q')
            break;
        
        
    }
    
    return 0;
    
}
