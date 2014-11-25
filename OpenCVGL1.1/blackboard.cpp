//
//  blackboard.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/18.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//

#include "blackboard.h"

#include <cv.hpp>
#include <core.hpp>
#include <highgui.hpp>
#include <objdetect.hpp>
#include <imgproc.hpp>

#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

void detectLine(Mat &input,Mat &processed);

int main (int argc, char **argv)
{
    
//    
//    // 1. prepare VideoCapture Object
//    VideoCapture cap;
//    Mat frame_cap,frame_pro;
//    std::string input_index;
//    if(argc >= 2){ // capture from video file
//        input_index = argv[1];
//        cap.open(input_index);
//    }else{ // capture from camera
//        cap.open(0);
//    }
//    
//    cvNamedWindow ("Capture", CV_WINDOW_AUTOSIZE);
//    cvNamedWindow ("Project", CV_WINDOW_AUTOSIZE);
//    
//    if(!cap.isOpened()){
//        printf("no input video\n");
//        return 0;
//    }
//    else
//    {
//        bool loop_flag = true;
//        while(loop_flag){
//            cap >> frame_cap;
//            if(frame_cap.empty()){
//                break;
//            }
//            detectLine(frame_cap, frame_cap);
//            imshow("Capture",frame_cap);
//            imshow("Project",frame_cap);
//            
//            // 5. process according to input key
//            int k = cvWaitKey(33);
//            switch(k){
//                case 'q':
//                case 'Q':
//                    loop_flag = false;
//                    break;
//            }
//        }
//    }
    
    
    cv::Mat input,processed;
    const char *input_file;
    const char* preset_file = "/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/figures3.jpg";
    
    if(argc==2){
        input_file=argv[1];
    }else{
        input_file=preset_file;
    }
    
    //2.read an image from the specified file
    input=cv::imread(input_file,1);
    if(input.empty()){
        fprintf(stderr,"cannot open %s\n",input_file);
        exit(0);
    }
    
    detectLine(input, processed);
    
    cv::namedWindow("processed image",1);

    cv::imshow("processed image",processed);

    cv::waitKey(0);

    
    
    
    return 0;
}

void detectLine(Mat &input,Mat &processed){
    
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    
    cvtColor(input,canny_output,CV_BGR2GRAY);  //色空間の変換(グレイスケール化)
    blur(canny_output, canny_output, Size(5,5) );//平滑化して誤差を減らす
    threshold(canny_output, canny_output, 115, 255, CV_THRESH_BINARY);//二値化
    Canny (canny_output, canny_output, 50,150, 3);//canny法によるエッジ検出
    
   
//    
//    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
//
//    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//    
//    for( int i=0; i < (int)contours.size(); i++ ) {
//        for(int j=0; j<(int)contours[i].size(); j++){
//            circle(drawing, contours[i][j], 10, Scalar(100,200,0),1);
//        }
//    }
//
//    for( int i = 0; i< contours.size(); i++ )
//    {
//        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//    }
//    
    
    // (5)検出結果表示用のウィンドウを確保し表示する
    
    canny_output.copyTo(processed);
    

    
}





