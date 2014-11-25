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
    
    
    // 1. prepare VideoCapture Object
    VideoCapture cap;
    Mat frame_cap,frame_pro;
    std::string input_index;
    if(argc >= 2){ // capture from video file
        input_index = argv[1];
        cap.open(input_index);
    }else{ // capture from camera
        cap.open(0);
    }
    
    cvNamedWindow ("Capture", CV_WINDOW_AUTOSIZE);
    cvNamedWindow ("Project", CV_WINDOW_AUTOSIZE);
    
    if(!cap.isOpened()){
        printf("no input video\n");
        return 0;
    }
    else
    {
        bool loop_flag = true;
        while(loop_flag){
            cap >> frame_cap;
            if(frame_cap.empty()){
                break;
            }
            detectLine(frame_cap, frame_cap);
            imshow("Capture",frame_cap);
            imshow("Project",frame_cap);
            
            // 5. process according to input key
            int k = cvWaitKey(33);
            switch(k){
                case 'q':
                case 'Q':
                    loop_flag = false;
                    break;
            }
        }
    }
    
    
    
    return 0;
}

void detectLine(Mat &input,Mat &processed){
    
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    
    cvtColor(input,canny_output,CV_BGR2GRAY);  //色空間の変換(グレイスケール化)
    blur(canny_output, canny_output, Size(5,5) );//平滑化して誤差を減らす
    Canny (canny_output, canny_output, 50,150, 3);//canny法によるエッジ検出
    //cvThreshold(img_gray, img_gray, 0, 255, CV_THRESH_BINARY);//二値化
    
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    vector<vector<Point> >::iterator it;
    for( it = contours.begin(); it != contours.end(); it++ )
        cout << *it << endl;
    
    

    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    
    for( int i=0; i < (int)contours.size(); i++ ) {
        for(int j=0; j<(int)contours[i].size(); j++){
            circle(drawing, contours[i][j], 10, Scalar(100,200,0),1);
        }
    }

    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }
    
    
    // (5)検出結果表示用のウィンドウを確保し表示する
    drawing.copyTo(processed);
    

    
}





