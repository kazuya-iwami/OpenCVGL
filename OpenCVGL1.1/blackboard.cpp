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

void detectLine(cv::Mat &input,cv::Mat &processed);

int main (int argc, char **argv)
{
    
    
    // 1. prepare VideoCapture Object
    cv::VideoCapture cap;
    cv::Mat frame;
    std::string input_index;
    if(argc >= 2){ // capture from video file
        input_index = argv[1];
        cap.open(input_index);
    }else{ // capture from camera
        cap.open(0);
    }
    
    cvNamedWindow ("Capture", CV_WINDOW_AUTOSIZE);
    
    if(!cap.isOpened()){
        printf("no input video\n");
        return 0;
    }
    else
    {
        bool loop_flag = true;
        while(loop_flag){
            cap >> frame;               
            if(frame.empty()){
                break;
            }
            detectLine(frame, frame);
            cv::imshow("Capture",frame);
            
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

void detectLine(cv::Mat &input,cv::Mat &processed){
    
    int i;
    IplImage  *src_img_prob = 0, *src_img_gray = 0;
    CvMemStorage *storage;
    CvSeq *lines = 0;
    CvPoint *point;
    
    // (1)画像の読み込み
    IplImage tmp = input;
    src_img_prob = &tmp;
    src_img_gray =cvCreateImage( cvGetSize(src_img_prob),IPL_DEPTH_8U,1); //1チャネル８ビットのIplImageを作成
    cvCvtColor(src_img_prob, src_img_gray, CV_BGR2GRAY);  //色空間の変換(グレイスケール化)
    
    // (2)ハフ変換のための前処理
    cvCanny (src_img_gray, src_img_gray, 50, 200, 3);
    storage = cvCreateMemStorage (0);
    
    // (4)確率的ハフ変換による線分の検出と検出した線分の描画
    lines = 0;
    lines = cvHoughLines2 (src_img_gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 50, 10);
    for (i = 0; i < lines->total; i++) {
        point = (CvPoint *) cvGetSeqElem (lines, i);
        cvLine (src_img_prob, point[0], point[1], CV_RGB (255, 0, 0), 3, 8, 0);
    }
    
    // (5)検出結果表示用のウィンドウを確保し表示する
    processed = cv::cvarrToMat(src_img_prob);
    
    //cvReleaseImage (&src_img_prob);
    //cvReleaseImage (&src_img_gray);
    cvReleaseMemStorage (&storage);

}



