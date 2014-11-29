//
//  blackboard.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/18.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//

#include "blackboard.h"

#include <stdio.h>

#include <cv.hpp>
#include <core.hpp>
#include <highgui.hpp>
#include <objdetect.hpp>
#include <imgproc.hpp>

#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

#define VERTEX_DISTANCE_COEFFICIENT 3.0//同一頂点とみなす距離の調整定数
#define APPROX_COEFFICIENT 0.013 //頂点認識の調整係数

enum Vertex_Kind {
    NONE_EDGE,
    A_EDGE,
    T_EDGE,
    Y_EDGE,
    L_EDGE
};

void detectLine(Mat &input,Mat &processed);
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour,Scalar color);

class TmpVertex {//０自身　1最短　２２番め
public:
    int num_of_sides;//3:３辺からなる　2:2辺からなる 1:エラー
    double dst[3];
    int k[3],l[3];
    TmpVertex(int k,int l);
};

TmpVertex::TmpVertex(int k_,int l_) {
    dst[0]=0.0;
    dst[1]=dst[2]=10000.0;
    k[0]=k_;
    l[0]=l_;
    num_of_sides=1;
}

class Contour {
public:
    Point point;
    bool used;
    int vertex_id;//所属している頂点の添字
    Contour(Point p,bool u);
    
};


Contour::Contour(Point p,bool u){
    point=p;
    used=u;
    
}

class Vertex { //頂点クラス
public:
    Point point;
    int num_of_sides;//辺の数
    Vertex_Kind kind;
    int attached_vertex[3];
    Vertex(Point p,int t);
};
Vertex::Vertex(Point p,int t){
    point=p;
    num_of_sides=t;
    kind=NONE_EDGE;
    attached_vertex[0]=attached_vertex[1]=attached_vertex[2]=-1;

}

double distance(Point p1,Point p2){
    double dst=sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    return dst;
}



int main (int argc, char **argv)
{
    
//    //動画からの読み込み
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
    
    //画像からの読み込み
    cv::Mat input,processed;
    const char *input_file;
    const char* preset_file = "/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/figures4.jpg";
    
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
    vector<vector<Contour>> approx_contours;//補正後の頂点
    
    cvtColor(input,canny_output,CV_BGR2GRAY);  //色空間の変換(グレイスケール化)
    blur(canny_output, canny_output, Size(3,3) );//平滑化して誤差を減らす
    threshold(canny_output, canny_output, 115, 255, CV_THRESH_BINARY);//二値化
    Canny (canny_output, canny_output, 50,150, 3);//canny法によるエッジ検出
    
    blur(canny_output, canny_output, Size(3,3) );//線が細い場合穴が空いて正しく物体認識出来ないのでこの２行入れる 線が細い場合危険
    threshold(canny_output, canny_output, 15, 255, CV_THRESH_BINARY);
    
    findContours(canny_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    rectangle(drawing, Point(0,0), canny_output.size(), Scalar(255,255,255),CV_FILLED);
    
    for( int i = 0; i< contours.size(); i+=2 ) //何故かほぼ同じ領域が２つ連続していたので省く
    {

        vector<cv::Point> approx;
        
        //Ramer–Douglas–Peucker algorithmによる多角形近似
        if (std::fabs(cv::contourArea(contours[i])) < 20 )//小さいobjectは排除
            continue;
        approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * APPROX_COEFFICIENT,true);
        vector<Contour> tmp_approx;
        for( int j= 0; j< approx.size(); j++ ){
            Contour tmp(approx[j],false);
            tmp_approx.push_back(tmp);
            //cout << i << " "<<j<<endl;
        }
        approx_contours.push_back(tmp_approx);
        
//            for(int j=0; j<(int)contours[i].size(); j++){//特徴点の表示
//                line(drawing, contours[i][j], contours[i][j], Scalar(200,250,250));
//            }
        
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
        setLabel(drawing, to_string(i), approx,color);
        
    }
    
    //VERTEX_DISTANCE決定
    //各頂点間の最小距離の平均を元に、どの距離までを同一頂点とみなすか決める。
    double ave_min_dst=0.0;
    
    for( int j = 0; j< approx_contours[0].size(); j++ ){
        double min_dst=1000.0;
        
        for( int k = 1; k< approx_contours.size(); k++ ){
            for( int l= 0; l< approx_contours[k].size(); l++ ){
                double dst = distance(approx_contours[0][j].point,approx_contours[k][l].point);
                if(min_dst > dst){
                    min_dst=dst;
                }
            }
        }
        ave_min_dst+=min_dst;
    }
    ave_min_dst=ave_min_dst/approx_contours[0].size()*VERTEX_DISTANCE_COEFFICIENT;
    cout << "ave_min_dst :"<<ave_min_dst<<endl;
    
    //頂点抽出
    vector<TmpVertex> tmp_vertexes;//一時的な頂点保存クラス
    vector<Vertex> vertexes;
    
    for( int i = 0; i< approx_contours.size(); i++ ){
        //cout << "approx_contours[" << i << "]" << endl;
        for( int j= 0; j< approx_contours[i].size(); j++ ){//ある頂点について
            if(approx_contours[i][j].used==true)continue;//すでに頂点に決まっているものは無視
            TmpVertex tmp(i,j);
            
            //cout << i <<" "<< j << endl;

            //circle(drawing, approx_contours[7][1].point, 3,Scalar(0,0,0),2);
            //circle(drawing, approx_contours[6][6].point, 3,Scalar(200,0,0),2);
            //cout << approx_contours[i][j].point << endl;
            for( int k = 1; k< approx_contours.size(); k++ ){
                if(k==i)continue;
                
                double min_dst=1000.0;
                int min_l=0;
                
                for( int l= 0; l< approx_contours[k].size(); l++ ){
                    if(approx_contours[k][l].used==true)continue;
                    double dst = distance(approx_contours[i][j].point,approx_contours[k][l].point);
                    if(min_dst > dst){//ある領域内の最短点求める
                        min_dst=dst;
                        min_l=l;
                    }
                    
                }
                //同じ領域から２点取らないように
                if(tmp.dst[1] > min_dst){//最短更新
                    tmp.dst[2]=tmp.dst[1];
                    tmp.k[2]=tmp.k[1];
                    tmp.l[2]=tmp.l[1];
                    tmp.dst[1] = min_dst;
                    tmp.k[1]=k;
                    tmp.l[1]=min_l;
                    
                }else if(tmp.dst[2] > min_dst){//二番目に近い点更新
                    tmp.dst[2] = min_dst;
                    tmp.k[2]=k;
                    tmp.l[2]=min_l;
                }

            }
            //頂点生成
            if(tmp.dst[2] < ave_min_dst){//３辺からなる頂点
                tmp.num_of_sides=3;
                approx_contours[tmp.k[0]][tmp.l[0]].used=true;
                approx_contours[tmp.k[1]][tmp.l[1]].used=true;
                approx_contours[tmp.k[2]][tmp.l[2]].used=true;
                approx_contours[tmp.k[0]][tmp.l[0]].vertex_id=(int)vertexes.size();
                approx_contours[tmp.k[1]][tmp.l[1]].vertex_id=(int)vertexes.size();
                approx_contours[tmp.k[2]][tmp.l[2]].vertex_id=(int)vertexes.size();
                Vertex vertex((approx_contours[tmp.k[0]][tmp.l[0]].point
                                    +approx_contours[tmp.k[1]][tmp.l[1]].point
                                    +approx_contours[tmp.k[2]][tmp.l[2]].point)*(1.0/3.0),tmp.num_of_sides);
                vertexes.push_back(vertex);
                
            }else if(tmp.dst[1] < ave_min_dst){//２辺からなる頂点
                tmp.num_of_sides=2;
                approx_contours[tmp.k[0]][tmp.l[0]].used=true;
                approx_contours[tmp.k[1]][tmp.l[1]].used=true;
                approx_contours[tmp.k[0]][tmp.l[0]].vertex_id=(int)vertexes.size();
                approx_contours[tmp.k[1]][tmp.l[1]].vertex_id=(int)vertexes.size();
                Vertex vertex((approx_contours[tmp.k[0]][tmp.l[0]].point
                                    +approx_contours[tmp.k[1]][tmp.l[1]].point)*(1.0/2.0),tmp.num_of_sides);
                vertexes.push_back(vertex);
                //circle(drawing, approx_contours[tmp.k[0]][tmp.l[0]].point,3,Scalar(200,0,0),2);
            }else{
                //頂点取得失敗失敗！
                cout << "頂点取得失敗!" << endl;
                tmp.num_of_sides=1;
                Vertex vertex(Point(0,0),tmp.num_of_sides);
                vertexes.push_back(vertex);
            }
            tmp_vertexes.push_back(tmp);

        }
    }

    //頂点分類
    
    //各頂点のL,T字判別
    
    for(int i=0;i<vertexes.size();i++){//tmp_vertexesとvertexesは添字共有している
        if(vertexes[i].num_of_sides==2){
            
            int next_0,prev_0,next_1,prev_1;
            
            if(tmp_vertexes[i].l[0]==0){
                prev_0=(int)approx_contours[tmp_vertexes[i].k[0]].size() - 1;
            }else prev_0 = tmp_vertexes[i].l[0]-1;
            
            if(tmp_vertexes[i].l[0] == ((int)approx_contours[tmp_vertexes[i].k[0]].size() -1)  ){
                next_0=0;
            }else next_0 = tmp_vertexes[i].l[0]+1;
            
            if(tmp_vertexes[i].l[1]==0){
                prev_1=(int)approx_contours[tmp_vertexes[i].k[1]].size() - 1;
            }else prev_1 = tmp_vertexes[i].l[1]-1;
            
            if(tmp_vertexes[i].l[1] == ((int)approx_contours[tmp_vertexes[i].k[1]].size() -1)  ){
                next_1=0;
            }else next_1 = tmp_vertexes[i].l[1]+1;
            
            //まず同じ方向にある頂点の組を１つ見つける
            int other_0,other_1;//同じ方向の頂点に属していない特徴点
            //内積を求め、正なら同じ方向
            if((approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)
                .dot(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)//隣の頂点の値を取得
                   < norm(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[0]][next_0].vertex_id;
                }else vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[1]][next_1].vertex_id;
                
                other_0=prev_0;
                other_1=prev_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)//隣の頂点の値を取得
                   < norm(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[0]][next_0].vertex_id;
                }else vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[1]][prev_1].vertex_id;
                
                other_0=prev_0;
                other_1=next_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)//隣の頂点の値を取得
                   < norm(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[0]][prev_0].vertex_id;
                }else vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[1]][next_1].vertex_id;
                
                other_0=next_0;
                other_1=prev_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)//隣の頂点の値を取得
                   < norm(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[0]][prev_0].vertex_id;
                }else vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[1]][prev_1].vertex_id;
                
                other_0=next_0;
                other_1=next_1;
            }
            
            
            //ここで、other_0,1の頂点が同じ方向ならL型、逆方向ならT型と判断
            //これでは正しく取得できない場合もあるので改善の余地あり
            if((approx_contours[tmp_vertexes[i].k[0]][other_0].point - vertexes[i].point)
               .dot(approx_contours[tmp_vertexes[i].k[1]][other_1].point - vertexes[i].point)>0){
                //L型の場合
                vertexes[i].kind=L_EDGE;
                circle(drawing, vertexes[i].point,3,Scalar(0,0,0),2);

                if(norm(approx_contours[tmp_vertexes[i].k[0]][other_0].point - vertexes[i].point)
                   < norm(approx_contours[tmp_vertexes[i].k[1]][other_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][other_0].vertex_id;
                }else{
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[1]][other_1].vertex_id;
                }
                
                
            }else{
                //T型の場合
                vertexes[i].kind=T_EDGE;
                vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][next_0].vertex_id;
                vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][prev_0].vertex_id;
                circle(drawing, vertexes[i].point,3,Scalar(0,200,0),2);
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][other_0].point - vertexes[i].point)
                   < norm(approx_contours[tmp_vertexes[i].k[1]][other_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][other_0].vertex_id;
                }else{
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[1]][other_1].vertex_id;
                }
                
            }
            
            
            //

               

        }//2頂点の場合終了
        
        if(vertexes[i].num_of_sides==3){
            int tmp_l[3][2];
            Point tmp_vector[3];
            int tmp_counter=0;
            int flag=0;
            

            for(int j=0;j<3;j++){ //頂点を構成する特徴点の前後にある特徴点を取得
                if(tmp_vertexes[i].l[j]==0){
                    tmp_l[j][0]=(int)approx_contours[tmp_vertexes[i].k[j]].size() - 1;
                }else tmp_l[j][0] = tmp_vertexes[i].l[j]-1;
                
                if(tmp_vertexes[i].l[j] == ((int)approx_contours[tmp_vertexes[i].k[j]].size() -1)  ){
                    tmp_l[j][1]=0;
                }else tmp_l[j][1] = tmp_vertexes[i].l[j]+1;
                
            }
            
            for(int n=0;n<3 && flag==0;n++){
                for (int m=n+1; m<3 && flag==0; m++) {
                    for(int p=0;p<2 && flag==0;p++){
                        for(int q=0; q<2 && flag==0;q++){
                            if((approx_contours[tmp_vertexes[i].k[n]][tmp_l[n][p]].point - vertexes[i].point)
                               .dot(approx_contours[tmp_vertexes[i].k[m]][tmp_l[m][q]].point - vertexes[i].point)/
                               norm(approx_contours[tmp_vertexes[i].k[n]][tmp_l[n][p]].point - vertexes[i].point)/
                               norm(approx_contours[tmp_vertexes[i].k[m]][tmp_l[m][q]].point - vertexes[i].point) > 0.93969){//cos(20°)
                                if(norm(approx_contours[tmp_vertexes[i].k[n]][tmp_l[n][p]].point - vertexes[i].point) <
                                   norm(approx_contours[tmp_vertexes[i].k[m]][tmp_l[m][q]].point - vertexes[i].point)){
                                    vertexes[i].attached_vertex[tmp_counter]=approx_contours[tmp_vertexes[i].k[n]][tmp_l[n][p]].vertex_id;
                                    tmp_vector[tmp_counter]=approx_contours[tmp_vertexes[i].k[n]][tmp_l[n][p]].point- vertexes[i].point;
                                }else{
                                    vertexes[i].attached_vertex[tmp_counter]=approx_contours[tmp_vertexes[i].k[m]][tmp_l[m][q]].vertex_id;
                                    tmp_vector[tmp_counter]=approx_contours[tmp_vertexes[i].k[m]][tmp_l[m][q]].point- vertexes[i].point;
                                }
                                tmp_counter++;
                                if(tmp_counter==3){//３頂点見つけたはずなので角度取得
                                    flag=1;
                                    double sum_angle=0.0;
                                    sum_angle=acos(tmp_vector[0].dot(tmp_vector[1])/norm(tmp_vector[0])/norm(tmp_vector[1]));
                                    sum_angle+=acos(tmp_vector[0].dot(tmp_vector[2])/norm(tmp_vector[0])/norm(tmp_vector[2]));
                                    sum_angle+=acos(tmp_vector[1].dot(tmp_vector[2])/norm(tmp_vector[1])/norm(tmp_vector[2]));
                                    
                                    if(sum_angle < 6.108651){
                                        //A型
                                        vertexes[i].kind=A_EDGE;
                                        circle(drawing, vertexes[i].point,3,Scalar(0,200,200),2);
                                        //circle(drawing, vertexes[2].point,3,Scalar(0,200,200),2);
                                        //circle(drawing, vertexes[4].point,3,Scalar(0,200,200),2);

                                    }else{
                                        //Y型
                                        vertexes[i].kind=Y_EDGE;
                                        circle(drawing, vertexes[i].point,3,Scalar(150,100,150),2);

                                    }
                                }
                                
                            }
                        }
                    }
                }
            }
            
            
            
            
        }//３頂点の場合終了
    }
    
    
    drawing.copyTo(processed);
    
}

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour,Scalar color)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;
    
    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(contour);
    
    cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    //cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    cv::putText(im, label, pt, fontface, scale,color, thickness, 8);
}

//TODO 自前で類似線を合わせる処理つくる
//実際の線の太さを想定して取得してみる
//頂点データ保存

//L字の取得が甘い　２頂点前後の計４点すべてが別頂点に入る可能性あり



