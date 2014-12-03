//
//  blackboard.cpp
//  OpenCVGL1.1
//
//  Created by Iwami kazuya on 2014/11/18.
//  Copyright (c) 2014年 kazuya. All rights reserved.
//
//参考URL
//https://opencv-code.com/tutorials/detecting-simple-shapes-in-an-image/
//http://opencv.jp/opencv-2.1/cpp/structural_analysis_and_shape_descriptors.html
//http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html




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

enum Vertex_Type {
    NONE_EDGE,
    A_EDGE,
    T_EDGE,
    Y_EDGE,
    L_EDGE
};

enum Vertex_Kind {
    VERTEX_KIND_NONE,
    
    KIND_L1,
    KIND_L2,
    KIND_L3,
    KIND_L4,
    KIND_L5,
    KIND_L6,
    
    KIND_A1,
    KIND_A2,
    KIND_A3,
    
    KIND_Y1,
    KIND_Y2,
    KIND_Y3,
    KIND_Y4,
    KIND_Y5,
    
    KIND_T1,
    KIND_T2,
    KIND_T3,
    KIND_T4
};

enum Edge_Kind{
    KIND_EDGE_NONE,
    KIND_EDGE_ERR,
    KIND_PLUS,
    KIND_MINUS,
    KIND_FORE,
    KIND_BACK
};



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


class Vertex_Existing_Kind{
public:
    Vertex_Kind vertex_kind;
    bool flag;
    Vertex_Existing_Kind(Vertex_Kind k);
};

Vertex_Existing_Kind::Vertex_Existing_Kind(Vertex_Kind k){
    vertex_kind = k;
    flag=true;
}


class Edge{
public:
    int vertex_number1, edge_number1, vertex_number2, edge_number2;
    Edge_Kind edge_kind;
    Edge(int v1,int e1, int v2, int e2);
};

Edge::Edge(int v1,int e1, int v2, int e2){
    vertex_number1=v1;
    vertex_number2=v2;
    edge_number1=e1;
    edge_number2=e2;
    edge_kind=KIND_EDGE_NONE;
}


class Vertex { //頂点クラス
public:
    Point point;
    int num_of_sides;//辺の数
    Vertex_Type type;
    int attached_vertex[3];
    vector<Vertex_Existing_Kind> existing_kind_list;
    Vertex(Point p,int t);
};

Vertex::Vertex(Point p,int t){
    point=p;
    num_of_sides=t;
    type=NONE_EDGE;
    attached_vertex[0]=attached_vertex[1]=attached_vertex[2]=-1;

}







//関数定義



void detecteVertex(Mat &input,vector<Vertex> &vertexes,vector<Edge> &edges);
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour,Scalar color);
bool edgeExists(vector<Edge> edges,int v1,int e1, int v2, int e2);
int genEdgeLabel(vector<Vertex> &vertex, vector<Edge> &edges);
Edge_Kind getEdgeKind(Vertex_Kind kind, int edge_n,bool out_dir);
bool calcEdge(vector<Vertex> &vertexes, Edge &edge);
bool checkLocateRight(Point p1,Point p2);

double distance(Point p1,Point p2){
    double dst=sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    return dst;
}





int main (int argc, char **argv)
{
    
    
    vector<Vertex> vertexes;//頂点のベクタ
    vector<Edge> edges;
    
    
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
    cv::Mat input;
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
    
    detecteVertex(input, vertexes, edges);
    genEdgeLabel(vertexes, edges);
    
    cv::namedWindow("processed image",1);

    cv::imshow("processed image",input);

    cv::waitKey(0);

    
    
    
    return 0;
}

void detecteVertex(Mat &input,vector<Vertex> &vertexes,vector<Edge> &edges){
    
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
    
    
    
    
    //頂点生成
    vector<TmpVertex> tmp_vertexes;//一時的な頂点保存クラス
    
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
            //vertexのインスタンス化
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
                    vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[0]][next_0].vertex_id;
                }else vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[1]][next_1].vertex_id;
                
                other_0=prev_0;
                other_1=prev_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][next_0].point - vertexes[i].point)//隣の頂点の値を取得
                   < norm(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[0]][next_0].vertex_id;
                }else vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[1]][prev_1].vertex_id;
                
                other_0=prev_0;
                other_1=next_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)//隣り合う頂点の登録
                   < norm(approx_contours[tmp_vertexes[i].k[1]][next_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[0]][prev_0].vertex_id;
                }else vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[1]][next_1].vertex_id;
                
                other_0=next_0;
                other_1=prev_1;
            }else if((approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)
                     .dot(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)>0){
                
                if(norm(approx_contours[tmp_vertexes[i].k[0]][prev_0].point - vertexes[i].point)//隣り合う頂点の登録
                   < norm(approx_contours[tmp_vertexes[i].k[1]][prev_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[0]][prev_0].vertex_id;
                }else vertexes[i].attached_vertex[2]=approx_contours[tmp_vertexes[i].k[1]][prev_1].vertex_id;
                
                other_0=next_0;
                other_1=next_1;
            }
            
            
            //ここで、other_0,1の頂点が同じ方向ならL型、逆方向ならT型と判断
            //これでは正しく取得できない場合もあるので改善の余地あり
            if((approx_contours[tmp_vertexes[i].k[0]][other_0].point - vertexes[i].point)
               .dot(approx_contours[tmp_vertexes[i].k[1]][other_1].point - vertexes[i].point)>0){
                //L型の場合
                vertexes[i].type=L_EDGE;
                
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L1));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L2));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L3));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L4));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L5));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_L6));
                
                circle(input, vertexes[i].point,3,Scalar(200,200,200),2);
                
                vertexes[i].attached_vertex[1] = vertexes[i].attached_vertex[2];//仕様上こうなってます

                if(norm(approx_contours[tmp_vertexes[i].k[0]][other_0].point - vertexes[i].point)//隣り合う頂点の登録
                   < norm(approx_contours[tmp_vertexes[i].k[1]][other_1].point - vertexes[i].point)){
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][other_0].vertex_id;
                }else{
                    vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[1]][other_1].vertex_id;
                }
                
                //並び替え
                if(checkLocateRight(vertexes[vertexes[i].attached_vertex[1]].point - vertexes[i].point,
                                    vertexes[vertexes[i].attached_vertex[0]].point - vertexes[i].point)){
                    int tmp_a0 = vertexes[i].attached_vertex[0];
                    vertexes[i].attached_vertex[0] = vertexes[i].attached_vertex[1];
                    vertexes[i].attached_vertex[1] =tmp_a0;
                    
                }
                
                
                
            }else{
                //T型の場合
                vertexes[i].type=T_EDGE;
                
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_T1));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_T2));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_T3));
                vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_T4));
                
                vertexes[i].attached_vertex[0]=approx_contours[tmp_vertexes[i].k[0]][other_0].vertex_id;//隣り合う頂点の登録
                vertexes[i].attached_vertex[1]=approx_contours[tmp_vertexes[i].k[1]][other_1].vertex_id;
                circle(input, vertexes[i].point,3,Scalar(0,200,0),2);
                
                //並び替え
                if(checkLocateRight(vertexes[vertexes[i].attached_vertex[2]].point - vertexes[i].point,
                                    vertexes[vertexes[i].attached_vertex[1]].point - vertexes[i].point)){
                    int tmp_a0 = vertexes[i].attached_vertex[0];
                    vertexes[i].attached_vertex[0] = vertexes[i].attached_vertex[1];
                    vertexes[i].attached_vertex[1] =tmp_a0;

                }
            }
            
            
            //

               

        }//2頂点の場合終了
        
        if(vertexes[i].num_of_sides==3){//3頂点の場合
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
                                    double angle1,angle2,angle3=0.0;
                                    angle1=acos(tmp_vector[0].dot(tmp_vector[1])/norm(tmp_vector[0])/norm(tmp_vector[1]));
                                    angle2=acos(tmp_vector[0].dot(tmp_vector[2])/norm(tmp_vector[0])/norm(tmp_vector[2]));
                                    angle3=acos(tmp_vector[1].dot(tmp_vector[2])/norm(tmp_vector[1])/norm(tmp_vector[2]));
                                    
                                    
                                    if(angle1+angle2+angle3 < 6.108651){
                                        //A型
                                        vertexes[i].type=A_EDGE;
                                        
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_A1));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_A2));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_A3));
                                        
                                        circle(input, vertexes[i].point,3,Scalar(0,200,200),2);
                                        // vertexes[i].attached_vertex[2]が中央の接点になるように接点並び替え
                                        int tmp_a0=vertexes[i].attached_vertex[0];
                                        int tmp_a1=vertexes[i].attached_vertex[1];
                                        int tmp_a2=vertexes[i].attached_vertex[2];
                                        
                                        
                                        if(angle1>angle2+angle3-0.174533){
                                            if(checkLocateRight(tmp_vector[0], tmp_vector[1])){
                                                vertexes[i].attached_vertex[0]=tmp_a1;
                                                vertexes[i].attached_vertex[1]=tmp_a0;
                                                
                                            }
                                        }else if(angle3>angle2+angle1-0.174533){
                                            if(checkLocateRight(tmp_vector[1], tmp_vector[2])){
                                                vertexes[i].attached_vertex[0]=tmp_a2;
                                                vertexes[i].attached_vertex[2]=tmp_a0;
                                                
                                            }else{
                                                vertexes[i].attached_vertex[0]=tmp_a1;
                                                vertexes[i].attached_vertex[1]=tmp_a2;
                                                vertexes[i].attached_vertex[2]=tmp_a0;
                                            }
                                        }else{
                                            if(checkLocateRight(tmp_vector[0], tmp_vector[2])){
                                                vertexes[i].attached_vertex[0]=tmp_a2;
                                                vertexes[i].attached_vertex[1]=tmp_a0;
                                                vertexes[i].attached_vertex[2]=tmp_a1;
                                                
                                            }else{
                                                vertexes[i].attached_vertex[1]=tmp_a2;
                                                vertexes[i].attached_vertex[2]=tmp_a1;
                                            }
                                        }
                                        
                                        //circle(drawing, vertexes[2].point,3,Scalar(0,200,200),2);
                                        //circle(drawing, vertexes[4].point,3,Scalar(0,200,200),2);

                                    }else{
                                        //Y型
                                        vertexes[i].type=Y_EDGE;
                                        
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_Y1));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_Y2));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_Y3));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_Y4));
                                        vertexes[i].existing_kind_list.push_back(Vertex_Kind(KIND_Y5));
                                        
                                        circle(input, vertexes[i].point,3,Scalar(150,100,150),2);
                                        
                                        //並び替え
                                        if(checkLocateRight(vertexes[vertexes[i].attached_vertex[0]].point - vertexes[i].point,
                                                            vertexes[vertexes[i].attached_vertex[2]].point - vertexes[i].point)){
                                            int tmp_a1 = vertexes[i].attached_vertex[1];
                                            vertexes[i].attached_vertex[1] = vertexes[i].attached_vertex[2];
                                            vertexes[i].attached_vertex[2] =tmp_a1;
                                            
                                        }


                                    }
                                }
                                
                            }
                        }
                    }
                }
            }
            
        }//３頂点の場合終了
    }//頂点分類終了
    
    
    
    
    //edge_listの生成
    for(int i=0;i<vertexes.size();i++){
        if(vertexes[i].type == L_EDGE){
            for(int edge_n1=0; edge_n1<2; edge_n1++){
                if(vertexes[vertexes[i].attached_vertex[edge_n1]].type == L_EDGE){
                    for(int edge_n2=0;edge_n2<2;edge_n2++){
                        if(i == vertexes[vertexes[i].attached_vertex[edge_n1]].attached_vertex[edge_n2]){//対応する２頂点検出
                            if(edgeExists(edges, i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2) == false){//まだその辺が登録されていなかったら
                                edges.push_back(Edge( i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2));
                            }
                        }
                    }
                }else {
                    for(int edge_n2=0;edge_n2<3;edge_n2++){
                        if(i == vertexes[vertexes[i].attached_vertex[edge_n1]].attached_vertex[edge_n2]){//対応する２頂点検出
                            if(edgeExists(edges, i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2) == false){//まだその辺が登録されていなかったら
                                edges.push_back(Edge( i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2));
                            }
                        }
                    }
                }
                
            }
        }else {
            for(int edge_n1=0; edge_n1<3; edge_n1++){
                if(vertexes[vertexes[i].attached_vertex[edge_n1]].type == L_EDGE){
                    for(int edge_n2=0;edge_n2<2;edge_n2++){
                        if(i == vertexes[vertexes[i].attached_vertex[edge_n1]].attached_vertex[edge_n2]){//対応する２頂点検出
                            if(edgeExists(edges, i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2) == false){//まだその辺が登録されていなかったら
                                edges.push_back(Edge( i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2));
                            }
                        }
                    }
                }else {
                    for(int edge_n2=0;edge_n2<3;edge_n2++){
                        if(i == vertexes[vertexes[i].attached_vertex[edge_n1]].attached_vertex[edge_n2]){//対応する２頂点検出
                            if(edgeExists(edges, i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2) == false){//まだその辺が登録されていなかったら
                                edges.push_back(Edge( i, edge_n1, vertexes[i].attached_vertex[edge_n1], edge_n2));
                            }
                        }
                    }
                }
                
            }
        }
    }
    
    
    
    for(int i=0;i<vertexes.size();i++){//頂点のラベル表示
        cv::putText(input, to_string(i),vertexes[i].point , cv::FONT_HERSHEY_SIMPLEX, 1,Scalar(200,200,200), 1, 8);
        
    }
    
    
}

bool checkLocateRight(Point p1,Point p2){ // P1→P2について　trueなら右にfalseなら左にある。
    if((p1.x * p2.y - p1.y * p2.x) > 0){
        return  true;
    }else return false;
}


//その辺が既に登録されているか
bool edgeExists(vector<Edge> edges,int v1,int e1, int v2, int e2){    for(int i=0;i<edges.size();i++){
        if(edges[i].vertex_number1==v1 && edges[i].edge_number1 == e1 && edges[i].vertex_number2==v2 && edges[i].edge_number2 == e2){
            return true;
        }
        if(edges[i].vertex_number1==v2 && edges[i].edge_number1 == e2 && edges[i].vertex_number2==v1 && edges[i].edge_number2 == e1){
            return true;
        }
    }
    return false;
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




//線画のラベリング関数
int genEdgeLabel(vector<Vertex> &vertex, vector<Edge> &edges){ //err 0; １つに決定 1 決定しない 2;
    
    while(1){
        bool delete_flag=false;
        for(int i =0; i<edges.size();i++){
            if(edges[i].edge_kind == KIND_EDGE_ERR)return 0;
            delete_flag = calcEdge(vertex, edges[i]);
        }
        if(delete_flag == false){//何も消されなかったら抜ける
            for(int i =0; i<edges.size();i++){
                if(edges[i].edge_kind == KIND_EDGE_NONE)return 2;
            }
            
            return 1;
        }
    }
    
    return 0;
    
}


Edge_Kind getEdgeKind(Vertex_Kind kind, int edge_n ,bool out_dir){//edgeのv1ならout_dir=true v2ならfalse
    Edge_Kind KIND_OUT,KIND_IN;
    if(out_dir){
        KIND_OUT=KIND_FORE;
        KIND_IN=KIND_BACK;
    }else{
        KIND_OUT=KIND_BACK;
        KIND_IN=KIND_FORE;
    }
    
    if(kind==KIND_A1 && edge_n == 0)return KIND_IN;
    if(kind==KIND_A1 && edge_n == 1)return KIND_OUT;
    if(kind==KIND_A1 && edge_n == 2)return KIND_PLUS;
    
    if(kind==KIND_A2 && edge_n == 0)return KIND_PLUS;
    if(kind==KIND_A2 && edge_n == 1)return KIND_PLUS;
    if(kind==KIND_A2 && edge_n == 2)return KIND_MINUS;
    
    if(kind==KIND_A3 && edge_n == 0)return KIND_MINUS;
    if(kind==KIND_A3 && edge_n == 1)return KIND_MINUS;
    if(kind==KIND_A3 && edge_n == 2)return KIND_PLUS;
    
    
    if(kind==KIND_Y1 && edge_n == 0)return KIND_PLUS;
    if(kind==KIND_Y1 && edge_n == 1)return KIND_PLUS;
    if(kind==KIND_Y1 && edge_n == 2)return KIND_PLUS;
    
    if(kind==KIND_Y2 && edge_n == 0)return KIND_IN;
    if(kind==KIND_Y2 && edge_n == 1)return KIND_OUT;
    if(kind==KIND_Y2 && edge_n == 2)return KIND_MINUS;
    
    if(kind==KIND_Y3 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_Y3 && edge_n == 1)return KIND_MINUS;
    if(kind==KIND_Y3 && edge_n == 2)return KIND_IN;
    
    if(kind==KIND_Y4 && edge_n == 0)return KIND_MINUS;
    if(kind==KIND_Y4 && edge_n == 1)return KIND_IN;
    if(kind==KIND_Y4 && edge_n == 2)return KIND_OUT;
    
    if(kind==KIND_Y5 && edge_n == 0)return KIND_MINUS;
    if(kind==KIND_Y5 && edge_n == 1)return KIND_MINUS;
    if(kind==KIND_Y5 && edge_n == 2)return KIND_MINUS;
    
    
    if(kind==KIND_L1 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_L1 && edge_n == 1)return KIND_IN;
    
    if(kind==KIND_L2 && edge_n == 0)return KIND_IN;
    if(kind==KIND_L2 && edge_n == 1)return KIND_OUT;
    
    if(kind==KIND_L3 && edge_n == 0)return KIND_PLUS;
    if(kind==KIND_L3 && edge_n == 1)return KIND_OUT;
    
    if(kind==KIND_L4 && edge_n == 0)return KIND_IN;
    if(kind==KIND_L4 && edge_n == 1)return KIND_PLUS;
    
    if(kind==KIND_L5 && edge_n == 0)return KIND_MINUS;
    if(kind==KIND_L5 && edge_n == 1)return KIND_IN;
    
    if(kind==KIND_L6 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_L6 && edge_n == 1)return KIND_MINUS;
    
    
    if(kind==KIND_T1 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_T1 && edge_n == 1)return KIND_IN;
    if(kind==KIND_T1 && edge_n == 2)return KIND_OUT;
    
    if(kind==KIND_T2 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_T2 && edge_n == 1)return KIND_IN;
    if(kind==KIND_T2 && edge_n == 2)return KIND_IN;

    if(kind==KIND_T3 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_T3 && edge_n == 1)return KIND_IN;
    if(kind==KIND_T3 && edge_n == 2)return KIND_PLUS;

    if(kind==KIND_T4 && edge_n == 0)return KIND_OUT;
    if(kind==KIND_T4 && edge_n == 1)return KIND_IN;
    if(kind==KIND_T4 && edge_n == 2)return KIND_MINUS;
    
 
    return KIND_EDGE_NONE;
}

bool calcEdge(vector<Vertex> &vertexes,Edge &edge){//何か削除したら戻り値 true
    bool erase_flag1[6],erase_flag2[6];
    memset(erase_flag1,0,sizeof(erase_flag1));//消すところfalseにする
    memset(erase_flag2,0,sizeof(erase_flag2));
    
    for(int i =0; i<vertexes[edge.vertex_number1].existing_kind_list.size();i++){
        if(vertexes[edge.vertex_number1].existing_kind_list[i].flag==false)break;
        for(int j=0; j<vertexes[edge.vertex_number2].existing_kind_list.size();j++){
            if(vertexes[edge.vertex_number2].existing_kind_list[j].flag==false)break;

            if((getEdgeKind(vertexes[edge.vertex_number1].existing_kind_list[i].vertex_kind, edge.edge_number1, true) ==
               getEdgeKind(vertexes[edge.vertex_number2].existing_kind_list[j].vertex_kind, edge.edge_number2, true) &&
               edge.edge_kind==KIND_EDGE_NONE)
               ||
               (getEdgeKind(vertexes[edge.vertex_number1].existing_kind_list[i].vertex_kind, edge.edge_number1, true) ==
                getEdgeKind(vertexes[edge.vertex_number2].existing_kind_list[j].vertex_kind, edge.edge_number2, true) &&
                getEdgeKind(vertexes[edge.vertex_number1].existing_kind_list[i].vertex_kind, edge.edge_number1, true) == edge.edge_kind)){//２頂点の型の組み合わせが存在 かつ　指定されていればその辺の型を優先
                   
                //Edge_Kind test1=getEdgeKind(vertexes[edge.vertex_number1].existing_kind_list[i].vertex_kind, edge.edge_number1, true);
                //Edge_Kind test2=getEdgeKind(vertexes[edge.vertex_number2].existing_kind_list[j].vertex_kind, edge.edge_number2, true);
                   
                erase_flag1[i]=true;//存在したら消さない
                erase_flag2[j]=true;
            }
        }
    }
    
    bool erase_flag=false;
    for(int i =0; i<vertexes[edge.vertex_number1].existing_kind_list.size();i++){
        if(erase_flag1[i]==false && vertexes[edge.vertex_number1].existing_kind_list[i].flag){
            vertexes[edge.vertex_number1].existing_kind_list[i].flag = false;
            erase_flag=true;

        }
    }
    for(int i =0; i<vertexes[edge.vertex_number2].existing_kind_list.size();i++){
        if(erase_flag2[i]==false && vertexes[edge.vertex_number2].existing_kind_list[i].flag){
            vertexes[edge.vertex_number2].existing_kind_list[i].flag = false;
            erase_flag=true;
        }
    }
    
    int n1=0;//trueの数
    int n2=0;
    Edge_Kind tmp_edge=KIND_EDGE_NONE;
    for(int i =0; i<vertexes[edge.vertex_number1].existing_kind_list.size();i++){
        if(vertexes[edge.vertex_number1].existing_kind_list[i].flag)n1++;
        tmp_edge=getEdgeKind(vertexes[edge.vertex_number1].existing_kind_list[i].vertex_kind, edge.edge_number1, true);
    }
    for(int i =0; i<vertexes[edge.vertex_number2].existing_kind_list.size();i++){
        if(vertexes[edge.vertex_number2].existing_kind_list[i].flag)n2++;
    }
    
    if(n1==1 && n2==1){
        edge.edge_kind=tmp_edge;
    }
    if(n1==0 || n2==0){
        edge.edge_kind=KIND_EDGE_ERR;
        printf("CALC ERR!!\n");
    }
    
    return erase_flag;

}




//TODO 自前で類似線を合わせる処理つくる
//実際の線の太さを想定して取得してみる
//頂点データ保存

//L字の取得が甘い　２頂点前後の計４点すべてが別頂点に入る可能性あり



