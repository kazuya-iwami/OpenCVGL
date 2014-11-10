#include <core.hpp>
#include <imgproc.hpp>
#include <highgui.hpp>
#include <stdio.h>

#define FLAG 1 //0:direct access 1:built-in function

const char* preset_file = "/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/fruits.jpg";

void convertColorToGray(cv::Mat &input,cv:: Mat &processed);
void blur(cv::Mat &input, cv::Mat &processed);
void edge(cv::Mat &input, cv::Mat &processed);

int main(int argc, char *argv[]){
    const char *input_file;
    //prepare Mat objects for input image and output image
    cv::Mat input,processed;
    
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
    
    //convertColorToGray(input,processed);
    //edge(input,processed);
    //cv::GaussianBlur(input, processed,cv::Size(5,5),10,10);
    cv::Sobel(input, processed,  -1, 1, 1,5);
    //5.create windows
    
    cv::namedWindow("original image",1);
    cv::namedWindow("processed image",1);
    
    //6.show images
    
    cv::imshow("original image",input);
    cv::imshow("processed image",processed);
    
    //7.wait key input
    cv::waitKey(0);
    
    //8.save the processed result
    cv::imwrite("/Users/kazuya/Git/OpenCVGL/OpenCVGL1.1/processd.jpg",processed);
    
    return 0;
    
}

void blur(cv::Mat &input, cv::Mat &processed){
    
    cv::Size s=input.size();
    processed.create(s,CV_8UC3);
    
    int filter[3][3]={
        {1,2,1},
        {2,4,2},
        {1,2,1}};
    int sum_of_filter = 16;
    
    
    for (int i = 0; i < s.height-2; i++){
        
        uchar *ptr1;
        cv::Vec3b *ptr2;
        
        for (int j = 0; j < s.width-2; j++){
            cv::Vec3b sum = {0,0,0};
            for (int k = 0; k < 3; k++){
                for (int l = 0; l < 3; l++){
                    ptr1=input.ptr<uchar>(i+k,j+l);
                    
                    sum[0] += filter[k][l]*(uchar)ptr1[0]/sum_of_filter;
                    sum[1] += filter[k][l]*(uchar)ptr1[1]/sum_of_filter;
                    sum[2] += filter[k][l]*(uchar)ptr1[2]/sum_of_filter;
                }
            }
            ptr2=processed.ptr<cv::Vec3b>(i,j);
            *ptr2 = sum;
        }
    }
    
    
}

void edge(cv::Mat &input, cv::Mat &processed){
    
    cv::Size s=input.size();
    processed.create(s,CV_8UC3);
    
    
    
    
    int filter[3][3]={
        {1,2,1},
        {2,4,2},
        {1,2,1}};
    int sum_of_filter = 16;
    
    
    for (int i = 0; i < s.height-2; i++){
        
        uchar *ptr1;
        cv::Vec3b *ptr2;
        
        for (int j = 0; j < s.width-2; j++){
            cv::Vec3b sum = {0,0,0};
            for (int k = 0; k < 3; k++){
                for (int l = 0; l < 3; l++){
                    ptr1=input.ptr<uchar>(i+k,j+l);
                    
                    sum[0] += filter[k][l]*(uchar)ptr1[0]/sum_of_filter;
                    sum[1] += filter[k][l]*(uchar)ptr1[1]/sum_of_filter;
                    sum[2] += filter[k][l]*(uchar)ptr1[2]/sum_of_filter;
                }
            }
            ptr2=processed.ptr<cv::Vec3b>(i,j);
            *ptr2 = sum;
        }
    }
    
    
}



void convertColorToGray(cv::Mat &input,cv::Mat &processed)
{
#if FLAG //use built-in function
    
    //4.convert color to gray
    cv::Mat temp;
    std::vector<cv::Mat> planes;
    cv::cvtColor(input,temp,CV_BGR2YCrCb);
    cv::split(temp,planes);
    processed=planes[0];
#else
    //3.create Mat for output image
    cv::Size s=input.size();
    processed.create(s,CV_8UC1);
    
    for (int j=0;j<s.height;j++){
        uchar *ptr1,*ptr2;
        ptr1=input.ptr<uchar>(j);
        ptr2=processed.ptr<uchar>(j);
        
        //4.convert color to gray
        
        for(int i=0;i<s.width;i++){
            double y=0.114*(double)ptr1[0]+0.587*(double)ptr1[1]+0.299*(double)ptr1[2];
            if(y>255){y=255;}
            if(y<0){y=0;}
            
            *ptr2=(uchar)y;
            
            ptr1 +=3;
            ptr2++;
        }
    }
#endif
}