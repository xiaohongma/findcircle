
#include <numeric>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#include "utils.h"
void imshowResize(const cv::String& winname, const cv::Mat& img)
{
    namedWindow(winname, CV_WINDOW_NORMAL);
    cvResizeWindow(winname.c_str(), 700,700);
    imshow(winname, img);
}
// size at least 2*2
 void estimateCenterDistance(vector< Point > centers, double* mean_object_dist, double* angle)
{
    RotatedRect rect = minAreaRect(centers);
    *angle = rect.angle;
    //cout << "angle"<< rect.angle <<endl;
    //取外接矩形四个定点处的n*n个圆，计算圆心距。
    Point2f vertices[4];
    rect.points(vertices);
    //vector<double> dist1(centers.size());
    //double min;
    // int index;
    Point2f corner_4[4];//距离外接矩形四个顶点最近的圆心
    int num = centers.size();
    for (int i = 0; i < 4; i++) {
        double  min_dist = 1000000;
        int  index = 0;
        double dist1 = 0;
        for(int j = 0; j< num; j++) {
            Point a = centers.at(j);
            //cout << (a.x -vertices[i].x)*(a.x -vertices[i].x) +(a.y -vertices[i].y)*(a.y -vertices[i].y)<<endl;
            dist1 = (a.x -vertices[i].x)*(a.x -vertices[i].x) +(a.y -vertices[i].y)*(a.y -vertices[i].y);
            //dist[j] = (centers[j][0]-vertices[i][0])^2 +(centers[j][1]-vertices[i][1])^2;
            if(dist1<min_dist) {
                min_dist = dist1;
                index = j;
            }
        }
        corner_4[i] = centers[index];
        //centers.clear();
        // circle(img,corner_4[i], 5,Scalar(155,50,255),-1,4,0);
        //line(img, vertices[i], vertices[(i+1)%4], Scalar(255));
    }
    

 
    vector<Point> selected_object;//角点附近的圆心
    vector<double> object_dist; //圆心距离
    //  double object_space[2];

    for (int i = 0; i < 4; i++) {
        //double  min_dist = 1000000;
       // int  index = 0;
        double dist1[num];
        //int num = 2;
        // while(1){
        for(int j = 0; j< num; j++) {
            Point a = centers.at(j);

            dist1[j] = (a.x -corner_4[i].x)*(a.x -corner_4[i].x) +(a.y -corner_4[i].y)*(a.y -corner_4[i].y);
          //  if(dist1 ==0) continue;

        //   if(dist1<min_dist) {
         //       min_dist = dist1;
          //      index = j;
          //  }
        }
        sort(dist1,dist1+num);
        
        // depend on whether the object is more than 3*3 or not 
        if(num>=9){
         object_dist.push_back(sqrt(dist1[1]));
         object_dist.push_back(sqrt(dist1[2]));
         object_dist.push_back(sqrt(dist1[4])/2);
         object_dist.push_back(sqrt(dist1[5])/2);
        } else{
          object_dist.push_back(sqrt(dist1[1]));
          object_dist.push_back(sqrt(dist1[2]));
        }
        // circle(img,selected_object[i], 5,Scalar(155,50,255),-1,4,0);
        // for(int m = 0; m <num ;m++)
        // cout << "dist " <<sqrt(dist1[m])<< endl;

        

    }
    sort(object_dist.begin(),object_dist.end());
   // for(int i = 0; i <object_dist.size();i++){
    //    cout << "dist " << object_dist.at(i)<< endl;
   // }

    //average of center dist,exclude the maximun and minimun
    *mean_object_dist = accumulate(object_dist.begin()+1,object_dist.end()-1, 0.0)/(object_dist.size()-2);
   // double  xmean_object_dist = accumulate(object_dist.begin(),object_dist.end(), 0.0)/object_dist.size();
    cout << "mean_object_dist"<<*mean_object_dist<< endl;
   // cout << "xmean_object_dist"<<xmean_object_dist<< endl;

    //calculate the top right point of sub-box
   // *bl = Point(corner_4[0].x, corner_4[0].y);


}

 void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size )
{
    cv::Mat src = cv::cvarrToMat(image);
    // imshow("Adaptive", src);
    const int cn = src.channels();
    cv::Mat dx(src.rows, src.cols, CV_16SC(cn));
    cv::Mat dy(src.rows, src.cols, CV_16SC(cn));

    cv::Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);

    CvMat _dx = dx, _dy = dy;
    _AdaptiveFindThreshold(&_dx, &_dy, low, high);

}
 

// 仿照matlab，自适应求高低两个门限
 void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high)
{
    CvSize size;
    IplImage *imge=0;
    int i,j;
    CvHistogram *hist;
    int hist_size = 255;
    float range_0[]= {0,256};
    float* ranges[] = { range_0 };
    double PercentOfPixelsNotEdges = 0.7;
    size = cvGetSize(dx);
    imge = cvCreateImage(size, IPL_DEPTH_32F, 1);
    // 计算边缘的强度, 并存于图像中
    float maxv = 0;
    for(i = 0; i < size.height; i++ )
    {
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        float* _image = (float *)(imge->imageData + imge->widthStep*i);
        for(j = 0; j < size.width; j++)
        {
            _image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
            maxv = maxv < _image[j] ? _image[j]: maxv; //保存最大的边缘强度

        }
    }
    if(maxv == 0) {
        high = 0;
        low = 0;
        cvReleaseImage( &imge );
        return;
    }

    // 计算直方图，bins范围是灰度范围，统计量为梯度值。选择梯度比例为0.7处的灰度值作为高阈值。
    range_0[1] = maxv;
    hist_size = (int)(hist_size > maxv ? maxv:hist_size);//选择bin的范围，如果灰度值达不到255，就用较小值
    hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    cvCalcHist( &imge, hist, 0, NULL );
    int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
    float sum=0;
    int icount = hist->mat.dim[0].size;
    //cout <<icount <<endl;

    float *h = (float*)cvPtr1D( hist->bins, 0 );
    for(i = 0; i < icount; i++)
    {
        sum += h[i];
        if( sum > total )
            break;
    }
    // 计算高低门限
    *high = (i+1) * maxv / hist_size ;
    *low = *high * 0.5;
    cvReleaseImage( &imge );
    cvReleaseHist(&hist);
}

void printoutPath(vector<Path>& allpath)
{
        if(allpath.empty()){
        cout <<"no segmentation path have been found"<<endl;
    }else if(allpath.size()==1){
        cout << "just one path found!" <<endl;
        Path bestpath = allpath.at(0);
        for(int j = 0;j<bestpath.path.size();j++){
            cout<<"path"<<bestpath.path.at(j).p<<"direction"<<bestpath.path.at(j).direction<<endl; 
            }
        
    }else{
        int remaining = allpath.at(0).tag;
        int index = 0;
        for(int i = 1;i<allpath. size();i++){
            //vector<Step> path = allpath.at(i).path; 
            int tag = allpath.at(i).tag;//remaining points
           if(tag<remaining) {index = i;remaining = tag;}
        } 
        cout <<"more than one path found, return the best path"<< endl;
        Path bestpath = allpath.at(index);
         for(int j = 0;j<bestpath.path.size();j++){
            cout<<"path"<<bestpath.path.at(j).p<<"direction"<<bestpath.path.at(j).direction<<endl; 
            }
        
    }
}
