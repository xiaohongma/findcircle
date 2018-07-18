#include "utils.h"
#include <numeric>
#include <algorithm>
using namespace cv;
using namespace std;
void imshowResize(const String& winname, Mat& img)
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
