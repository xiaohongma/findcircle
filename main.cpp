#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/mpl/accumulate.hpp>
#include <numeric>
#include <algorithm>
#include "dfs.h"
#include "bfs.h"
#include "utils.h"
using namespace cv;
using namespace std;
#define WINDOWNAME "original  image"

/**
 * we can use this method slelect roi by hand, draging out a rectangle in the image.It is a MouseCallback function
 */
void selectROI(int event, int x, int y, int flags, void* img);
/**
 save ROI as a Mat type image, and write into build dictonary
 */
void saveROI(Mat& img, Rect rect);




//prior: radius, n*n the num in each rectangle(error<3)
// the number(threshold) of circles that can be segmented(0.9)  
//可乐21, 24*9 4*6 农夫11.7 28*10pixel 4*7
double assume_radius = 11.7;
double test_count = 28*10;
int m =7;//x
int n =4;//y
bool g_bDrawingBox = false;
Rect g_rectangle = Rect(-1,-1,0,0);

int main(int argc, char **argv) {
    Mat srcImage = imread("2_texture.jpg");
     imshowResize(WINDOWNAME,srcImage);
    setMouseCallback(WINDOWNAME, selectROI, (void*) &srcImage);
    waitKey(0);
    //canny edge
    Mat distImg, tmpImg;
    Mat roi = imread("roi.jpg");
    distImg = roi;
    cvtColor(roi, tmpImg, CV_BGR2GRAY);
    GaussianBlur(tmpImg,tmpImg,Size(9,9),2,2);
    IplImage ipltmp = tmpImg;
    Mat edges;
    double canny_low;
    double canny_high;
    int min_radius = 0.8*assume_radius;
    int max_radius = 1.2*assume_radius;
    int min_dist = 2*assume_radius;
    AdaptiveFindThreshold(&ipltmp, &canny_low, &canny_high);
    double canny_thresroud = canny_high;
    Canny(tmpImg,edges, canny_thresroud, canny_thresroud/2);
    imshowResize("canny edge",edges);
 
    // hough circles, this can be optimized to reduce running time
    int acc_threshold = 30;// make the 2*n*n > count>= n*n
    vector<Vec3f> circles;
    //自适应寻找hough变换的阈值acc_threshold(这里设置步长为1，以后可以自适应步长)
    HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                 min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
    while(1) {
        if(circles.size() < test_count) {
            acc_threshold = acc_threshold-1;
            HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:" << circles.size()<<endl;
        } else if(circles.size() > 2*test_count) {
            acc_threshold = acc_threshold+1;
            HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:"<< circles.size()<<endl;
        } else {


            break;
        }
    }
    //estimate  circle radius
    vector<Point> centers;
    vector<int> radiuses;
    for (int i = 0; i< circles.size(); i++) {
        Point center(round(circles[i][0]), round(circles[i][1]));
        centers.push_back(center);
        int radius = round(circles[i][2]);
        radiuses.push_back(radius);
        // circle(distImg,center, 5,Scalar(155,50,255),-1,4,0);
        // circle(distImg,center,radius,Scalar(0,255,0),2,4,0);
        // putText(distImg,to_string(i),center,0 ,1, Scalar(0,0,255),4,8);
    }
    double mean_radius =  std::accumulate(begin(radiuses),end(radiuses), 0.0)/radiuses.size();
    cout << "mean radius" << mean_radius <<endl;
    // estimate mean distance
    double mean_object_dist;
    double angle;
    estimateCenterDistance(centers,&mean_object_dist, &angle);//圆心距离
    cout << mean_object_dist<< endl;
    
    //deep first search to  segment ROI into several rectangle
    int direction[8][2] = {{m-1,n-1},{n-1,m-1},{-(m-1),-(n-1)},{-(n-1),-(m-1)},{m-1,-(n-1)},{-(m-1),n-1},{n-1,-(m-1)},{-(n-1),m-1}};
    
  
    // visited map to record the visiting status of circle centers
    Mat  visited = Mat(distImg.size(),CV_8UC1,Scalar(255));
    for (int i = 0; i< centers.size(); i++) {

        visited.at<uchar>(centers.at(i)) = 0;
          circle(distImg,centers.at(i),5,Scalar(155,50,255),-1,4,0);
        //  circle(visited,p, 5,Scalar(155,50,255),-1,4,0);
    }
    int count_segmentation =0;//cout >0 indicates that we have at least one route to segment roi   successfully
    vector<Step> path;
    vector<Path> allpath;
   dfs(direction,visited,centers,mean_radius,mean_object_dist,distImg,&count_segmentation,path,allpath);
    // bfs(direction,roi.size(),centers,mean_radius,mean_object_dist,distImg,&count_segmentation);
    cout <<"count_segmentation " << count_segmentation<<endl;
    printoutPath(allpath);
 

    return 0;
}


void selectROI(int event, int x, int y, int flags, void* img)
{

    Mat& image = *(Mat*) img;
    switch( event ) {
    // 鼠标移动消息
    case EVENT_MOUSEMOVE: {
        if (g_bDrawingBox) {
            //绘制标识为真，则记录下长和宽到RECT变量中
            g_rectangle.width = x - g_rectangle.x;
            g_rectangle.height = y - g_rectangle.y;
        }
    }
    break;
    //左键按下消息
    case EVENT_LBUTTONDOWN: {
        g_bDrawingBox = true;
        g_rectangle = Rect(x, y, 0, 0);
        // cout << "Hello Button down" << endl;
    }
    break;
    //左键抬起消息
    case EVENT_LBUTTONUP: {
        g_bDrawingBox = false;
        if (g_rectangle.width < 0) {
            g_rectangle.x += g_rectangle.width;
            g_rectangle.width *= -1;
        }

        if (g_rectangle.height < 0) {
            g_rectangle.y += g_rectangle.height;
            g_rectangle.height *= -1;
        }
        saveROI(image, g_rectangle);


    }
    break;
    default:
        printf("error!\n");
    }
}
void saveROI(Mat& img, Rect rect)
{
    Mat roi = img(rect);
    imwrite("roi.jpg",roi);
    imshow("roi",roi);
    //rectangle(img, rect.tl(), rect.br(), Scalar(0, 255,0), 2, 1, 0);
    destroyWindow(WINDOWNAME);
    //imshowResize("original image",img);
}








