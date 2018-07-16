#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/mpl/accumulate.hpp>
#include <numeric>
#include "dfs.h"
#include "utils.h"
#include "bfs.h"
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

/**
 * this two methods are used to adaptive canny edge detection.
 */
void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size =3);
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high);

//prior: radius, n*n
//可乐21, 24*9 农夫11.7 28*10pixel
double assume_radius = 21;
double test_count = 24*9;
int m =6;//x
int n =4;//y
bool g_bDrawingBox = false;
Rect g_rectangle = Rect(-1,-1,0,0);

int main(int argc, char **argv) {
    Mat srcImage = imread("1_texture.jpg");
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
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        centers.push_back(center);
        int radius = cvRound(circles[i][2]);
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
    
    //deep first search to segment ROI into several rectangle
    int direction[8][2] = {{m-1,n-1},{n-1,m-1},{-(m-1),-(n-1)},{-(n-1),-(m-1)},{m-1,-(n-1)},{-(m-1),n-1},{n-1,-(m-1)},{-(n-1),m-1}};
    // visited map to record the visiting status of circle centers
    Mat  visited = Mat(distImg.size(),CV_8UC1,Scalar(255));
    for (int i = 0; i< centers.size(); i++) {

        visited.at<uchar>(centers.at(i)) = 0;
        //  circle(visited,p1, 5,Scalar(155,50,255),-1,4,0);
        //  circle(visited,p, 5,Scalar(155,50,255),-1,4,0);
    }
    int count_segmentation =0;//cout >0 indicates that we have at least one route to segment roi   successfully
   dfs(direction,visited,centers,mean_radius,mean_object_dist,distImg,&count_segmentation);
    // bfs(direction,roi.size(),centers,mean_radius,mean_object_dist,distImg,&count_segmentation);
    cout <<"count_segmentation" << count_segmentation<<endl;

    //cv::rectangle(distImg,sub_rect,Scalar(255),1,8,0);

    imshowResize("first box",distImg);
    waitKey(0);

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






