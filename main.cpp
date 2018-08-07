#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include "utils.h"
#include "segmentation.h"
#include "feature_circle.h"
#include "contours.h"
#include "dfs.h"
using namespace cv;
using namespace std;
#define WINDOWNAME "original  image"
#define USE_CIRCLE 0;

/**
 * we can use this method slelect roi by hand, draging out a rectangle in the image.It is a MouseCallback function
 */
void selectROI(int event, int x, int y, int flags, void* img);
/**
 save ROI as a Mat type image, and write into build dictonary
 */
void saveROI(Mat& img, Rect rect);
void saveMask(int event, int x,int y, int flags, void* img);

void getContour(Mat& input, Mat& mat_for_show);
double angle( Point pt1, Point pt2, Point pt0 );




//prior: radius, n*n the num in each rectangle(error<3)
// the number(threshold) of circles that can be segmented(0.9)  
//可乐21, 24*9 4*6 农夫11.7 28*10pixel 4*7 315*163.3
double assume_radius = 21;
int m =6;//x
int n =4;//y 
float width = 194.4;
float height = 297.8;  
bool g_bDrawingBox = false;
Rect g_rectangle = Rect(-1,-1,0,0);
vector<Point> mask_vertex;


int main(int argc, char **argv) {
     /*Mat conx = imread("contour_358.jpg",0);
    Mat con = ~conx;
    Mat dist; 
    Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT,Size(50,50));
    morphologyEx(con,dist,cv::MORPH_CLOSE,kernel);
    Mat xx;
    threshold(con,xx,100,255,THRESH_BINARY);
     Mat mat_for_show = Mat(con.size(),CV_8UC3,Scalar(0));
    	std::vector<cv::Point> vec_points_max_poly;
	getContour(xx, mat_for_show, vec_points_max_poly); 
    Point point;
    find_start_point(conx,mat_for_show,&point);
    circle(mat_for_show,point,10,Scalar(255),-1);
    imshowResize("contour",mat_for_show);
    
    waitKey(0);*/
    
    
    
     /*Mat mat_projection_jpg = imread("kuoluo/37.bmp",0);
     Mat dist; 
     Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT,Size(20,20));
     morphologyEx(mat_projection_jpg,dist,cv::MORPH_CLOSE,kernel);
     imshowResize("original",mat_projection_jpg);
     imshowResize("dist",dist);
     waitKey(0);

      Mat mat_pro ;
      threshold(dist,mat_pro,100,255,THRESH_BINARY);
      Mat mat_for_show = Mat(mat_projection_jpg.size(),CV_8UC3,Scalar(0));
     getContour(mat_pro,mat_for_show);
     imshowResize("mat_projection_gray",mat_projection_jpg);
     waitKey(0);*/

    
  for(int i = 4;i<=40;i++){
      
    //int i = 5 ;
    Mat srcImage = imread("kuoluo/"+to_string(i)+"_texture.bmp",0);
    Mat mask = imread("kuoluo/"+to_string(i)+".bmp",0);
    threshold(mask,mask,100,255,THRESH_BINARY);
    Mat roi;
    srcImage.copyTo(roi,mask);
    
    imshowResize("the "+to_string(i)+" img",roi);
    waitKey(0);
   
    vector<int> params;
    params.push_back(round(width));
    params.push_back(round(height));  
    params.push_back(m);
    params.push_back(n);
    params.push_back(round(assume_radius));
    
    int t = clock();
    segmentation_roi(roi,mask,FEATURE_USE_CIRCLE,USE_DFS,params);
    int t2 = clock();
   cout <<"time "<< (t2-t)/CLOCKS_PER_SEC<<endl;
    
    }
   
    return 0;
}

//void segmentation_roi(Mat& img, int feature, int method, vector<float> params){
    
    
//}

void saveMask(int event,int x,int y,int flags, void* img){
    switch(event) {
        case EVENT_LBUTTONUP:
        {
            cout <<"mask position "<<x<<" "<<y<<endl;
            mask_vertex.push_back(cv::Point(x,y));
        }
        
    }
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

/*double angle( Point pt1, Point pt2, Point pt0 )
{    
    double dx1 = pt1.x - pt0.x; 
    double dy1 = pt1.y - pt0.y;  
    double dx2 = pt2.x - pt0.x;  
    double dy2 = pt2.y - pt0.y;    
    double angle_line = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);//余弦值
    return acos(angle_line)*180/3.141592653; 
}


void getContour(Mat& input, Mat& mat_for_show){
     Mat mat_projection_binary = input;
    // Mat mat_for_show = Mat(input.size(),CV_8UC3,Scalar(0));
    // Mat matforshow = mat_projection_jpg;
		std::vector<cv::Vec4i> vec_hierarchy_mask;
		std::vector<std::vector<cv::Point>> vec_vec_contours_mask;
		cv::findContours(mat_projection_binary, vec_vec_contours_mask, vec_hierarchy_mask, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		//提取轮廓的最小外接多边形
		std::vector<std::vector<cv::Point>> vec_points_polys_phoxi(vec_vec_contours_mask.size());
		std::multimap<int, int, std::greater<int>> multimap_polys_points; //按轮廓尺寸对poly进行排序
		int n_poly_approx_scale = 30;
		for (int i = 0; i < vec_vec_contours_mask.size(); i++)
		{
            RNG rng;
			cv::approxPolyDP(cv::Mat(vec_vec_contours_mask[i]), vec_points_polys_phoxi[i], n_poly_approx_scale, true);
			//drawContours(mat_for_show, vec_vec_contours_mask, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point());
             Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			//cv::drawContours(mat_for_show, vec_points_polys_phoxi, i, color, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
			multimap_polys_points.insert(std::make_pair(vec_vec_contours_mask[i].size(), i));
		}
		std::vector<cv::Point> vec_points_max_poly;
		std::multimap<int, int, std::greater<int>>::iterator iter_multimap_polys_points = multimap_polys_points.begin();
		vec_points_max_poly = vec_points_polys_phoxi[iter_multimap_polys_points->second];
        polylines(mat_for_show,vec_points_max_poly,true,Scalar(255,255,255));
        
         
         vector<double> angles;
        
      angles.push_back(abs(angle(vec_points_max_poly.back(),vec_points_max_poly.at(1),vec_points_max_poly.at(0))-90));
      
        
        int num = vec_points_max_poly.size();
        vec_points_max_poly.push_back(vec_points_max_poly.at(0));
        for(int i = 1; i<num;i++){
            double a = angle(vec_points_max_poly.at(i-1),vec_points_max_poly.at(i+1),vec_points_max_poly.at(i));
            angles.push_back(abs(a-90));
            cout <<"angle"<<a<<endl;
        }
        
        for(int i = 0;i<num;i++){
            double a =  angles.at(i);
            if(a<10){
                circle(mat_for_show,vec_points_max_poly.at(i),10,Scalar(255),-1);
            }
            cout<<"angles"<< angles.at(i)<<endl;
        }
        
        //std::vector<double>::iterator biggest = std::min_element(std::begin(angles), std::end(angles));
   // std::cout << "Max element is " << *biggest<< " at position " << std::distance(std::begin(angles), biggest) << std::endl;
        
       // drawContours(mat_for_show, vec_points_max_poly,0, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point());
        imshowResize("contour",mat_for_show);
        waitKey(0);
}*/



