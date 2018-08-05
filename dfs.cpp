
#include "utils.h"
#include "feature_contour.h"
#include "contours.h"
#include "dfs.h"
#include "polygon_calculation.hpp"
using namespace std;


void findtl(cv::Mat& visited, cv::Mat& img,cv::Point* point)
{
    int distance = visited.rows*visited.rows+visited.cols*visited.cols;
    Point tlpoint(visited.cols,visited.rows);
    for(int y  =0; y<visited.rows;y++){
        uchar* row = visited.ptr<uchar>(y);
        for(int x = 0;x<visited.cols;x++){
            if(row[x]==0){
                int d = x*x+y*y;
                if(d<distance){
                    distance = d;
                    tlpoint = Point(x,y);
                   // break;
                }
             }
        }
    }
     *point = tlpoint;
    
}

// decide whether or not to continue dfs, waiting for improvement
bool isContinue(float score){
    
   // sort(score.begin(),score.end());//the maximun of score
    if(score>=0.8){
        return true;
    } 
    return false;
}




/*bool isEnd(cv::Mat& visited, std::vector< int > params)
{

    bool isend = false;
    int width = params.at(0);
    int height = params.at(1);
    int count = 0;
    for(int i = 0;i<visited.rows;i++){
        uchar* row = visited.ptr<uchar>(i);
        for(int j = 0;j<visited.cols;j++){
            if(row[j]==0) count++;
        }
    }
   
    if(count<width*height){
        cout<<"unvisited area"<<count << endl;
        isend = true;
    }

    return isend;
}*/



void setVisited(cv::Mat& visited, cv::Mat& img,vector<Point>& visit_points,uchar value)
{
    if(visit_points.empty()) return;
    int num = visit_points.size();
    for(int i = 0;i<num;i++){
        visited.at<uchar>(visit_points.at(i))=value;
    }
     
    /*
    int i = 0;
    while(true){
        fstream file;
        file.open("contour_"+to_string(i)+".jpg",ios_base::in);
        if(!file){
            imwrite("contour_"+to_string(i)+".jpg",visited);
            break;
        }else{
            i++;
        }
    }
    */
    



    // visited(bounding) = value;
     
    //rectangle(img,bounding,Scalar(value,0,0),1,16);
   
   // imshowResize("xx",img);
  //  imshowResize("visited",visited);
   // waitKey(0);
    
}

void intersection_rect_mask(cv::Rect& rect, cv::Mat& mask, float* ratio)
{
    int x0 = rect.tl().x;
    int y0 = rect.tl().y;
    int x_end = rect.br().x;
    int y_end = rect.br().y;
    
    int count = 0;
    for(int x = x0;x<x_end;x++){
        for(int y = y0;y<y_end;y++){
            if(mask.at<uchar>(Point(x,y))==255)
                count++;
        }
    }
    *ratio = (float)count/(rect.width*rect.height);
    cout <<"ratio" <<*ratio<<endl;
    
}

void find_start_point(cv::Mat& visited, cv::Mat& img,cv::Point* point){
   
    Mat mat_projection_jpg = ~visited;
	Mat dist;
	Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(7, 7));
	morphologyEx(mat_projection_jpg, dist, cv::MORPH_OPEN, kernel);
	kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(21, 21));
	morphologyEx(dist, dist, cv::MORPH_CLOSE, kernel);
	/*imshowResize("original", mat_projection_jpg);
	waitKey(1);
	imshowResize("dist", dist);
	waitKey(1);*/

	Mat mat_pro;
	threshold(dist, mat_pro, 100, 255, THRESH_BINARY);
	Mat mat_for_show = Mat(mat_projection_jpg.size(), CV_8UC3, Scalar(0));
	std::vector<cv::Point> vec_points_max_poly;
	getContour(mat_pro, mat_for_show, vec_points_max_poly);
	//waitKey(1);

	// 测试
	if (!is_counterclockwise(vec_points_max_poly)) 
	{
		std::reverse(vec_points_max_poly.begin(), vec_points_max_poly.end());
	}
	auto vec_n_index = get_recommended_point_index_to_dfs(vec_points_max_poly);
    if(vec_n_index.empty()) return;
    *point = vec_points_max_poly[vec_n_index.at(0)];
    cout<<"best points numbe"<<*point<<endl;
    cv::circle(mat_for_show,*point, 10, cv::Scalar(255, 125, 125), -1); 
    imshowResize("contours",mat_for_show);

	/*for (auto n_index : vec_n_index) 
	{
		cv::circle(mat_for_show, vec_points_max_poly[n_index], 10, cv::Scalar(255, 125, 125), -1);
		cv::namedWindow("test", cv::WINDOW_FREERATIO);
		imshowResize("test", mat_for_show);
		cv::waitKey(0); // ms
	}
	cv::namedWindow("test", cv::WINDOW_FREERATIO);
	imshowResize("test", mat_for_show);
	cv::waitKey(0); // ms
    */
}





