
#include "utils.h"
#include "feature_contour.h"
#include "contours.h"
#include "dfs.h"
#include "polygon_calculation.hpp"
using namespace std;




// decide whether or not to continue dfs, waiting for improvement
bool isContinue(float score){
    
   // sort(score.begin(),score.end());//the maximun of score
    if(score>=0.8){
        return true;
    } 
    return false;
}





void setVisited(cv::Mat& visited, cv::Mat& img,vector<Point>& visit_points,uchar value)
{
    if(visit_points.empty()) return;
    int num = visit_points.size();
    for(int i = 0;i<num;i++){
        visited.at<uchar>(visit_points.at(i))=value;
    }
     
    
    // visited(bounding) = value;
     
    //rectangle(img,bounding,Scalar(value,0,0),1,16);
   
    //imshowResize("xx",img);
    //imshowResize("visited",visited);
   // waitKey(0);
    
}


void find_start_point(cv::Mat& visited, cv::Mat& img,std::vector<int>& params,vector<Point>& key_pts){
   
    Mat mask = ~visited;
    // open process on mask 
    Mat opened_mask;
    Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(params.at(PARAM_OPEN_SCALE_LOC), params.at(PARAM_OPEN_SCALE_LOC)));
	morphologyEx(mask, opened_mask, cv::MORPH_OPEN, kernel);
   // imshowResize("after open",opened_mask);
   // waitKey(0);
    //binary input img
    Mat mat_pro;
	threshold(opened_mask, mat_pro, 100, 255, THRESH_BINARY);
    // find contours
    std::vector<cv::Vec4i> vec_hierarchy_mask;
	std::vector<std::vector<cv::Point>> vec_vec_contours_mask;
    cv::findContours(mat_pro, vec_vec_contours_mask, vec_hierarchy_mask, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // fill up the small contours.(the area less than 0.1*box_area)
    double thres = params.at(PARAM_HEIGHT_LOC)*params.at(PARAMS::PARAM_WIDTH_LOC)*0.1;
    for(int i =0;i<vec_vec_contours_mask.size();i++){
        vector<Point> ctr = vec_vec_contours_mask.at(i);
        if(contourArea(ctr)<thres){
       // cout <<"Area "<<contourArea(ctr)<<endl;
        drawContours(mat_pro, vec_vec_contours_mask, i, Scalar(0), -1, 8, vector<Vec4i>(), 0, Point());
        }
    }
    //imshowResize("filled small contours",mat_pro);
   // waitKey(0);
    cout<<" find "<<vec_vec_contours_mask.size()<<" contours"<<endl;

    //close the mask map 
    Mat closed_mask;
    kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(params.at(PARAM_CLOSE_SCALE_LOC), params.at(PARAM_CLOSE_SCALE_LOC)));
	morphologyEx(mat_pro, closed_mask, cv::MORPH_CLOSE, kernel);
    //imshowResize("after close",closed_mask);
    //waitKey(0);
    //Mat mat_for_show = Mat(img.size(), CV_8UC3, Scalar(0));
    Mat mat_for_show = img.clone();
    // return the biggest polygon
	std::vector<cv::Point> vec_points_max_poly;
	getPolygon(closed_mask, mat_for_show,params, vec_points_max_poly);

	// find the best point and its two neighbor points in polygon
	if (!is_counterclockwise(vec_points_max_poly)) 
	{
		std::reverse(vec_points_max_poly.begin(), vec_points_max_poly.end());
	}
	int max_len = round(1.2*max(params.at(PARAM_WIDTH_LOC),params.at(PARAM_HEIGHT_LOC)));
	auto vec_n_index = get_recommended_point_index_to_dfs(vec_points_max_poly,max_len);
    if(vec_n_index.empty()) 
    {
        cout<<"no recommendation point"<<endl;
        return;
        
    }
    Point best_point = vec_points_max_poly[vec_n_index.at(0)];
    int next_index = (vec_n_index.at(0)<(vec_points_max_poly.size()-1))?(vec_n_index.at(0)+1):0;
    int pre_index = (vec_n_index.at(0)>0)?(vec_n_index.at(0)-1):(vec_points_max_poly.size()-1);
    Point pre_point = vec_points_max_poly.at(pre_index);
    Point next_point = vec_points_max_poly.at(next_index);
    key_pts.push_back(best_point);
    key_pts.push_back(pre_point);
    key_pts.push_back(next_point);
    
    /*circle(mat_for_show,best_point,10,Scalar(255),-1);
    imshowResize("polygon",mat_for_show);
    waitKey(0);*/

}

void get_rotated_rect(Mat& img,vector<Point> key_pts, int direction[],RotatedRect& ro_rect){
    int width = direction[0];
    int height = direction[1];
    
    Vec2f v1(key_pts.at(1).x-key_pts.at(0).x,key_pts.at(1).y-key_pts.at(0).y);
    Vec2f v2(key_pts.at(2).x-key_pts.at(0).x,key_pts.at(2).y-key_pts.at(0).y);
    
     float angle1 = acos(v1.dot(Vec2f(1,0))/(get_module(v1)))*180/CV_PI*sng(get_cross_product(Vec2f(1,0),v1));
     float angle2 = acos(v2.dot(Vec2f(1,0))/(get_module(v2)))*180/CV_PI*sng(get_cross_product(Vec2f(1,0),v2));

    cout<<"angle1 "<<angle1<<" angle2 "<<angle2<<endl;
   
   float angle12 = acos(v1.dot(v2)/(get_module(v1)*get_module(v2)))*180/CV_PI;
   float bias = 0.5*(angle12-90)*sng(get_cross_product(v1,v2));
    cout <<"bias "<< bias << endl;
   angle1 = angle1+bias;
   angle2 = angle2-bias;
    v1 = v1*width/sqrt(v1.dot(v1));
    v2 = v2*height/sqrt(v2.dot(v2));
    Point center (0.5*(v1+v2)(0)+key_pts.at(0).x,0.5*(v1+v2)(1)+key_pts.at(0).y);
   // RotatedRect rr(center,Size2f(width,height),angle1);
    RotatedRect rr(center,Size2f(width,height),angle1);
    cout<<"after bais angle1 "<<angle1<<" angle2 "<<angle2<<endl;
    cout <<"rotated angle"<<rr.angle<<" width "<<rr.size.width<<endl;
    ro_rect = rr;
    
   /* //Mat mat_for_show = img.clone();
   Mat mat_for_show = Mat(img.size(), CV_8UC3, Scalar(0));
  //  circle(mat_for_show,center,10,Scalar(255));
    DrawRotatedRect(mat_for_show,rr,Scalar(255));
    line(mat_for_show,key_pts[0],key_pts[1],Scalar(255));
    line(mat_for_show,key_pts[0],key_pts[2],Scalar(255));
//
     
    cout<<"best points numbe"<<key_pts.at(0)<<endl;
    cv::circle(mat_for_show,key_pts.at(0), 10, cv::Scalar(255, 125, 125), -1); 
    cv::circle(mat_for_show,key_pts.at(1), 10, cv::Scalar(255, 125, 125), -1); 
    cv::circle(mat_for_show,key_pts.at(2), 10, cv::Scalar(255, 125, 125), -1); 
    putText(mat_for_show,to_string(1),key_pts.at(1),FONT_HERSHEY_SCRIPT_SIMPLEX,4,Scalar(255),4);
    imshowResize("key_pts",mat_for_show);
    waitKey(0);*/
}



