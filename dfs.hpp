#ifndef DfsForDeconstruction_HPP
#define DfsForDeconstruction_HPP
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
# include <opencv2/opencv.hpp>
#include "utils.h"
#include "polygon_calculation.hpp"
# include "Feature.hpp"
#include "feature_circle.hpp"


 class DfsForDeconstruction{
     // 
    private:
        cv::Mat visited;
        cv::Mat img;
        cv::Mat mask;
        std::vector<int> params;
       std::vector<cv::Vec2i> direction;
       std::vector<StepInfo> path;
public: 
     DfsForDeconstruction( cv::Mat& img, cv::Mat& mask, std::vector<int>& params){
     this->img = img;
     this->mask = mask;
     this->params = params;
     this->visited = ~mask;// visited map: used in dfs. It is reverse of mask
    int width =  this->params.at(0);
    int height =  this->params.at(1);
    this->direction.push_back(cv::Vec2i(width, height));
    this->direction.push_back(cv::Vec2i(height, width));
    //direction ={{width,height},{height,width},{-width,height},{width,-height},{-width,-height},{-height,width},{height,-width},{-height,-width}};
    }

 public:
    void dfs(Feature& feature, std::vector<PathInfo>& allpaths){
        
        if (isEnd()) {
            PathInfo pathInfo;
            pathInfo.path=path;
            pathInfo.tag = 0000;//unused tag
            allpaths.push_back(pathInfo);
            return;
        }

        std::vector<cv::Point> key_pts;
        // find key_pts in polygon
        find_start_point(key_pts);
    
        if(key_pts.empty()) return;

        for (int i = 0; i<direction.size(); i++) {
            float score=0;
            std::vector<cv::Point> visit_points;
            cv::RotatedRect final_r_rect;
            extendAdjustRect(visited, key_pts,direction[i],feature,&score,visit_points,final_r_rect);
            //如果周围可以有邻接矩形
            if(isContinue(score)) {
                StepInfo step;
                //circle(img,point,5,Scalar(255),-1);
                // set the m*n rect into flase
                setVisited(visit_points,255);
                // cout <<"visited direction" <<i<<" visited bl"<<point<<endl;
                step.direction = i;
                step.r_rect =final_r_rect;
                step.score = score;
                path.push_back(step);
                dfs(feature, allpaths);
                setVisited(visit_points,0);
                path.pop_back();
            }

        }
  
    }

 public:
     
     /**
 * this function is used to check whether there are rectangles can be extented.
 @param visited is a map record the visiting status.
 @param feature indicates whether or not we use each feature, it is stored in fixed order:circle,contour,texture
 @param param stores prior params we have known. it is also in fixed order.first two is the width and height of the box, the third and forth are the numbers of cans along each side. And the fifth is stored radius. 
 @param direction is the search direction
 @param basePoint is the bottom left point of probably existing rectangle
 @param img roi image, in case we need breakpoint or debug
 @param score return the score in this step.
 */

void extendAdjustRect( cv::Mat& visited, std::vector<cv::Point>& key_pts,cv::Vec2i direct,Feature& feature, float* score, std::vector< cv::Point >& visit_points, cv::RotatedRect& bounding_r_rect){
    // identify which class the feature belong to(CircleFeature,Contour Feature,TextureFeature)
    if(typeid(*(feature.GetClassType())) == typeid(CircleFeature)){
        CircleFeature* cf = dynamic_cast<CircleFeature*>(&feature);
         cv::waitKey(30000);
        cf->extend_by_circle(visited, key_pts,direct,score,visit_points,bounding_r_rect);
        //extend_by_circle(key_pts,direct,units,score,visit_points,bounding_r_rect);
    }else if(1){
        // ContourFeature
    }else if(1){
        //TextureFeature
    }
    
}
 
 private :
     /**
 this function used to determine when we have visited whole mask. 
 @param visited nothing to say.
 @param path, allpath, same with the above
 */
bool isEnd()
{

   // bool isend = false;
    /*int m = params.at(2);
    int n = params.at(3);
    int count = 0;
    
    for(int i = 0;i<units.size();i++){
        if(visited.at<uchar>(units.at(i))==0) count++;
    }

    if(count<m*n*0.5){
        std::cout<<"remaining units "<<count << std::endl;
        isend = true;
    } */
    
    //return isend;
    cv::Mat mask = ~visited;
	std::vector<std::vector<cv::Point>> vec_vec_contours_mask;
    findContours(mask, vec_vec_contours_mask, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(vec_vec_contours_mask.empty()){
        std::cout << "vitsited all points! " << std::endl;
        return true;
    }
    
    int n__connected_area = 0;
    int unit = params.at(PARAM_HEIGHT_LOC)*params.at(PARAMS::PARAM_WIDTH_LOC);// area of one box
    for(int i =0;i<vec_vec_contours_mask.size();i++){
        std::vector<cv::Point> ctr = vec_vec_contours_mask.at(i);
        n__connected_area += contourArea(ctr);
    }
    std::cout << "visited map remaining area "<<n__connected_area/(unit)<<std::endl;
    if(n__connected_area<unit){
        std::cout << "finishedddddddddddddddddddddddddddddddddddd visit"<<std::endl;
        return true;
    }
    
    return false;

}


 private :
     /**
 * this is used to set visited or unvisited of centers in the extended rectangle.
 @param visited visited map record visiting status, same size with  roi image
 @param point bottom left point, same with the input in extendAdjustRect()
 @param img, direction  the same with corresponding params in function extendAdjustRect()
 @param value uchar value(0 or 255) is to be written into center point
 */

void setVisited(std::vector<cv::Point>& visit_points,uchar value){
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


 private:
/**
* @brief this function is used to find the start point to segment. First we extract do mophology process in the mask( open), then we get the contours of mask. There are still many small contours, we just fill them and reserve the main contours. Then, we use mophology close operator to enlarge the reserved contour.Notice that the struct size is much smaller in open operator than in close operator. Then, we use getPolygon function to get polygons of corresponding contours and return the biggest polygon. The best start point in the polygon should consider following conditions: 1. the angle is near 90. 2. is the convex point. 3. both of the two side have proper length.
* 
* @param visited p_visited:...
* @param img p_img:...
* @param params p_params:...
* @param key_pts p_key_pts:...
*/
void find_start_point(std::vector<cv::Point>& key_pts){
     cv::Mat mask = ~visited;
    // open process on mask 
    cv::Mat opened_mask;
    cv::Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(params.at(PARAM_OPEN_SCALE_LOC), params.at(PARAM_OPEN_SCALE_LOC)));
	morphologyEx(mask, opened_mask, cv::MORPH_OPEN, kernel);
   // imshowResize("after open",opened_mask);
   // waitKey(0);
    //binary input img
    cv::Mat mat_pro;
	threshold(opened_mask, mat_pro, 100, 255, cv::THRESH_BINARY);
    // find contours
    std::vector<cv::Vec4i> vec_hierarchy_mask;
	std::vector<std::vector<cv::Point>> vec_vec_contours_mask;
    cv::findContours(mat_pro, vec_vec_contours_mask, vec_hierarchy_mask, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(vec_vec_contours_mask.empty()){
        std::cout << "no remaining contours " << std::endl;
        return;
    }
    
    // fill up the small contours.(the area less than 0.1*box_area)
    double thres = params.at(PARAM_HEIGHT_LOC)*params.at(PARAMS::PARAM_WIDTH_LOC)*0.1;
    int n_count_small_contour;
    for(int i =0;i<vec_vec_contours_mask.size();i++){
        std::vector<cv::Point> ctr = vec_vec_contours_mask.at(i);
        if(contourArea(ctr)<thres){
            n_count_small_contour++;
       // cout <<"Area "<<contourArea(ctr)<<endl;
        drawContours(mat_pro, vec_vec_contours_mask, i, cv::Scalar(0), -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }
    }
    //imshowResize("filled small contours",mat_pro);
   // waitKey(0);
    std::cout<<" find "<<vec_vec_contours_mask.size()<<" contours"<<std::endl;
    if(vec_vec_contours_mask.size() == n_count_small_contour){
        std::cout << "no remaining big contours" << std::endl;
        return;
    }

    //close the mask map 
    cv::Mat closed_mask;
    kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(params.at(PARAM_CLOSE_SCALE_LOC), params.at(PARAM_CLOSE_SCALE_LOC)));
	morphologyEx(mat_pro, closed_mask, cv::MORPH_CLOSE, kernel);
    //imshowResize("after close",closed_mask);
    //waitKey(0);
    //Mat mat_for_show = Mat(img.size(), CV_8UC3, Scalar(0));
    cv::Mat mat_for_show = img.clone();
    // return the biggest polygon
	std::vector<cv::Point> vec_points_max_poly;
	getPolygon(closed_mask, mat_for_show,params, vec_points_max_poly);

	// find the best point and its two neighbor points in polygon
	if (!is_counterclockwise(vec_points_max_poly)) 
	{
		std::reverse(vec_points_max_poly.begin(), vec_points_max_poly.end());
	}
	int max_len = round(1.2*std::max(params.at(PARAM_WIDTH_LOC),params.at(PARAM_HEIGHT_LOC)));
	auto vec_n_index = get_recommended_point_index_to_dfs(vec_points_max_poly,max_len);
    if(vec_n_index.empty()) 
    {
        std::cout<<"no recommendation point"<<std::endl;
        return;
        
    }
    cv::Point best_point = vec_points_max_poly[vec_n_index.at(0)];
    int next_index = (vec_n_index.at(0)<(vec_points_max_poly.size()-1))?(vec_n_index.at(0)+1):0;
    int pre_index = (vec_n_index.at(0)>0)?(vec_n_index.at(0)-1):(vec_points_max_poly.size()-1);
    cv::Point pre_point = vec_points_max_poly.at(pre_index);
    cv::Point next_point = vec_points_max_poly.at(next_index);
    key_pts.push_back(best_point);
    key_pts.push_back(pre_point);
    key_pts.push_back(next_point);
    /*circle(mat_for_show,best_point,10,cv::Scalar(255),-1);
    imshowResize("polygon",mat_for_show);
    cv::waitKey(0);*/
    }
    
 private :
    /**
* @brief decide whether or not we continue to visit
* 
* @param score p_score:
* @return bool 
*/
bool isContinue(float score){
      
   // sort(score.begin(),score.end());//the maximun of score
    if(score>=0.8){
        return true;
    } 
    return false;
}

 private:

void getPolygon(cv::Mat& input, cv::Mat& mat_for_show, std::vector< int >& params, std::vector< cv::Point >& vec_points_max_poly){
    cv::Mat mat_projection_binary = input;
	// Mat mat_for_show = Mat(input.size(),CV_8UC3,Scalar(0));
	// Mat matforshow = mat_projection_jpg;
	std::vector<cv::Vec4i> vec_hierarchy_mask;
	std::vector<std::vector<cv::Point>> vec_vec_contours_mask;
	cv::findContours(mat_projection_binary, vec_vec_contours_mask, vec_hierarchy_mask, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(vec_hierarchy_mask.empty()) {
        std:: cout<<"no remaining contours"<<std::endl;
       return;   
    }
	//提取轮廓的最小外接多边形
	std::vector<std::vector<cv::Point>> vec_points_polys_phoxi(vec_vec_contours_mask.size());
   
	std::multimap<int, int, std::greater<int>> multimap_polys_points; //按轮廓尺寸对poly进行排序
	int n_poly_approx_scale=params.at(PARAM_POLY_APPROX_SCALE_LOC);
	for (int i = 0; i < vec_vec_contours_mask.size(); i++)
	{
		cv::RNG rng;
		cv::approxPolyDP(cv::Mat(vec_vec_contours_mask[i]), vec_points_polys_phoxi[i], n_poly_approx_scale, true);
		//drawContours(mat_for_show, vec_vec_contours_mask, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point());
		//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//cv::drawContours(mat_for_show, vec_points_polys_phoxi, i, color, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
		//multimap_polys_points.insert(std::make_pair(vec_vec_contours_mask[i].size(), i));
        multimap_polys_points.insert(std::make_pair(contourArea(vec_vec_contours_mask[i]), i));
        std::cout<<"size"<<contourArea(vec_vec_contours_mask[i])<<std::endl;
      //  imshowResize("contourii", mat_for_show);
     //   waitKey(0);
    

	}
	
	std::multimap<int, int, std::greater<int>>::iterator iter_multimap_polys_points = multimap_polys_points.begin();
	vec_points_max_poly = vec_points_polys_phoxi[iter_multimap_polys_points->second];
	polylines(mat_for_show, vec_points_max_poly, true, cv::Scalar(255, 255, 255));

	//imshowResize("contour", mat_for_show);
	//waitKey(0);
}

 private:
     
std::vector<int> get_recommended_point_index_to_dfs(
	const std::vector<cv::Point> &vec_point_poly_,
	float n_rectangle_max_len_,
	float f_absolute_error_angle_ = 10.0, // 正常角度范围
	float f_relatvie_error_angle_ = 2.0 // 非正常最大角度范围，std::abs(f_angle - 90) / f_absolute_error_angle_
	){
        std::vector<float> vec_f_score(vec_point_poly_.size(), 0);
	std::vector<int> vec_n_index(vec_point_poly_.size());

	for (int n_index = 0; n_index < vec_point_poly_.size(); ++n_index)
	{
		vec_n_index[n_index] = n_index;
		if (is_convex_corner(vec_point_poly_, n_index))
		{
			float f_angle = get_angle(vec_point_poly_, n_index);
			float f_angle_diff_rate = std::abs(f_angle - 90) / f_absolute_error_angle_;
			if (f_angle_diff_rate < f_relatvie_error_angle_)
			{
				cv::Vec2f vec2f_pre = get_vec2i_pre(vec_point_poly_, n_index);
				float f_vec2f_pre_module = std::sqrt(vec2f_pre.dot(vec2f_pre));
				cv::Vec2f vec2f_next = get_vec2i_next(vec_point_poly_, n_index);
				float f_vec2f_next_module = std::sqrt(vec2f_next.dot(vec2f_next));

				vec_f_score[n_index] =
					1.0 
					* (0.15f + std::min(f_relatvie_error_angle_ - 0.4f, f_relatvie_error_angle_ - f_angle_diff_rate) / f_relatvie_error_angle_) // 角度
					* std::min(f_vec2f_pre_module, n_rectangle_max_len_) / n_rectangle_max_len_ // 边长
					* std::min(f_vec2f_next_module, n_rectangle_max_len_) / n_rectangle_max_len_ // 边长
					;
			}
		}
	}

	vec_n_index.erase(
		std::remove_if(
		vec_n_index.begin(),
		vec_n_index.end(),
		[&vec_f_score](int x) {return vec_f_score[x] <= 0; }
		),
		vec_n_index.end()
		);
	std::sort(
		vec_n_index.begin(),
		vec_n_index.end(),
		[&vec_f_score](int x, int y) {return vec_f_score[x] > vec_f_score[y]; }
	);
	return vec_n_index; // 移动语义
    }



 };

#endif





