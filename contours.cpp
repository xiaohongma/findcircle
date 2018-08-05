#include <iostream>
#include <opencv2/opencv.hpp>
#include "polygon_calculation.hpp"
#include "contours.h"
#include "utils.h"
using namespace cv;
using namespace std;
void getContour(Mat& input, Mat& mat_for_show, std::vector<cv::Point>& vec_points_max_poly){
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
		//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//cv::drawContours(mat_for_show, vec_points_polys_phoxi, i, color, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
		//multimap_polys_points.insert(std::make_pair(vec_vec_contours_mask[i].size(), i));
        multimap_polys_points.insert(std::make_pair(ComputePolygonArea(vec_vec_contours_mask[i]), i));
        cout<<"size"<<ComputePolygonArea(vec_vec_contours_mask[i])<<endl;
      //  imshowResize("contourii", mat_for_show);
     //   waitKey(0);
    

	}
	
	std::multimap<int, int, std::greater<int>>::iterator iter_multimap_polys_points = multimap_polys_points.begin();
	vec_points_max_poly = vec_points_polys_phoxi[iter_multimap_polys_points->second];
	polylines(mat_for_show, vec_points_max_poly, true, Scalar(255, 255, 255));

	//imshowResize("contour", mat_for_show);
	//waitKey(0);
}



// 得到推荐点的索引
std::vector<int> get_recommended_point_index_to_dfs(
	const std::vector<cv::Point> &vec_point_poly_,
	float n_rectangle_max_len_,
	float f_absolute_error_angle_, // 正常角度范围
	float f_relatvie_error_angle_ // 非正常最大角度范围，std::abs(f_angle - 90) / f_absolute_error_angle_
	)
{
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


double ComputePolygonArea(const vector<Point> &points)
{
    int point_num = points.size();
    if(point_num < 3)return 0.0;
    double s = points[0].y * (points[point_num-1].x - points[1].x);
    for(int i = 1; i < point_num; ++i)
        s += points[i].y * (points[i-1].x - points[(i+1)%point_num].x);
    return fabs(s/2.0);
}


