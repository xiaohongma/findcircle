#ifndef CONTOURS
#define CONTOURS


#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
void getPolygon(Mat& input, Mat& mat_for_show, std::vector<cv::Point>& vec_points_max_poly);

std::vector<int> get_recommended_point_index_to_dfs(
	const std::vector<cv::Point> &vec_point_poly_,
	float n_rectangle_max_len_ = 300,
	float f_absolute_error_angle_ = 5.0, // 正常角度范围
	float f_relatvie_error_angle_ = 2.0 // 非正常最大角度范围，std::abs(f_angle - 90) / f_absolute_error_angle_
	);

double ComputePolygonArea(const vector<Point> &points);

#endif
