// cv_test.cpp : Defines the entry point for the console application.

#include "polygon_calculation.hpp"
#include "utils.h"
#include "contours.h"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{


	Mat mat_projection_jpg = imread("contour_25.jpg", 0);
    mat_projection_jpg = ~mat_projection_jpg;
	Mat dist;
	Mat kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(7, 7));
	morphologyEx(mat_projection_jpg, dist, cv::MORPH_OPEN, kernel);
	kernel = getStructuringElement(cv::MorphShapes::MORPH_RECT, Size(21, 21));
	morphologyEx(dist, dist, cv::MORPH_CLOSE, kernel);
	imshowResize("original", mat_projection_jpg);
	waitKey(1);
	imshowResize("dist", dist);
	waitKey(1);

	Mat mat_pro;
	threshold(dist, mat_pro, 100, 255, THRESH_BINARY);
	Mat mat_for_show = Mat(mat_projection_jpg.size(), CV_8UC3, Scalar(0));
	std::vector<cv::Point> vec_points_max_poly;
	getContour(mat_pro, mat_for_show, vec_points_max_poly);
	waitKey(1);

	// ≤‚ ‘
	if (!is_counterclockwise(vec_points_max_poly)) 
	{
		std::reverse(vec_points_max_poly.begin(), vec_points_max_poly.end());
	}
	auto vec_n_index = get_recommended_point_index_to_dfs(vec_points_max_poly);

	for (auto n_index : vec_n_index) 
	{
		cv::circle(mat_for_show, vec_points_max_poly[n_index], 10, cv::Scalar(255, 125, 125), -1);
		cv::namedWindow("test", cv::WINDOW_FREERATIO);
		imshowResize("test", mat_for_show);
		cv::waitKey(0); // ms
	}
	cv::namedWindow("test", cv::WINDOW_FREERATIO);
	imshowResize("test", mat_for_show);
	cv::waitKey(0); // ms


	return 0;
}

