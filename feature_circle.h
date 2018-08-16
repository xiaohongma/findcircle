#ifndef FEA_CIRCLE
#define FEA_CIRCLE
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
/**
* @brief we use the circle feature to get the score of each step, now we just count circles in every patch as the score. If the score is too low, we will not visit next step. Next step, Combine with the returned score based on other features, we will get an overall score using function isContinue(vector<float> score) 
* 
* @param visited p_visited: visit map
* @param img p_img: score img
* @param direction p_direction:
* @param basePoint p_basePoint:bl point, i do not want to say
* @param params p_params: you can see the description in the reference fucntion
* @param score p_score: return score
*/
void extend_by_circle(Mat& visited, Mat& img,Mat& mask,vector<Point> key_pts, int direction[], vector<int>& params,vector<Point>& centers,float* score,vector<Point>& visit_points, RotatedRect& bounding_r_rect);
void circle_belong_rect( RotatedRect& rect, double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect);

void circle_belong_rect( Rect& rect, double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect);

void eat_padding(Rect& inrect, Rect& range_rect,Mat& visited, Rect& outrect );
//count the points on the bounding of circle
//references: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm 
void points_on_circle(Point center, int radius, vector<Point>& points);

void raster_circle(Point center,
         int radius, vector<Point>& points);
void draws_circles(Mat& img,  Mat& visited,vector<int> params,vector<Point>& centers);
void find_nearest_point(vector<Point>& centers, Point basepoint,Point* nearest_point);

void find_bounding_rect(Mat& visited,Mat& img,RotatedRect& r_rect,vector<Point>& centers,int radius,RotatedRect& bounding_rect,vector<Point>& remaining_circles);

void rotate_rect_contains( RotatedRect& rect, vector<Point>& centers,vector<bool>& belong_rect);
void count_points_in_rotated_rect(RotatedRect& rotated_rect,vector<Point>& rotated_rect_points);
void DrawRotatedRect(cv::Mat& img, cv::RotatedRect& rr, cv::Scalar color);
void rotateImage(Mat& src,RotatedRect r_rect, Mat& dst);
#endif
