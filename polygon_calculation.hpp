#ifndef POLYGON
#define POLYGON

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#define NULL_POINT cv::Point(-1, -1);

// 得到多边形的下一顶点
inline int get_index_next(const std::vector<cv::Point> & vec_point_poly_, int n_index_)
{
	return (n_index_ + 1) % vec_point_poly_.size();
}

// 得到多边形的上一顶点
inline int get_index_pre(const std::vector<cv::Point> & vec_point_poly_, int n_index_)
{
	return (vec_point_poly_.size() + n_index_ - 1) % vec_point_poly_.size();
}

// 多边形顶点指向下一顶点的向量
inline cv::Vec2i get_vec2i_next(const std::vector<cv::Point> & vec_point_poly_, int n_index_)
{
	int n_index_next = (n_index_ + 1) % vec_point_poly_.size();
	return cv::Vec2i(vec_point_poly_[n_index_next] - vec_point_poly_[n_index_]);
}

// 得到多边形顶点指向上一顶点的向量
inline cv::Vec2i get_vec2i_pre(const std::vector<cv::Point> & vec_point_poly_, int n_index_)
{
	int n_index_pre = (vec_point_poly_.size() + n_index_ - 1) % vec_point_poly_.size();
	return cv::Vec2i(vec_point_poly_[n_index_pre] - vec_point_poly_[n_index_]);
}

// 得到叉乘
inline int get_cross_product(const cv::Vec2i &vec2i_1_, const cv::Vec2i &vec2i_2_)
{
	return vec2i_1_[0] * vec2i_2_[1] - vec2i_1_[1] * vec2i_2_[0];
}

// 得到叉乘
inline float get_cross_product(const cv::Vec2f &vec2f_1_, const cv::Vec2f &vec2f_2_)
{
	return vec2f_1_[0] * vec2f_2_[1] - vec2f_1_[1] * vec2f_2_[0];
}


// 得到两个向量夹角，0-180
inline float get_angle(const cv::Vec2f &vec2f_1_, const cv::Vec2f &vec2f_2_)
{
	return std::acos(vec2f_2_.dot(vec2f_1_) / std::sqrt(vec2f_1_.dot(vec2f_1_) * vec2f_2_.dot(vec2f_2_))) * 180 / 3.14159265;
}

// 得到多边形某一顶点的夹角，0-180
inline float get_angle(const std::vector<cv::Point> & vec_point_poly_, int n_index_)
{
	cv::Vec2f vec2f_1 = get_vec2i_pre(vec_point_poly_, n_index_);
	cv::Vec2f vec2f_2 = get_vec2i_next(vec_point_poly_, n_index_);
	return std::acos(vec2f_2.dot(vec2f_1) / std::sqrt(vec2f_1.dot(vec2f_1) * vec2f_2.dot(vec2f_2))) * 180 / 3.14159265;
}



inline bool is_convex_corner(const cv::Vec2i &vec2i_1_, const cv::Vec2i &vec2i_2_, bool b_is_counterclockwise_ = true)
{
	int n_cross_product = get_cross_product(vec2i_1_, vec2i_2_);
	return n_cross_product == 0 || !((n_cross_product > 0) ^ b_is_counterclockwise_);
}

// 判断多边形某顶点的凹凸性，默认要求多边形内部的存储顺序为逆时针排列
inline bool is_convex_corner(const std::vector<cv::Point> & vec_point_poly_, int n_index_, bool b_is_counterclockwise_ = true)
{
	auto vec2i_1 = -get_vec2i_pre(vec_point_poly_, n_index_);
	auto vec2i_2 = get_vec2i_next(vec_point_poly_, n_index_);

	return is_convex_corner(vec2i_1, vec2i_2, b_is_counterclockwise_);
}


// 判断多边形内部的存储顺序，是否为逆时针排列
inline bool is_counterclockwise(std::vector<cv::Point> & vec_point_poly_)
{
	if (vec_point_poly_.size() < 3) return true;

	int n_index = 0;
	for (size_t i = 1; i < vec_point_poly_.size(); ++i)
	{
		if (vec_point_poly_[i].x < vec_point_poly_[n_index].x
			|| (vec_point_poly_[i].x == vec_point_poly_[n_index].x && vec_point_poly_[i].y < vec_point_poly_[n_index].y)
			)
		{
			n_index = i;
		}
	}
	// n_index对应的点一定是凸角
	return is_convex_corner(vec_point_poly_, n_index);
}


// 线段是否相交，不含端点
inline bool is_segment_cross(cv::Point point_1_, cv::Point point_2_, cv::Point point_3_, cv::Point point_4_)
{
	float f_area_abc = (point_1_.x - point_3_.x) * (point_2_.y - point_3_.y) - (point_1_.y - point_3_.y) * (point_2_.x - point_3_.x);
	float f_area_abd = (point_1_.x - point_4_.x) * (point_2_.y - point_4_.y) - (point_1_.y - point_4_.y) * (point_2_.x - point_4_.x);
	if (f_area_abc * f_area_abd >= 0)
	{
		return false;
	}
	float f_area_cda = (point_3_.x - point_1_.x) * (point_4_.y - point_1_.y) - (point_3_.y - point_1_.y) * (point_4_.x - point_1_.x);
	float f_area_cdb = f_area_cda + f_area_abc - f_area_abd;
	if (f_area_cda * f_area_cdb >= 0)
	{
		return false;
	}
	return true;
}

// 得到两线段交点；不相交，则返回 NULL_POINT
inline cv::Point get_point_segment_cross(cv::Point point_1_, cv::Point point_2_, cv::Point point_3_, cv::Point point_4_)
{

	// 三角形abc 面积的2倍  
	float f_area_abc = (point_1_.x - point_3_.x) * (point_2_.y - point_3_.y) - (point_1_.y - point_3_.y) * (point_2_.x - point_3_.x);
	// 三角形abd 面积的2倍  
	float f_area_abd = (point_1_.x - point_4_.x) * (point_2_.y - point_4_.y) - (point_1_.y - point_4_.y) * (point_2_.x - point_4_.x);
	// 面积符号相同则两点在线段同侧,不相交 (对点在线段上的情况,本例当作不相交处理);  
	if (f_area_abc*f_area_abd >= 0)
	{
		return NULL_POINT;
	}

	// 三角形cda 面积的2倍  
	float f_area_cda = (point_3_.x - point_1_.x) * (point_4_.y - point_1_.y) - (point_3_.y - point_1_.y) * (point_4_.x - point_1_.x);
	// 三角形cdb 面积的2倍  
	float f_area_cdb = f_area_cda + f_area_abc - f_area_abd;
	// 面积符号相同则两点在线段同侧,不相交 (对点在线段上的情况,本例当作不相交处理);  
	if (f_area_cda * f_area_cdb >= 0)
	{
		return NULL_POINT;
	}

	//计算交点坐标  
	float f_t = f_area_cda / (f_area_abd - f_area_abc);
	float f_dx = f_t * (point_2_.x - point_1_.x);
	float f_dy = f_t * (point_2_.y - point_1_.y);
	return cv::Point(point_1_.x + f_dx, point_1_.y + f_dy);
}

// 点斜式求两直线交点
inline cv::Point get_point_line_cross(cv::Vec2f vec2f_1_, cv::Point2f point_1, cv::Vec2f vec2f_2_, cv::Point2f point_2)
{
	if (point_1 == point_2) return point_1;
	float det = get_cross_product(vec2f_1_, vec2f_2_);

	if (det < 0.000001 && det > -0.000001) return NULL_POINT;
	cv::Vec2f v3 = point_2 - point_1;

	float t1 = get_cross_product(v3, vec2f_2_) / det;
	return point_1 + cv::Point2f(vec2f_1_ * t1);
}

// 两点间距离
inline float get_distance(cv::Point point_1_, cv::Point point_2_)
{
	return std::sqrt((point_1_.x - point_2_.x) * (point_1_.x - point_2_.x) + (point_1_.y - point_2_.y) * (point_1_.y - point_2_.y));
}

// 得到向量的模长
inline float get_module(cv::Vec2i v)
{
	return std::sqrt(v.dot(v));
}

#endif
