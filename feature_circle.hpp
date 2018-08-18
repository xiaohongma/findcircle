#ifndef FEA_CIRCLE_HPP
#define FEA_CIRCLE_HPP
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "dfs.hpp"
#include "polygon_calculation.hpp"
#include "Feature.hpp"
class CircleFeature : public Feature {
    private:
        //cv::Mat visited;
        cv::Mat img;
        cv::Mat mask;
        std::vector<int> params;
        std::vector<cv::Vec2i> direction;
        std::vector<cv::Point> centers;
public:
     CircleFeature(cv::Mat& img,cv::Mat& mask,std::vector<int>& params){
     this->img = img;
     this->mask = mask;
     this->params = params;
     detection_circles();
        
    }

private :
    /**
* @brief detect circles in the img. first we use adaptive canny to detect the edge, then use the canny threshold in hough circle detection.In the following, we can use the mask area to constraint the circle number.
* 
* @param img p_img:...
* @param params p_params:...
* @param centers p_centers:...
*/
void detection_circles(){
    
    cv::Mat img_blur;
    blur(img,img_blur,cv::Size(3,3));
    IplImage ipltmp = img_blur;
    double canny_low;
    double canny_high;
    int ass_radius = params.at(4);
    int min_radius = 0.8*ass_radius; 
    int max_radius = 1.2*ass_radius;
    int min_dist = 2*ass_radius;
    AdaptiveFindThreshold(&ipltmp, &canny_low, &canny_high);
    double canny_thresroud = canny_high;
    //Canny
    /*cout<<"canny threshold"<< canny_thresroud<<endl;
    Mat edges;
    Canny(img_blur,edges,canny_thresroud,canny_thresroud*0.5);
    imshowResize("canny edge",edges);*/
    
 
    // hough circles, this can be optimized to reduce running time
    int acc_threshold = round(3.14*2*ass_radius*0.15);// make the 2*n*n > count>= n*n
    std::vector<cv::Vec3f> circles;

    HoughCircles(img,circles,CV_HOUGH_GRADIENT,1,
                 min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
    
    if(circles.empty()){
        std::cout<<"no circle detected using hough circle detection, please ensure the right params"<<std::endl;
        return; 
    }
   // Mat mat_for_show(img.size(),CV_8UC3,Scalar(0));
    for (int i = 0; i< circles.size(); i++) {
        cv::Point center(round(circles[i][0]), round(circles[i][1]));
        centers.push_back(center);
      //  circle(mat_for_show,center,20,Scalar(255));

    }
    //imshowResize("circles",mat_for_show);
    // waitKey(0);

    }
    
    
    
public:
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
void extend_by_circle(cv::Mat& visited,std::vector<cv::Point> key_pts,cv::Vec2i direction,float* score,std::vector<cv::Point>& visit_points, cv::RotatedRect& bounding_r_rect){
   // refine the rect using circle center position
    bool refine  =true;
    if(centers.empty()) {
        std::cout<<"no circle input in extend rect step "<<std::endl;
        return;
    }
    // extend rect(not use circle feature);
    cv::RotatedRect r_rect;
    get_rotated_rect(img,key_pts,cv::Size2i(direction[0],direction[1]),r_rect);
    // get bounding box of circles, which inside or intersection of input rect;
    cv::RotatedRect bounding_rect1;
    std::vector<cv::Point> remaining_circles;
    int radius = params.at(4);
    find_bounding_rect(visited,r_rect,radius,bounding_rect1,remaining_circles);
    
  /*  cv::Mat test(visited.size(),CV_8UC3,cv::Scalar(0));
    int num = centers.size();
    for(int i = 0; i<num;i++){
         circle(test,centers.at(i),radius,cv::Scalar(155));
    }
    DrawRotatedRect(test,bounding_rect1,cv::Scalar(0,255,0));
    circle(test,key_pts.at(0), 10, cv::Scalar(0, 125, 0), -1);
    circle(test,key_pts.at(1), 10, cv::Scalar(255, 125, 125), -1);
    circle(test,key_pts.at(2), 10, cv::Scalar(255, 125, 125), -1);*/

  
    if( bounding_rect1.size.empty()){
        std::cout <<"no bounding rect"<<std::endl;
        return;
    }
    //refine use circle center position and polygon edge. In the following, we relocation the rect angle and start point based on circle centers along the two sides.
    if(refine){
        // two segments extracted from polygon, insecting in the start point.
        cv::Vec3f line1_params;
        line1_params[0] = key_pts.at(1).y-key_pts.at(0).y;
        line1_params[1] = key_pts.at(0).x - key_pts.at(1).x;
        line1_params[2] = key_pts.at(1).x*key_pts.at(0).y-key_pts.at(0).x*key_pts.at(1).y;
        
        cv::Vec3f line2_params;
        line2_params[0] = key_pts.at(2).y-key_pts.at(0).y;
        line2_params[1] = key_pts.at(0).x - key_pts.at(2).x;
        line2_params[2] = key_pts.at(2).x*key_pts.at(0).y-key_pts.at(0).x*key_pts.at(2).y;
        // find circles nearest to the two sides separately.
        std::vector<float> d_line1;
        float min_d_line1 = 100000;
        int ind_line1 = 0;
        std::vector<cv::Point> line1_nearest_point;
        for(int i = 0;i<remaining_circles.size();i++){
            cv::Vec3f v_point(remaining_circles.at(i).x,remaining_circles.at(i).y,1);
            float d_scale = sqrt(line1_params[0]*line1_params[0]+line1_params[1]*line1_params[1]);
            float d1 = abs(line1_params.dot(v_point))/d_scale;
            d_line1.push_back(d1);
            if(d1<min_d_line1){
                min_d_line1 = d1;
                ind_line1 = i;
            }
        }
        
        for(int i = 0; i<d_line1.size();i++){
            if(d_line1.at(i)-min_d_line1<radius){
                 line1_nearest_point.push_back(remaining_circles.at(i));
                //  circle(test,remaining_circles.at(i),radius,Scalar(255),-1);
            }
        }
        
        
        std::vector<float> d_line2;
        float min_d_line2 = 100000;
        std::vector<cv::Point> line2_nearest_point;
        for(int i = 0;i<remaining_circles.size();i++){
            cv::Vec3f v_point(remaining_circles.at(i).x,remaining_circles.at(i).y,1);
            float d_scale = sqrt(line2_params[0]*line2_params[0]+line2_params[1]*line2_params[1]);
            float d2 = abs(line2_params.dot(v_point))/d_scale;
            d_line2.push_back(d2);
            if(d2<min_d_line2){
                min_d_line2 = d2;
            }
        }
        
        for(int i = 0; i<d_line2.size();i++){
            if(d_line2.at(i)-min_d_line2<radius){
                 line2_nearest_point.push_back(remaining_circles.at(i));
                // circle(test,remaining_circles.at(i),radius,Scalar(255),-1);
            }
        }
        
        cv::Vec4f line1,line2;
        fitLine(line1_nearest_point,line1,cv::DistanceTypes::DIST_L2,0,0.01,0.01);
        fitLine(line2_nearest_point,line2,cv::DistanceTypes::DIST_L2,0,0.01,0.01);
       cv::Point A1,B1;
       // incase that the slope is infinity(x=b)
         if(abs(line1[0])<0.01){
             A1 = cv::Point(line1[2],0);
             B1 = cv::Point(line1[2],visited.size().height);
        }else{
            A1 =  cv::Point(0,line1[3]-(line1[1]*line1[2]/line1[0]));
            B1 = cv::Point(visited.size().width,(visited.size().width-line1[2])*line1[1]/line1[0]+line1[3]);   
        }
        
        cv::Point A2,B2;
        if(abs(line2[0])<0.01){
             A2 = cv::Point(line2[2],0);
             B2 = cv::Point(line2[2],visited.size().height);
        }else{
             A2 = cv::Point(0,line2[3]-(line2[1]*line2[2]/line2[0]));
             B2 =  cv::Point(visited.size().width,(visited.size().width-line2[2])*line2[1]/line2[0]+line2[3]);
        }
               
       /* line(test,A1,B1,Scalar(0,255,0));
        line(test,A2,B2,Scalar(0,255,0));
       circle(test,inter_point,20,Scalar(0,0,255),-1);
       circle(test,Point(line1[2],line1[3]),5,Scalar(0,255,0),-1);
       circle(test,Point(line2[2],line2[3]),5,Scalar(0,255,0),-1);*/
       
       cv::Point inter_point =  get_point_segment_cross(A1,B1,A2,B2);
       std::cout<<"inter_point"<<inter_point<<std::endl;
       
       //incase we missed the circle near basepoint
       cv::Point new_base_point;
       find_nearest_center(remaining_circles,inter_point,&new_base_point);
       if(get_distance(new_base_point,inter_point)>radius){
           new_base_point = inter_point;
           std::cout <<"missed the corner circles"<<std::endl;
        }
    
        std::vector<cv::Point> new_key_pts;
        new_key_pts.push_back(new_base_point);
        new_key_pts.push_back(cv::Point(line1[2],line1[3]));
        new_key_pts.push_back(cv::Point(line2[2],line2[3]));
    
        cv::RotatedRect ro_rect;
        cv::Size2i new_rect_size(direction[0] -radius,direction[1] -radius);
        get_rotated_rect(img,new_key_pts,new_rect_size,ro_rect);
        remaining_circles.clear();
     
        find_bounding_rect(visited,ro_rect,radius,bounding_rect1,remaining_circles);
        if( bounding_rect1.size.empty()){
            std::cout <<"no bounding rect"<<std::endl;
            return;
        }
    
      
    }
    
    
    // here we just consider the score is circle_num/total simply
    int circle_count = remaining_circles.size();
    int m = params.at(2);
    int n  = params.at(3);
    //score <=1
    *score = (circle_count/(m*n+0.0))>1?1:circle_count/(m*n+0.0);
     std::cout<<"score" << *score<<std::endl;
    // return bounding box of all circle
    bounding_r_rect = bounding_rect1;
    
    //return visited points
    std::vector<cv::Point> rotated_rect_points;
    count_points_in_rotated_rect(bounding_r_rect,rotated_rect_points);
    std::vector<cv::Point> remaining_rotated_rect_points;
    for(int i = 0; i <rotated_rect_points.size();i++){
       if(mask.at<uchar>(rotated_rect_points.at(i))==255 && visited.at<uchar>(rotated_rect_points.at(i))==0){
           remaining_rotated_rect_points.push_back(rotated_rect_points.at(i));
         //  test.at<cv::Vec3b>(rotated_rect_points.at(i)) = 255;
           
        }
           
    }
   visit_points  = remaining_rotated_rect_points;
   /*imshowResize("visited",visited);
    imshowResize("test",test);
    cv::waitKey(0); */

}
    
private:
/**
* @brief find circles inside the rect or have intersection with the rect.
* 
* @param rect p_rect:...
* @param radius p_radius:...
* @param centers p_centers:...
* @param img p_img:...
* @param belong_rect p_belong_rect:...
*/
void circle_belong_rect( cv::RotatedRect& rect, double radius,std::vector<bool>& belong_rect){
    
    int num = centers.size();
  
    cv::Point2f corner_pts[4]; 
    rect.points(corner_pts);
    cv::Point2f* lastItemPointer = (corner_pts + sizeof corner_pts / sizeof corner_pts[0]);
    std::vector<cv::Point2f> contour_pts(corner_pts, lastItemPointer);
    for(int i  = 0; i<num;i++){
        double d = pointPolygonTest(contour_pts,centers.at(i),true);
        if(d>=-radius)
            belong_rect.at(i) = true;// intersection or inside the rect
    }
    //DrawRotatedRect(img,rect2,Scalar(255));
   // circle(img,re_tl,20,Scalar(255),-1);
   // circle(img,rect_center,20,Scalar(255),-1);
   // imshowResize("xxxx",img);
   // waitKey(0);
    
}
private:

/**
* @brief count the points on the bounding of circle. references: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm 
* 
* @param center p_center:...
* @param radius p_radius:...
* @param points p_points:...
*/
void raster_circle(cv::Point center,
         int radius, std::vector<cv::Point>& points){
    int x0 = center.x;
    int y0 = center.y;
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;
    points.push_back(cv::Point(x0,y0+radius));
    points.push_back(cv::Point(x0,y0-radius));
    points.push_back(cv::Point(x0 + radius,y0));
    points.push_back(cv::Point(x0 - radius,y0));
    
    while(x < y) 
    {
        if(f >= 0) 
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;    
        points.push_back(cv::Point(x0 + x, y0 + y));
        points.push_back(cv::Point(x0 + y, y0 + x));
        points.push_back(cv::Point(x0 - y, y0 + x));
        points.push_back(cv::Point(x0 - x, y0 + y));
        points.push_back(cv::Point(x0 - x, y0 - y));
        points.push_back(cv::Point(x0 - y, y0 - x));
        points.push_back(cv::Point(x0 + y, y0 - x));
        points.push_back(cv::Point(x0 + x, y0 - y));
    }
}
private:
/**
* @brief find the nearest center of input centers to the given point.
* 
* @param centers p_centers:...
* @param basepointcv::Pointsepoint:...
* @param nearest_point p_nearest_point:...
*/
void find_nearest_center( std::vector<cv::Point>& centers,cv::Point basepoint,cv::Point* nearest_point){
    int distance = 10000*10000;
    cv::Point best_point(basepoint);
    
    for(int i = 0;i <centers.size();i++){
        int d = (centers.at(i).x -basepoint.x)*(centers.at(i).x -basepoint.x)+(centers.at(i).y -basepoint.y)*(centers.at(i).y -basepoint.y);
        if(d<distance){
            distance = d;
            best_point = centers.at(i);
        }
    }
     *nearest_point = best_point;
}

private:
/**
* @brief this function is used to find all centers that have intersection or inside the input rect.
* 
* @param visited p_visited:...
* @param img p_img:...
* @param r_rect p_r_rect:...
* @param centers p_centers:...
* @param radius p_radius:...
* @param bounding_rect p_bounding_rect:...
* @param remaining_circles p_remaining_circles:...
*/
void find_bounding_rect(cv::Mat& visited, cv::RotatedRect& r_rect,int radius,cv::RotatedRect& bounding_rect,std::vector<cv::Point>& remaining_circles){
    
   // DrawRotatedRect(img,r_rect,Scalar(255));
   // imshowResize("rect",img);
   // waitKey(0);
    cv::RotatedRect whole(cv::Point(visited.cols,0),cv::Point(0,0),cv::Point(0,visited.rows));
    std::vector<cv::Point> rotate_intersect_pts;
    rotatedRectangleIntersection(r_rect,whole,rotate_intersect_pts);
    double inters_area = contourArea(rotate_intersect_pts);
    float ratio = inters_area/(r_rect.size.area());
    std::cout <<"ratio "<<ratio << std::endl;
    if(ratio <=0.8){
       std::cout <<"ratio "<<ratio <<" is too small"<<std::endl;
        return;    
    }
    //intersection with the whole image
    cv::RotatedRect inters_ro_rect = minAreaRect(rotate_intersect_pts);

    
    
    
   int num = centers.size();
    std::vector<bool> belong_rectx(num);
    circle_belong_rect(inters_ro_rect,radius, belong_rectx);
   
    
    std::vector <cv::Point> remaining;
    std::vector<cv::Point> circle_points;

    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    for (int i = 0; i < num; i++) {
        //circle(visited,centers.at(i),radius,Scalar(155),1,4,0);
        if(belong_rectx.at(i)&&visited.at<uchar>(centers.at(i))==0){
            remaining.push_back(centers.at(i));
            std::vector<cv::Point> pts;
           // points_on_circle(centers.at(i),radius,pts);
            raster_circle(centers.at(i),radius,pts);
            circle_points.insert(circle_points.end(),pts.begin(),pts.end());
        }
    }
     
    if(remaining.empty()){
        std::cout <<"no remaining circles"<<std::endl;
        return;
    }
       std::cout <<"remaining"<<remaining.size()<<std::endl;
       
  
          
   cv::Mat visited_clone(visited.size(),CV_8UC1,cv::Scalar(0));
   for(int i = 0;i <centers.size();i++){
       circle(visited_clone,centers.at(i),radius,cv::Scalar(255));
      // visited_clone.at<uchar>(circle_points.at(i))=255;
    }
    


   // should bounding all the point in circle
    remaining_circles = remaining;
   bounding_rect =  minAreaRect(circle_points);
   
   /*DrawRotatedRect(visited_clone,bounding_rect,Scalar(255,100,100));
   imshowResize("xxxxx",visited_clone);
   waitKey(0);*/

}
private:
/**
* @brief count all point inside the rotatedrect
* 
* @param rotated_rect p_rotated_rect:...
* @param rotated_rect_points p_rotated_rect_points:...
*/
void count_points_in_rotated_rect(cv::RotatedRect& rotated_rect,std::vector<cv::Point>& rotated_rect_points){
    cv::Rect bounding_rect = rotated_rect.boundingRect();
    
    int x0 = bounding_rect.tl().x;
    int y0 = bounding_rect.tl().y;
    int x_end = bounding_rect.br().x;
    int y_end = bounding_rect.br().y;
    cv::Point2f corner_pts[4]; 
    rotated_rect.points(corner_pts);
    
    cv::Point2f* lastItemPointer = (corner_pts + sizeof corner_pts / sizeof corner_pts[0]);
    std::vector<cv::Point2f> contour_pts(corner_pts, lastItemPointer);
    
    for(int x = x0;x<x_end;x++){ 
        for(int y = y0;y<y_end;y++){
            double a = pointPolygonTest(contour_pts,cv::Point(x,y),false);
            if(a>=0)
                rotated_rect_points.push_back(cv::Point(x,y));
        }
    }
    
}

public:
    Feature*  GetClassType(void){
        return this;
    }
    

};

#endif
