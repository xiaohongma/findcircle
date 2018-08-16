
#include "feature_circle.h"
#include "utils.h"
#include "dfs.h"
#include "polygon_calculation.hpp"
void extend_by_circle(Mat& visited, Mat& img,Mat& mask,vector<Point> key_pts, int direction[], vector<int>& params,vector<Point>& centers,float* score,vector<Point>& visit_points, RotatedRect& bounding_r_rect)
{
   
    bool refine  =true;
    if(centers.empty()) return;
     int radius = params.at(4);
    //Mat visited_clone(visited.size(),CV_8UC1,Scalar(0));
    Mat test(visited.size(),CV_8UC3,Scalar(0));
     Mat visited_clone = visited.clone();
    int num = centers.size();
    for(int i = 0; i<num;i++){
        circle(visited_clone,centers.at(i),radius,Scalar(155));
         circle(test,centers.at(i),radius,Scalar(155));
    }
    
    //imshowResize("visited_clone",visited_clone); 
   // waitKey(0);  

    RotatedRect r_rect;
    get_rotated_rect(img,key_pts,direction,r_rect);
     //DrawRotatedRect(test,r_rect,Scalar(0,255,0));
    RotatedRect bounding_rect1;
    vector<Point> remaining_circles;
    find_bounding_rect(visited_clone,img,r_rect,centers,radius,bounding_rect1,remaining_circles);
    //DrawRotatedRect(test,bounding_rect1,Scalar(0,255,0));
   //// cv::circle(test,key_pts.at(0), 10, cv::Scalar(0, 125, 0), -1);
   // cv::circle(test,key_pts.at(1), 10, cv::Scalar(255, 125, 125), -1);
   // cv::circle(test,key_pts.at(2), 10, cv::Scalar(255, 125, 125), -1);

  
    if( bounding_rect1.size.empty()){
        cout <<"no bounding rect"<<endl;
        return;
    }
    if(refine){
       // line(test,key_pts[0],key_pts[1],Scalar(255));
        //line(test,key_pts[0],key_pts[2],Scalar(255));
        Vec3f line1_params;
        line1_params[0] = key_pts.at(1).y-key_pts.at(0).y;
        line1_params[1] = key_pts.at(0).x - key_pts.at(1).x;
        line1_params[2] = key_pts.at(1).x*key_pts.at(0).y-key_pts.at(0).x*key_pts.at(1).y;
        
        Vec3f line2_params;
        line2_params[0] = key_pts.at(2).y-key_pts.at(0).y;
        line2_params[1] = key_pts.at(0).x - key_pts.at(2).x;
        line2_params[2] = key_pts.at(2).x*key_pts.at(0).y-key_pts.at(0).x*key_pts.at(2).y;
        
        vector<float> d_line1;
        float min_d_line1 = 100000;
        int ind_line1 = 0;
        vector<Point> line1_nearest_point;
        for(int i = 0;i<remaining_circles.size();i++){
            Vec3f v_point(remaining_circles.at(i).x,remaining_circles.at(i).y,1);
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
                  circle(test,remaining_circles.at(i),radius,Scalar(255),-1);
            }
        }
        
        
        vector<float> d_line2;
        float min_d_line2 = 100000;
        vector<Point> line2_nearest_point;
        for(int i = 0;i<remaining_circles.size();i++){
            Vec3f v_point(remaining_circles.at(i).x,remaining_circles.at(i).y,1);
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
                 circle(test,remaining_circles.at(i),radius,Scalar(255),-1);
            }
        }
        
        Vec4f line1,line2;
        fitLine(line1_nearest_point,line1,cv::DistanceTypes::DIST_L2,0,0.01,0.01);
        fitLine(line2_nearest_point,line2,cv::DistanceTypes::DIST_L2,0,0.01,0.01);
        //make the two line are vertical with each other
       /* double theta = acos(line1[0]*line2[0]+line1[1]*line2[1])*180/CV_PI;
        cout<<"theta "<<theta<<endl;
        float r = get_cross_product(Vec2f(line1[0],line1[1]),Vec2f(line2[0],line2[1]));
        float bais_theta = (theta-90)/2*CV_PI/180;
        //rotation A' = (x*cos(theta)-y*sin(theta),y*cos(theta)+x*sin(theta)); A=(x,y)
        if(r>0){
            //line one is lower;
            bais_theta = -bais_theta;
        }
        //refine params;
        float p1_l1 = line1[0]*cos(bais_theta)-line1[1]*sin(bais_theta);
        float p2_l1 = line1[1]*cos(bais_theta)+line1[0]*sin(bais_theta);
        line1[0] = p1_l1/sqrt(p1_l1*p1_l1+p2_l1*p2_l1);
         line1[1] = p2_l1/sqrt(p1_l1*p1_l1+p2_l1*p2_l1);
            
        float p1_l2 = line2[0]*cos(bais_theta)-line2[1]*sin(bais_theta);
        float p2_l2 = line2[1]*cos(bais_theta)+line2[0]*sin(bais_theta);
        line2[0] = p1_l2/sqrt(p1_l2*p1_l2+p2_l2*p2_l2);
         line2[1] = p2_l2/sqrt(p1_l2*p1_l2+p2_l2*p2_l2);*/
       Point A1,B1;
         if(abs(line1[0])<0.01){
             A1 = Point(line1[2],0);
             B1 = Point(line1[2],test.size().height);
        }else{
            A1 =  Point(0,line1[3]-(line1[1]*line1[2]/line1[0]));
            B1 = Point(test.size().width,(test.size().width-line1[2])*line1[1]/line1[0]+line1[3]);   
        }
        line(test,A1,B1,Scalar(0,255,0));
        
        Point A2,B2;
        if(abs(line2[0])<0.01){
             A2 = Point(line2[2],0);
             B2 = Point(line2[2],test.size().height);
        }else{
             A2 = Point(0,line2[3]-(line2[1]*line2[2]/line2[0]));
             B2 =  Point(test.size().width,(test.size().width-line2[2])*line2[1]/line2[0]+line2[3]);
        }

        line(test,A2,B2,Scalar(0,255,0));
       Point inter_point =  get_point_segment_cross(A1,B1,A2,B2);
       cout<<"inter_point"<<inter_point<<endl;
       circle(test,inter_point,20,Scalar(0,0,255),-1);
       circle(test,Point(line1[2],line1[3]),5,Scalar(0,255,0),-1);
       circle(test,Point(line2[2],line2[3]),5,Scalar(0,255,0),-1);
       
       //incase we missed the circle near basepoint
       Point new_base_point;
       find_nearest_point(remaining_circles,inter_point,&new_base_point);
       if(get_distance(new_base_point,inter_point)>radius){
           new_base_point = inter_point;
           cout <<"missed the corner circle"<<endl;
    }
    
    vector<Point> new_key_pts;
    new_key_pts.push_back(new_base_point);
    new_key_pts.push_back(Point(line1[2],line1[3]));
    new_key_pts.push_back(Point(line2[2],line2[3]));
    
    RotatedRect ro_rect;
    int new_direct[2];
   new_direct[0] =  direction[0] -radius;
   new_direct[1] = direction[1] -radius;
    get_rotated_rect(img,new_key_pts,new_direct,ro_rect);
     remaining_circles.clear();
     
    find_bounding_rect(visited_clone,test,ro_rect,centers,radius,bounding_rect1,remaining_circles);
    if( bounding_rect1.size.empty()){
        cout <<"no bounding rect"<<endl;
        return;
     }
    
      
    }
    

    bounding_r_rect = bounding_rect1;
   vector<Point> rotated_rect_points;
   count_points_in_rotated_rect(bounding_r_rect,rotated_rect_points);
   vector<Point> remaining_rotated_rect_points;
   for(int i = 0; i <rotated_rect_points.size();i++){
       if(mask.at<uchar>(rotated_rect_points.at(i))==255 && visited.at<uchar>(rotated_rect_points.at(i))==0){
           remaining_rotated_rect_points.push_back(rotated_rect_points.at(i));
          // visited_clone.at<uchar>(rotated_rect_points.at(i))=255;
         //  test.at<cv::Vec3b>(rotated_rect_points.at(i)) = 255;
           
        }
           
    }


   visit_points  = remaining_rotated_rect_points;
    

   // imshowResize("visited_clone",visited_clone); 
   
     //waitKey(0);  

    // here we just consider the score is circle_num/total simply
   //score <=1
    int circle_count = remaining_circles.size();
    int m = params.at(2);
    int n  = params.at(3);
    *score = (circle_count/(m*n+0.0))>1?1:circle_count/(m*n+0.0);
     cout<<"score" << *score<<endl;
}


void find_bounding_rect(Mat& visited,Mat& img,RotatedRect& r_rect,vector<Point>& centers,int radius,RotatedRect& bounding_rect,vector<Point>& remaining_circles){
    
   // DrawRotatedRect(img,r_rect,Scalar(255));
   // imshowResize("rect",img);
   // waitKey(0);
    RotatedRect whole(Point(visited.cols,0),Point(0,0),Point(0,visited.rows));
    vector<Point> rotate_intersect_pts;
    rotatedRectangleIntersection(r_rect,whole,rotate_intersect_pts);
    double inters_area = contourArea(rotate_intersect_pts);
    float ratio = inters_area/(r_rect.size.area());
    cout <<"ratio "<<ratio << endl;
    if(ratio <=0.8){
        cout <<"ratio "<<ratio <<" is too small"<<endl;
        return;    
    }
    //intersection with the whole image
    RotatedRect inters_ro_rect = minAreaRect(rotate_intersect_pts);

    
    
    
   int num = centers.size();
    vector<bool> belong_rectx(num);
    circle_belong_rect(inters_ro_rect,radius,centers,img, belong_rectx);
   
    
    vector <Point> remaining;
    vector<Point> circle_points;

    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    for (int i = 0; i < num; i++) {
        //circle(visited,centers.at(i),radius,Scalar(155),1,4,0);
        if(belong_rectx.at(i)&&visited.at<uchar>(centers.at(i))==0){
            remaining.push_back(centers.at(i));
            vector<Point> pts;
           // points_on_circle(centers.at(i),radius,pts);
            raster_circle(centers.at(i),radius,pts);
            circle_points.insert(circle_points.end(),pts.begin(),pts.end());
        }
    }
     
    if(remaining.empty()){
        cout <<"no remaining circles"<<endl;
        return;
    }
       cout <<"remaining"<<remaining.size()<<endl;
       
  
          
   Mat visited_clone(visited.size(),CV_8UC1,Scalar(0));
   for(int i = 0;i <centers.size();i++){
       circle(visited_clone,centers.at(i),radius,Scalar(255));
      // visited_clone.at<uchar>(circle_points.at(i))=255;
    }
    


   // should bounding all the point in circle
    remaining_circles = remaining;
   bounding_rect =  minAreaRect(circle_points);
   
  // DrawRotatedRect(visited_clone,bounding_rect,Scalar(255,100,100));
 //  imshowResize("xxxxx",visited_clone);
  // waitKey(0);

}


void circle_belong_rect( RotatedRect& rect, double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect){
    
    int num = centers.size();
  
    Point2f corner_pts[4]; 
    rect.points(corner_pts);
    Point2f* lastItemPointer = (corner_pts + sizeof corner_pts / sizeof corner_pts[0]);
    vector<Point2f> contour_pts(corner_pts, lastItemPointer);
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

void draws_circles(Mat& img, Mat& visited,vector<int> params,vector<Point>& centers){

    IplImage ipltmp = img;
    double canny_low;
    double canny_high;
    int ass_radius = params.at(4);
    int min_radius = 0.8*ass_radius; 
    int max_radius = 1.2*ass_radius;
    int min_dist = 2*ass_radius;
    AdaptiveFindThreshold(&ipltmp, &canny_low, &canny_high);
    double canny_thresroud = canny_high;
    //Canny
    cout<<"canny threshold"<< canny_thresroud<<endl;
    //Mat edges;
   // Canny(img,edges, canny_thresroud, canny_thresroud/2);
   // imshowResize("canny edge",edges);
    //waitKey(0);
    
 
    // hough circles, this can be optimized to reduce running time
    int acc_threshold = round(3.14*2*ass_radius*0.15);// make the 2*n*n > count>= n*n
    vector<Vec3f> circles;
    //自适应寻找hough变换的阈值acc_threshold(这里设置步长为1，以后可以自适应步长)

    HoughCircles(img,circles,CV_HOUGH_GRADIENT,1,
                 min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
    
    if(circles.empty()){
        
       return; 
    }
    for (int i = 0; i< circles.size(); i++) {
        Point center(round(circles[i][0]), round(circles[i][1]));
        centers.push_back(center);
       // circle(visited,center,ass_radius,Scalar(155));

    }

}



void raster_circle(Point center,
         int radius, vector<Point>& points)
{
    int x0 = center.x;
    int y0 = center.y;
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;
    points.push_back(Point(x0,y0+radius));
    points.push_back(Point(x0,y0-radius));
    points.push_back(Point(x0 + radius,y0));
    points.push_back(Point(x0 - radius,y0));
    
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
        points.push_back(Point(x0 + x, y0 + y));
        points.push_back(Point(x0 + y, y0 + x));
        points.push_back(Point(x0 - y, y0 + x));
        points.push_back(Point(x0 - x, y0 + y));
        points.push_back(Point(x0 - x, y0 - y));
        points.push_back(Point(x0 - y, y0 - x));
        points.push_back(Point(x0 + y, y0 - x));
        points.push_back(Point(x0 + x, y0 - y));
    }
}

void find_nearest_point(vector<Point>& centers, Point basepoint,Point* nearest_point){
    int distance = 10000*10000;
    Point best_point(basepoint);
    
    for(int i = 0;i <centers.size();i++){
        int d = (centers.at(i).x -basepoint.x)*(centers.at(i).x -basepoint.x)+(centers.at(i).y -basepoint.y)*(centers.at(i).y -basepoint.y);
        if(d<distance){
            distance = d;
            best_point = centers.at(i);
        }
    }
     *nearest_point = best_point;
}



void count_points_in_rotated_rect(RotatedRect& rotated_rect,vector<Point>& rotated_rect_points){
    Rect bounding_rect = rotated_rect.boundingRect();
    
    int x0 = bounding_rect.tl().x;
    int y0 = bounding_rect.tl().y;
    int x_end = bounding_rect.br().x;
    int y_end = bounding_rect.br().y;
    Point2f corner_pts[4]; 
    rotated_rect.points(corner_pts);
    
    Point2f* lastItemPointer = (corner_pts + sizeof corner_pts / sizeof corner_pts[0]);
    vector<Point2f> contour_pts(corner_pts, lastItemPointer);
    
    for(int x = x0;x<x_end;x++){ 
        for(int y = y0;y<y_end;y++){
            double a = pointPolygonTest(contour_pts,Point(x,y),false);
            if(a>=0)
                rotated_rect_points.push_back(Point(x,y));
                
        }
    }
    
}


void DrawRotatedRect(cv::Mat& img, cv::RotatedRect& rr, cv::Scalar color)
{
    cv::Point2f pts[4];
    rr.points(pts);
    for (int i = 0; i < 4; i++)
    {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, 4);
    }
}



