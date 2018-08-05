
#include "feature_circle.h"
#include "utils.h"
#include "dfs.h"
void extend_by_circle(Mat& visited, Mat& img,Mat& mask,int direction[], Point basePoint, vector<int>& params,vector<Point>& centers,float* score,vector<Point>& visit_points)
{
    if(centers.empty()) return;
     int radius = params.at(4);
    
     Point p2(basePoint.x+direction[0],basePoint.y + direction[1]);//br point
     Rect rect(basePoint,p2);
    Rect whole(Point(0,0),visited.size());
    rect &= whole;
     
    float ratio;
     intersection_rect_mask(rect,mask,&ratio);

         if( ratio<=0.8  ){
             *score =0;
        
         return;
       }
    

    
     
   int num = centers.size();
    vector<bool> belong_rectx(num);
    circle_belong_rect(rect,radius,centers,img, belong_rectx);
   
    
    vector <Point> remaining;
    vector<Point> circle_points;
    Mat visited_clone = visited.clone();
    // Mat visited_clone(visited.size(),CV_8UC1,Scalar(0));
    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    int circle_count = 0;
    for (int i = 0; i < num; i++) {
        circle(visited_clone,centers.at(i),radius,Scalar(155),1,4,0);
        if(belong_rectx.at(i)&&visited.at<uchar>(centers.at(i))==0){
            circle_count++;
            remaining.push_back(centers.at(i));
            vector<Point> pts;
           // points_on_circle(centers.at(i),radius,pts);
            raster_circle(centers.at(i),radius,pts);
            circle_points.insert(circle_points.end(),pts.begin(),pts.end());
        }
    }
    
    if(remaining.empty()){
        //setVisited(visited,img,mask,rect,255);;
       // bounding = rect;
        *score = 0;
        cout <<"no remaining circles"<<endl;
        return;
    }


    cout<<"basepoint"<<basePoint<<endl;
    cout <<"direction" <<direction[0]<<" "<<direction[1]<< endl;
    cout <<"remaining"<<remaining.size()<<endl;

   // should bounding all the point in circle
   Rect b_rect = boundingRect(circle_points);
    Rect final_rect;
    Rect padding_rect(Point(0,0),visited_clone.size());
    eat_padding(b_rect,padding_rect,visited_clone,final_rect);

  // bounding = final_rect;
   
   // vector<Point> visit_points;
   
    int x0 = final_rect.tl().x;
    int y0 = final_rect.tl().y;
    int x_end = final_rect.br().x;
    int y_end = final_rect.br().y;
    
   
    for(int x = x0;x<x_end;x++){ 
        for(int y = y0;y<y_end;y++){
            if(mask.at<uchar>(Point(x,y))==255  && visited.at<uchar>(Point(x,y))==0)
                visit_points.push_back(Point(x,y));
                
        }
    }
    
   //rectangle(img, final_rect,Scalar(255),1,16);
   imshowResize("visited_clone",visited_clone);
   imshowResize("XXX",img);
   waitKey(0);  

       

 //  cout <<"bounding box4 size "<<b_rect4.size()<<endl;
    // here we just think the score is circle_num/total simply
    int m = params.at(2);
     int n  = params.at(3); 
    *score = float(circle_count)/(m*n);
     cout<<"score" << *score<<endl;
    
     
    
}

void circle_belong_rect( Rect& rect, double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect){
    // Point p(basePoint.x,basePoint.y+1);
    /* Point brpoint(tlpoint.x+direction[0], tlpoint.y + direction[1]);//br

    Rect rect(tlpoint,brpoint);*/
    int num = centers.size();
    Point rect_center = 0.5*(rect.br()+rect.tl());
    Point h(abs(rect.tl().x-rect.br().x)/2,abs(rect.tl().y-rect.br().y)/2);// the radius of rect
    // calculate the distance from centers to rect, if the distance less than radius, we  think the circle still belong to this rectangle.
    for(int i  = 0; i < num ;i ++ ){
        Point p(abs(centers.at(i).x-rect_center.x), abs(centers.at(i).y-rect_center.y));
        Point p2(max((p.x-h.x),0),max((p.y-h.y),0));
        belong_rect.at(i) = (p2.dot(p2)<=radius*radius)||(rect.contains(centers.at(i)));
        
    }
    
   // rectangle(img,rect,Scalar(255,0,0),1,16);

}


void eat_padding(cv::Rect& inrect, Rect& range_rect, cv::Mat& visited, cv::Rect& outrect)
{
    
    uchar value = 155;
   Point high_corner = inrect.br();
   Point low_corner = inrect.tl();
   
   Point high_thres = range_rect.br();
   Point low_thres = range_rect.tl();
   

   int y_low = low_corner.y;
   //int xx = low_corner.x;
   for(int y = low_corner.y-1;y>=low_thres.y;y--){
       uchar* row = visited.ptr<uchar>(y);
       int x=low_thres.x;
       for( x = low_thres.x;x<=high_thres.x;x++){
            if((row[x] == value)&&(low_corner.x<=x)&&(x<high_corner.x))
               break;
    }
    //xx = x;
     if(x<(high_thres.x-1))
            break;
        y_low =y;
    
  }
  
  int y_high = high_corner.y;
  for(int y = high_corner.y+1;y<high_thres.y;y++){
       uchar* row = visited.ptr<uchar>(y);
       int x = low_thres.x;
       for( x = low_thres.x;x<high_thres.x;x++){
            if((row[x] == value)&&(low_corner.x<=x)&&(x<high_corner.x))
               break;
    }
    
     if(x<(high_thres.x-1))
            break;
        y_high =y;
  }
 // cout <<"y_high"<<y_high<<endl;
  int x_low = low_corner.x;
  for(int x=low_corner.x-1;x>=low_thres.x;x--){
      int y = y_low;
      for( y = y_low;y<y_high;y++){
        if((visited.ptr<uchar>(y)[x]==value) &&(low_corner.y<=y)&&(y<high_corner.y))
            break;
    }
    if(y<(y_high-1)) break;
    x_low = x;
}

  int x_high = high_corner.x;
  for(int x=high_corner.x+1;x<high_thres.x;x++){
      int y = y_low;
      for( y = y_low;y<y_high;y++){
        if((visited.ptr<uchar>(y)[x]==value) &&(low_corner.y<=y)&&(y<high_corner.y))
            break;
    }
    if(y<(y_high-1)) break;
    x_high = x;
  }

outrect = Rect(Point(x_low,y_low),Point(x_high,y_high));
//cout <<"x_low"<<x_low<<endl;
//cout <<"y_low" << y_low <<endl;
//cout << "top left"<<outrect.tl()<<endl;
//out << "bottom right"<<outrect.br()<<endl;
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
    
    
    
    
    /*Mat mCanny_Gray,mThres_Gray;

    double CannyAccThresh = threshold(img,mThres_Gray,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);

    double CannyThresh = 0.5 * CannyAccThresh;

    Canny(img,mCanny_Gray,CannyThresh,CannyAccThresh);
    imshowResize("mCanny_Gray",mCanny_Gray);*/
 
    // hough circles, this can be optimized to reduce running time
    int acc_threshold = round(3.14*2*ass_radius*0.15);// make the 2*n*n > count>= n*n
    vector<Vec3f> circles;
    //自适应寻找hough变换的阈值acc_threshold(这里设置步长为1，以后可以自适应步长)

    HoughCircles(img,circles,CV_HOUGH_GRADIENT,1,
                 min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
    
    if(circles.empty()){
        
       return; 
    }
    /*while(1) {
        if(circles.size() < test_count) {
            acc_threshold = acc_threshold-1;
            HoughCircles(img,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:" << circles.size()<<endl;
        } else if(circles.size() > 2*test_count) {
            acc_threshold = acc_threshold+1;
            HoughCircles(img,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:"<< circles.size()<<endl;
        } else {


            break;
        }
    }*/
    for (int i = 0; i< circles.size(); i++) {
        Point center(round(circles[i][0]), round(circles[i][1]));
        centers.push_back(center);
      //  circle(visited,center,ass_radius,Scalar(255,0,0),-1);

    }

}

void points_on_circle(Point center, int radius, vector<Point>& points)
{
    int x0 = center.x;
    int y0 = center.y;
    int x = radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (radius << 1);

    while (x >= y)
    {
        points.push_back(Point(x0 + x, y0 + y));
        points.push_back(Point(x0 + y, y0 + x));
        points.push_back(Point(x0 - y, y0 + x));
        points.push_back(Point(x0 - x, y0 + y));
        points.push_back(Point(x0 - x, y0 - y));
        points.push_back(Point(x0 - y, y0 - x));
        points.push_back(Point(x0 + y, y0 - x));
        points.push_back(Point(x0 + x, y0 - y));

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        
        if (err > 0)
        {
            x--;
            dx += 2;
            err += dx - (radius << 1);
        }
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
