#include "dfs.h"
using namespace std;

void dfs(int direction[8][2], Mat& visited,vector<Point> centers, double radius, double mean_dist, Mat& img, int* count_segmentation,vector<Step>& path, vector<Path>& allpaths)
{
    // all of circle have been setted to flase
    int num = abs(direction[0][1])*abs(direction[0][0]);
    if (isEnd(visited,centers,path,allpaths,num)) {
        (*count_segmentation)++;
        
        return;
    }
    Point point;// bottom left point
     Step step;
    findblcenter(visited,centers,&point);
    //cout << centers.size()<<endl;

    for (int i = 0; i<8; i++) {
        //if have not visited
        //如果周围可以有邻接矩形
        if(extendAdjustRect(visited, centers,direction[i],point, radius,mean_dist,img)) {
            
            // set the m*n rect into flase
            setVisited(visited,point,direction[i],255,radius, mean_dist,centers,img);
           step.direction = i;
           step.p = point;
           path.push_back(step);
            dfs(direction, visited,centers, radius, mean_dist,img,count_segmentation,path,allpaths);
            setVisited(visited,point,direction[i],0,radius, mean_dist,centers,img);//back to set it to 0
            path.pop_back();
        }

    }

}


// return of bool vector that indicates whether the center belong to this recantangle.(1:belong 0:not belong)
//belong means the circle is inner of the rect or have interesection in the bounding
void circle_belong_rect( Point blpoint, int direction[], double radius, double dist, vector<Point> centers,Mat& img,vector<bool>& belong_rect){
     Point p1(blpoint.x-sng(direction[0])*radius, blpoint.y-radius*sng(direction[1]));
     Point p2(p1.x, p1.y + direction[1]*dist+ 2*radius*sng(direction[1]));
     Point p3(p1.x+direction[0]*dist+2*radius*sng(direction[0]), p1.y);

    Rect rect(p2,p3);
    int num = centers.size();
    Point rect_center = 0.5*(rect.br()+rect.tl());
    Point h(abs(rect.tl().x-rect.br().x)/2,abs(rect.tl().y-rect.br().y)/2);// the radius of rect
    // calculate the distance from centers to rect, if the distance less than radius, we  think the circle still belong to this rectangle.
    for(int i  = 0; i < num ;i ++ ){
        Point p(abs(centers.at(i).x-rect_center.x), abs(centers.at(i).y-rect_center.y));
        Point p2(max((p.x-h.x),0),max((p.y-h.y),0));
        belong_rect.at(i) = (p2.dot(p2)<=radius*radius)||(rect.contains(centers.at(i)));
        
    }
    //rectangle(img,rect,Scalar(255,0,0),1,16);

}

bool extendAdjustRect(Mat& visited,vector<Point> centers,int direction[], Point basePoint, double radius,double dist, Mat& img)
{
    
    int num = centers.size();
    vector<bool> belong_rectx(num);
     circle_belong_rect( basePoint, direction, radius, dist,centers,img, belong_rectx);
    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    int count = 0;
    for (int i = 0; i < num; i++) {
        if(belong_rectx.at(i)&&visited.at<uchar>(centers.at(i))==0)
            count++;
    }
     
    //standard number in every rectangle
    int standard_c = (abs(direction[0])+1)*(abs(direction[1])+1);
    if(abs(standard_c-count )>3 ) {
        return false;
    }
    cout << "containing point"<<count << endl;


    return true; 


}

bool isEnd(cv::Mat& visited, std::vector< cv::Point > centers, std::vector< Step >& path, std::vector<Path>& allpaths,int num)
{
    //int num = centers.size();
     int count =0;
    for( int i = 0; i < centers.size(); i ++ ) {
       
        if(visited.at<uchar>(centers.at(i))==0) {
              count++;
              
             //continue;
        }
    }
        // if the remaining circles is less than  m*n(num of circles in one box), we think it is a right segmentation
        if (count<round(1*num)){ 
            Path p;
             p.path = path;
            p.tag = count;
            allpaths.push_back(p); 
           // for(int i = 0; i<path.size();i++){
             //   cout<<"path"<<path.at(i).p<<"direction"<<path.at(i).direction<<endl;
        //}
        return true;
    }

return false; 

}

double sng(double x) {
    return (x <0)? -1 : (x> 0);
}
//find the bottom center of circles
void findblcenter(Mat& visited, vector<Point> centers, Point* bl) {

    vector<Point> remaining;
    for (int i = 0; i < centers.size(); i++) {
        if(visited.at<uchar>(centers.at(i))==0) {
            remaining.push_back(centers.at(i));
            //circle(img,centers.at(i),radius,Scalar(0,0,100),-1,4,0);
        }
    }
   
    //waiting for modification
    if(remaining.empty()|| remaining.size() ==1) {
        return; 
    }


    int x_lb = 0;
    int y_lb = visited.rows;
    double distance = (remaining.at(0).x-x_lb)*(remaining.at(0).x-x_lb)+(remaining.at(0).y-y_lb)*(remaining.at(0).y-y_lb);
    int index = 0;
    for(int i =1; i< remaining.size(); i++) {
        double d2 = (remaining.at(i).x-x_lb)*(remaining.at(i).x-x_lb)+(remaining.at(i).y-y_lb)*(remaining.at(i).y-y_lb);
        if(d2 < distance) {
            distance = d2;
            index = i;
        }
    }
    cout << "index" << index << endl;

    *bl = remaining.at(index);
    

}

void setVisited(Mat& visited, Point blpoint, int direction[],uchar a, double radius, double dist, vector<Point> centers, Mat& img){
    int num = centers.size();
    vector<bool> belong_rect(num);
     circle_belong_rect( blpoint, direction, radius, dist,centers,img, belong_rect);
     
     for(int i = 0; i<num ;i++){
         if(belong_rect.at(i)){
             visited.at<uchar>(centers.at(i)) = a;
             circle(img,centers.at(i), radius,Scalar(a,a,a),6,8,0);
        }
             
    }
    
    //rectangle(img,rect,Scalar(255,0,0),1,16);
     //circle(img,blpoint,10,Scalar(255,0,0),-1,16,0);
    imshowResize("xx",img);
     waitKey(0);
         
}




