
#include <numeric>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#include "utils.h"
void imshowResize(const cv::String& winname, const cv::Mat& img)
{
    namedWindow(winname, CV_WINDOW_NORMAL);
    cvResizeWindow(winname.c_str(), 700,700);
    imshow(winname, img);
}

 void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size )
{
    cv::Mat src = cv::cvarrToMat(image);
    // imshow("Adaptive", src);
    const int cn = src.channels();
    cv::Mat dx(src.rows, src.cols, CV_16SC(cn));
    cv::Mat dy(src.rows, src.cols, CV_16SC(cn));

    cv::Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);

    CvMat _dx = dx, _dy = dy;
    _AdaptiveFindThreshold(&_dx, &_dy, low, high);

}
 

// 仿照matlab，自适应求高低两个门限
 void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high)
{
    CvSize size;
    IplImage *imge=0;
    int i,j;
    CvHistogram *hist;
    int hist_size = 255;
    float range_0[]= {0,256};
    float* ranges[] = { range_0 };
    double PercentOfPixelsNotEdges = 0.7;
    size = cvGetSize(dx);
    imge = cvCreateImage(size, IPL_DEPTH_32F, 1);
    // 计算边缘的强度, 并存于图像中
    float maxv = 0;
    for(i = 0; i < size.height; i++ )
    {
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        float* _image = (float *)(imge->imageData + imge->widthStep*i);
        for(j = 0; j < size.width; j++)
        {
            _image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
            maxv = maxv < _image[j] ? _image[j]: maxv; //保存最大的边缘强度

        }
    }
    if(maxv == 0) {
        high = 0;
        low = 0;
        cvReleaseImage( &imge );
        return;
    }

    // 计算直方图，bins范围是灰度范围，统计量为梯度值。选择梯度比例为0.7处的灰度值作为高阈值。
    range_0[1] = maxv;
    hist_size = (int)(hist_size > maxv ? maxv:hist_size);//选择bin的范围，如果灰度值达不到255，就用较小值
    hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    cvCalcHist( &imge, hist, 0, NULL );
    int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
    float sum=0;
    int icount = hist->mat.dim[0].size;
    //cout <<icount <<endl;

    float *h = (float*)cvPtr1D( hist->bins, 0 );
    for(i = 0; i < icount; i++)
    {
        sum += h[i];
        if( sum > total )
            break;
    }
    // 计算高低门限
    *high = (i+1) * maxv / hist_size ;
    *low = *high * 0.5;
    cvReleaseImage( &imge );
    cvReleaseHist(&hist);
}

int sng(double x) {
    return (x <0)? -1 : (x> 0);
}



int rotateImg(cv::String read_path,cv::String out_path,float angle){
 
  IplImage* src= cvLoadImage(read_path.c_str(),1);
	//cvNamedWindow( "src", 1 );
     cv::Mat mat1 = cv::cvarrToMat(src);
	imshowResize(" src img",mat1);
	float anglerad=CV_PI*angle/180.0;
    //输入图像的大小
	int w = src->width;
	int h = src->height;
	//旋转后图像的大小
	int w_dst = int(fabs(h*sin(anglerad))+fabs(w*cos(anglerad)));
	int h_dst = int(fabs(w * sin(anglerad)) +fabs(h * cos(anglerad)));
	CvSize rect;
	rect.height=h_dst;
	rect.width=w_dst;
	//中间变量
	IplImage *des=cvCreateImage(rect,src->depth,src->nChannels);
	//旋转后的图像
	IplImage *des_rot=cvCreateImage(rect,src->depth,src->nChannels);
	//用0填充
	//cvFillImage(des,0);
    cvSet(des,0);

	//设置roi区域，将原图copy到roi区域
	CvRect roi;
	roi.x=(w_dst-w)/2;
	roi.y=(h_dst-h)/2;
	roi.height=h;
	roi.width=w;
	cvSetImageROI(des,roi);
	cvCopy(src,des,NULL);
	cvResetImageROI(des);
 	//旋转矩阵
	float m[6];
 	CvMat M = cvMat( 2, 3, CV_32F, m );
 
 	m[0] = (float)cos(-anglerad);
 	m[1] = (float)sin(-anglerad);
 	m[3] = -m[1];
 	m[4] = m[0];
 	// 将旋转中心移至图像中间
 	m[2] = w_dst*0.5f;  
 	m[5] = h_dst*0.5f;  
 	cvGetQuadrangleSubPix( des, des_rot, &M);
	//cvNamedWindow( "dst", 1 );
    cv::Mat mat = cv::cvarrToMat(des_rot);
	/*cvShowImage("dst",des_rot);
	cvReleaseImage(&src);
	cvReleaseImage(&des);
	cvReleaseImage(&des_rot);*/
    imwrite(out_path,mat);
    imshowResize("rotated img",mat);
	waitKey(0);
	return 0;
}
