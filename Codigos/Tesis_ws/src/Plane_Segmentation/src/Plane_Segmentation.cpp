 //opencv header files
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "image_transport/image_transport.h"
 #include "sensor_msgs/image_encodings.h"
 #include "cv_bridge/cv_bridge.h"

 #include "iostream"
 #include "stdio.h" //standard input-output-library
 #include "algorithm" //functions on sequences, like find, sort etc. 
 #include "stdlib.h" //includes random number generation
 #include "time.h" //get date and time information
 #include "math.h"
 #include "string.h" //manipulate strings and arrays
 #include "iomanip"
 #include "fstream"
 #include "vector"
 #include "thread"
 
 #include "opencv/cv.h"
 #include "opencv/highgui.h"
 #include "opencv/cxcore.h"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/video/tracking.hpp"
 #include "opencv2/core/core.hpp"
 
 #include "init_structures.h" //initialise structures used in this algorithm
 //#include "img_proc_fcns.h" //various image processing/computer vision functions
 #include "capture.h" //image capture 
 #include "ms_overwrite_safe_buffer.h"

 
 #define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
 #define Window_H 1.3*(proc_H)+20
 #define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(#img, img);
 using namespace std;
 using namespace cv;

Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue, F_Cr, F_Cb, F_a, channel_1[4], channel_2[4], channel_3[4], F_iic, src, src_out, dst, histImage; // Mat Declarations

double proc_W, proc_H;

void combine_channels(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic){
	
	//Reference on passing parameters-by-reference in C/C++: http://stackoverflow.com/questions/11235187/opencv-changing-mat-inside-a-function-mat-scope
	//Reference on adding matrices without saturation: http://answers.opencv.org/question/13769/adding-matrices-without-saturation/
	  Cr.convertTo(Cr, CV_32S);
	  Cb.convertTo(Cb, CV_32S);
	  a.convertTo(a, CV_32S);
		//cv::addWeighted(Cr, 0.5, Cb, 0.5, 0.0, iic, CV_32S);
	  cv::addWeighted((Cr+Cb), 0.25, a, 0.5, 0.0, iic, CV_32S);
	  iic.convertTo(iic, CV_8UC1);
}

void solve_derivatives(cv::Mat src, cv::Mat& grad){

    Mat src_gray;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray 
    cvtColor( src, src_gray, CV_RGB2GRAY );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
              
              

}

void histogram(cv::Mat src, cv::Mat& hist){
              
              Mat dst;
              
              vector<Mat> bgr_planes;
              split( src, bgr_planes );

              /// Establish the number of bins
              int histSize = 256;

              /// Set the ranges ( for B,G,R) )
              float range[] = { 0, 256 } ;
              const float* histRange = { range };

              bool uniform = true; bool accumulate = false;

              Mat b_hist, g_hist, r_hist;

              /// Compute the histograms:
              calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
              calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
              calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

              // Draw the histograms for B, G and R
              int hist_w = 512; int hist_h = 400;
              int bin_w = cvRound( (double) hist_w/histSize );

              Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

              /// Normalize the result to [ 0, histImage.rows ]
              normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
              normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
              normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

              /// Draw for each channel
              for( int i = 1; i < histSize; i++ )
              {
                  line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                                   Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                                   Scalar( 255, 0, 0), 2, 8, 0  );
                  line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                                   Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                                   Scalar( 0, 255, 0), 2, 8, 0  );
                  line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                                   Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                                   Scalar( 0, 0, 255), 2, 8, 0  );
              }
              
              hist=histImage;
                  
}


void InitializeImageVisualizers()
{
	namedWindow( "HSV", 1 ); 
    namedWindow( "YCrCb", 1 ); 
    namedWindow( "Lab", 1 ); 
    namedWindow( "F_hue", 1 ); 
    namedWindow( "F_sat", 1 ); 
    namedWindow( "F_Cb", 1 ); 
    namedWindow( "F_Cr", 1 ); 
    namedWindow( "F_a", 1 ); 
    cvNamedWindow( "F_iic", 1);
    cvNamedWindow( "Solve_Derivative", 1);
    cvNamedWindow( "Histogram", 1);
    cvNamedWindow( "Prueba", 1);
    //Window Names
}


void ShowImages()
{
	imshow("HSV",F_hsv);
	imshow("YCrCb",F_YCrCb);
	imshow("Lab",F_lab);
	imshow("F_hue",F_hue);
	imshow("F_sat",F_sat);
	imshow("F_Cr",F_Cr);
	imshow("F_Cb",F_Cb);
	imshow("F_a",F_a);
	imshow("F_iic",F_iic);
	imshow("Solve_Derivative", src_out);
	imshow("Histogram", histImage);	
}

void CalculateImageFeatures()
{
	cvtColor( frame, F_hsv, 	CV_BGR2HSV );	//Convert to HSV
	cvtColor( frame, F_YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
	cvtColor( frame, F_lab, 	CV_BGR2Lab );	//Convert to Lab

	cv::split(F_hsv,channel_1); //cvsplit Divides a multi-channel array into separate single-channel arrays
	F_hue=channel_1[0];
	F_sat=channel_1[1];

	cv::split(F_YCrCb,channel_2);
	F_Cr=channel_2[1];
	F_Cb=channel_2[2];

	cv::split(F_lab,channel_3);
	F_a=channel_3[1];
	combine_channels(F_Cr, F_Cb, F_a, F_iic); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
	normalize(F_iic, F_iic, 0, 255, CV_MINMAX);

	src=frame;
	solve_derivatives(src,src_out); // Solver Derivative Calculation

	histogram(src,histImage); // Histogram Calculation
}

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            MAIN           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
int main( int argc, char** argv ) 
{
	ros::init(argc, argv, "Plane_Segmentation");
	ros::NodeHandle nh;


	bool disp_img, refresh;



	VideoCapture cap("./eng_stat_obst.avi"); // Declare capture form
	//VideoCapture cap(0);


	while (nh.ok()) 
	{
		cap >> frame; // Image from Cam to Mat Variable
		cvtColor(frame, gray, CV_BGR2GRAY); // Convert to GrayScale
		waitKey(1); // Wait Time

		int input=cvWaitKey(40);
		if ((char)input==32)
		{
			std::swap(prevgray, gray);
		}
		//safe window

		if( prevgray.data )
		{		  
			CalculateImageFeatures();
			waitKey(1);
			ShowImages();
			//cvShowImage("Prueba",F_Cb2);
		}

		if(waitKey(30)>=0)
		{
			break;
		}
		std::swap(prevgray, gray);
		waitKey(1);
	}

	ros::spinOnce();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	ros::spin();

	return 0;
}
