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
#include <float.h>

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

#include "../egbis/image.h"
#include "../slic_mod/slic.h"


#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;

Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue, F_Cr, F_Cb, F_a, channel_1[4], channel_2[4], channel_3[4], F_iic, src, src_out, dst, histImage, temp_grad[3], sobel[3], F_mag, F_ang, F_lbp, image_pub, hist_prueba, histImage2; // Mat Declarations

IplImage *frame2, *F_iic2, *image_pub2;

double proc_W, proc_H;
VideoCapture cap;


void ShowImages()
{
	/*
	imshow("HSV",F_hsv);
	//imshow("YCrCb",F_YCrCb);
	//imshow("Lab",F_lab);
	imshow("F_hue",F_hue);
	imshow("F_sat",F_sat);
	//imshow("F_Cr",F_Cr);
	//imshow("F_Cb",F_Cb);
	//imshow("F_a",F_a);
	imshow("F_iic",F_iic);
	//imshow("Sobel_Derivative", src_out);
	//imshow("Histogram", histImage);	
	imshow("F_mag", F_mag);
	imshow("F_ang", F_ang);
	imshow("LBP", F_lbp);
	//imshow("Prueba", ANG);
	*/
	
	
	//DISPLAY_IMAGE_XY(false, F_hsv, 0 , 0);
	//DISPLAY_IMAGE_XY(false, F_YCrCb, 1 , 0);
	//DISPLAY_IMAGE_XY(false, F_lab, 2 , 0);
	DISPLAY_IMAGE_XY(true, frame, 0 , 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	//DISPLAY_IMAGE_XY(true, gray, 1 , 0);
	DISPLAY_IMAGE_XY(true, F_hue, 1 , 0);
	cv::resizeWindow("F_hue", 160, 120);
	//cv::resizeWindow("F_hue", 160, 120);
	DISPLAY_IMAGE_XY(true, F_sat, 2 , 0);
	cv::resizeWindow("F_sat", 160, 120);
	//cv::resizeWindow("F_sat", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_Cr, 5 , 0);
	//DISPLAY_IMAGE_XY(true, F_Cb, 6 , 0);
	//DISPLAY_IMAGE_XY(true, F_a, 7 , 0);
	DISPLAY_IMAGE_XY(true, F_iic, 3 , 0);
	cv::resizeWindow("F_iic", 160, 120);
	DISPLAY_IMAGE_XY(true, F_mag, 4 , 0);
	cv::resizeWindow("F_mag", 160, 120);
	DISPLAY_IMAGE_XY(true, F_ang, 5 , 0);
	cv::resizeWindow("F_ang", 160, 120);
	DISPLAY_IMAGE_XY(true, F_lbp, 6 , 0);
	cv::resizeWindow("F_lbp", 160, 120);
}

void CalculateIIC(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic){
	
	//Reference on passing parameters-by-reference in C/C++: http://stackoverflow.com/questions/11235187/opencv-changing-mat-inside-a-function-mat-scope
	//Reference on adding matrices without saturation: http://answers.opencv.org/question/13769/adding-matrices-without-saturation/
	  Cr.convertTo(Cr, CV_32S);
	  Cb.convertTo(Cb, CV_32S);
	  a.convertTo(a, CV_32S);
		//cv::addWeighted(Cr, 0.5, Cb, 0.5, 0.0, iic, CV_32S);
	  cv::addWeighted((Cr+Cb), 0.25, a, 0.5, 0.0, iic, CV_32S);
	  iic.convertTo(iic, CV_8UC1);
}



void CalculateMagnitudeOrientationOfGradients()
{
	Scharr(gray, temp_grad[0], gray.depth(), 1, 0, 1, 0, BORDER_DEFAULT);
	convertScaleAbs(temp_grad[0], sobel[1], 1, 0);

	Scharr(gray, temp_grad[1], gray.depth(), 0, 1, 1, 0, BORDER_DEFAULT);
	convertScaleAbs(temp_grad[1], sobel[2], 1, 0);

	Mat abs_grad_x = abs(temp_grad[0]);
	Mat abs_grad_y = abs(temp_grad[1]);
	F_mag = abs_grad_x + abs_grad_y;
	F_mag = 255 - F_mag;
	F_ang = 0*F_mag;
	float result;
	//ANG = atan2( temp_grad[1] ,temp_grad[0]);
	for (int y = 0; y < temp_grad[1].rows; y++) 
	{
		for (int x = 0; x < temp_grad[1].cols; x++) 
		{
			float valueX = sobel[1].at<uchar>(y,x);
			float valueY = sobel[2].at<uchar>(y,x);
			float result = fastAtan2(valueY,valueX);
			F_ang.at<uchar>(y, x) = (uchar)result;
		}
	}

	normalize(F_mag, F_mag, 0, 255, CV_MINMAX);
	convertScaleAbs(F_mag, F_mag, 1, 0); 
	normalize(F_ang, F_ang, 0, 255, CV_MINMAX);
	convertScaleAbs(F_ang, F_ang, 1, 0);	
}

Mat CalculateLBP(Mat img){
    Mat dst = Mat::zeros(img.rows-2, img.cols-2, CV_8UC1);
    for(int i=1;i<img.rows-1;i++) {
        for(int j=1;j<img.cols-1;j++) {
            uchar center = img.at<uchar>(i,j);
            unsigned char code = 0;
            code |= ((img.at<uchar>(i-1,j-1)) > center) << 7;
                code |= ((img.at<uchar>(i-1,j)) > center) << 6;
            code |= ((img.at<uchar>(i-1,j+1)) > center) << 5;
            code |= ((img.at<uchar>(i,j+1)) > center) << 4;
            code |= ((img.at<uchar>(i+1,j+1)) > center) << 3;
            code |= ((img.at<uchar>(i+1,j)) > center) << 2;
            code |= ((img.at<uchar>(i+1,j-1)) > center) << 1;
            code |= ((img.at<uchar>(i,j-1)) > center) << 0;
            dst.at<uchar>(i-1,j-1) = code;
        }
    }
    return dst;
}


void Histogram(cv::Mat src, cv::Mat& hist){
              
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
  imshow("histograma",hist);
      
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
	CalculateIIC(F_Cr, F_Cb, F_a, F_iic); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
	normalize(F_iic, F_iic, 0, 255, CV_MINMAX);

	src=frame;
	//sobel_derivatives(src,src_out); // Solver Derivative Calculation
	
	CalculateMagnitudeOrientationOfGradients();

	F_lbp = CalculateLBP(gray);
	
//	Histogram(src,histImage); // Histogram Calculation
}


void CameraSetup()
{
	cap = VideoCapture(1); // Declare capture form Video: "eng_stat_obst.avi"
	//VideoCapture cap(0); //Camera integrada de la computadora
	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
	proc_W = 160;//160
	proc_H = 120;//120
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
	// Good reference: http://superuser.com/questions/897420/how-to-know-which-framerate-should-i-use-to-capture-webcam-with-ffmpeg
	cap.set(CV_CAP_PROP_FPS, 15);	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
    namedWindow( "view", 1 );
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        image_pub=cv_ptr->image;
        //image_pub=cv_bridge::toCvShare(msg, "bgr8")->image;
            image_pub2 = cvCreateImage(cvSize(160,120), IPL_DEPTH_8U, 3);
    image_pub2->imageData = (char *) image_pub.data;
        imshow("view",image_pub);
        // imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //  waitKey(30);       

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void SuperPixels(cv::Mat src)
{
  namedWindow( "SuperPixels", 1 ); 
  frame2 = cvCreateImage(cvSize(160,120), IPL_DEPTH_8U, 3);
  frame2->imageData = (char *) src.data;
  
      /* Yield the number of superpixels and weight-factors from the user. */
  IplImage *lab_image = cvCloneImage(frame2);
  cvCvtColor(frame2, lab_image, CV_BGR2Lab);
  int w = frame2->width, h = frame2->height;
  //int nr_superpixels = atoi(argv[2]);
  int nr_superpixels = 400;
  //int nc = atoi(argv[3]);
  int nc = 40;
  double step = sqrt((w * h) / (double) nr_superpixels)*3;

  /* Perform the SLIC superpixel algorithm. */
 
  Slic slic;
  slic.generate_superpixels(lab_image, step, nc);
  slic.create_connectivity(lab_image);
  slic.store_superpixels(frame2);
  slic.display_contours(frame2, CV_RGB(255,0,0));
  //slic.display_center_grid(frame2, CV_RGB(0,255,0));
  slic.display_number_grid(frame2, CV_RGB(0,255,0));
  slic.show_histograms(1,20);
  //slic.calculate_histograms(frame2);
  //slic.colour_with_cluster_means(frame2);
  cvShowImage("SuperPixels", frame2);
  cvWaitKey(10);
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
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1); //Publisher
  ros::Rate loop_rate(5);
  
	CameraSetup();
  waitKey(1);
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback); // Subscriber Function
  
  CvScalar s; 
  
	while (nh.ok()) 
	{
		//cap >> frame; // Image from Cam to Mat Variable
		if (!cap.read(frame)) 
		{
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}
		cv:resize(frame, frame, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(frame, gray, CV_BGR2GRAY); // Convert to GrayScale
		waitKey(1); // Wait Time
    

		if( prevgray.data )
		{		  
			CalculateImageFeatures();       
			ShowImages();
				
			
			//Publisher	Code
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", F_lbp).toImageMsg();
      pub.publish(msg);
      waitKey(1);
      ////////////////
    //   IplImage* frame_ipl = cvCreateImage(cvSize(frame.cols,frame.rows), IPL_DEPTH_8U, 3);
    //   frame_ipl->imageData = (char *) frame.data;
    //   cvShowImage("frame_prueba",frame_ipl);
      
    //   int A=(((frame.cols/4)*3)-(frame.cols/4))-1;
    //   int B=((frame.rows-(frame.rows/12))-((frame.rows/4)*3))-1;
    //   IplImage* frame_window=NULL;
    //   frame_window=cvCreateImage(cvSize(A,B),IPL_DEPTH_8U,3);
      
    //   for(int y = 0; y < frame.rows; y=y+1)
    //   {
    //       for(int x = 0; x < frame.cols; x=x+1)
    //     	{
    //           s=cvGet2D(frame_ipl,y,x); 
    //           if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows/4)*3)
    //           {
    //               s.val[0]=75;
    //               s.val[1]=248;
    //               s.val[2]=251;
    //               cvSet2D(frame_ipl,y,x,s);
    //           } 
              
    //           if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows-(frame.rows/12)))
    //           {
    //               s.val[0]=75;
    //               s.val[1]=248;
    //               s.val[2]=251;
    //               cvSet2D(frame_ipl,y,x,s);
                                   
    //           } 
              
    //           if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4))
    //           {
    //               s.val[0]=75;
    //               s.val[1]=248;
    //               s.val[2]=251;
    //               cvSet2D(frame_ipl,y,x,s);
                               
    //           }
              
    //           if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4)*3)
    //           {
    //               s.val[0]=75;
    //               s.val[1]=248;
    //               s.val[2]=251;
    //               cvSet2D(frame_ipl,y,x,s);
                               
    //           }
    //       }                               
    //   }
    //  cvShowImage("frame_prueba",frame_ipl);
     
     
    //  for(int y = 0; y < B; y=y+1)
    //   {
    //       for(int x = 0; x < A; x=x+1)
    //     	{
              
             
    //            s=cvGet2D(frame_ipl,y,x); 
    //                s.val[0]=0;
    //               s.val[1]=0;
    //               s.val[2]=0;
    //            cvSet2D(frame_window,y,x,s);
              
               
              
              
           
    //       }                               
    //   }  
     
    //   int xx=0;
    //   int yy=0;
    //   for(int y = 0; y < frame.rows; y=y+1)
    //   {
    //       for(int x = 0; x < frame.cols; x=x+1)
    //     	{
              
    //           if (x>=(frame.cols/4)+1 && x<=(frame.cols/4)*3-1 && y>=((frame.rows/4)*3)+1 && y<=(frame.rows-(frame.rows/12)-1))
    //           {
    //            s=cvGet2D(frame_ipl,y,x); 
    //            cvSet2D(frame_window,yy,xx,s);
    //            xx=xx+1;
    //            if(xx>((frame.cols/4)*3)-(frame.cols/4)-2)
    //            {
    //            xx=0;
    //            yy=yy+1;
    //            } 
    //           } 
              
           
    //       }                               
    //   }      
    // //  cvShowImage("f2",frame_window); 
      
    //   cvShowImage("f2",frame_window); 
    //   hist_prueba=frame_window;
    //   Histogram(hist_prueba,histImage2);
    //   imshow("histograma_ventana",histImage2);
      
      
      
      
      SuperPixels(frame);	
		}



		if(waitKey(30)>=0)
		{
			break;
		}
		std::swap(prevgray, gray);
		waitKey(1);
	  ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
