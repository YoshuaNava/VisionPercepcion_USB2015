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
#include "../src/slic.h"


#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;

Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue, F_Cr, F_Cb, F_a, channel_1[4], channel_2[4], channel_3[4], F_iic, src, src_out, dst, histImage, temp_grad[3], sobel[3], F_mag, F_ang, F_lbp, image_pub, hist_prueba, histImage2,superpixel_img; // Mat Declarations
double valor_hist;

IplImage *frame2, *F_iic2, *image_pub2, *superpixel_img2;



double proc_W, proc_H;
VideoCapture cap;

int cvCreateTrackbar(
const char* trackbar_name,
const char* window_name,
int* posicion,
int count,
CvTrackbarCallback on_change
);


void ShowImages()
{
	imshow("Frame",frame);
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
  
      
}


void CalculateImageFeatures()
{
	cvtColor( frame, F_hsv, 	CV_BGR2HSV );	//Convert to HSV
	cvtColor( frame, F_YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
	cvtColor( frame, F_lab, 	CV_BGR2Lab );	//Convert to Lab

	cv::split(F_hsv,channel_1); //cvsplit Divides a multi-channel array into separate single-channel arrays
	F_hue=channel_1[0];
	Scalar intensity = F_hue.at<uchar>(2, 2);
	cout << intensity.val[0] << endl;
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
	
	Histogram(src,histImage); // Histogram Calculation
}


void CameraSetup()
{
	cap = VideoCapture(1); // Declare capture form Video: "eng_stat_obst.avi"
	//VideoCapture cap(0); //Camera integrada de la computadora
	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
	proc_W = 160;//160
	proc_H = 120;//120
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);//640
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);//360
	// Good reference: http://superuser.com/questions/897420/how-to-know-which-framerate-should-i-use-to-capture-webcam-with-ffmpeg
	cap.set(CV_CAP_PROP_FPS, 15);	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        image_pub=cv_ptr->image;
        //image_pub=cv_bridge::toCvShare(msg, "bgr8")->image;
        image_pub2 = cvCreateImage(cvSize(160,120), IPL_DEPTH_8U, 3);
        image_pub2->imageData = (char *) image_pub.data;
        //imshow("view",image_pub);
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
  superpixel_img2 = cvCreateImage(cvSize(160,120), IPL_DEPTH_8U, 3);
  superpixel_img2=frame2;
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
  slic.display_contours(superpixel_img2, CV_RGB(255,0,0));
  cvShowImage("SuperPixels", superpixel_img2);
  
  slic.colour_with_cluster_means(frame2);
  	  cvShowImage("Cluster", frame2);

  cvWaitKey(10);
}

void Comparison(cv::Mat src_base, cv::Mat src_test1, cv:: Mat src_test2){
    Mat hsv_base;
    Mat hsv_test1;
    Mat hsv_test2;
    Mat hsv_half_down;

    /// Load three images with different environment settings



    /// Convert to HSV
    cvtColor( src_base, hsv_base, COLOR_BGR2HSV );
    cvtColor( src_test1, hsv_test1, COLOR_BGR2HSV );
    cvtColor( src_test2, hsv_test2, COLOR_BGR2HSV );

    hsv_half_down = hsv_base( Range( hsv_base.rows/2, hsv_base.rows - 1 ), Range( 0, hsv_base.cols - 1 ) );

    /// Using 50 bins for hue and 60 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };


    /// Histograms
    MatND hist_base;
    MatND hist_half_down;
    MatND hist_test1;
    MatND hist_test2;

    /// Calculate the histograms for the HSV images
    calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
    normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_half_down, 1, channels, Mat(), hist_half_down, 2, histSize, ranges, true, false );
    normalize( hist_half_down, hist_half_down, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_test1, 1, channels, Mat(), hist_test1, 2, histSize, ranges, true, false );
    normalize( hist_test1, hist_test1, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_test2, 1, channels, Mat(), hist_test2, 2, histSize, ranges, true, false );
    normalize( hist_test2, hist_test2, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Apply the histogram comparison methods
    for( int i = 0; i < 4; i++ )
    {
        int compare_method = i;
        double base_base = compareHist( hist_base, hist_base, compare_method );
        double base_half = compareHist( hist_base, hist_half_down, compare_method );
        double base_test1 = compareHist( hist_base, hist_test1, compare_method );
        double base_test2 = compareHist( hist_base, hist_test2, compare_method );

        printf( " Method [%d] Perfect, Base-Half, Base-Test(1), Base-Test(2) : %f, %f, %f, %f \n", i, base_base, base_half , base_test1, base_test2 );
        if(i==0)
        {
        valor_hist=base_test2;
        }
    }

    printf( "Done \n" );
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
  int bandera=0;
  namedWindow( "TrackBar", 1 );
  cvCreateTrackbar("Eje B", "TrackBar", &bandera, 5 );
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
      
      IplImage* frame_ipl = cvCreateImage(cvSize(frame.cols,frame.rows), IPL_DEPTH_8U, 3);
      frame_ipl->imageData = (char *) frame.data;
      
      
      int A=(((frame.cols/4)*3)-(frame.cols/4))-1;
      int B=((frame.rows-(frame.rows/12))-((frame.rows/4)*3))-1;
      IplImage* frame_window=NULL;
      frame_window=cvCreateImage(cvSize(A,B),IPL_DEPTH_8U,3);
      
      for(int y = 0; y < frame.rows; y=y+1)
      {
          for(int x = 0; x < frame.cols; x=x+1)
        	{
              s=cvGet2D(frame_ipl,y,x); 
              if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows/4)*3)
              {
                  s.val[0]=75;
                  s.val[1]=248;
                  s.val[2]=251;
                  //cvSet2D(frame_ipl,y,x,s);
              } 
             
              if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows-(frame.rows/12)))
              {
                  s.val[0]=75;
                  s.val[1]=248;
                  s.val[2]=251;
                  //cvSet2D(frame_ipl,y,x,s);
                                   
              } 
              
              if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4))
              {
                  s.val[0]=75;
                  s.val[1]=248;
                  s.val[2]=251;
                  //cvSet2D(frame_ipl,y,x,s);                               
              }
              
              if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4)*3)
              {
                  s.val[0]=75;
                  s.val[1]=248;
                  s.val[2]=251;
                  //cvSet2D(frame_ipl,y,x,s);                             
              }
          }                               
      }
 
     
     
     for(int y = 0; y < B; y=y+1)
      {
          for(int x = 0; x < A; x=x+1)
        	{           
               s=cvGet2D(frame_ipl,y,x); 
                   s.val[0]=0;
                  s.val[1]=0;
                  s.val[2]=0;
               cvSet2D(frame_window,y,x,s);                     
          }                               
      }  
     
      int xx=0;
      int yy=0;
      for(int y = 0; y < frame.rows; y=y+1)
      {
          for(int x = 0; x < frame.cols; x=x+1)
        	{
              
              if (x>=(frame.cols/4)+1 && x<=(frame.cols/4)*3-1 && y>=((frame.rows/4)*3)+1 && y<=(frame.rows-(frame.rows/12)-1))
              {
               s=cvGet2D(frame_ipl,y,x); 
               cvSet2D(frame_window,yy,xx,s);
               xx=xx+1;
               if(xx>((frame.cols/4)*3)-(frame.cols/4)-2)
               {
               xx=0;
               yy=yy+1;
               } 
              }                          
          }                               
      }      

      
      
      /*
      if (bandera>2)
      {
      cvSaveImage("Save_Window5.png",frame_window);
      }
      */
      
      hist_prueba=frame_window;
      Histogram(hist_prueba,histImage2);
      imshow("Histogram_Save_Window",histImage2);
      
      IplImage* safe_window=NULL;//inicializo imagen
      safe_window=cvLoadImage("/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Save_Window.jpg",1);
      Mat hist_save_window_1,hist_save_window_2;
       /*
      hist_save_window_1=safe_window;
      Histogram(hist_save_window_1,hist_save_window_2);
      imshow("Safe_Window",hist_save_window_2);
       
       Mat model, img1, img2;
       model = imread( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Save_Window.png", 1 );

       img1 = imread( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Save_Window2.png", 1 );
       
       
       //img2 = imread( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Save_Window5.png", 1 );
       Comparison(model,img1,hist_prueba);
       */
  
       SuperPixels(frame);	
       xx=0;
       yy=0;
       IplImage* cluster_img=NULL;
       cluster_img=cvCreateImage(cvSize(A,B),IPL_DEPTH_8U,3);
       Mat img_cluster;

      for(int y = 0; y < frame.rows; y=y+1)
      {
          for(int x = 0; x < frame.cols; x=x+1)
        	{
              
              if (x>=(frame.cols/4)+1 && x<=(frame.cols/4)*3-1 && y>=((frame.rows/4)*3)+1 && y<=(frame.rows-(frame.rows/12)-1))
              {
              
               s=cvGet2D(frame2,y,x); 
               cvSet2D(cluster_img,yy,xx,s);
               xx=xx+1;
               if(xx>((frame.cols/4)*3)-(frame.cols/4)-2)
               {
               xx=0;
               yy=yy+1;
               } 
              } 
              
           
          }                               
      } 
      img_cluster=cluster_img;
      imshow("Cluster_Window",img_cluster);
    /*  
       hist_save_window_1=img_cluster;
       Histogram(hist_save_window_1,hist_save_window_2);
       imshow("Histograma_Cluster",hist_save_window_2);
       */
      if (bandera>2)
      {
      cvSaveImage("Plane2.png",cluster_img);
      }
     
       Mat model, img1, img2;
       model = imread( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Plane1.png", 1 );

       img1 = imread( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/Plane2.png", 1 );
       
    /*   
   hist_save_window_1=img_cluster;
    Histogram(hist_save_window_1,hist_save_window_2);
       Comparison(model,img1,hist_save_window_1);
       */
     Mat ventana;
     Mat img_out;
     img_out=frame;
   ventana=frame;

    /*
      for(int y = 0; y < frame.rows; y=y+1)
      {
          for(int x = 0; x < frame.cols; x=x+1)
        	{
             Vec3b color = frame.at<Vec3b>(y, x);           
             cosa.at<Vec3b>(y, x) = color;
          }                               
      } 
  */
  Size size(79,19);//the dst image size,e.g.100x100
  resize(ventana,ventana,size);//resize image
  yy=0;
  xx=0;
  int vent_x=0;
  int vent_y=0;
    for(int y = 0; y < frame.rows; y=y+20)
        {
          for(int x = 0; x < frame.cols; x=x+80)
        	{ 
                for(int yy = y; yy < y+19; yy=yy+1)
                {
                  for(int xx = x; xx < x+79; xx=xx+1)
                	{
           
                             Vec3b color = frame.at<Vec3b>(yy, xx);           
                              ventana.at<Vec3b>(vent_y, vent_x) = color;
                              
                              vent_x=vent_x+1;
                              
                                                           
                         
                  }
                  vent_x=0;   
                  vent_y=vent_y+1;                           
                }
                
                vent_y=0; 
                vent_x=0;
                hist_save_window_1=ventana;
                Histogram(hist_save_window_1,hist_save_window_2);
                Comparison(model,img1,hist_save_window_1);
                imshow("pruebaprueba",ventana);
                cout << valor_hist << endl;
              
                if(valor_hist>0.1)              
                {
                  for(int yy = y; yy < y+19; yy=yy+1)
                    {
                  for(int xx = x; xx < x+79; xx=xx+1)
                	   {
           
                             Vec3b color = frame.at<Vec3b>(yy, xx);  
                            color.val[0]=255;
                            color.val[1]=255;
                             color.val[2]=255;      
                              img_out.at<Vec3b>(yy, xx) = color;
                              
                             
                              
                                                           
                         
                       }                          
                    }
                }
                
        }                               
          }
          imshow("Plane_Segmentation",img_out);
          /*
      hist_save_window_1=ventana;
      Histogram(hist_save_window_1,hist_save_window_2);
      imshow("holaaa",hist_save_window_1);
      Comparison(model,img1,hist_save_window_1);
     */

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
