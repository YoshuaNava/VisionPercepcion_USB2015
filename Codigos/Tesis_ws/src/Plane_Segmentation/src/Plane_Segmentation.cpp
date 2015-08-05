 //opencv header files
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "image_transport/image_transport.h"
 #include "sensor_msgs/image_encodings.h"
 #include "cv_bridge/cv_bridge.h"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/video/tracking.hpp"
 #include "opencv2/core/core.hpp"
 #include "iostream"
 #include "stdio.h"
 #include "stdlib.h"
 #include "math.h"
 #include "string.h"
 #include "iomanip"
 #include <opencv/cv.h>
 #include <opencv/highgui.h>
 #include <opencv/cxcore.h>

 #include <stdio.h> //standard input-output-library
 #include <algorithm> //functions on sequences, like find, sort etc. 
 #include <stdlib.h> //includes random number generation
 #include <time.h> //get date and time information
 #include <string.h> //manipulate strings and arrays

 #include "init_structures.h" //initialise structures used in this algorithm
 #include "img_proc_fcns.h" //various image processing/computer vision functions
 #include "capture.h" //image capture 
 #include "ms_overwrite_safe_buffer.h"
 #include "ms_communications_loop.h"
 #include <iostream>
 #include <fstream>
 #include <vector>
 #include <thread>
 
 #define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
 #define Window_H 1.3*(proc_H)+20
 #define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(#img, img);
 using namespace std;
 using namespace cv;



double proc_W, proc_H;



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
    
    Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue; // Mat Declarations
    IplImage *F_hsv2, *F_YCrCb2, *F_lab2, *F_sat2, *F_hue2; // Iplimage* Declarations
    bool disp_img, refresh;
   
    VideoCapture cap(1); // Declare capture form
    namedWindow( "HSV", 1 ); 
    namedWindow( "YCrCb", 1 ); 
    namedWindow( "Lab", 1 ); 
    namedWindow( "Prueba", 1 ); 
    //Window Names
   
    while (nh.ok()) 
    {
    
          cap >> frame; //Capture from webcam to Mat image
          IplImage* image_pub; // Transfer Mat to IplImage
          
          image_pub=cvCreateImage(cvSize(frame.cols,frame.rows),8,3); // Rezise
          IplImage ipltemp=frame;
          cvCopy(&ipltemp,image_pub);
         
          cap >> frame;
          cvtColor(frame, gray, CV_BGR2GRAY); // Convert to GrayScale
          waitKey(1); // Wait Time
          
          
          
          int input=cvWaitKey(40);
          if ((char)input==32)
          {
              std::swap(prevgray, gray);
          }
          
          if( prevgray.data )
          {
          
              cvtColor( frame, F_hsv, 	CV_BGR2HSV );	//Convert to HSV
              cvtColor( frame, F_YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
              cvtColor( frame, F_lab, 	CV_BGR2Lab );	//Convert to Lab
              F_hsv2=cvCreateImage(cvSize(F_hsv.cols,F_hsv.rows),8,3); // Rezise
              IplImage ipltemp=F_hsv;
              cvCopy(&ipltemp,F_hsv2);
              cvShowImage("Prueba",F_hsv2);
              //cvCvtPixToPlane( F_hsv2, F_hue2, F_sat2, 0, 0 );
            /*  
            
              Para realizar esta parte del codigo, hay que hacer la conversión respectiva a IplOmage.
              cvtColor( image_pub, F_hsv, 	CV_BGR2HSV );	//Convert to HSV
              cvCvtColor( image_pub, F_YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
              cvCvtColor( image_pub, F_lab, 	CV_BGR2Lab );	//Convert to Lab
              cvCvtPixToPlane( F_hsv, F_hue, F_sat, 0, 0 );
              
              cvShowImage("Lab",image_pub);
              //view iamges separately
              /*
              if (disp_img){	DISPLAY_IMAGE_XY(refresh, F_hue, 2, 0);	 
                            DISPLAY_IMAGE_XY(refresh, F_sat, 3, 0); }
                            */
              imshow("HSV",F_hsv);
              imshow("YCrCb",F_YCrCb);
              imshow("Lab",F_lab);
              //cvShowImage("Lab",image_pub);
          
          }
          
         if(waitKey(30)>=0)
         break;
         std::swap(prevgray, gray);
         waitKey(1);
           
        
    }
    
    ros::spinOnce();
    cap.release(); //Destroy the Capture from webcam
    destroyAllWindows(); //Destroy the windows
    

    ros::spin();

    return 0;
}
