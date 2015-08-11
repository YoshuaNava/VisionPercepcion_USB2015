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
 #include "ms_communications_loop.h"

 
 #define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
 #define Window_H 1.3*(proc_H)+20
 #define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(#img, img);
 using namespace std;
 using namespace cv;



double proc_W, proc_H;

void combine_channels(IplImage* Cr, IplImage* Cb, IplImage* a, IplImage* iic){
    unsigned char * CrPixelData = (unsigned char *)(Cr->imageData);
    unsigned char * CbPixelData = (unsigned char *)(Cb->imageData);
    unsigned char * aPixelData = (unsigned char *)(a->imageData);
    unsigned char * iicPixelData = (unsigned char *)(iic->imageData);
    
    cvZero(iic);
    
    for (int y = 0; y < iic->height; y++) {
        for (int x = 0; x < iic->width; x++) {
            
            //int Cr_index = (y*Cr.width+x);
            //int Cb_index = (y*Cb.width+x);
            //int a_index = (y*a.width+x);
            int index = (y*iic->width+x);
            iicPixelData[index] = cvRound((CrPixelData[index] + CbPixelData[index] + 2*(aPixelData[index]))/4);
        }
    }
  
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
    
    Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue, F_Cr, F_Cb, F_a, channel_1[4], channel_2[4], channel_3[4], F_iic; // Mat Declarations
    IplImage *F_hsv2, *F_YCrCb2, *F_lab2, *F_sat2, *F_hue2, *F_Cr2, *F_Cb2, *F_a2, *F_iic2,  *gray_image, *frame2; // Iplimage* Declarations
    bool disp_img, refresh;
       
    VideoCapture cap(0); // Declare capture form
    
    namedWindow( "HSV", 1 ); 
    namedWindow( "YCrCb", 1 ); 
    namedWindow( "Lab", 1 ); 
    namedWindow( "F_hue", 1 ); 
    namedWindow( "F_sat", 1 ); 
    namedWindow( "F_Cb", 1 ); 
    namedWindow( "F_Cr", 1 ); 
    namedWindow( "F_a", 1 ); 
    cvNamedWindow( "Prueba", 1); 
    //Window Names
   
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
          
          frame2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
          frame2->imageData = (char *) frame.data;
          
          F_hsv2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
          F_YCrCb2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
          F_lab2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
                    
                    
          F_hue2= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
          F_sat2= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
          
          F_Cr2=cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
          F_Cb2= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
                
          cvCvtColor( frame2, F_hsv2, 	CV_BGR2HSV );	//Convert to HSV
          cvCvtColor( frame2, F_YCrCb2, CV_BGR2YCrCb );	//Convert to YCrCb
          cvCvtColor( frame2, F_lab2, 	CV_BGR2Lab );	//Convert to Lab
          
          cvCvtPixToPlane( F_hsv2, F_hue2, F_sat2, 0, 0 );
          cvCvtPixToPlane( F_YCrCb2, 0, F_Cr2, F_Cb2, 0 );
          
          
          F_iic2= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
          F_iic2->imageData = (char *) frame.data;
          combine_channels(F_Cr2, F_Cb2, F_a2, F_iic2); //D = (A + B + 2*C)/4 //illumination invariant color channel combination

//F_Cb2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
//F_Cb2->imageData = (char *) F_Cb.data;

//F_a2=cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
//F_a2->imageData = (char *) F_a.data;

//F_iic2 = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);


              //combine_channels(F_Cr2, F_Cb2, F_a2, F_iic2); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
              //combine_channels(F_Cr, F_Cb, F_a, F_iic);
              // cvCvtPixToPlane( F.hsv, F.hue, F.sat, 0, 0 );
              //cvCvtPixToPlane( F.YCrCb, 0, F.Cr, F.Cb, 0 );
              //c
             /* 
              IplImage* image_pub; // Transfer Mat to IplImage   
              image_pub=cvCreateImage(cvSize(F_hue.cols,F_hue.rows),8,3); // Rezise
              IplImage ipltemp=F_hue;
               cvCopy(&ipltemp,image_pub);
*/
              imshow("HSV",F_hsv);
              imshow("YCrCb",F_YCrCb);
              imshow("Lab",F_lab);
              imshow("F_hue",F_hue);
              imshow("F_sat",F_sat);
              imshow("F_Cr",F_Cr);
              imshow("F_Cb",F_Cb);
              imshow("F_a",F_a);
              //imshow("Prueba",frame);
             cvShowImage("Prueba",F_Cb2);
        
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
