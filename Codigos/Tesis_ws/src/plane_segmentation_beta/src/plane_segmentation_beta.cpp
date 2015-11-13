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

//#include "img_proc_fcns.h" //various image processing/computer vision functions

#include "../slic_mod/slic.h"
#include "../src/functions.h"

#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;

Mat frame, gray, prevgray, F_hsv, F_YCrCb, F_lab, F_sat, F_hue, F_Cr, F_Cb, F_a, channel_1[4], channel_2[4], channel_3[4], F_iic, src, src_out, dst, histImage, temp_grad[3], sobel[3], F_mag, F_ang, F_lbp, image_pub, hist_prueba, histImage2, superpixel_img, Cluster, Window_Cluster, frame_out, Save_Window; // Mat Declarations

IplImage *image_pub2;

double proc_W, proc_H;
VideoCapture cap;

Slic slic;


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
	
	

	DISPLAY_IMAGE_XY(true, frame, 0 , 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	//DISPLAY_IMAGE_XY(true, gray, 1 , 0);
	//DISPLAY_IMAGE_XY(true, F_hue, 1 , 0);
	cv::resizeWindow("F_hue", 160, 120);
	//cv::resizeWindow("F_hue", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_sat, 2 , 0);
	cv::resizeWindow("F_sat", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_iic, 3 , 0);
	cv::resizeWindow("F_iic", 160, 120);
	DISPLAY_IMAGE_XY(true, F_mag, 4 , 0);
	cv::resizeWindow("F_mag", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_ang, 5 , 0);
	cv::resizeWindow("F_ang", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_lbp, 6 , 0);
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


// Can be optimized with intrinsics!!!
Mat CalculateLBP(Mat img)
{
  Mat dst = Mat::zeros(img.rows-2, img.cols-2, CV_8UC1);
  const int dx[8] = {-1, -1, -1, 0, +1, +1, +1, 0};
  const int dy[8] = {-1, 0, +1, +1, +1, 0, -1, -1};
  uchar center, code, periphery_value;

  for(int i=1; i<img.rows-1 ;i++)
  {
    for(int j=1; j<img.cols-1 ;j++)
    {
        center = img.ptr<uchar>(i)[j];
        code = 0;
        for(int k=0; k<8 ;k++)
        {
          periphery_value = (img.ptr<uchar>(i+dx[k]))[j+dy[k]];
          code |= (periphery_value > center) << 7-k;
      
        }
        (dst.ptr<uchar>(i-1))[j-1] = code;
    }
  }
  return dst;
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
        cvReleaseImage(&image_pub2);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



void SuperPixels(cv::Mat src)
{

    

  Mat sub2=src.clone();
  Mat sub=src.clone();   
  Mat img_out1,img_out2,img_out_pixels;
  int xx,yy,r,g,b,bb,rr,gg,r_seed,g_seed,b_seed;
  int superpixels_x, superpixels_y;  
  Vec3b color,color_seed;
  Mat img_out_seed,img_out_seed2;

  IplImage frame2 = (IplImage)sub;  // deallocating memory
  IplImage frame3 = (IplImage)sub2; // deallocating memory
  
  /* Yield the number of superpixels and weight-factors from the user. */
  
  IplImage *lab_image = cvCloneImage(&frame2);
  cvCvtColor(&frame2, lab_image, CV_BGR2Lab);
  int w = lab_image->width, h = lab_image->height;
  int nr_superpixels = 400;

  int nc = 40;
  double step = sqrt((w * h) / (double) nr_superpixels)*3;

  /* Perform the SLIC superpixel algorithm. */
 
  slic.clear_data();
  slic.generate_superpixels(lab_image, step, nc);
  slic.create_connectivity(lab_image);
  slic.colour_with_cluster_means(&frame3);
  Cluster=&frame3;
  slic.store_superpixels(&frame2);
  //slic.calculate_histograms(&frame2);

  slic.export_superpixels_to_files(&frame2);
  slic.display_contours(&frame2, CV_RGB(255,0,0));
  slic.display_number_grid(&frame2, CV_RGB(0,255,0));

 
  Mat  img_ventanita1,img_ventanita2,seed1,seed2;
  Mat img_prueba1,img_prueba2;
  int xxx,yyy;
  slic.export_superpixels_data(19,src,img_ventanita1,img_prueba1,xxx,yyy,seed1);
  slic.export_superpixels_data(33,src,img_ventanita2,img_prueba2,xxx,yyy,seed2);
  imshow("src",src);
  imshow("img_prueba",img_prueba1);
   imshow("img_prueba2",img_prueba2);
  Mat Superpixel=&frame2; 
  Mat result;
  //CompareCluster(src,Cluster,result);

  // img_ventanita1,img_ventanita2,seed1,seed2;
  //Mat img_prueba1,img_prueba2;
  //int xxx,yyy;
  //slic.export_superpixels_data(&frame2,19,Cluster,img_ventanita1,img_prueba1,xxx,yyy,seed1);
  // slic.export_superpixels_data(&frame2,24,Cluster,img_ventanita2,img_prueba2,xxx,yyy,seed2);
 
  Mat histred, histgreen, histblue;
  float B_max_x,B_max_y,G_max_x,G_max_y,R_max_x,R_max_y; 
  float B_max_x2,B_max_y2,G_max_x2,G_max_y2,R_max_x2,R_max_y2; 
  Histogram(img_prueba1,histred,histgreen,histblue,B_max_x,B_max_y,G_max_x,G_max_y,R_max_x,R_max_y);
  imshow("histred1",histred);
   imshow("histblue1",histblue); 
      imshow("histgreen1",histgreen); 
  Histogram(img_prueba2,histred,histgreen,histblue,B_max_x2,B_max_y2,G_max_x2,G_max_y2,R_max_x2,R_max_y2);  
    imshow("histred2",histred); 
       imshow("histblue2",histblue); 
      imshow("histrgreen2",histgreen); 
  cout << R_max_x << ";" << R_max_y << ";" << R_max_x2 << ";" << R_max_y2 << endl;
 
   
  imshow("Superpixels",Superpixel); // Image Frame with Superpixels 
  //imshow("Result",result);
  imshow("Cluster",Cluster);

}


void CameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

 // cap = VideoCapture("LaboratorioMaleta.avi");
    cap = VideoCapture("PasilloLabA.avi");
	//cap = VideoCapture("eng_stat_obst.avi");
	

	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
	proc_W = 160;//160
	proc_H = 120;//120
	//proc_W = 320;//160
	//proc_H = 240;//120
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	// Good reference: http://superuser.com/questions/897420/how-to-know-which-framerate-should-i-use-to-capture-webcam-with-ffmpeg
	cap.set(CV_CAP_PROP_FPS, 30);
}


int threshold_value = 0;
int threshold_type = 0;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src_F_Mag,dst_filtered_F_Mag;
char* window_name = "Threshold Demo";

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

void Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold( src_F_Mag, dst_filtered_F_Mag, threshold_value, max_BINARY_value,threshold_type );
  imwrite( "/home/rafael/VisionPercepcion_USB2015/Codigos/Tesis_ws/src/plane_segmentation_beta/src/F_Mag_Image.jpg", dst_filtered_F_Mag );
  imshow( window_name, dst_filtered_F_Mag );
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
  ros::Rate loop_rate(40);
  
	CameraSetup();
	cap.read(frame);
  waitKey(10);
  //image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback); // Subscriber Function
  
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
		waitKey(10); // Wait Time
    




		if( prevgray.data )
		{		  
  //DISPLAY_IMAGE_XY(true, frame, 0 , 0);
  //cv::resizeWindow("frame", proc_W, proc_H);
//      waitKey(1);

		  CalculateImageFeatures(); 
			DrawSaveWindow(frame,frame_out);//Draw the Yellow Save Window And Return the Same Image with the Yellow Save Window
			//imshow("Yellow_Window",frame_out);
			SuperPixels(frame);	
      CreateSaveWindow(frame,Save_Window); // Create a Image of the Save Window
      SuperPixels(frame);// Calculate the Superpixels  
      CreateWindowCluster(Cluster,Window_Cluster);// Get the Save Window of the Cluster Image 
 /*     
    	Mat F_Mag=F_mag.clone();
      Size size(640,480);
      resize(F_Mag,F_Mag,size);
      //imshow("F_MAG",F_Mag);

      Mat FMAG=F_mag.clone();
      Vec3b ColorBase;
      ColorBase[0]=0;
      ColorBase[1]=0;
      ColorBase[2]=0;
      
      Vec3b Cub_1,Cub_2,Cub_3,Cub_4,Cub_5,Cub_6;

      
      
           for(int y = F_mag.rows-1; y>1; y=y-1)
              {
                 for(int x = F_mag.cols-1; x >1  ; x=x-1)
                	   {
           
                             Cub_1=FMAG.at<Vec3b>(y, x);  
                             Cub_2=FMAG.at<Vec3b>(y, x-1); 
                             Cub_3=FMAG.at<Vec3b>(y-1, x); 
                             Cub_4=FMAG.at<Vec3b>(y-1, x-1);
                             Cub_5=FMAG.at<Vec3b>(y-1, x-2);
                             Cub_6=FMAG.at<Vec3b>(y, x-2); 
                             double Promedio=(Cub_1[0]+Cub_1[1]+Cub_1[2]+Cub_2[0]+Cub_2[1]+Cub_2[2]+Cub_3[0]+Cub_3[1]+Cub_3[2]+Cub_4[0]+Cub_4[1]+Cub_4[2]+Cub_5[0]+Cub_5[1]+Cub_5[2]+Cub_6[0]+Cub_6[1]+Cub_6[2])/18;
                             if (Promedio==0)
                              {
                              FMAG.at<Vec3b>(y-2, x)= ColorBase;
                              FMAG.at<Vec3b>(y-2, x-1)= ColorBase;
                              FMAG.at<Vec3b>(y-2, x-2)= ColorBase;
                              } 
                             
 
                       }                          
               } 
               */
      //imshow("MAG",FMAG);
  
   src_F_Mag=F_mag.clone();     
 dst_filtered_F_Mag=F_mag.clone();


  /// Create Trackbar to choose type of Threshold

                  
    
  /// Call the function to initialize
  Threshold_Demo( 0, 0 );  
  
  
  
  
 
    Mat dst_canny, cdst;
    cdst=frame.clone();
    Canny(dst_filtered_F_Mag, dst_canny, 50, 200, 3); 
 
     vector<Vec4i> lines;
    HoughLinesP( dst_canny, lines, 1, CV_PI/180, 30, 30, 10);
    

    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( cdst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }

    
  imshow("canny",cdst);
  
  Mat red_line=frame.clone();
int init_point_x=0;
int init_point_y=(frame.rows/2)+10;
int final_point_x,final_point_y;
double rect_distance;
double rect_distance_base=1000;
Vec3b red_base;
int base_1,base_2,base_3;
cout << cdst.cols << endl;
             for(int x = 40 ; x < cdst.cols ; x=x+1)
              {
                 
                  for(int y = 0; y < cdst.rows; y=y+1)
                	   {
                      red_base=cdst.at<Vec3b>(y, x); 
                      base_1=red_base[0];
                                            base_2=red_base[1];
                                                                  base_3=red_base[2];
                      if ((base_1==0) && (base_2==0) && (base_3==255))
                        {
                        double value_a=(x-init_point_x)*(x-init_point_x);
                        double value_b=(y-init_point_y)*(y-init_point_y);
                        rect_distance=sqrt(abs(value_a)+abs(value_b));
                        //cout << "hola" << endl;
                        if(rect_distance<rect_distance_base)
                        {
                        rect_distance_base=rect_distance;
                        final_point_x=x;
                        final_point_y=y;
                        
                         } 
                                }
 
                             
 
                       } 
                                                    line( red_line, Point(init_point_x, init_point_y),
            Point(final_point_x, final_point_y), Scalar(0,0,255), 3, 8 ); 
            init_point_x=final_point_x;
            init_point_y=final_point_y;  
            rect_distance_base=1000; 
                  } 
  imshow("red_lines",red_line);
			ShowImages();
				

/*
   int frame_width=   cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   VideoWriter video("PasilloLabB.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

   for(;;){

       cap >> frame;
       video.write(frame);
       imshow( "Frame", frame );
       char c = (char)waitKey(33);
       if( c == 27 ) break;
    }
    */
			//Publisher	Code
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", F_lbp).toImageMsg();
      pub.publish(msg);
      waitKey(1);
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
