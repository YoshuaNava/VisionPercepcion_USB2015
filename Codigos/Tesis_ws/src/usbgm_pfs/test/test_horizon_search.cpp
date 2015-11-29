
#include "../include/global.h"

#include "../src/prob_fns.cpp"
//#include "slic_superpixels/slic.h"



//macros for stopwatch
//REFERENCE: https://sites.google.com/site/mikesapi/downloads/ground-plane-segmentation-and-autonomous-guidance
#define CV_TIMER_START(X)       	double X = (double)cvGetTickCount();
#define CV_TIMER_STOP(Y, STRING) 	double Y = (double)cvGetTickCount() - X; \
                                            printf("Time @ [%s] = %gms\n", \
                                            STRING, Y/((double)cvGetTickFrequency()*1000.) );

#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;


cv::Mat frame, seg_image, gray, prevgray; // Mat Declarations

cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny;
cv::Mat thrsh_sobel;


double proc_W, proc_H;
VideoCapture cap;

Slic slic;
vector<Superpixel> superpixel_list;


void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0 , 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, gray, 1 , 0);
	cv::resizeWindow("gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, borders_sobel, 2 , 0);
	cv::resizeWindow("borders_sobel", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, borders_canny, 3 , 0);
	cv::resizeWindow("borders_canny", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, thrsh_sobel, 4 , 0);
	cv::resizeWindow("thrsh_sobel", proc_W, proc_H);
}


void CalculateMagnitudeOrientationOfGradients()
{
	//GaussianBlur(gray, gray, Size(5, 5), 0, 0 );
	cv::Scharr(gray, temp_grad[0], gray.depth(), 1, 0, 1, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[0], sobel[1], 1, 0);

	cv::Scharr(gray, temp_grad[1], gray.depth(), 0, 1, 1, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[1], sobel[2], 1, 0);

	cv::Mat abs_grad_x = abs(temp_grad[0]);
	cv::Mat abs_grad_y = abs(temp_grad[1]);
	borders_sobel = abs_grad_x + abs_grad_y;
	borders_sobel = 255 - borders_sobel;

	normalize(borders_sobel, borders_sobel, 0, 255, CV_MINMAX);
	convertScaleAbs(borders_sobel, borders_sobel, 1, 0); 
	//F_mag = 255 - F_mag;
}

void CalculateCanny()
{
	int edgeThresh = 1;
	int lowThreshold = 10;
	int const max_lowThreshold = 100;
	int ratio = 5;
	int kernel_size = 3;
	GaussianBlur(gray, borders_canny, Size(5, 5), 0, 0);

	/// Canny detector
	Canny(borders_canny, borders_canny, lowThreshold, lowThreshold*ratio, kernel_size, true);
}




void superpixels(cv::Mat src)
{
  namedWindow( "SuperPixels", 1 ); 
  IplImage frame2 = (IplImage)src; // Reference on deallocating memory: http://stackoverflow.com/questions/12635978/memory-deallocation-of-iplimage-initialised-from-cvmat
  
      /* Yield the number of superpixels and weight-factors from the user. */
  IplImage *lab_image = cvCloneImage(&frame2);
  cvCvtColor(&frame2, lab_image, CV_BGR2Lab);
  int w = lab_image->width, h = lab_image->height;
  //int nr_superpixels = atoi(argv[2]);
  int nr_superpixels = 400;
  //int nc = atoi(argv[3]);
  int nc = 40;
  double step = sqrt((w * h) / (double) nr_superpixels)*3;

  /* Perform the SLIC superpixel algorithm. */
 
  slic.clear_data();
  slic.generate_superpixels(lab_image, step, nc);
  slic.create_connectivity(lab_image);
  	//slic.colour_with_cluster_means(&frame2);
  slic.store_superpixels(&frame2);
  //slic.calculate_histograms(&frame2);
  

  // slic.export_superpixels_to_files(&frame2);
   slic.display_contours(&frame2, CV_RGB(255,0,0));
   slic.display_number_grid(&frame2, CV_RGB(0,255,0));
   superpixel_list = slic.get_superpixels();
  //slic.show_histograms(1,32);

  //slic.display_center_grid(frame2, CV_RGB(0,255,0));
  //slic.calculate_histograms(frame2);

  cvShowImage("SuperPixels", &frame2);
  //cvReleaseImage(&frame2);
  cvReleaseImage(&lab_image);
  cvWaitKey(10);
}


void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

  cap = VideoCapture(0);
	//cap = VideoCapture("eng_stat_obst.avi");
	

	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
//	proc_W = 160;//160
//	proc_H = 120;//120
	proc_W = 320;//160
	proc_H = 240;//120
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	// Good reference: http://superuser.com/questions/897420/how-to-know-which-framerate-should-i-use-to-capture-webcam-with-ffmpeg
	cap.set(CV_CAP_PROP_FPS, 30);
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
	ros::Rate loop_rate(100);

	cameraSetup();
	cap.read(frame);
	waitKey(10);
    
	while (nh.ok()) 
	{
		CV_TIMER_START(X)
		//cap >> frame; // Image from Cam to Mat Variable
		if (!cap.read(frame)) 
		{
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}

		CV_TIMER_STOP(A, "Received image from camera")
		cv:resize(frame, frame, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(frame, gray, CV_BGR2GRAY);
		waitKey(1); // Wait Time

		
		seg_image = frame.clone();
		superpixels(seg_image);
		CV_TIMER_STOP(B, "Superpixels processed")
		CalculateMagnitudeOrientationOfGradients();
		CalculateCanny();
		CV_TIMER_STOP(C, "Magnitude and angle of image gradients calculated")

		cv::adaptiveThreshold(borders_sobel, thrsh_sobel, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, proc_H/2+1, 0);
		CV_TIMER_STOP(D, "Thresholding magnitude of image gradients");

		showImages();
		CV_TIMER_STOP(Z, "Loop finished")
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
