
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
cv::Mat temp_grad[3], sobel[3], F_mag, F_ang;


double proc_W, proc_H;
VideoCapture cap;

Slic slic;
vector<Superpixel> superpixel_list;


void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0 , 0);
	cv::resizeWindow("frame", proc_W, proc_H);
}


void CalculateMagnitudeOrientationOfGradients()
{
	cv::Scharr(gray, temp_grad[0], gray.depth(), 1, 0, 1, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[0], sobel[1], 1, 0);

	cv::Scharr(gray, temp_grad[1], gray.depth(), 0, 1, 1, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[1], sobel[2], 1, 0);

	cv::Mat abs_grad_x = abs(temp_grad[0]);
	cv::Mat abs_grad_y = abs(temp_grad[1]);
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
		waitKey(1); // Wait Time

		
		waitKey(1);
		seg_image = frame.clone();
		superpixels(seg_image);
		CV_TIMER_STOP(B, "Superpixels processed")

		CV_TIMER_STOP(C, "Prior probability calculated")
		showImages();


		// if(waitKey(30)>=0)
		// {
		// 	break;
		// }
		// waitKey(1);
		CV_TIMER_STOP(D, "Loop finished")
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
