
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

cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny, borders_combined;
cv::Mat img_lines, contoured_img;
vector<Vec4i> lines;
vector<cv::Point> lines_points;
vector<cv::Point> polynomial_fit;


double proc_W, proc_H;
VideoCapture cap;

Slic slic;
vector<Superpixel> superpixel_list;


void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0, 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, gray, 1, 0);
	cv::resizeWindow("gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, borders_combined, 2, 0);
	cv::resizeWindow("borders_combined", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, img_lines, 3, 0);
	cv::resizeWindow("img_lines", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, contoured_img, 4, 0);
	cv::resizeWindow("contoured_img", proc_W, proc_H);
}


void calculateSobel()
{
	GaussianBlur(gray, gray, Size(5, 5), 0, 0 );
	cv::Sobel(gray, temp_grad[0], gray.depth(), 2, 0, 3, 15, 0, BORDER_DEFAULT);
	cv::Sobel(gray, temp_grad[1], gray.depth(), 0, 2, 3, 15, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[0], sobel[0]);
	cv::convertScaleAbs(temp_grad[1], sobel[1]);
	addWeighted(sobel[0], 0.5, sobel[1], 0.5, 0, borders_sobel);

	cv::Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	cv::Mat skel(borders_sobel.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat eroded, dilated;
	cv::erode(borders_sobel, eroded, element);
	cv::dilate(eroded, dilated, element); // temp = open(img)
	cv::subtract(borders_sobel, dilated, dilated);
	cv::bitwise_or(skel, dilated, skel);
	eroded.copyTo(borders_sobel);
}


void calculateCanny()
{
	int edgeThresh = 1;
	int lowThreshold = 3;
	int const max_lowThreshold = 100;
	int ratio = 5;
	int kernel_size = 3;
	GaussianBlur(gray, borders_canny, Size(3, 3), 0, 0);

	/// Canny detector
	Canny(borders_canny, borders_canny, lowThreshold, lowThreshold*ratio, kernel_size, true);
}


void findLinesHough()
{
	float line_slope;
	cv::Point aux_point;
	img_lines = frame.clone();
	lines_points.clear();
	
	HoughLinesP(borders_combined, lines, 1, CV_PI/180, 130, 20, 10);
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		aux_point = cv::Point(l[0], l[1]);
		lines_points.push_back(aux_point);
		aux_point = cv::Point(l[2], l[3]);
		lines_points.push_back(aux_point);

		if(abs(l[0] - l[2]) != 0)
		{
			line_slope = (l[1] - l[3])/(l[0] - l[2]);
			if(abs(line_slope) < 1.0)
			{
				cv::line(img_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
			}
		}
	}
}


void fitPolynomialFloorContour()
{
	approxPolyDP(lines_points, polynomial_fit, 3, false);
	contoured_img = frame.clone();
	// for( size_t i = 0; i < polynomial_fit.size(); i++ )
	// {
	// 	cv::line(contoured_img, cv::Point(polynomial_fit[i].x, polynomial_fit[i].y), cv::Point(polynomial_fit[i+1].x, polynomial_fit[i+1].y), cv::Scalar(0,0,255), 3, CV_AA);
	// }
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
  

  //cap = VideoCapture(0);
	cap = VideoCapture("eng_stat_obst.avi");
	

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
		calculateSobel();
		calculateCanny();
		addWeighted(borders_sobel, 0.5, borders_canny, 0.5, 0.0, borders_combined);
		CV_TIMER_STOP(C, "Canny and Sobel edge detectors")
		findLinesHough();
		CV_TIMER_STOP(D, "Find lines using Hough Transform")
		fitPolynomialFloorContour();
		CV_TIMER_STOP(E, "Fit a polynomial to the points given by the Hough Transform")
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
