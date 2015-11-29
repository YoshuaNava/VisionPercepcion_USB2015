
#include "../include/global.h"



#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;

Mat frame, gray, prevgray; // Mat Declarations

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
	
	
	//DISPLAY_IMAGE_XY(false, F_hsv, 0 , 0);
	//DISPLAY_IMAGE_XY(false, F_YCrCb, 1 , 0);
	//DISPLAY_IMAGE_XY(false, F_lab, 2 , 0);
	DISPLAY_IMAGE_XY(true, frame, 0 , 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	//DISPLAY_IMAGE_XY(true, gray, 1 , 0);
	// DISPLAY_IMAGE_XY(true, F_hue, 1 , 0);
	// cv::resizeWindow("F_hue", 160, 120);
	//cv::resizeWindow("F_hue", 160, 120);
	// DISPLAY_IMAGE_XY(true, F_sat, 2 , 0);
	// cv::resizeWindow("F_sat", 160, 120);
	//cv::resizeWindow("F_sat", 160, 120);
	//DISPLAY_IMAGE_XY(true, F_Cr, 5 , 0);
	//DISPLAY_IMAGE_XY(true, F_Cb, 6 , 0);
	//DISPLAY_IMAGE_XY(true, F_a, 7 , 0);
	// DISPLAY_IMAGE_XY(true, F_iic, 3 , 0);
	// cv::resizeWindow("F_iic", 160, 120);
	// DISPLAY_IMAGE_XY(true, F_mag, 4 , 0);
	// cv::resizeWindow("F_mag", 160, 120);
	// DISPLAY_IMAGE_XY(true, F_ang, 5 , 0);
	// cv::resizeWindow("F_ang", 160, 120);
	// DISPLAY_IMAGE_XY(true, F_lbp, 6 , 0);
	// cv::resizeWindow("F_lbp", 160, 120);
}





void SuperPixels(cv::Mat src)
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
  

  slic.export_superpixels_to_files(&frame2);
  slic.display_contours(&frame2, CV_RGB(255,0,0));
  slic.display_number_grid(&frame2, CV_RGB(0,255,0));
  //slic.show_histograms(1,32);

  //slic.display_center_grid(frame2, CV_RGB(0,255,0));
  //slic.calculate_histograms(frame2);

  cvShowImage("SuperPixels", &frame2);
  //cvReleaseImage(&frame2);
  cvReleaseImage(&lab_image);
  cvWaitKey(10);
}


void CameraSetup()
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
			ShowImages();
			//Publisher	Code
			waitKey(1);
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
