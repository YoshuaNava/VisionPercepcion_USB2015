
#include <global.h>

#include <slic_modified/slic.h>
#include <egbis/egbis.h>


using namespace std;
using namespace cv;

VideoCapture cap;

cv::Mat frame, seg_image, gray, superpixels_contours_img; // Mat Declarations
Egbis egbis;

void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0, 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, gray, 1, 0);
	cv::resizeWindow("gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, seg_image, 2, 0);
	cv::resizeWindow("seg_image", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, superpixels_contours_img, 3, 0);
	cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
}


void egbisSuperpixels()
{
	seg_image = egbis.generateSuperpixels(frame, gray);
    superpixels_contours_img = egbis.outlineSuperpixelsContours(cv::Scalar(255,0,0));
    egbis.calculateSuperpixelCenters();
    egbis.storeSuperpixelsMemory();
    superpixels_contours_img = egbis.displayCenterGrid(superpixels_contours_img, cv::Scalar(0,255,0));
}



void cameraSetup()
{
  	cap = VideoCapture(0);
//	cap = VideoCapture("../eng_stat_obst.avi");
	
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
	printf("******************************************\n");
	printf("Probabilistic Ground Plane Segmentation\n");
	printf("Authors: Rafael Colmenares and Yoshua Nava\n");
	printf("******************************************\n");
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
		printf("#######################################\n");
		CV_TIMER_STOP(A, "Received image from camera")
		cv:resize(frame, frame, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(frame, gray, CV_BGR2GRAY);
		waitKey(1); // Wait Time

		egbisSuperpixels();
		CV_TIMER_STOP(B, "Superpixels processed")

		showImages();

		CV_TIMER_STOP(Z, "Loop finished")
		printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	return 0;
}
