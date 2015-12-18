
#include <global.h>

#include <probabilistic_functions.h>
#include <segmentation_handler.h>
#include <hough_horizon_search.h>
#include <gps_sapienza/img_proc_functions.h>

using namespace std;
using namespace cv;
using namespace ProbFloorSearch;

cv::Mat superpixels_contours_img, floor_prior; // Mat Declarations
cv::Mat borders_combined, img_lines, poly_boundary_img, superpixels_below_boundary, floor_prob_map;
cv::Mat bayes_prob_floor, coloured_bayes_floor;

VideoCapture cap;
vector<Superpixel> superpixels_list;

SegmentationHandler seg_handler("SLIC");
HoughHorizon hough_searcher(proc_H, proc_W);
GPSSapienza::Features features;
GPSSapienza::Features* featuresPtr = &features;



void showImages()
{
	DISPLAY_IMAGE_XY(true, features.rgb, 0, 0);
	cv::resizeWindow("features.rgb", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.gray, 1, 0);
	cv::resizeWindow("features.gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.hue, 2, 0);
	cv::resizeWindow("features.hue", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.sat, 3, 0);
	cv::resizeWindow("features.sat", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.iic, 4, 0);
	cv::resizeWindow("features.iic", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.mag, 0, 1);
	cv::resizeWindow("features.mag", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.ang32, 1, 1);
	cv::resizeWindow("features.ang32", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.lbp, 2, 1);
	cv::resizeWindow("features.lbp", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, superpixels_contours_img, 3, 1);
	cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);	
}


void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

//    cap = VideoCapture(0);
	cap = VideoCapture("eng_stat_obst.avi");
	//cap = VideoCapture("Laboratorio.avi");
	//cap = VideoCapture("LaboratorioMaleta.avi");
	//cap = VideoCapture("PasilloLabA.avi");
	//cap = VideoCapture("PasilloLabB.avi");
 // cap = VideoCapture("Laboratorio4.avi");
//  cap = VideoCapture("Calle1.avi");
	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
	
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
	cap.read(features.rgb);

	printf("******************************************\n");
	printf("Probabilistic Ground Plane Segmentation\n");
	printf("Authors: Rafael Colmenares and Yoshua Nava\n");
	printf("******************************************\n");
	waitKey(0);
	
    
	while (nh.ok()) 
	{
		CV_TIMER_START(X)
		//cap >> frame; // Image from Cam to Mat Variable
		if (!cap.read(features.rgb)) 
		{
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}

		printf("#######################################\n");
		CV_TIMER_STOP(A, "Received image from camera")
		cv:resize(features.rgb, features.rgb, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(features.rgb, features.gray, CV_BGR2GRAY);
		waitKey(1); // Wait Time

		
		GPSSapienza::calculateImageFeatures(featuresPtr);
		CV_TIMER_STOP(B, "Image features ready")
		superpixels_contours_img = features.rgb.clone();
		seg_handler.segmentImage(features.rgb, features.gray);
		superpixels_list = seg_handler.getSuperpixels();
		superpixels_contours_img = seg_handler.getContoursImage();
		CV_TIMER_STOP(C, "Superpixels processed")

		showImages();
		CV_TIMER_STOP(Z, "Loop finished")
		printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
