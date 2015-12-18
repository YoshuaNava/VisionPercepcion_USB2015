
#include <global.h>

#include <probabilistic_functions.h>
#include <segmentation_handler.h>
#include <hough_horizon_search.h>


using namespace std;
using namespace cv;
using namespace ProbFloorSearch;

cv::Mat frame, gray, superpixels_contours_img, floor_prior; // Mat Declarations
cv::Mat borders_combined, img_lines, poly_boundary_img, superpixels_below_boundary, floor_prob_map;
cv::Mat bayes_prob_floor, coloured_bayes_floor;

VideoCapture cap;
vector<Superpixel> superpixels_list;

SegmentationHandler seg_handler("SLIC");
HoughHorizon hough_searcher(proc_H, proc_W);


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
	cap.read(frame);

	printf("******************************************\n");
	printf("Probabilistic Ground Plane Segmentation\n");
	printf("Authors: Rafael Colmenares and Yoshua Nava\n");
	printf("******************************************\n");
	waitKey(0);
	
    
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

		
		superpixels_contours_img = frame.clone();
		seg_handler.segmentImage(frame, gray);
		superpixels_list = seg_handler.getSuperpixels();
		superpixels_contours_img = seg_handler.getContoursImage();
		CV_TIMER_STOP(B, "Superpixels processed")
		floor_prior = ProbFns::getFloorPrior(frame, superpixels_list);
		CV_TIMER_STOP(C, "Prior probability calculated")
		// hough_searcher.doSimpleEstimation(frame, gray, superpixels_contours_img, floor_prior, superpixels_list);
		hough_searcher.doProbabilisticEstimation(frame, gray, superpixels_contours_img, floor_prior, superpixels_list);
		hough_searcher.doBayesianEstimation(frame, gray, superpixels_contours_img, floor_prior, superpixels_list);
		CV_TIMER_STOP(D, "Tagged superpixels below Hough-based bayesian boundary")


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
