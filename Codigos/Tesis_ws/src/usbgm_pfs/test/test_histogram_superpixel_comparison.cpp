
#include <global.h>

#include <segmentation_handler.h>

using namespace std;
using namespace cv;
using namespace ProbFloorSearch;

cv::Mat frame, seg_image, gray, superpixels_contours_img; // Mat Declarations
vector<Superpixel> superpixels_list;

VideoCapture cap;

SegmentationHandler segHandler("EGBIS");


void compareHistograms(int base_sp_id, int test1_sp_id, int test2_sp_id)
{
	printf("Superpixels histogram comparison. Methods: 1) Correlation, 2) Chi-Square, 3) Intersection, and 4) Bhattacharyya distance\n");


	for( int i = 0; i < 4; i++ )
    {
		if((superpixels_list.size() >= base_sp_id) && (superpixels_list.size() >= test1_sp_id) && (superpixels_list.size() >= test2_sp_id))
		{
			int compare_method = i;
			double base_base = compareHist( superpixels_list[base_sp_id].get_histogram(), superpixels_list[base_sp_id].get_histogram(), compare_method );
		    double base_test1 = compareHist( superpixels_list[base_sp_id].get_histogram(), superpixels_list[test1_sp_id].get_histogram(), compare_method );
		    double base_test2 = compareHist( superpixels_list[base_sp_id].get_histogram(), superpixels_list[test2_sp_id].get_histogram(), compare_method );
		    printf( " Method [%d]: Superpixel %i against itself, Superpixels %i and %i, Superpixels %i and %i : %f,   %f,   %f \n", i, base_sp_id, base_sp_id, test1_sp_id, base_sp_id, test2_sp_id, base_base, base_test1, base_test2 );
		}
	}

}


void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0, 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, superpixels_contours_img, 1, 0);
	cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
}



void cameraSetup()
{
  	cap = VideoCapture(0);
	//cap = VideoCapture("../eng_stat_obst.avi");
	
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
		segHandler.segmentImage(frame, gray);
		superpixels_list = segHandler.getSuperpixels();
		superpixels_contours_img = segHandler.getContoursImage();
		CV_TIMER_STOP(B, "Superpixels processed")


		compareHistograms(11, 23, 104);
		CV_TIMER_STOP(C, "Three superpixel histograms compared")
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
