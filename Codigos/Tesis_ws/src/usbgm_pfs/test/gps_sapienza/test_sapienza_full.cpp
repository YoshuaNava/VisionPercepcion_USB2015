
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

const string segmentationAlgorithm = "EGBIS";
SegmentationHandler seg_handler(segmentationAlgorithm);
HoughHorizon hough_searcher(proc_H, proc_W);

GPSSapienza::Features features;
GPSSapienza::Features* featuresPtr = &features;
GPSSapienza::Statistics statistics;
GPSSapienza::Statistics* statisticsPtr = &statistics;
GPSSapienza::Model safewindow_model;
GPSSapienza::Model* safewindow_modelPtr = &safewindow_model;
GPSSapienza::Algorithm_parameters alg_params;


cv::Mat generateColorSegmentedImage(cv::Mat mask, cv::Mat img, cv::Scalar colour)
{
	cv::Mat output = img.clone();
	int x, y;
	uchar blue_tonality, red_tonality;
	for(x=0; x<img.cols ;x++)
	{
		for(y=0; y<img.rows ;y++)
		{
			if(mask.at<uchar>(y,x) == 255)
			{
				output.at<cv::Vec3b>(y, x)[0] = colour.val[0];
				output.at<cv::Vec3b>(y, x)[1] = colour.val[1];
				output.at<cv::Vec3b>(y, x)[2] = colour.val[2];
			}
		}
	}

	// cvtColor(gray, gray, CV_GRAY2RGB);
	// addWeighted(superpixels_below_boundary, 0.7, gray, 0.3, 0.0, superpixels_below_boundary);
	return output;
}


void showImages()
{
	DISPLAY_IMAGE_XY(true, features.rgb, 0, 0);
	cv::resizeWindow("features.rgb", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.gray, 1, 0);
	cv::resizeWindow("features.gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, superpixels_contours_img, 2, 0);
	cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
	if(segmentationAlgorithm == "EGBIS")
	{
		features.seg_img = features.seg_img * floor(255/features.superpixels_list.size());
	}
	DISPLAY_IMAGE_XY(true, features.seg_img, 3, 0);
	cv::resizeWindow("features.seg_img", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.hue, 0, 1);
	cv::resizeWindow("features.hue", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.sat, 1, 1);
	cv::resizeWindow("features.sat", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.mag, 2, 1);
	cv::resizeWindow("features.mag", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.ang32, 3, 1);
	cv::resizeWindow("features.ang32", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.lbp, 4, 1);
	cv::resizeWindow("features.lbp", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.iic, 5, 1);
	cv::resizeWindow("features.iic", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.post1, 4, 4);
	cv::resizeWindow("features.post1", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, features.floor_boundary, 5, 4);
	cv::resizeWindow("features.floor_boundary", proc_W, proc_H);
}


void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

    // cap = VideoCapture(0);
	cap = VideoCapture("eng_stat_obst.avi");
	// cap = VideoCapture("Laboratorio.avi");
	// cap = VideoCapture("LaboratorioMaleta.avi");
	// cap = VideoCapture("PasilloLabA.avi");
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
	
	alg_params.img_size = cv::Size(proc_W, proc_H);
	GPSSapienza::init_stats(alg_params.img_size, statisticsPtr, 1);
	GPSSapienza::init_model(alg_params.img_size, safewindow_modelPtr);
	GPSSapienza::init_features(alg_params.img_size, featuresPtr);
	GPSSapienza::init_images_img_proc(alg_params.img_size);
	
    
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
		features.superpixels_list = seg_handler.getSuperpixels();
		superpixels_contours_img = seg_handler.getContoursImage();
		features.seg_img = seg_handler.getSegmentedImage();
		CV_TIMER_STOP(C, "Superpixels processed")

		statistics.prior_img = ProbFns::getFloorPrior(features.rgb, features.superpixels_list);		
		CV_TIMER_STOP(D, "Floor prior prepared")
		
		GPSSapienza::superPixelStats(features, statisticsPtr);
		CV_TIMER_STOP(E, "Superpixels statistics calculated")
	
		GPSSapienza::updatePrior(statisticsPtr, featuresPtr);
		CV_TIMER_STOP(F, "Prior probability updated")
		
		
		GPSSapienza::getModel(featuresPtr, safewindow_modelPtr);
		CV_TIMER_STOP(G, "Captured safe window model")
		
		GPSSapienza::displayHistograms(safewindow_modelPtr);
		CV_TIMER_STOP(H, "Showing features histograms")


		GPSSapienza::featureAnalysis(featuresPtr, safewindow_modelPtr, statisticsPtr);
		CV_TIMER_STOP(I, "Analyzing features with G-stat")

		GPSSapienza::displayAnalyzedFeatures(features);
		CV_TIMER_STOP(J, "Showing analyzed features")

		GPSSapienza::probAnalysis2(featuresPtr, statisticsPtr);
		CV_TIMER_STOP(K, "Probabilistic analysis ready")

		GPSSapienza::updateParams(features.bin_class_result, statisticsPtr, featuresPtr);
		CV_TIMER_STOP(L, "Expectation-Maximization completed")
		
		features.floor_boundary = features.bin_class_result.clone();

		GPSSapienza::findObstacleBoundary(features.floor_boundary);
		CV_TIMER_STOP(M, "Extracted the floor boundary")
		
		// hough_searcher.setMinScoreHough(70);
		// hough_searcher.doProbabilisticEstimation(features.rgb, features.gray, superpixels_contours_img, statistics.prior_img, features.superpixels_list);
		// // statistics.prior_img = hough_searcher.getProbabilisticFloorEstimate();
		// // // hough_searcher.doBayesianEstimation(features.rgb, features.gray, superpixels_contours_img, statistics.prior_img, features.superpixels_list);
		// // // statistics.prior_img = hough_searcher.getBayesianFloorEstimate();
		// // hough_searcher.showImages();
		// CV_TIMER_STOP(N, "Hough-based floor boundary search")
		// cv::Mat probMask = (hough_searcher.getProbabilisticFloorEstimate() > 0.7);
		// 	cv::imshow("hough search", hough_searcher.getTaggedSuperpixelsImage());
		// 	cv::imshow("hough probabilistic mask", probMask);
		// cv::Mat resulting_mask;
		// features.floor_boundary.copyTo(resulting_mask, probMask);
		// cv::addWeighted(resulting_mask, 0.5, probMask, 0.5, 0, resulting_mask);
		// 	cv::imshow("resulting_mask", (resulting_mask>200));
		
		// // cv::imshow("hough poly", hough_searcher.getPolyBoundaryImage());
		
		
		// cv::Mat fused_boundary = generateColorSegmentedImage((resulting_mask > 150), features.rgb, cv::Scalar(255,0,0));
		
		
		// cv::imshow("fused_boundary", fused_boundary);
		
		
		showImages();
		CV_TIMER_STOP(Z, "Loop finished")
		printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	 	ros::spinOnce();
        // cv::waitKey(0);
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return EXIT_SUCCESS;
}
