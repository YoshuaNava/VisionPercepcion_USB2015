

#include <segmentation_handler.h>

using namespace ProbFloorSearch;

SegmentationHandler::SegmentationHandler(std::string segmentationType)
{
	this->segmentationType = segmentationType;
}


SegmentationHandler::~SegmentationHandler() 
{
    clearData();
}


/*
 * Clear the data as saved by the algorithm.
 *
 * Input : -
 * Output: -
 */
void SegmentationHandler::clearData() 
{
	superpixels_list.clear();
}


void SegmentationHandler::slicSuperpixels()
{
	this->seg_image = frame.clone();
	IplImage frame2 = (IplImage)seg_image; // Reference on deallocating memory: http://stackoverflow.com/questions/12635978/memory-deallocation-of-iplimage-initialised-from-cvmat

	/* Yield the number of superpixels and weight-factors from the user. */
	IplImage *lab_image = cvCloneImage(&frame2);
	cvCvtColor(&frame2, lab_image, CV_BGR2Lab);
	int w = lab_image->width, h = lab_image->height;
	int nr_superpixels = 6*w;
	int nc = 20;
	double step = sqrt((w * h) / (double) nr_superpixels)*3;

	/* Perform the SLIC superpixel algorithm. */
	slic.clear_data();
	slic.generate_superpixels(lab_image, step, nc);
	slic.create_connectivity(lab_image);
	slic.store_superpixels(this->seg_image, this->gray);
	this->superpixels_list = slic.get_superpixels();
	this->seg_image = slic.get_segmented_image();

	cvReleaseImage(&lab_image);
}


void SegmentationHandler::egbisSuperpixels()
{
	this->seg_image = egbis.generateSuperpixels(frame, gray);
    egbis.calculateSuperpixelCenters();
    egbis.storeSuperpixelsMemory();
	this->superpixels_list = egbis.getSuperpixels();
}


void SegmentationHandler::segmentImage(cv::Mat frame, cv::Mat gray)
{
	this->frame = frame;
	this->gray = gray;
	if(this->segmentationType == "EGBIS")
	{
		egbisSuperpixels();
	}
	else if (this->segmentationType == "SLIC")
	{
		slicSuperpixels();	
	}
}


vector<Superpixel> SegmentationHandler::getSuperpixels()
{
	return this->superpixels_list;
}


cv::Mat SegmentationHandler::getSegmentedImage()
{
	return this->seg_image;
}


cv::Mat SegmentationHandler::getContoursImage()
{
	if(this->segmentationType == "SLIC")
	{
		this->superpixels_contours_img = frame.clone();
		IplImage frame2 = (IplImage)superpixels_contours_img; // Reference on deallocating memory: http://stackoverflow.com/questions/12635978/memory-deallocation-of-iplimage-initialised-from-cvmat
		slic.display_contours(&frame2, CV_RGB(255,0,0));
		//slic.display_center_grid(&frame2, CV_RGB(0,255,0));
		slic.display_number_grid(&frame2, CV_RGB(0,255,0));
		this->superpixels_contours_img = cv::cvarrToMat(&frame2, true, true, 0);
	}
	else if(this->segmentationType == "EGBIS")
	{
		this->superpixels_contours_img = egbis.outlineSuperpixelsContours(cv::Scalar(255,0,0));
		this->superpixels_contours_img = egbis.displayCenterGrid(superpixels_contours_img, cv::Scalar(0,255,0));
	}
	return this->superpixels_contours_img;
}
