#ifndef SEGMENTATION_HANDLER_H
#define SEGMENTATION_HANDLER_H

#include <slic_modified/slic.h>
#include <egbis/egbis.h>




namespace ProbFloorSearch
{
	class SegmentationHandler
	{
		private:
			cv::Mat frame, seg_image, gray, superpixels_contours_img;
			vector<Superpixel> superpixels_list;	
			Slic slic;
			Egbis egbis;
			std::string segmentationType = "";
			void slicSuperpixels();
			void egbisSuperpixels();
			void clearData();
		public:
			SegmentationHandler(std::string segmentationType);
			~SegmentationHandler();
			
			void segmentImage(cv::Mat frame, cv::Mat gray);
			vector<Superpixel> getSuperpixels();
			cv::Mat getContoursImage();
	};
}

#endif