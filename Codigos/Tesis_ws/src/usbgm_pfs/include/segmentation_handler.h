#ifndef SEGMENTATION_HANDLER_H
#define SEGMENTATION_HANDLER_H

#include <slic_modified/slic.h>
#include <egbis/egbis.h>



namespace ProbFloorSearch
{
	class SegmentationHandler
	{
		private:
			cv::Mat rgb;
			cv::Mat gray;
			cv::Mat seg_image;
			cv::Mat superpixels_contours_img;
			cv::Mat color_clusters_img;
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
			
			void segmentImage(cv::Mat rgb, cv::Mat gray);
			vector<Superpixel> getSuperpixels();
			cv::Mat getSegmentedImage();
			cv::Mat getContoursImage();
			cv::Mat getColorClustersImage();
	};
}

#endif