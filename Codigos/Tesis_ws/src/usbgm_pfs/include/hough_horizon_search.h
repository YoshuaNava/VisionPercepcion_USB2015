#ifndef HOUGH_HORIZON_SEARCH_H
#define HOUGH_HORIZON_SEARCH_H

#include <global.h>

namespace ProbFloorSearch
{
	class ProbFns
	{
		public: 
			cv::Mat getFloorPrior(cv::Mat img, vector<Superpixel> superpixels);
	};
}


#endif
