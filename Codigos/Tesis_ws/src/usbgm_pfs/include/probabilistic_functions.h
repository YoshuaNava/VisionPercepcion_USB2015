#ifndef PROBABILISTIC_FUNCTIONS_H
#define PROBABILISTIC_FUNCTIONS_H

#include <global.h>

namespace ProbFloorSearch
{
	class ProbFns
	{
		public: 
			static cv::Mat getFloorPrior(cv::Mat img, vector<Superpixel> superpixels);
	};
}

#endif
