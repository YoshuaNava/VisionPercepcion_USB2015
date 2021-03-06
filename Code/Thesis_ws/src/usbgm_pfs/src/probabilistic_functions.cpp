
#include <probabilistic_functions.h>

using namespace ProbFloorSearch;


cv::Mat ProbFns::getFloorPrior(cv::Mat img, vector<Superpixel> superpixels)
{
	//REFERENCE: https://sites.google.com/site/mikesapi/downloads/ground-plane-segmentation-and-autonomous-guidance
	float lambda, Y1, Y0, prob_floor;
	lambda = 5.0/img.rows;
	Y0 = 1;
	Y1 = 1;
	int i, j, k, row, x_coord, y_coord;
	vector<cv::Point> points;
	cv::Mat priorMatrix = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
	for(i=0; i < superpixels.size() ;i++)
	{
			row = superpixels[i].get_center().y;
			prob_floor = 1.0/(1 + ((Y1/Y0) * (exp(lambda*(img.rows-row)) - 1)));
			//printf("%f\n", prob_floor);
			points = superpixels[i].get_points();
			for(j=0; j < points.size() ;j++)
			{
				x_coord = points[j].x;
				y_coord = points[j].y;
				priorMatrix.at<float>(y_coord, x_coord) = prob_floor;
			}

	}
	return priorMatrix;
}
