#ifndef HOUGH_HORIZON_SEARCH_H
#define HOUGH_HORIZON_SEARCH_H

#include <global.h>

namespace ProbFloorSearch
{
	class HoughHorizon
	{
		private:
			cv::Mat frame, gray;
			cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny, borders_combined;
			cv::Mat img_lines, floor_boundary_img, superpixels_below_boundary, floor_prob_map;
			cv::Mat bayes_prob_floor, coloured_bayes_floor;
			vector<cv::Vec4i> lines;
			vector<cv::Point> lines_dataset;
			int lines_history = 5;
			deque<vector<cv::Point>> acc_lines_points;
			vector<cv::Point> polynomial_fit;
			vector<int> superpixel_is_floor;		
			
			double proc_W, proc_H;
			cv::VideoCapture cap;
			vector<Superpixel> superpixels_list;
			
			Slic slic;
			Egbis egbis;
			int poly_degree = 3;
			Eigen::VectorXd poly_coeff;
			
			int superpixels_history = 5;
			deque<vector<int>> superpixels_isfloor_samples;
			vector<double> superpixels_floor_prob;

		public: 
			void calculateSobel();
			void calculateCanny();
			void findLinesHough();
			void fitPolynomialFloorContour();
			void drawPolynomialFloorBoundary();
			void findSuperpixelsBelowBoundary();
			void calculateFloorProbability();
			void drawProbabilisticFloor();
			void calculateBayesianEstimateFloor();
	};
}


#endif
