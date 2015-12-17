
#ifndef HOUGH_HORIZON_SEARCH_H
#define HOUGH_HORIZON_SEARCH_H

#include <global.h>
#include <libsuperpixel/superpixel.h>



//REFERENCE: http://stackoverflow.com/questions/16796732/how-to-sort-vector-of-points-based-on-a-y-axis
struct cvPointComparator {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} cvPointComparator;


namespace ProbFloorSearch
{
	class HoughHorizon
	{
		private:
			double img_W, img_H;
			cv::Mat frame, gray, superpixels_contours_img, floor_prior; // Mat Declarations
			cv::Mat borders_combined, img_lines, poly_boundary_img, superpixels_below_boundary, floor_prob_map;
			cv::Mat bayes_prob_floor, coloured_bayes_floor;
			
			vector<cv::Point> lines_dataset;
			int lines_history = 5;
			deque<vector<cv::Point>> acc_lines_points;
			
			int poly_degree = 3;
			Eigen::VectorXd poly_coeff;
			
			vector<Superpixel> superpixels_list;			
			vector<int> superpixel_is_floor;
			int superpixels_history = 5;
			deque<vector<int>> superpixels_isfloor_samples;
			vector<double> superpixels_floor_prob;
			
			char algorithmType = ' ';
			void clearData();
		public: 
			HoughHorizon(double img_W, double img_H);
			~HoughHorizon();
			
			cv::Mat getBordersImage();
			cv::Mat getLinesImage();
			cv::Mat getPolyBoundaryImage();
			cv::Mat getTaggedSuperpixelsImage();
			cv::Mat getColouredBayesImage();
			cv::Mat getProbabilisticFloorEstimate();
			cv::Mat getBayesianFloorEstimate();
			
			void showImages();
			void calculateSobelCannyBorders();
			void findLinesHough();
			void fitPolynomialFloorContour();
			void drawPolynomialFloorBoundary();
			void findSimpleSuperpixelsBelowBoundary();
			void findProbSuperpixelsBelowBoundary();
			void calculateFloorProbability();
			void drawProbabilisticFloor();
			void calculateBayesianEstimateFloor();
			
			void doSimpleEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list);
			void doProbabilisticEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list);
			void doBayesianEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list);
	};
}


#endif
