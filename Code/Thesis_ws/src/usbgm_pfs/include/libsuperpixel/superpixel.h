#ifndef SUPERPIXEL_H
#define SUPERPIXEL_H

/* superpixel.h.
 *
 * Written by: Yoshua Nava
 *
 * This file contains the class elements of the class Superpixel. The purpose of
 * this class is to serve as a container of RGB Superpixels segmented by different
 * algorithms, so that they can be treated separately as an entity.
 *
 */

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"

#define point2Dvec vector<cv::Point>

using namespace std;

class Superpixel 
{
	private:
		int id;
		int num_points;
		cv::Point center;
		cv::Mat histogram;
		point2Dvec points;
		cv::Rect bounding_rect;
		cv::Mat pixels;
		cv::Mat pixels_gray;
		cv::Mat pixels_mask;
		float probability_floor;
		float avg_depth;

		void init_structures(int num_points);
		void clear_data();
	public:
		/* Class constructors and deconstructors. */
		Superpixel();
		Superpixel(int id, int num_points);
		Superpixel(int id, cv::Point center);
		Superpixel(int id, int num_points, cv::Point center, cv::Mat histogram, point2Dvec points);
		~Superpixel();

		int get_id();
		cv::Point get_center();
		point2Dvec get_points();
		cv::Mat get_histogram();
		cv::Mat get_pixels();
		cv::Mat get_pixels_gray();
		cv::Mat get_pixels_mask();
		cv::Rect get_bounding_rect();
		
		void print_everything();
		void add_point(cv::Point point);
		void add_pixels_information(cv::Mat img, cv::Mat gray, vector<vector<int>> clusters);
		void calculate_histogram();
		void export_to_jpeg(IplImage *img);
};

#endif
