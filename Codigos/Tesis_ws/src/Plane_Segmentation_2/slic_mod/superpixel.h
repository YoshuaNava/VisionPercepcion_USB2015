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
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
//#include <float.h>
using namespace std;

#define RGBcolourFrequencyChart vector<vector<int>>	//Taken from the original slic class
#define point2Dvec vector<cv::Point>

class Superpixel 
{
	private:
		int id;
		int num_points;
		cv::Point center;
		RGBcolourFrequencyChart histogram;
		point2Dvec points;
		cv::Mat pixel_mask;
		cv::Rect bounding_rect;
		cv::Mat pixels;

		void init_structures(int num_points);
		void clear_data();
	public:
		/* Class constructors and deconstructors. */
		Superpixel();
		Superpixel(int id, int num_points);
		Superpixel(int id, cv::Point center);
		Superpixel(int id, int num_points, cv::Point center, RGBcolourFrequencyChart histogram, point2Dvec points);
		~Superpixel();

		cv::Point get_center();
		point2Dvec get_points();
		RGBcolourFrequencyChart get_histogram();

		static RGBcolourFrequencyChart init_FrequencyChart();
		
		void print_everything();
		void add_point(cv::Point point);
		void add_histogram_colorFrequencies(int R, int G, int B);
		void calculate_img_pixel_mask(IplImage *img);
		void export_to_jpeg();
};

#endif