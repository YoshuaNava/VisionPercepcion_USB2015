/*
 *  Copyright (C) <2014>  <Michael Sapienza>
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef IMG_PROC_FUNCTIONS_H
#define IMG_PROC_FUNCTIONS_H

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

#include "init_structures.h"


namespace GPSSapienza
{
	cv::Mat superpixels_histogram;
	cv::Mat stats_disp;
	cv::Mat mask[NUM_FEATURES]; //Images for use as masks eg:superpixels
	static int hist_size = 256;
	float range[] = { 0, 256 } ;
	const float* hist_range = { range };
	static double alpha = 0.6;
	static double beta = 1.- alpha;
	
	void calculateIIC(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic);
	void calculateMagnitudeOrientationOfGradients(cv::Mat gray, cv::Mat& F_mag, cv::Mat& F_ang);
	void calculateLBP(cv::Mat frame, cv::Mat& lbp);
	void calculateImageFeatures(GPSSapienza::Features* featuresPtr);
	void init_images_img_proc(cv::Size img_size);
	void superPixelStats(Features features, Statistics* stats);
	static inline double getPrior(int h, cv::Rect* R);
	void updatePrior(Statistics *stats, Features* features);
	void GetModel(Features* features, Model* model);
}

#endif