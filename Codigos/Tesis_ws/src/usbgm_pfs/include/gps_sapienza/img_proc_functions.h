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
	cv::Mat bin[NUM_FEATURES]; //images to use as masks with colour analysis
	static int hist_size = 256;
	float range[] = { 0, 256 } ;
	const float* hist_range = { range };
	static double alpha = 0.6;
	static double beta = 1.- alpha;
	static int vmin = 10, vmax = 256, smin = 5; // min/max values for Variance/Saturation mask
	
	const char * HIST_VAL = "Val Image Histogram";
	const char * HIST_EDGE = "Edge Image Histogram";
	const char * HIST_HUE = "Hue Image Histogram";
	const char * HIST_SAT = "Sat Image Histogram";
	const char * LBP_HIST 		= "LBP Image Histogram"; // window name
	const char * iic_HIST 		= "IIC Image Histogram"; // window name	
	const char * GBS_HIST 		= "GBS Image Histogram"; // window name
	
	static int dim_9	= 9;
    static int dim_16 	= 16;
    static int dim_32 	= 32;
    static int dim_64 	= 64;
    static int dim_128 	= 128;
	static int range_256 	= 256;
	static int range_181 	= 181;
	static int range_91 	= 91;
	float range_256_arr[] = {float(0),float(range_256-1)};
	float range_181_arr[] = {float(0),float(range_181-1)};
	float range_91_arr[] = {float(0),float(range_91-1)};
	float range_2pi_arr[] = {-CV_PI,CV_PI};
	const float* range_256_ptr = range_256_arr;
	const float* range_181_ptr = range_181_arr;
	const float* range_91_ptr = range_91_arr;
	const float* range_2pi_ptr = range_2pi_arr;
	cv::Size HistSize;	
	cv::Mat HistImgH, HistImgS, HistImgV; //Images to display histograms
	cv::Mat EdgeHist_img, LBPhist_img, iichist_img;
	cv::Mat GhistImg, GhistImg2;		
	
	void calculateIIC(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic);
	void calculateMagnitudeOrientationOfGradients(cv::Mat gray, cv::Mat& F_mag, cv::Mat& F_ang);
	void calculateLBP(cv::Mat frame, cv::Mat& lbp);
	void calculateImageFeatures(GPSSapienza::Features* featuresPtr);
	void init_images_img_proc(cv::Size img_size);
	static inline double getPrior(int h, cv::Rect* R);
	void superPixelStats(Features features, Statistics* stats);
	void updatePrior(Statistics *stats, Features* features);
	void getModel(Features* features, Model* model);
	static inline cv::Scalar hue2rgb( float hue );
	static inline void printGHistogram(int hist_size, cv::Mat histogram, cv::Mat &hist_img, const char *Window, bool flag);
	static inline void printHistogram(int hist_size, cv::Mat &histogram, cv::Mat &hist_img, const char *Window, bool flag);
	void displayHistograms(Model* model);
	
	
}

#endif