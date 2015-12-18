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
	void calculateIIC(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic);
	void calculateMagnitudeOrientationOfGradients(cv::Mat gray, cv::Mat& F_mag, cv::Mat& F_ang);
	void calculateLBP(cv::Mat frame, cv::Mat& lbp);
	void calculateImageFeatures(GPSSapienza::Features* featuresPtr);
}

#endif