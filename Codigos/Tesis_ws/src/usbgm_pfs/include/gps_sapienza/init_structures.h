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


#ifndef INIT_STRUCTURES_H
#define INIT_STRUCTURES_H

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

#include <libsuperpixel/superpixel.h>

#define NUM_FEATURES    6
#define ZERO	0 //Binary image zero=0 & one=255
#define ONE	255


namespace GPSSapienza
{
    struct statistics {
    
        int* id;
        int* size;
        int* gray_id;
        cv::Scalar* mean;
        cv::Scalar* stdDev;
        cv::Rect* box; 
        int no_features;
    
    
        double* P_Gt;
        double* P_Gf;
        cv::Mat prior_img;
    
        double *P_FgGt, *P_FgGf;
    
        double* P_GtgF;//posterior
        double* P_GfgF;
    
        double* G_score;
    
        int nos; //nuber of segments
        int img_w;
        int img_h;
    
        double *L1, *L0;
        double *Z1, *Z0, *gmax;
    
        CvHistogram *H_SF[NUM_FEATURES];
        CvHistogram *H_G1[NUM_FEATURES], *H_G1_DISP[NUM_FEATURES];
        CvHistogram *H_G0[NUM_FEATURES], *H_G0_DISP[NUM_FEATURES];
    
    };
    typedef struct statistics Statistics;
    
    
    struct model {
    
        cv::Rect* box; //safe area
        cv::Mat mask; //safe area
        
        //Basic Stats
        cv::Scalar* mean;
        cv::Scalar* stdDev;
    
        CvHistogram* H_M[NUM_FEATURES],*H_M_DISP[NUM_FEATURES];
        int* dim;
    
    };
    typedef struct model Model;
    
    
    struct features {
        cv::Mat rgb, gray;
        //Edges
        cv::Mat mag, ang32, P_ang;
    
        //Colour
        cv::Mat hsv, lab, YCrCb;
        cv::Mat hue, sat, val; 
        cv::Mat Cr, a, Cb, iic;
        cv::Mat P_hue, P_sat, P_val;
    
        //LBP
        cv::Mat lbp, P_lbp;
    
        //Posterior
        cv::Mat post0, post1, post_ratio, P_X1[5], P_X0[5];
    
        //Classification
        cv::Mat bin_class_result;
        cv::Mat seg_img;
        vector<Superpixel> superpixels_list;
    };
    
    typedef struct features Features;

    struct algorithm_parameters {
        cv::Size img_size;
        
    };
    
    typedef struct algorithm_parameters Algorithm_parameters;


	void init_stats(cv::Size img_size, Statistics* S, bool init);
    void init_model(cv::Size img_size, cv::Rect SafeRegion, Model* M);
    void init_features(cv::Size img_size, Features * F);
    
}


#endif