
#include "init_structures.h"
using namespace std;

namespace GPSSapienza
{
	void init_stats(cv::Size img_size, Statistics* stats, bool init)
	{
		static int range = 256;
		stats->no_features = NUM_FEATURES;
	
		if(init) stats->id = new int[range];
		for (int i=0; i<range; i++){
			stats->id[i] = 0;
		}
	
		if(init) stats->size = new int[range];
		for (int i=0; i<range; i++){
			stats->size[i] = 0;
		}
	
		if(init) stats->gray_id = new int[range];
		for (int i=0; i<range; i++){
			stats->gray_id[i] = 0;
		}
	
		if(init) stats->mean = new cv::Scalar[range];
		for (int i=0; i<range; i++){
			stats->mean[i] = cv::Scalar(0,0,0,0);
		}
	
		if(init) stats->stdDev = new cv::Scalar[range];
		for (int i=0; i<range; i++){
			stats->stdDev[i] = cv::Scalar(0,0,0,0);
		}
	
		if(init) stats->box = new cv::Rect[range];
		for (int i=0; i<range; i++){
			stats->box[i] = cv::Rect(0,0,0,0);
		}
	
		if(init) stats->P_Gt = new double[range];
		if(init) stats->P_Gf = new double[range];
		if(init) stats->P_GtgF = new double[range];
		if(init) stats->P_GfgF = new double[range];
		for (int i=0; i<range; i++){
			stats->P_Gt[i] = 0.;
			stats->P_Gf[i] = 0.;
		
			stats->P_GtgF[i] = 0.;
			stats->P_GfgF[i] = 0.;
		}
	
		if(init) stats->P_FgGt = new double[range * stats->no_features];
		if(init) stats->P_FgGf = new double[range * stats->no_features];
	
		for (int i=0; i<range; i++){
			for (int j=0; j<stats->no_features; j++){
				stats->P_FgGt[stats->no_features*i + j] = 0.;
				stats->P_FgGf[stats->no_features*i + j] = 0.;
			}
		}
	
	

		stats->prior_img = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);

	
		stats->nos = 0; //number of segments
		stats->img_w = img_size.width;
		stats->img_h = img_size.height;
	
		unsigned int iseed = (unsigned int)time(NULL);
		srand (iseed);
		//double min = 0.001;
		double min = 0.01;
		double max = 40.01;
	
		if(init) stats->L1 = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->L1[i] = 0.01;
		}
	
		if(init) stats->L0 = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->L0[i] = 0.01;
		}
	
		if(init) stats->gmax = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->gmax[i] = 2.;
		}
	
		if(init) stats->Z1 = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->Z1[i] = (1/stats->L1[i])*(1-exp(-stats->L1[i]*stats->gmax[i]));
		}
	
		if(init) stats->Z0 = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->Z0[i] = (1/stats->L0[i])*(exp(stats->L0[i]*stats->gmax[i])-1);
		}
	
		if(init) stats->G_score = new double[stats->no_features];
		for (int i=0; i<stats->no_features; i++){
			stats->G_score[i] = 0;
		}
	
		//Histogram Initialization
		static int dim_40 	= 40;
		static int range_40 	= 40;
		float range_40_arr[] = {float(0.),float(range_40-1)};
		float* range_40_ptr = range_40_arr;
	
		if(init){
		for(int i=0;i<stats->no_features;i++){
		stats->H_G1[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		stats->H_G0[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		stats->H_G1_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		stats->H_G0_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		}
		}
		if(!init){
			for(int i=0;i< stats->no_features; i++)
			{
				cvClearHist(stats->H_G1[i]);
				cvClearHist(stats->H_G0[i]);
				cvClearHist(stats->H_G1_DISP[i]);
				cvClearHist(stats->H_G0_DISP[i]);
			}
		}
	
		//Histogram Initialization
		static int dim_9	= 9;
		static int dim_32 	= 32;
	
		static int range_256 	= 256;
		static int range_181 	= 181;
	
		float range_256_arr[] = {float(0.),float(range_256-1)};
		float range_181_arr[] = {float(0.),float(range_181-1)};
		float range_2pi_arr[] = {-CV_PI,CV_PI};
	
		float* range_256_ptr = range_256_arr;
		float* range_181_ptr = range_181_arr;
		float* range_2pi_ptr = range_2pi_arr;
	
		if(init){
			stats->H_SF[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
			stats->H_SF[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
			stats->H_SF[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
			stats->H_SF[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
			stats->H_SF[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
			stats->H_SF[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		}
		if(!init){
			for(int i=0;i<stats->no_features;i++){
				cvClearHist(stats->H_SF[i]);
			}
		}
	}
	
	
	
	void init_model(cv::Size img_size, Model* model)
	{
		int n = 1; //number of model regions
		cv::Rect SafeRegion = cv::Rect( cvRound(img_size.width/3),//+25,
					(img_size.height)-cvRound(img_size.height/7),
					cvRound(img_size.width/3),
					cvRound(img_size.height/8) );
	
		model->box = &SafeRegion;
	 
		model->mask = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		cv::Mat roi = (model->mask(SafeRegion));
		roi.setTo(255);
		// cv::imshow("roi", model->mask);
	
		model->mean = new cv::Scalar[n];
		for (int i=0; i<n; i++){
			model->mean[i] = cv::Scalar(0,0,0,0);
		}
	
		model->stdDev = new cv::Scalar[n];
		for (int i=0; i<n; i++){
			model->stdDev[i] = cv::Scalar(0,0,0,0);
		}
	
		//Histogram Initialization
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
	
		float* range_256_ptr = range_256_arr;
		float* range_181_ptr = range_181_arr;
		float* range_91_ptr = range_91_arr;
		float* range_2pi_ptr = range_2pi_arr;
	
		model->dim = new int[NUM_FEATURES];
		model->dim[0] = 32;
		model->dim[1] = 9;
		for (int i=2; i<NUM_FEATURES; i++){
			model->dim[i] = 32;
		}
			
		model->H_M[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		model->H_M[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
		model->H_M[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
		model->H_M[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		model->H_M[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		model->H_M[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
	
		model->H_M_DISP[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		model->H_M_DISP[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
		model->H_M_DISP[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
		model->H_M_DISP[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		model->H_M_DISP[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		model->H_M_DISP[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
	}





	void init_features(cv::Size img_size, Features* features)
	{
		features->rgb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		features->gray = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->mag = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->ang32 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		features->P_ang = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);	
		features->hsv = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		features->lab = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		features->YCrCb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		features->hue = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->sat = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->val = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->Cr = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->a = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->Cb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->iic = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->P_hue = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		features->P_sat = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		features->P_val = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		features->lbp  = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->P_lbp = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		features->post0 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		features->post1 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		features->post_ratio = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		features->bin_class_result = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		features->seg_img = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
	
		for(int i=0;i<5;i++){
			features->P_X1[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			features->P_X0[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		}
	}


	
}
