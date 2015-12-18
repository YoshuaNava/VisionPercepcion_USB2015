
#include "init_structures.h"

namespace GPSSapienza
{
	void init_stats(cv::Size img_size, Statistics* S, bool init)
	{
		static int range = 256;
		S->no_features = NUM_FEATURES;
	
		if(init) S->id = new int[range];
		for (int i=0; i<range; i++){
		S->id[i] = 0;
		}
	
		if(init) S->size = new int[range];
		for (int i=0; i<range; i++){
		S->size[i] = 0;
		}
	
		if(init) S->gray_id = new int[range];
		for (int i=0; i<range; i++){
		S->gray_id[i] = 0;
		}
	
		if(init) S->mean = new cv::Scalar[range];
		for (int i=0; i<range; i++){
		S->mean[i] = cv::Scalar(0,0,0,0);
		}
	
		if(init) S->stdDev = new cv::Scalar[range];
		for (int i=0; i<range; i++){
		S->stdDev[i] = cv::Scalar(0,0,0,0);
		}
	
		if(init) S->box = new cv::Rect[range];
		for (int i=0; i<range; i++){
		S->box[i] = cv::Rect(0,0,0,0);
		}
	
		if(init) S->P_Gt = new double[range];
		if(init) S->P_Gf = new double[range];
		if(init) S->P_GtgF = new double[range];
		if(init) S->P_GfgF = new double[range];
		for (int i=0; i<range; i++){
		S->P_Gt[i] = 0.;
		S->P_Gf[i] = 0.;
	
		S->P_GtgF[i] = 0.;
		S->P_GfgF[i] = 0.;
		}
	
		if(init) S->P_FgGt = new double[range*S->no_features];
		if(init) S->P_FgGf = new double[range*S->no_features];
	
		for (int i=0; i<range; i++){
			for (int j=0; j<S->no_features; j++){
				S->P_FgGt[S->no_features*i + j] = 0.;
				S->P_FgGf[S->no_features*i + j] = 0.;
			}
		}
	
	

		S->prior_img = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);

	
		S->nos = 0; //number of segments
		S->img_w = img_size.width;
		S->img_h = img_size.height;
	
		unsigned int iseed = (unsigned int)time(NULL);
		srand (iseed);
		//double min = 0.001;
		double min = 0.01;
		double max = 40.01;
	
		if(init) S->L1 = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->L1[i] = 0.01;
		}
	
		if(init) S->L0 = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->L0[i] = 0.01;
		}
	
		if(init) S->gmax = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->gmax[i] = 2.;
		}
	
		if(init) S->Z1 = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->Z1[i] = (1/S->L1[i])*(1-exp(-S->L1[i]*S->gmax[i]));
		}
	
		if(init) S->Z0 = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->Z0[i] = (1/S->L0[i])*(exp(S->L0[i]*S->gmax[i])-1);
		}
	
		if(init) S->G_score = new double[S->no_features];
		for (int i=0; i<S->no_features; i++){
			S->G_score[i] = 0;
		}
	
		//Histogram Initialization
		static int dim_40 	= 40;
		static int range_40 	= 40;
		float range_40_arr[] = {float(0.),float(range_40-1)};
		float* range_40_ptr = range_40_arr;
	
		if(init){
		for(int i=0;i<S->no_features;i++){
		S->H_G1[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		S->H_G0[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		S->H_G1_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		S->H_G0_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
		}
		}
		if(!init){
			for(int i=0;i< S->no_features; i++)
			{
				cvClearHist(S->H_G1[i]);
				cvClearHist(S->H_G0[i]);
				cvClearHist(S->H_G1_DISP[i]);
				cvClearHist(S->H_G0_DISP[i]);
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
			S->H_SF[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
			S->H_SF[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
			S->H_SF[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
			S->H_SF[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
			S->H_SF[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
			S->H_SF[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		}
		if(!init){
			for(int i=0;i<S->no_features;i++){
				cvClearHist(S->H_SF[i]);
			}
		}
	}
	
	
	
	void init_model(cv::Size img_size, cv::Rect SafeRegion, Model* M)
	{
		int n = 1; //number of model regions
	
		M->box = &SafeRegion;
	
		M->mask = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
	
		//cvSetImageROI( M->mask, SafeRegion );
		M->mask = cv::Mat::ones(img_size.height, img_size.width, CV_32FC1);
		//cvSet(M->mask,cv::ScalarAll(ONE),0);
		//cvResetImageROI( M->mask );
	
		//M->mean = new cv::Scalar[n];
		for (int i=0; i<n; i++){
			M->mean[i] = cv::Scalar(0,0,0,0);
		}
	
		//M->stdDev = new cv::Scalar[n];
		for (int i=0; i<n; i++){
			M->stdDev[i] = cv::Scalar(0,0,0,0);
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
	
		M->dim = new int[NUM_FEATURES];
		M->dim[0] = 32;
		M->dim[1] = 9;
		for (int i=2; i<NUM_FEATURES; i++){
			M->dim[i] = 32;
		}
	
		M->H_M[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		M->H_M[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
		M->H_M[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
		M->H_M[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		M->H_M[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		M->H_M[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
	
		M->H_M_DISP[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		M->H_M_DISP[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
		M->H_M_DISP[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
		M->H_M_DISP[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
		M->H_M_DISP[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
		M->H_M_DISP[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
	}





	void init_features(cv::Size img_size, Features * F)
	{
		F->rgb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		F->gray = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->mag = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->ang32 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		F->P_ang = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);	
		F->hsv = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		F->lab = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		F->YCrCb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		F->hue = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->sat = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->val = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->Cr = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->a = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->Cb = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->iic = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->P_hue = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		F->P_sat = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		F->P_val = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		F->lbp  = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		F->P_lbp = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		F->post0 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		F->post1 = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		F->post_ratio = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
	
		F->bin_class_result = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
	
		for(int i=0;i<5;i++){
			F->P_X1[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			F->P_X0[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
		}
	}







	
}





