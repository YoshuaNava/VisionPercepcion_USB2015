
#include "img_proc_functions.h"

namespace GPSSapienza{


	void calculateIIC(cv::Mat Cr, cv::Mat Cb, cv::Mat a, cv::Mat& iic)
	{	
		//Reference on passing parameters-by-reference in C/C++: http://stackoverflow.com/questions/11235187/opencv-changing-mat-inside-a-function-mat-scope
		//Reference on adding matrices without saturation: http://answers.opencv.org/question/13769/adding-matrices-without-saturation/
		Cr.convertTo(Cr, CV_32S);
		Cb.convertTo(Cb, CV_32S);
		a.convertTo(a, CV_32S);
		cv::addWeighted((Cr+Cb), 0.25, a, 0.5, 0.0, iic, CV_32S);
		iic.convertTo(iic, CV_8UC1);
		normalize(iic, iic, 0, 255, CV_MINMAX);
	}
	
	
	
	void calculateMagnitudeOrientationOfGradients(cv::Mat gray, cv::Mat& F_mag, cv::Mat& F_ang)
	{
		cv::Mat temp_grad[3], sobel[3];
		Scharr(gray, temp_grad[0], gray.depth(), 1, 0, 1, 0, cv::BORDER_DEFAULT);
		convertScaleAbs(temp_grad[0], sobel[1], 1, 0);
	
		Scharr(gray, temp_grad[1], gray.depth(), 0, 1, 1, 0, cv::BORDER_DEFAULT);
		convertScaleAbs(temp_grad[1], sobel[2], 1, 0);
	
		F_mag = abs(temp_grad[0]) + abs(temp_grad[1]);  //abs_grad_x + abs_grad_y
		F_mag = 255 - F_mag;
		F_ang = 0*F_mag;
		float result;
		float valueX;
		float valueY;
		for (int y = 0; y < temp_grad[1].rows; y++) 
		{
			for (int x = 0; x < temp_grad[1].cols; x++) 
			{
				valueX = sobel[1].at<uchar>(y,x);
				valueY = sobel[2].at<uchar>(y,x);
				result = cv::fastAtan2(valueY,valueX);
				F_ang.at<uchar>(y, x) = (uchar)result;
			}
		}
		normalize(F_mag, F_mag, 0, 255, CV_MINMAX);
		convertScaleAbs(F_mag, F_mag, 1, 0); 
		normalize(F_ang, F_ang, 0, 255, CV_MINMAX);
		convertScaleAbs(F_ang, F_ang, 1, 0);	
	}
	
	
	// // Can be optimized with intrinsics!!!
	void calculateLBP(cv::Mat frame, cv::Mat& lbp)
	{
		lbp = cv::Mat::zeros(frame.rows-2, frame.cols-2, CV_8UC1);
		const int dx[8] = {-1, -1, -1, 0, +1, +1, +1, 0};
		const int dy[8] = {-1, 0, +1, +1, +1, 0, -1, -1};
		uchar center, code, periphery_value;
		
		for(int i=1; i<frame.rows-1 ;i++)
		{
			for(int j=1; j<frame.cols-1 ;j++)
			{
				center = frame.ptr<uchar>(i)[j];
				code = 0;
				for(int k=0; k<8 ;k++)
				{
					periphery_value = (frame.ptr<uchar>(i+dx[k]))[j+dy[k]];
					code |= (periphery_value > center) << 7-k;
				}
				(lbp.ptr<uchar>(i-1))[j-1] = code;
			}
		}
	}
	
	
	void calculateImageFeatures(GPSSapienza::Features* featuresPtr)
	{
		cv::Mat hsv_channels[3], ycrcb_channels[3], lab_channels[3];
		cvtColor( featuresPtr->rgb, featuresPtr->hsv, 	CV_BGR2HSV );	//Convert to HSV
		cvtColor( featuresPtr->rgb, featuresPtr->lab, 	CV_BGR2Lab );	//Convert to Lab
		cvtColor( featuresPtr->rgb, featuresPtr->YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
	
		cv::split(featuresPtr->hsv, hsv_channels); //cvsplit Divides a multi-channel array into separate single-channel arrays
		featuresPtr->hue = hsv_channels[0];
		featuresPtr->sat = hsv_channels[1];
		featuresPtr->val = hsv_channels[2];
		cv::split(featuresPtr->YCrCb, ycrcb_channels);
		featuresPtr->Cr = ycrcb_channels[1];
		featuresPtr->Cb = ycrcb_channels[2];
		cv::split(featuresPtr->lab, lab_channels);
		featuresPtr->a = lab_channels[1];
		calculateIIC(featuresPtr->Cr, featuresPtr->Cb, featuresPtr->a, featuresPtr->iic); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
		
		calculateMagnitudeOrientationOfGradients(featuresPtr->gray, featuresPtr->mag, featuresPtr->ang32);	
		calculateLBP(featuresPtr->gray, featuresPtr->lbp);
	}


	void init_images_img_proc(cv::Size img_size)
	{
		int i;
		stats_disp = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
	
		// //View_Histogram Graph Based Segmentation
		// int GBShist_size = 256;			// size of histogram (number of bins)
		// float range_0[]={0,256};
		// float* ranges[] = { range_0 };
		// hist = cvCreateHist(1, &GBShist_size, CV_HIST_ARRAY, ranges, 1);
	
		// int LBPhist_size = 32;
		// LBPBOXhist = cvCreateHist(1, &LBPhist_size, CV_HIST_ARRAY, ranges, 1);
		// hist_img = cvCreateImage(cvSize(255,200), 8, 1);
	
	
		// //Histogram Analysis
		// HistSize = cvSize(128,100);
		// HistImgH = cvCreateImage( HistSize, 8, 3 );
		// HistImgS = cvCreateImage( HistSize, 8, 3 );
		// HistImgV = cvCreateImage( HistSize, 8, 3 );
	
		// EdgeHist_img = cvCreateImage(HistSize, 8, 1);
	
		// LBPhist_img = cvCreateImage(HistSize, 8, 1);
		// iichist_img = cvCreateImage(HistSize, 8, 1);
		// GhistImg = cvCreateImage(cvSize(512,100), 8, 1);
		// GhistImg2 = cvCreateImage(cvSize(120,100), 8, 1);
	
		for(i=0; i<NUM_FEATURES ;++i)
		{
		     bin[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
		     mask[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		//     sobel[i] = cvCreateImage( S, IPL_DEPTH_8U, 1);
		//     inv_prob[i] = cvCreateImage( S, 32, 1);
		//     temp[i] = cvCreateImage( S, 32, 1);
		//     PG[i] = cvCreateImage( S, 32, 1);
		//     PG_prev[i] = cvCreateImage( S, 32, 1);
		//     cvZero(PG_prev[i]);
		//     PG1_DISP[i] = cvCreateImage( cvSize(120,100), 8, 3);
		//     PG0_DISP[i] = cvCreateImage( cvSize(120,100), 8, 3);
		}
	
		// static int hdims = 32;
		// static int hrange = 181;
		// static int vrange = 256;
		// float hranges_arr[] = {float(0),float(hrange-1)};
		// float vranges_arr[] = {float(0),float(vrange-1)};
		// float* hranges = hranges_arr;
		// float* vranges = vranges_arr;
	
		// static int dim_40 	= 40;
		// static int range_40 	= 40;
		// float range_40_arr[] = {float(0),float(range_40-1)};
		// float* range_40_ptr = range_40_arr;
	
		// histH = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
		// histS = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &vranges, 1 );
		// histV = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &vranges, 1 );
		// static int dim_9 	= 9;
		// static int dim_32 	= 32;
		// static int range_32 	= 32;
		// float range_2pi_arr[] = {-CV_PI,CV_PI};
		// float* range_2pi_ptr = range_2pi_arr;
		// float range_32_arr[] = {float(0),float(range_32-1)};
		// float* range_32_ptr = range_32_arr;
		// float range_log_arr[] = {-30,30};
		// float* range_log_ptr = range_log_arr;
		// ANG_HIST = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
		// Ghist = cvCreateHist( 1, &vrange, CV_HIST_ARRAY, &range_log_ptr, 1 );
		// Ghist2 = cvCreateHist( 1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1 );
		// Ghist2DISP = cvCreateHist( 1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1 );
	
		// hist_temp = cvCreateImage( cvSize(120,100), 8, 3);
	}



void superPixelStats(Features features, Statistics* stats)
{
	int k = 0;
	cv::calcHist(&features.seg_img, 1, 0, cv::Mat(), superpixels_histogram, 1, &hist_size, &hist_range, true, false);
	stats_disp.setTo(cv::Scalar(255,255,255));
	for(int i=0; i < hist_size ;i++)
	{
		if ((cvRound(superpixels_histogram.at<float>(0)) > 0) && (features.superpixels_list.size() > i))
		{
			// std::cout << "i  " << i << "\n";
			// std::cout << "superpixels size  =  " << features.superpixels_list.size() << "\n";
			Superpixel curr_superpixel = features.superpixels_list[i];
			stats->id[k] = k;
			stats->gray_id[k] = i;
			cv::compare(features.seg_img, cv::Scalar(i), mask[0], cv::CMP_EQ);
			stats->size[k] = curr_superpixel.get_points().size(); //count number of pixels in current segment

			cv::meanStdDev(curr_superpixel.get_pixels_gray(), stats->mean[k], stats->stdDev[k], curr_superpixel.get_pixels_mask());
			//std::cout << "mean = " << stats->mean[k] << "	stddev = " << stats->stdDev[k] << "\n";

			stats->box[k] = curr_superpixel.get_bounding_rect();
 			// stats->P_Gt[k] = GetPrior(stats->img_h, &stats->box[k]);
			stats->P_Gt[k] = stats->prior_img.at<float>(curr_superpixel.get_center().y, curr_superpixel.get_center().x);
 			stats->P_Gf[k] =  1. - stats->P_Gt[k];

			stats_disp.setTo(cv::Scalar(stats->mean[k].val[0]), mask[0]);
			// stats->prior_img.setTo(cv::Scalar(stats->P_Gt[k]), mask[0]);

			cv::rectangle(stats_disp, cvPoint(stats->box[k].x,stats->box[k].y), cvPoint(stats->box[k].x+stats->box[k].width,stats->box[k].y+stats->box[k].height), CV_RGB(255,0,0), 1, 8, 0);

			cv::line(stats_disp, cvPoint(stats->box[k].x + (stats->box[k].width)/2 , stats->box[k].y + (stats->box[k].height)/2 ), cvPoint(stats->img_w/2,stats->img_h), CV_RGB(255,0,0), 1, 8, 0);

			k++;

		}

	}

}

void updatePrior(Statistics *stats, Features* features)
{	
	cv::Scalar mean1 = cv::Scalar(0,0,0,0);
	cv::Scalar mean0 = cv::Scalar(0,0,0,0);
	
	mean1.val[0] = cv::mean(features->post1, cv::Mat()).val[0];
    if(mean1.val[0] > 0)
	{
		for(int i = 0; i < stats->nos; i++ )
		{
			cv::compare(features->seg_img, stats->gray_id[i], mask[0], CV_CMP_EQ);
			cv::GaussianBlur(features->post1, features->post1, cv::Size(5, 5), 0, 0);
			cv::GaussianBlur(features->post0, features->post0, cv::Size(5, 5), 0, 0);
			
			mean1.val[0] = cv::mean(features->post1, mask[0]).val[0];
			mean0.val[0] = cv::mean(features->post0, mask[0]).val[0];
			
			if(mean1.val[0] > 0)
			{
				double prior1 = alpha*stats->P_Gt[i] + beta*mean1.val[0];
				double prior0 = alpha*stats->P_Gf[i] + beta*mean0.val[0];			
				stats->P_Gt[i] = prior1/(prior1+prior0);
				stats->P_Gf[i] = prior0/(prior1+prior0);
			}
			stats->prior_img.setTo(cv::Scalar(stats->P_Gt[i]), mask[0]);
		}
	}
}


void GetModel(Features* features, Model* model)
{
    int loop;
	cv:inRange(features->hsv, cv::Scalar(0, smin, min(vmin,vmax), 0), cv::Scalar(180, 256, max(vmin,vmax), 0), bin[0]);
	cv::bitwise_and(bin[0], model->mask, bin[1]);
	cv::imshow("safe window", bin[1]);
	loop = 200;

    static bool acc =1;
    static int j=0;
    if(j<loop){
        acc = 1;
        j++;
    }
    else{
        acc=0;
        j=0;
    }

//     cvCalcHist( &F->mag, 	M->H_M[0], acc, M->mask );
//     cvCalcHist( &F->ang32, 	M->H_M[1], acc, M->mask );
//     cvCalcHist( &F->hue, 	M->H_M[2], acc, bin[1] );
//     cvCalcHist( &F->sat, 	M->H_M[3], acc, M->mask );
//     cvCalcHist( &F->lbp, 	M->H_M[4], acc, M->mask );
//     cvCalcHist( &F->iic, 	M->H_M[5], acc, M->mask );

//     static int dim_9 = 9;
//     static int dim_32 = 32;

//     for(int i=0;i<N;i++){
//         cvCopyHist(M->H_M[i], &M->H_M_DISP[i]);
//     }
}


static inline double getPrior(int h, cv::Rect* R){
    //const static int hmax = h-1;
    const static double lambda = 3./(double)h;
    double height = (double)(h - (R->y + (R->height)/2)) ;

    return exp(-lambda*height);
}

// IplImage *contour_image; //display final binary segmentation

// CvHistogram *hist = NULL;	    // pointer to histogram object

// CvHistogram *histH, *histS, *histV; //Histograms of superpixels to match against model
// IplImage *HistImgH, *HistImgS, *HistImgV; //Images to display histograms

// CvHistogram *ANG_HIST = NULL;

// IplImage *inv_prob[NUM_FEATURES], *temp[NUM_FEATURES], *PG[NUM_FEATURES], *PG_prev[NUM_FEATURES], *PG1_DISP[NUM_FEATURES], *PG0_DISP[NUM_FEATURES]; //Images for use when calculating probabilities

// IplImage* DEPTH_MAP;//Display depth array

// IplImage* LBPhist_img, *iichist_img;
// CvHistogram *LBPBOXhist = NULL;

// IplImage* GhistImg;
// CvHistogram *Ghist = NULL;

// IplImage* GhistImg2;
// CvHistogram *Ghist2 = NULL;
// CvHistogram *Ghist2DISP = NULL;

// CvSize HistSize;


// void GetModel(IplImage* gray, Features* F, Model* M, bool dynamic)
// {
//     int loop;
//     cvInRangeS( F->hsv, cvScalar(0,   smin, min(vmin,vmax), 0),
//                         cvScalar(180, 256,  max(vmin,vmax), 0), bin[0] );

//     cvAnd(bin[0], M->mask, bin[1]);
//     if(dynamic) loop = 200;
//     if(!dynamic) loop = 0;
//     static bool acc =1;
//     //static int loop = 1;
//     static int j=0;
//     if(j<loop){
//         acc = 1;
//         j++;
//     }
//     else{
//         acc=0;
//         j=0;
//     }

//     cvCalcHist( &F->mag, 	M->H_M[0], acc, M->mask );
//     cvCalcHist( &F->ang32, 	M->H_M[1], acc, M->mask );
//     cvCalcHist( &F->hue, 	M->H_M[2], acc, bin[1] );
//     cvCalcHist( &F->sat, 	M->H_M[3], acc, M->mask );
//     cvCalcHist( &F->lbp, 	M->H_M[4], acc, M->mask );
//     cvCalcHist( &F->iic, 	M->H_M[5], acc, M->mask );

//     static int dim_9 = 9;
//     static int dim_32 = 32;

//     for(int i=0;i<N;i++){
//         cvCopyHist(M->H_M[i], &M->H_M_DISP[i]);
//     }
// }


// void FeatureAnalysis(Features *F, Model* M, Statistics *S, IplImage *gbs, bool dynamic)
// {
//     cvInRangeS( F->hsv, cvScalar(0,   smin, min(vmin,vmax), 0),
//                 cvScalar(180, 256,  max(vmin,vmax), 0), bin[0] );


//     for(int i=0;i<S->no_features;i++){
//         S->L1[i] = S->L1[i] > 0 ? S->L1[i] : 0.01;
//         S->L0[i] = S->L0[i] > 0 ? S->L0[i] : 0.01;
//         S->gmax[i] = S->gmax[i] > 0 ? S->gmax[i] : 1.;
//     }

//     for(int i = 0; i < S->nos; i++ )
//     {
//         cvCmpS(gbs, S->gray_id[i], mask[0], CV_CMP_EQ); //mask out image segmen

//         cvAnd(bin[0], mask[0], bin[1]);

//         cvCalcHist( &F->mag,   S->H_SF[0], 0, mask[0] );
//         cvCalcHist( &F->ang32, S->H_SF[1], 0, mask[0] ); //NULL=mask
//         cvCalcHist( &F->hue,   S->H_SF[2], 0, bin[1] ); //NULL=mask
//         cvCalcHist( &F->sat,   S->H_SF[3], 0, mask[0] );
//         cvCalcHist( &F->lbp,   S->H_SF[4], 0, mask[0] ); //NULL=mask
//         cvCalcHist( &F->iic,   S->H_SF[5], 0, mask[0] );

//         for(int j=0;j<S->no_features;j++){
//             S->G_score[j] = Gstat(M->H_M[j], S->H_SF[j], M->dim[j]);
//             S->P_FgGt[S->no_features*i+j] = EXP_DIST_1(S->Z1[j], S->L1[j], S->G_score[j]); //0.9*exp(-lambda*G_scoreV);
//             S->P_FgGf[S->no_features*i+j] = EXP_DIST_0(S->Z0[j], S->L0[j], S->G_score[j]); //1. - S->P_VgGt[i];
//             cvSet(PG[j], cvScalar( S->G_score[j] ), mask[0]);
//             cvSet(F->P_X1[j], cvScalar(S->P_FgGt[S->no_features*i+j]/(S->P_FgGt[S->no_features*i+j]+S->P_FgGf[S->no_features*i+j])), mask[0]);
//             cvSet(F->P_X0[j], cvScalar(S->P_FgGf[S->no_features*i+j]/(S->P_FgGt[S->no_features*i+j]+S->P_FgGf[S->no_features*i+j])), mask[0]);
//         }


//     }

//     if(dynamic){
//         static bool flag = 0;
//         for(int i=0;i<S->no_features;i++){
//         if(flag){
//         cvAddWeighted(PG[0], 0.5, PG_prev[0], 0.5, 0, PG[0]);
//         }
//         cvCopy(PG[0], PG_prev[0], NULL);
//         }
//         flag =1;
//     }
// }

// void ProbAnalysis2(Features *F, Statistics* S, IplImage* gbs)
// {
//     double t = (double)(0*10 + 300)*0.1 - 30;

//     for(int i = 0; i < S->nos; i++ )
//     {
//         cvCmpS(gbs, S->gray_id[i], mask[0], CV_CMP_EQ); //mask out image segment

//         for(int j=0;j<S->no_features;j++){
//         S->P_FgGt[S->no_features*i+j] = S->P_FgGt[S->no_features*i+j] > 0.00001 ? S->P_FgGt[S->no_features*i+j] : LP;

//         }
//         double post1 =
//         (S->P_Gt[i])*(S->P_FgGt[S->no_features*i+0])*(S->P_FgGt[S->no_features*i+1])*(S->P_FgGt[S->no_features*i+2])*(S->P_FgGt[S->
//         no_features*i+3])*(S->P_FgGt[S->no_features*i+4])*(S->P_FgGt[S->no_features*i+5]);
//         double post0 =
//         (S->P_Gf[i])*(S->P_FgGf[S->no_features*i+0])*(S->P_FgGf[S->no_features*i+1])*(S->P_FgGf[S->no_features*i+2])*(S->P_FgGf[S->
//         no_features*i+3])*(S->P_FgGf[S->no_features*i+4])*(S->P_FgGf[S->no_features*i+5]);


//         S->P_GtgF[i] = post1 / (post1 + post0);
//         S->P_GfgF[i] = post0 / (post1 + post0);

//         cvSet(F->post1, cvScalar(S->P_GtgF[i]), mask[0]);
//         cvSet(F->post0, cvScalar(S->P_GfgF[i]), mask[0]);
//         cvSet(F->post_ratio, cvScalar(log(post1/post0)), mask[0]);
//     }


//     cvCalcHist( &F->post_ratio, Ghist, 0, 0);
//     cvThreshold(F->post_ratio, F->bin_class_result, t, ONE, CV_THRESH_BINARY);
//     cvNormalize(F->post_ratio,inv_prob[0], 0, 1, CV_MINMAX);
// }


// void UpdateParams(IplImage* T, Statistics *S, Features *F, bool dynamic)
// {
//     //initializations
//     static int loop;
//     static int GmaxInt;
//     static int L1Int;
//     static int L0Int;
//     static int dim_40 = 40;
//     static bool weighted_mean=0;
//     static double mean_trial;
//     static bool truncated = 1;

//     if(dynamic) loop = 200;
//     else loop = 0;

//     static bool acc = 1;
//     static int j=0;
//     static double gmax=5;
//     static double mean1=0.;
//     static double mean0=0.;
//     static CvScalar mean;
//     const static double min_mean1 = 0.025;
//     const static double max_mean1 = 10;
//     const static double min_mean0 = 0.25;

//     cvCopy(T, bin[4],0);
//     cvNot(T,bin[3]);
//     cvNormalize(F->post_ratio,inv_prob[0], 0, 1, CV_MINMAX);
//     cvAbsDiffS(inv_prob[0], inv_prob[3], cvScalar(1));

//     if(j<loop){
//     acc = 1;
//     j++;
//     }else{
//     acc=0;
//     j=0;
//     }

//     for(int i=0;i<S->no_features;i++)
// 	{
//         cvCopy(PG[i],temp[5],0);
//         cvNormalize(temp[5],temp[5],0,39,CV_MINMAX);
//         cvCalcHist( &temp[5], Ghist2, 1, NULL);
//         cvCopyHist(Ghist2, &Ghist2DISP);

//         cvMinMaxLoc( PG[i], NULL, &gmax, NULL, NULL, NULL);
//         if(dynamic){
//             S->gmax[i] = (0.5*gmax + 0.5*S->gmax[i]) > 0 ? (0.5*gmax + 0.5*S->gmax[i]) : 0.5;
//         }else{
//             S->gmax[i] = gmax > 0 ? gmax : 0.5;
//         }

//         if(weighted_mean){
//             cvMul(inv_prob[0],PG[i],inv_prob[1],1);
//             cvAvgSdv(inv_prob[1], &mean, NULL, NULL);
//         }else {
//             cvAvgSdv(PG[i], &mean, NULL, bin[4]);
//         }
//         mean.val[0] = mean.val[0] > min_mean1 ? mean.val[0] : min_mean1;
//         mean.val[0] = mean.val[0] > max_mean1 ? max_mean1 : mean.val[0];
//         if (mean1==0.) mean1 = mean.val[0];
//         if(dynamic){
//             mean1 = 0.5*mean1 + 0.5*mean.val[0];
//         }else{
//             mean1 = mean.val[0];
//         }
//         if(truncated){
//             if(mean1<(gmax/2)) mean1 = mean1 - gmax/(exp(gmax/mean1)-1);
//             if(mean1>=(gmax/2)) mean1 = (gmax/2);
//         }

//         S->L1[i] = (1./(mean1)) > 0 ? (1./(mean1)) : 1 ;

//         cvSet( hist_temp, cvScalarAll(255), 0 );
//         cvCalcHist( &PG[i], S->H_G1[i], acc, bin[4]);

//         cvCopyHist(S->H_G1[i], &S->H_G1_DISP[i]);


//         draw_dist(40, S->gmax[i], S->Z1[i], S->L1[i], hist_temp, 1);
//         cvAddWeighted( hist_temp, 0.5, PG1_DISP[i], 0.5, 0, PG1_DISP[i] );

//     }

// 	cvSet( hist_temp, cvScalarAll(255), 0 );

// 	if(weighted_mean){
// 		//cvMul(F->post0,PG[i],inv_prob[2],1);
// 		cvMul(inv_prob[3],PG[i],inv_prob[2],1);
// 		cvAvgSdv(inv_prob[2], &mean, NULL, NULL);
// 	}else{
// 		cvAvgSdv(PG[i], &mean, NULL, bin[3]);
// 	}

// 	mean.val[0] = gmax-mean.val[0];
// 	mean.val[0] = mean.val[0] > min_mean0 ? mean.val[0] : min_mean0;

// 	if (mean0==0.) mean0 = mean.val[0];
// 	if(dynamic){
// 		mean0 = 0.5*mean0 + 0.5*mean.val[0];
// 	}else{
// 		mean0 = mean.val[0];
// 	}
// 	if(truncated){
// 		if(mean0<(gmax/2)) mean0 = mean0 - gmax/(exp(gmax/mean0)-1);
// 		if(mean0>=(gmax/2)) mean0 = (gmax/2);
// 	}
// 	S->L0[i] = (1./(mean0)) > 0 ? (1./(mean0)) : 1 ;
// 	cvCalcHist( &PG[i], S->H_G0[i], acc, bin[3]);

// 	cvCopyHist(S->H_G0[i], &S->H_G0_DISP[i]);

// 	draw_dist(40, S->gmax[i], S->Z0[i], S->L0[i], hist_temp, 0);

// 	cvAddWeighted( hist_temp, 0.5, PG0_DISP[i], 0.5, 0, PG0_DISP[i] );

// 	S->Z1[i] = (1/S->L1[i])*(1-exp(-S->L1[i]*S->gmax[i]));
// 	S->Z0[i] = (1/S->L0[i])*(exp(S->L0[i]*S->gmax[i])-1);

	
// }



}

