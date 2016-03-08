
#include "img_proc_functions.h"
#include <global.h>

namespace GPSSapienza{


	void init_images_img_proc(cv::Size img_size)
	{
		int i;
		stats_disp = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC3);
		
		// //Histogram Analysis
		HistSize = cv::Size(128,100);
		HistImgH = cv::Mat::zeros(HistSize, CV_8UC3);

		HistImgS = cv::Mat::zeros(HistSize, CV_8UC3);
		HistImgV = cv::Mat::zeros(HistSize, CV_8UC3);

		EdgeHist_img = cv::Mat::zeros(HistSize, CV_8UC1);

		LBPhist_img = cv::Mat::zeros(HistSize, CV_8UC1);
		iichist_img = cv::Mat::zeros(HistSize, CV_8UC1);
		GhistImg = cv::Mat::zeros(cv::Size(512,100), CV_8UC1);
		GhistImg2 = cv::Mat::zeros(cv::Size(120,100), CV_8UC1);
		hist_temp = cv::Mat::zeros(cv::Size(120,100), CV_8UC3);

		for(i=0; i<NUM_FEATURES ;++i)
		{
			bin[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
			mask[i] = cv::Mat::zeros(img_size.height, img_size.width, CV_8UC1);
			inv_prob[i] =  cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			temp[i] =  cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			PG[i] =  cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			PG_prev[i] =  cv::Mat::zeros(img_size.height, img_size.width, CV_32FC1);
			
			PG1_DISP[i] = cv::Mat::zeros(cv::Size(120,100), CV_8UC3);;
			PG0_DISP[i] = cv::Mat::zeros(cv::Size(120,100), CV_8UC3);
		}
	}


	static inline double getPrior(int h, cv::Rect* R){
		//const static int hmax = h-1;
		const static double lambda = 3./(double)h;
		double height = (double)(h - (R->y + (R->height)/2)) ;

		return exp(-lambda*height);
	}

	void superPixelStats(Features features, Statistics* stats)
	{
		int k = 0;
		cv::calcHist(&features.seg_img, 1, 0, cv::Mat(), superpixels_histogram, 1, &hist_size, &hist_range, true, false);
		stats_disp.setTo(cv::Scalar(255,255,255));
		for(int i=0; i < hist_size ;i++)
		{
			if ((cvRound(superpixels_histogram.at<float>(0,1)) > 0) && (features.superpixels_list.size() > i))
			{
				Superpixel curr_superpixel = features.superpixels_list[i];
				stats->id[k] = k;
				stats->gray_id[k] = i;
				cv::compare(features.seg_img, cv::Scalar(i), mask[0], cv::CMP_EQ);
					// cv::imshow("mascara superpixel", mask[0]);
				stats->size[k] = curr_superpixel.get_points().size(); //count number of pixels in current segment
					
				cv::meanStdDev(curr_superpixel.get_pixels_gray(), stats->mean[k], stats->stdDev[k], curr_superpixel.get_pixels_mask());
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
		stats->nos = k;
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


	void getModel(Features* features, Model* model)
	{
		int loop;
		loop = 200;

		static bool acc = true;
		static int j=0;
		if(j<loop)
		{
			acc = true;
			j++;
		}
		else
		{
			acc = false;
			j = 0;
		}

		cv:inRange(features->hsv, cv::Scalar(0, smin, min(vmin,vmax), 0), cv::Scalar(180, 256, max(vmin,vmax), 0), bin[0]);
		cv::bitwise_and(bin[0], model->mask, bin[1]);
		cv::calcHist(&features->mag, 1, 0, model->mask, model->Hgram_M[0], 1, &model->dim[0], &range_256_ptr, true, acc);
		cv::calcHist(&features->ang32, 1, 0, model->mask, model->Hgram_M[1], 1, &model->dim[1], &range_2pi_ptr, true, acc);
		cv::calcHist(&features->hue, 1, 0, bin[1], model->Hgram_M[2], 1, &model->dim[2], &range_181_ptr, true, acc);
		cv::calcHist(&features->sat, 1, 0, model->mask, model->Hgram_M[3], 1, &model->dim[3], &range_256_ptr, true, acc);
		cv::calcHist(&features->lbp, 1, 0, model->mask, model->Hgram_M[4], 1, &model->dim[4], &range_256_ptr, true, acc);
		cv::calcHist(&features->iic, 1, 0, model->mask, model->Hgram_M[5], 1, &model->dim[5], &range_256_ptr, true, acc);
		
		// cv::imshow("safe window", model->mask);
		// cv::bitwise_and(features->mag, model->mask, features->mag);
		// cv::imshow("mag", features->mag);
		// cv::imshow("roi", features->mag(model->SafeRegion));
	}


	void displayHistograms(Model* model)
	{
		static int row = 1;
		for(int i=0;i<NUM_FEATURES;i++)
		{
			model->Hgram_M_DISP[i] = (model->Hgram_M[i]).clone();
		}
		// for(int i = 0; i < hist_size; i++ )
		// {
		//     int val = cvRound(model->Hgram_M_DISP[5].at<float>(i,0));
		// 	cout << "i = " << i << "   val = " << val << "\n";
		// }
		
		printHistogram(dim_32, model->Hgram_M[0], HistImgV, HIST_VAL, 1);
		printHistogram(dim_9, model->Hgram_M_DISP[1], EdgeHist_img, HIST_EDGE, 0);
		printHistogram(dim_32, model->Hgram_M_DISP[2], HistImgH, HIST_HUE, 1);
		printHistogram(dim_32, model->Hgram_M_DISP[3], HistImgS, HIST_SAT, 1);
		printHistogram(dim_32, model->Hgram_M_DISP[4], LBPhist_img, LBP_HIST, 0);
		printHistogram(dim_32, model->Hgram_M_DISP[5], iichist_img, iic_HIST, 0);
		
		DISPLAY_IMAGE_XY(true, HistImgH, 0, 2);
		cv::resizeWindow("HistImgH", HistImgH.cols, HistImgH.rows);
		DISPLAY_IMAGE_XY(true, HistImgS, 1, 2);
		cv::resizeWindow("HistImgS", HistImgS.cols, HistImgS.rows);
		DISPLAY_IMAGE_XY(true, HistImgV, 2, 2);
		cv::resizeWindow("HistImgV", HistImgV.cols, HistImgV.rows);
		DISPLAY_IMAGE_XY(true, EdgeHist_img, 3, 2);
		cv::resizeWindow("EdgeHist_img", EdgeHist_img.cols, EdgeHist_img.rows);
		DISPLAY_IMAGE_XY(true, LBPhist_img, 4, 2);
		cv::resizeWindow("LBPhist_img", LBPhist_img.cols, LBPhist_img.rows);
		DISPLAY_IMAGE_XY(true, iichist_img, 5, 2);
		cv::resizeWindow("iichist_img", iichist_img.cols, iichist_img.rows);
	}
	
	
	static inline cv::Scalar hue2rgb( float hue )
	{
		// hue to RGB conversion : coverts a given hue value to a RGB triplet for
		// display
		// parameters:
		// hue - hue value in range 0 to 180 (OpenCV implementation of HSV)
		// return value - CvScalar as RGB triple
		// taken from OpenCV 1.0 camshiftdemo.c example
		int rgb[3], p, sector;
		static const int sector_data[][3]= {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
		hue *= 0.033333333333333333333333333333333f;
		sector = cvFloor(hue);
		p = cvRound(255*(hue - sector));
		p ^= sector & 1 ? 255 : 0;
		
		rgb[sector_data[sector][0]] = 255;
		rgb[sector_data[sector][1]] = 0;
		rgb[sector_data[sector][2]] = p;
	
		return cv::Scalar(rgb[2], rgb[1], rgb[0], 0);
	}
	
	
	static inline void printGHistogram(int hist_size, cv::Mat histogram, cv::Mat& hist_img, const char *Window, bool flag)
	{
		float max_value;
		int bin_w = cvRound((double) hist_img.cols/hist_size);
		cv::normalize(histogram, histogram, 0, (10*hist_img.rows), CV_MINMAX);
	
		if(flag==0)
		{
			hist_img.setTo(255);
			for(int i = 0; i < hist_size; i++ )
			{
				int val = cvRound(histogram.at<float>(i,1));
				cv::rectangle( hist_img, cv::Point(i*bin_w, hist_img.rows),
								cv::Point((i+1)*bin_w, hist_img.rows - val),
								cv::Scalar(0), -1, 8, 0 );        }
		}
	}
	
	
	
	static inline void printHistogram(int hist_size, cv::Mat& histogram, cv::Mat& hist_img, const char *Window, bool flag)
	{
		float max_value;
		int bin_w = cvRound((double) hist_img.cols/hist_size);
		cv::normalize(histogram, histogram, 0, (4*hist_img.rows)/5, CV_MINMAX, -1, cv::Mat());
	
		if(flag==0)
		{
			hist_img.setTo(255);
			for(int i = 0; i < hist_size; i++ )
			{
				int val = cvRound(histogram.at<float>(i,0));
				cv::rectangle( hist_img, cv::Point(i*bin_w, hist_img.rows),
								cv::Point((i+1)*bin_w, hist_img.rows - val),
								cv::Scalar(0), -1, 8, 0 );
			}
		}
		else if(flag==1)
		{
			hist_img.setTo(0);
			for(int i = 0; i < hist_size; i++ )
			{
				int val = cvRound(histogram.at<float>(i,0));
				cv::Scalar color = hue2rgb(i*180.f/hist_size);
				cv::rectangle( hist_img, cv::Point(i*bin_w, hist_img.rows),
								cv::Point((i+1)*bin_w, hist_img.rows - val),
								color, -1, 8, 0 );
			}
		}
	}
	
	
	
	static inline double Gstat(cv::Mat histogram1, cv::Mat histogram2, int size)
	{
		double G = 0.;
		const static double SMALL = 0.000001;
		cv::normalize(histogram1, histogram1, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
		cv::normalize(histogram2, histogram2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	
		for(int i=0; i<size; i++)
		{
			double M = histogram1.at<float>(i,0); //sample
			double S = histogram2.at<float>(i,0); //model
			if(M == 0) M = SMALL;
			if(S == 0) S = SMALL;
			G += 2*(S*(log (S)) - S*(log (M)));
		}
	
		return G;
	}
	
	
	static inline double EXP_DIST_1(double z, double l, double g)
	{
		return (1/z)*exp(-l*g);
	}
	
	static inline double EXP_DIST_0(double z, double l, double g)
	{
		return (1/z)*exp(l*g);
	}
	
	void featureAnalysis(Features *features, Model *model, Statistics *stats)
	{
		cv:inRange(features->hsv, cv::Scalar(0, smin, min(vmin,vmax), 0), cv::Scalar(180, 256, max(vmin,vmax), 0), bin[0]);
		// long time = cvGetTickCount();
		
		for(int i=0; i<stats->no_features ;i++)
		{
			stats->L1[i] = stats->L1[i] > 0 ? stats->L1[i] : 0.01;
			stats->L0[i] = stats->L0[i] > 0 ? stats->L0[i] : 0.01;
			stats->gmax[i] = stats->gmax[i] > 0 ? stats->gmax[i] : 1.;
		}
		// cout << "Primer ciclo	" << (cvGetTickCount() - time)/((double)cvGetTickFrequency()*1000.) << "ms\n";
		// time = cvGetTickCount();
	
		for(int i=0; i<stats->nos ;i++)
		{
			cv::compare(features->seg_img, stats->gray_id[i], mask[0], CV_CMP_EQ);
			cv::bitwise_and(bin[0], mask[0], bin[1]);
			cv::calcHist(&features->mag, 1, 0, mask[0], stats->Hgram_SF[0], 1, &model->dim[0], &range_256_ptr, true, false);
			cv::calcHist(&features->ang32, 1, 0, mask[0], stats->Hgram_SF[1], 1, &model->dim[1], &range_2pi_ptr, true, false);
			cv::calcHist(&features->hue, 1, 0, bin[1], stats->Hgram_SF[2], 1, &model->dim[2], &range_181_ptr, true, false);
			cv::calcHist(&features->sat, 1, 0, mask[0], stats->Hgram_SF[3], 1, &model->dim[3], &range_256_ptr, true, false);
			cv::calcHist(&features->lbp, 1, 0, mask[0], stats->Hgram_SF[4], 1, &model->dim[4], &range_256_ptr, true, false);
			cv::calcHist(&features->iic, 1, 0, mask[0], stats->Hgram_SF[5], 1, &model->dim[5], &range_256_ptr, true, false);
	
			// cout << i << " A)  Segundo ciclo	" << (cvGetTickCount() - time)/((double)cvGetTickFrequency()*1000.) << "ms\n";
	
			for(int j=0; j<stats->no_features ;j++)
			{
				stats->G_score[j] = Gstat(model->Hgram_M[j], stats->Hgram_SF[j], model->dim[j]);
				stats->P_FgGt[stats->no_features*i+j] = EXP_DIST_1(stats->Z1[j], stats->L1[j], stats->G_score[j]); //0.9*exp(-lambda*G_scoreV);
				stats->P_FgGf[stats->no_features*i+j] = EXP_DIST_0(stats->Z0[j], stats->L0[j], stats->G_score[j]); //1. - S->P_VgGt[i];
				PG[j].setTo(cv::Scalar(stats->G_score[j]), mask[0]);
				features->P_X1[j].setTo(cv::Scalar(stats->P_FgGt[stats->no_features*i+j]	/	(stats->P_FgGt[stats->no_features*i+j] + stats->P_FgGf[stats->no_features*i+j])), mask[0]);
				features->P_X0[j].setTo(cv::Scalar(stats->P_FgGf[stats->no_features*i+j]	/	(stats->P_FgGt[stats->no_features*i+j] + stats->P_FgGf[stats->no_features*i+j])), mask[0]);
			}
			// cout << i << " B)  Segundo ciclo	" << (cvGetTickCount() - time)/((double)cvGetTickFrequency()*1000.) << "ms\n";
		}
		// cout << "Segundo ciclo	" << (cvGetTickCount() - time)/((double)cvGetTickFrequency()*1000.) << "ms\n";
		// time = cvGetTickCount();
		
	
		static bool flag = 0;
		for(int i=0; i<stats->no_features ;i++)
		{
			if(flag)
			{
				cv::addWeighted(PG[i], 0.5, PG_prev[i], 0.5, 0, PG[i]);
			}
			PG_prev[i] = PG[i].clone();
		}
		flag =1;
		// cout << "Tercer ciclo	" << (cvGetTickCount() - time)/((double)cvGetTickFrequency()*1000.) << "ms\n";
	}
	
	
	
	void displayAnalyzedFeatures(Features features)
	{
		DISPLAY_IMAGE_XY(true, features.P_X1[0], 0, 3);
		cv::resizeWindow("features.P_X1[0]", features.P_X1[0].cols, features.P_X1[0].rows);
		DISPLAY_IMAGE_XY(true, features.P_X1[1], 1, 3);
		cv::resizeWindow("features.P_X1[1]", features.P_X1[1].cols, features.P_X1[1].rows);
		DISPLAY_IMAGE_XY(true, features.P_X1[2], 2, 3);
		cv::resizeWindow("features.P_X1[2]", features.P_X1[2].cols, features.P_X1[2].rows);
		DISPLAY_IMAGE_XY(true, features.P_X1[3], 3, 3);
		cv::resizeWindow("features.P_X1[3]", features.P_X1[3].cols, features.P_X1[3].rows);
		DISPLAY_IMAGE_XY(true, features.P_X1[4], 4, 3);
		cv::resizeWindow("features.P_X1[4]", features.P_X1[4].cols, features.P_X1[4].rows);
		DISPLAY_IMAGE_XY(true, features.P_X1[5], 5, 3);
		cv::resizeWindow("features.P_X1[5]", features.P_X1[5].cols, features.P_X1[5].rows);
	}
	
	
	
	void probAnalysis2(Features *features, Statistics* stats)
	{
		double t = (double)(0*10 + 300)*0.1 - 30;
	
		for(int i=0; i<stats->nos; i++)
		{
			cv::compare(features->seg_img, stats->gray_id[i], mask[0], CV_CMP_EQ);
	
			for(int j=0; j<stats->no_features ;j++)
			{
				stats->P_FgGt[stats->no_features*i+j] = stats->P_FgGt[stats->no_features*i+j] > 0.00001 ? stats->P_FgGt[stats->no_features*i+j] : LP;
			}
			double post1 = (stats->P_Gt[i]) * (stats->P_FgGt[stats->no_features*i+0]); 
			post1 = post1 * (stats->P_FgGt[stats->no_features*i+1]) * (stats->P_FgGt[stats->no_features*i+2]);
			post1 = post1 * (stats->P_FgGt[stats->no_features*i+3]) * (stats->P_FgGt[stats->no_features*i+4]) * (stats->P_FgGt[stats->no_features*i+5]);
	
			double post0 = (stats->P_Gf[i]) * (stats->P_FgGf[stats->no_features*i+0]) * (stats->P_FgGf[stats->no_features*i+1]) * (stats->P_FgGf[stats->no_features*i+2]);
			post0 = post0 * (stats->P_FgGf[stats->no_features*i+3]) * (stats->P_FgGf[stats->no_features*i+4]) * (stats->P_FgGf[stats->no_features*i+5]);
	
			stats->P_GtgF[i] = post1 / (post1 + post0);
			stats->P_GfgF[i] = post0 / (post1 + post0);
			
			features->post1.setTo(cv::Scalar(stats->P_GtgF[i]), mask[0]);
			features->post0.setTo(cv::Scalar(stats->P_GtgF[i]), mask[0]);
			features->post_ratio.setTo(cv::Scalar(log(post1/post0)), mask[0]);
		}
	
		cv::calcHist(&features->post_ratio, 1, 0, cv::Mat(), Ghist, 1, &vrange, &range_log_ptr, true, false);
		cv::threshold(features->post_ratio, features->bin_class_result, t, 255, CV_THRESH_BINARY);
		features->bin_class_result.convertTo(features->bin_class_result, CV_8UC1);
		cv::normalize(features->post_ratio, inv_prob[0], 0, 1, CV_MINMAX);
	}
	
	
	
	
	
	
	
	
	void draw_dist(int range, double gmax, double Z, double L, cv::Mat& img, bool T)
	{
		int bin_w = cvRound((double)img.cols/(range));
	
		double value;
		for(int g=0; g<cvRound(gmax); g++)
		{
			if (T==1) 
			{
				value = 50*(EXP_DIST_1(Z, L, g));
			}
			if (T==0)
			{
				value = 50*(EXP_DIST_0(Z, L, g));
			}
	
			value = value > 0 ? value : 0.001;
			value = value > img.rows ? img.rows : value;
			//double value = 100*(value1+value0)/2;
	
			cv::rectangle(img, cv::Point(cvRound(g)*bin_w,img.rows), cv::Point((cvRound(g)+1)*bin_w, cvRound(img.rows - value)), CV_RGB(200,0,0), -1, 8, 0 );
	
		}
	
	}
	
	
	
	
	
	void updateParams(cv::Mat T, Statistics *stats, Features *features)
	{
		//initializations
		static int loop;
		static int GmaxInt;
		static int L1Int;
		static int L0Int;
		static int dim_40 = 40;
		static bool weighted_mean=0;
		static double mean_trial;
		static bool truncated = 1;
	
		loop = 200;
	
		static bool acc = true;
		static int j=0;
		static double gmax=5;
		static double mean1=0.;
		static double mean0=0.;
		static cv::Scalar mean, stddev;
		const static double min_mean1 = 0.025;
		const static double max_mean1 = 10;
		const static double min_mean0 = 0.25;
	
		bin[4] = T.clone();
		cv::bitwise_not(T, bin[3]);
		cv::normalize(features->post_ratio, inv_prob[0], 0, 1, CV_MINMAX);
		cv::absdiff(inv_prob[0], cv::Scalar(1), inv_prob[3]);
	
		if(j<loop)
		{
			acc = true;
			j++;
		}
		else
		{
			acc=false;
			j=0;
		}
	
		for(int i=0; i<stats->no_features ;i++)
		{
			temp[5] = PG[i].clone();
			cv::normalize(temp[5], temp[5], 0, 39, CV_MINMAX);
			cv::calcHist(&temp[5], 1, 0, cv::Mat(), Ghist2, 1, &dim_40, &range_40_ptr, true, false);
			Ghist2DISP = Ghist2.clone();
			cv::minMaxLoc(PG[i], NULL, &gmax, NULL, NULL, cv::Mat());
	
			stats->gmax[i] = (0.5*gmax + 0.5*stats->gmax[i]) > 0 ? (0.5*gmax + 0.5*stats->gmax[i]) : 0.5;
	
			if(weighted_mean)
			{
				inv_prob[1] = inv_prob[0] * PG[i];
				cv::meanStdDev(inv_prob[1], mean, stddev);
			}
			else
			{
				cv::meanStdDev(PG[i], mean, stddev);
			}
			mean.val[0] = mean.val[0] > min_mean1 ? mean.val[0] : min_mean1;
			mean.val[0] = mean.val[0] > max_mean1 ? max_mean1 : mean.val[0];
	
			if (mean1==0.) 
			{
				mean1 = mean.val[0];
			}
			mean1 = 0.5*mean1 + 0.5*mean.val[0];
			if(truncated){
				if(mean1<(gmax/2)) mean1 = mean1 - gmax/(exp(gmax/mean1)-1);
				if(mean1>=(gmax/2)) mean1 = (gmax/2);
			}
	
			stats->L1[i] = (1./(mean1)) > 0 ? (1./(mean1)) : 1 ;
	
			hist_temp = cv::Scalar(255, 255, 255);
	
			cv::calcHist(&PG[i], 1, 0, bin[4], stats->Hgram_G1[i], 1, &dim_40, &range_40_ptr, true, acc);
			stats->Hgram_G1_DISP[i] = stats->Hgram_G1[i].clone();
			draw_dist(40, stats->gmax[i], stats->Z1[i], stats->L1[i], hist_temp, 1);
			cv::addWeighted(hist_temp, 0.5, PG1_DISP[i], 0.5, 0, PG1_DISP[i]);
		
	
			hist_temp = cv::Scalar(255, 255, 255);
		
			if(weighted_mean)
			{
				inv_prob[2] = inv_prob[3] * PG[i];
				cv::meanStdDev(inv_prob[2], mean, stddev);
			}
			else
			{
				cv::meanStdDev(PG[i], mean, stddev);
			}
	
			mean.val[0] = gmax-mean.val[0];
			mean.val[0] = mean.val[0] > min_mean0 ? mean.val[0] : min_mean0;
	
			if (mean0==0.)
			{
				mean0 = mean.val[0];
			}
			mean0 = 0.5*mean0 + 0.5*mean.val[0];
		
			if(truncated)
			{
				if(mean0<(gmax/2))
				{
					mean0 = mean0 - gmax/(exp(gmax/mean0)-1);
				}
				if(mean0>=(gmax/2))
				{
					mean0 = (gmax/2);
				}
			}
			stats->L0[i] = (1./(mean0)) > 0 ? (1./(mean0)) : 1 ;
			cv::calcHist(&PG[i], 1, 0, bin[3], stats->Hgram_G0[i], 1, &dim_40, &range_40_ptr, true, acc);
			stats->Hgram_G0_DISP[i] = stats->Hgram_G0[i].clone();
		
			draw_dist(40, stats->gmax[i], stats->Z0[i], stats->L0[i], hist_temp, 0);
		
			cv::addWeighted(hist_temp, 0.5, PG0_DISP[i], 0.5, 0, PG0_DISP[i]);
		
			stats->Z1[i] = (1/stats->L1[i]) * (1-exp(-stats->L1[i] * stats->gmax[i]));
			stats->Z0[i] = (1/stats->L0[i]) * (exp(stats->L0[i] * stats->gmax[i])-1);
	
		}
	}





	void findObstacleBoundary(cv::Mat& Out)
	{
	
		static int y = (Out.rows) -1;
	
		bool flag0 = 0, flag1 = 0;
	
		for (int x=1; x < cvRound((Out.cols)/2)-1 ;x++) 
		{
			int x0 = cvRound((Out.cols)/2) - (x+1);
			int x1 = cvRound((Out.cols)/2) + (x+1);
	
			//		int In_index_0 	= (y*In->width+x0)*In->nChannels;
			//		int In_index_1 	= (y*In->width+x1)*In->nChannels;
	
			if(flag0 == 1 || Out.at<uchar>(y,x0) != 255){
				flag0 = 1;
				Out.at<uchar>(y,x0) = 0;
			}
			if(flag1 == 1 || Out.at<uchar>(y,x1) != 255){
				flag1 = 1;
				Out.at<uchar>(y,x1) = 0;
			}
		}
	
	
		for (int x=0; x < Out.cols ;x++) 
		{
			int flag = 0;
			for (int y=0; y < Out.rows ;y++) 
			{
				int y1 = (Out.rows) - (y+1);
	
				if(flag == 1 || Out.at<uchar>(y1,x) != 255){
					flag = 1;
					Out.at<uchar>(y1,x) = 0;
				}
			}
		}
	
	}

}