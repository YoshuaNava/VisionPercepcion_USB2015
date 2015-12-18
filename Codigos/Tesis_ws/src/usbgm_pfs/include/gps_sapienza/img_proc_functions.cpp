
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

}

