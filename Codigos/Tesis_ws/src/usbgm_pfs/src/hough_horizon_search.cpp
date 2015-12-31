
#include <hough_horizon_search.h>

using namespace ProbFloorSearch;

HoughHorizon::HoughHorizon(double img_H, double img_W)
{
	this->img_H = img_H;
	this->img_W = img_W;
}

HoughHorizon::~HoughHorizon()
{
	clearData();
}


void HoughHorizon::clearData()
{
	lines_dataset.clear();
	acc_lines_points.clear();
	superpixels_list.clear();
	superpixel_is_floor.clear();
	superpixels_isfloor_samples.clear();
	superpixels_floor_prob.clear();
}



cv::Mat HoughHorizon::getBordersImage()
{
	return this->borders_combined;	
}


cv::Mat HoughHorizon::getLinesImage()
{
	return this->img_lines;
}


cv::Mat HoughHorizon::getPolyBoundaryImage()
{
	return this->poly_boundary_img;
}


cv::Mat HoughHorizon::getTaggedSuperpixelsImage()
{
	return this->superpixels_below_boundary;
}


cv::Mat HoughHorizon::getColouredBayesImage()
{
	return coloured_bayes_floor;
}


cv::Mat HoughHorizon::getProbabilisticFloorEstimate()
{
	return this->floor_prob_map;
}


cv::Mat HoughHorizon::getBayesianFloorEstimate()
{
	return this->bayes_prob_floor;
}


void HoughHorizon::showImages()
{
	if(algorithmType == 's')
	{
		DISPLAY_IMAGE_XY(true, frame, 0, 0);
		cv::resizeWindow("frame", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, gray, 1, 0);
		cv::resizeWindow("gray", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, borders_combined, 2, 0);
		cv::resizeWindow("borders_combined", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, img_lines, 3, 0);
		cv::resizeWindow("img_lines", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, poly_boundary_img, 4, 0);
		cv::resizeWindow("poly_boundary_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_contours_img, 0, 1);
		cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_below_boundary, 1, 1);
		cv::resizeWindow("superpixels_below_boundary", proc_W, proc_H);
	}
	if(algorithmType == 'p')
	{
		DISPLAY_IMAGE_XY(true, frame, 0, 0);
		cv::resizeWindow("frame", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, gray, 1, 0);
		cv::resizeWindow("gray", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, borders_combined, 2, 0);
		cv::resizeWindow("borders_combined", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, img_lines, 3, 0);
		cv::resizeWindow("img_lines", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, poly_boundary_img, 4, 0);
		cv::resizeWindow("poly_boundary_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_contours_img, 0, 1);
		cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, floor_prob_map, 1, 1);
		cv::resizeWindow("floor_prob_map", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_below_boundary, 2, 1);
		cv::resizeWindow("superpixels_below_boundary", proc_W, proc_H);
	}
	if(algorithmType == 'b')
	{
		DISPLAY_IMAGE_XY(true, frame, 0, 0);
		cv::resizeWindow("frame", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, gray, 1, 0);
		cv::resizeWindow("gray", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, borders_combined, 2, 0);
		cv::resizeWindow("borders_combined", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, img_lines, 3, 0);
		cv::resizeWindow("img_lines", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, poly_boundary_img, 4, 0);
		cv::resizeWindow("poly_boundary_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_contours_img, 0, 1);
		cv::resizeWindow("superpixels_contours_img", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, floor_prob_map, 1, 1);
		cv::resizeWindow("floor_prob_map", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, superpixels_below_boundary, 2, 1);
		cv::resizeWindow("superpixels_below_boundary", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, bayes_prob_floor, 3, 1);
		cv::resizeWindow("bayes_prob_floor", proc_W, proc_H);
		DISPLAY_IMAGE_XY(true, coloured_bayes_floor, 4, 1);
		cv::resizeWindow("coloured_bayes_floor", proc_W, proc_H);
	}
}


void HoughHorizon::calculateSobelCannyBorders()
{
	cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny;
	int edgeThresh = 1;
	int lowThreshold = 3;
	int const max_lowThreshold = 100;
	int ratio = 5;
	int kernel_size = 3;
	//cv::equalizeHist(gray,gray);
	GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0 );
	cv::Sobel(gray, temp_grad[0], gray.depth(), 2, 0, 3, 15, 0, cv::BORDER_DEFAULT);
	cv::Sobel(gray, temp_grad[1], gray.depth(), 0, 2, 3, 15, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[0], sobel[0]);
	cv::convertScaleAbs(temp_grad[1], sobel[1]);
	addWeighted(sobel[0], 0.5, sobel[1], 0.5, 0, borders_sobel);
	

	//REFERENCE #1: http://stackoverflow.com/questions/16665742/a-good-approach-for-detecting-lines-in-an-image
	//REFERENCE #2: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
	cv::Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	cv::Mat skel(borders_sobel.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat eroded, dilated;
	cv::erode(borders_sobel, eroded, element);
	cv::dilate(eroded, dilated, element); // temp = open(img)
	cv::subtract(borders_sobel, dilated, dilated);
	cv::bitwise_or(skel, dilated, skel);
	eroded.copyTo(borders_sobel);

	GaussianBlur(gray, borders_canny, cv::Size(img_W/8+1-((int)img_W%8), (int)img_W/8+1-((int)img_W%8)), 0, 0);
	Canny(borders_canny, borders_canny, lowThreshold, lowThreshold*ratio, kernel_size, true);
	addWeighted(borders_sobel, 0.7, borders_canny, 0.3, 0.0, borders_combined);
}


void HoughHorizon::findLinesHough()
{
	cv::Vec4i line;
	vector<cv::Vec4i> lines;
	float line_slope;
	cv::Point aux_point;
	vector<cv::Point> lines_points;
	img_lines = frame.clone();
	HoughLinesP(borders_combined, lines, 1, CV_PI/180, 80, 20, 5);
	// Threshold for Sapienza dataset = 70
	// Threshold for ps3eye camera = 120
	
	for( size_t i = 0; i < lines.size(); i++ )
	{
		line = lines[i];
		if(abs(line[0] - line[2]) != 0)
		{
			line_slope = (float)(line[1] - line[3])/(line[0] - line[2]);
			if(abs(line_slope) < 1)
			{
				if((line[0] > img_W/2 && line_slope >= 0.0) || (line[2] < img_W/2 && line_slope <= 0.0))
				{
					if((line[1] < 2/3*img_H) || (line[3] < 2.0/3.0*img_H))
					{
						aux_point = cv::Point(line[0], line[1]);
						lines_points.push_back(aux_point);
						aux_point = cv::Point(line[2], line[3]);
						lines_points.push_back(aux_point);
						cv::line(img_lines, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 3, CV_AA);
					}
				}
			}
		}
	}
	if(acc_lines_points.size() < lines_history)
	{
		acc_lines_points.push_back(lines_points);
	}
	else
	{
		acc_lines_points.pop_front();
		acc_lines_points.push_back(lines_points);
	}
}



void HoughHorizon::fitPolynomialFloorContour()
{
	cv::Point aux_point;
	int j, k, l;
	for(j=0; j<acc_lines_points.size() ;j++)
	{
		for(k=0; k<acc_lines_points[j].size() ;k++)
		{
			lines_dataset.push_back(cv::Point(acc_lines_points[j][k].x, acc_lines_points[j][k].y));
		}
	}
	std::sort(lines_dataset.begin(), lines_dataset.end(), cvPointComparator);
	lines_dataset.erase( unique( lines_dataset.begin(), lines_dataset.end() ), lines_dataset.end() );

	Eigen::MatrixXd vandemonde(lines_dataset.size(), poly_degree+1);

	Eigen::VectorXd y(lines_dataset.size());
	for( size_t i = 0; i < lines_dataset.size(); i++ )
	{
		aux_point = lines_dataset[i];
		y(i) = aux_point.y;
		for(j=0; j<poly_degree+1 ;j++)
		{
			vandemonde(i,j) = pow(aux_point.x, j);
		}
	}
	poly_coeff = vandemonde.colPivHouseholderQr().solve(y);
}


void HoughHorizon::drawPolynomialFloorBoundary()
{
	poly_boundary_img = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);;
	int i, j;
	float poly_value;
	if(acc_lines_points[lines_history-1].size() > 0)
	{
		for(i=0; i<img_W ;i++)
		{
			poly_value = 0.0;
			for(j=0; j<poly_coeff.size() ;j++)
			{
				poly_value += poly_coeff(j)*pow(i, j);
			}

			if((poly_value >= 0) && (poly_value < img_H))
			{
				cv::circle(poly_boundary_img, cv::Point(i, poly_value), 1, CV_RGB(0,0,255), 3);
			}
		}
		for( size_t i = 0; i < lines_dataset.size(); i++ )
		{
			cv::Point aux_point;
			aux_point = lines_dataset[i];
			cv::circle(poly_boundary_img, cv::Point(aux_point.x, aux_point.y), 1, CV_RGB(0,255,0), 3);
		}
	}
	lines_dataset.clear();
}

void HoughHorizon::findSimpleSuperpixelsBelowBoundary()
{
	superpixels_below_boundary = poly_boundary_img.clone();
	superpixel_is_floor.clear();
	if(acc_lines_points[lines_history-1].size() > 0)
	{
		int i, j;
		point2Dvec points;
		float poly_value;
		int x_coord, y_coord;
		for(i=0; i<superpixels_list.size() ;i++)
		{
			cv::Point center = superpixels_list[i].get_center();
			poly_value = 0;
			for(j=0; j<poly_coeff.size() ;j++)
			{
				poly_value += poly_coeff(j)*pow(center.x, j);
			}

			if(poly_value < center.y)
			{
				superpixel_is_floor.push_back(1);
				points = superpixels_list[i].get_points();
				for(j=0; j < points.size() ;j++)
				{
					x_coord = points[j].x;
					y_coord = points[j].y;
					superpixels_below_boundary.at<cv::Vec3b>(y_coord, x_coord)[2] = 255;
				}
			}
			else
			{
				superpixel_is_floor.push_back(0);
			}
		}
		cvtColor(gray, gray, CV_GRAY2RGB);
		addWeighted(superpixels_below_boundary, 0.5, gray, 0.5, 0.0, superpixels_below_boundary);
	}
}



void HoughHorizon::findProbSuperpixelsBelowBoundary()
{
	superpixel_is_floor.clear();
	if(acc_lines_points[lines_history-1].size() > 0)
	{
		int i, j;
		float poly_value;
		for(i=0; i<superpixels_list.size() ;i++)
		{
			cv::Point center = superpixels_list[i].get_center();
			poly_value = 0;
			for(j=0; j<poly_coeff.size() ;j++)
			{
				poly_value += poly_coeff(j)*pow(center.x, j);
			}

			if(poly_value < center.y)
			{
				superpixel_is_floor.push_back(1);
			}
			else
			{
				superpixel_is_floor.push_back(0);
			}
		}

		if(superpixels_isfloor_samples.size() < superpixels_history)
		{
			superpixels_isfloor_samples.push_back(superpixel_is_floor);
		}
		else
		{
			superpixels_isfloor_samples.pop_front();
			superpixels_isfloor_samples.push_back(superpixel_is_floor);
		}
	}
}


void HoughHorizon::calculateFloorProbability()
{
	int i, j;
	superpixels_floor_prob.clear();
	vector<int> samples_count;
	for(i=0; i<superpixels_isfloor_samples.size() ;i++)
	{
		for(j=0; j<superpixels_isfloor_samples[i].size() ;j++)
		{
			if(superpixels_isfloor_samples[i][j] == 1)
			{
				if (superpixels_floor_prob.size() < j+1)
				{
					superpixels_floor_prob.push_back(1);	
				}
				else
				{
					superpixels_floor_prob[j]++;
				}
			}
			else
			{
				if (superpixels_floor_prob.size() < j+1)
				{
					superpixels_floor_prob.push_back(0);
				}
		 	}

			if(samples_count.size() < j+1)
			{
				samples_count.push_back(1);
			}
			else
			{
				samples_count[j]++;
			}
		}
	}

	for(i=0; i<samples_count.size() ;i++)
	{
		superpixels_floor_prob[i] = superpixels_floor_prob[i]/samples_count[i];
	}
}


void HoughHorizon::drawProbabilisticFloor()
{
	superpixels_below_boundary = poly_boundary_img.clone();
	floor_prob_map = cv::Mat::zeros(img_H, img_W, CV_32FC1);
	int i, j;
	point2Dvec points;
	int x_coord, y_coord;
	uchar blue_tonality, red_tonality;
	for(i=0; i<superpixels_list.size() ;i++)
	{
		if(i<superpixels_floor_prob.size())
		{
			if(superpixels_floor_prob[i] > 0)
			{
				blue_tonality = superpixels_floor_prob[i]*255;
				red_tonality = 255-superpixels_floor_prob[i]*255;
			}
			else
			{
				blue_tonality = 0;
				red_tonality = 0;
			}
			points = superpixels_list[i].get_points();
			for(j=0; j < points.size() ;j++)
			{
				x_coord = points[j].x;
				y_coord = points[j].y;
				superpixels_below_boundary.at<cv::Vec3b>(y_coord, x_coord)[0] = blue_tonality;
				superpixels_below_boundary.at<cv::Vec3b>(y_coord, x_coord)[2] = red_tonality;
				floor_prob_map.at<float>(y_coord, x_coord) = superpixels_floor_prob[i];
			}
		}
	}

	cvtColor(gray, gray, CV_GRAY2RGB);
	addWeighted(superpixels_below_boundary, 0.7, gray, 0.3, 0.0, superpixels_below_boundary);
}


void HoughHorizon::calculateBayesianEstimateFloor()
{
	coloured_bayes_floor = poly_boundary_img.clone();
	bayes_prob_floor = cv::Mat::zeros(img_H, img_W, CV_32FC1);
	int i, j;
	point2Dvec points;
	int x_coord, y_coord;
	uchar red_tonality;
	double prob_bayes_floor = 0;
	double prob_prior = 0;
	for(i=0; i<superpixels_list.size() ;i++)
	{
		if(i<superpixels_floor_prob.size())
		{
			prob_prior = floor_prior.at<float>(superpixels_list[i].get_center().y, superpixels_list[i].get_center().x);
			prob_bayes_floor = (superpixels_floor_prob[i] * prob_prior) / (superpixels_floor_prob[i] * prob_prior + 0.5 * (1-prob_prior));
			// cout << "prior floor =  " << prob_prior << "\n";
			// cout << "horizon floor =  " << superpixels_floor_prob[i] << "\n";
			// cout << "bayes floor =  " << prob_bayes_floor << "\n";
			red_tonality = prob_bayes_floor * 255;
			
			points = superpixels_list[i].get_points();
			for(j=0; j < points.size() ;j++)
			{
				x_coord = points[j].x;
				y_coord = points[j].y;
				bayes_prob_floor.at<float>(y_coord, x_coord) = prob_bayes_floor; 
				coloured_bayes_floor.at<cv::Vec3b>(y_coord, x_coord)[2] = red_tonality;
			}
		}
	}				
	addWeighted(coloured_bayes_floor, 0.7, gray, 0.3, 0.0, coloured_bayes_floor);
}



void HoughHorizon::doSimpleEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list)
{
	this->frame = frame.clone();
	this->gray = gray.clone();
	this->superpixels_contours_img = superpixels_contours_img;
	this->floor_prior = floor_prior.clone();
	this->superpixels_list = superpixels_list;
	this->algorithmType = 's';
	calculateSobelCannyBorders();
	findLinesHough();
	fitPolynomialFloorContour();
	drawPolynomialFloorBoundary();
	findSimpleSuperpixelsBelowBoundary();
//	showImages();
}


void HoughHorizon::doProbabilisticEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list)
{
	this->frame = frame.clone();
	this->gray = gray.clone();
	this->superpixels_contours_img = superpixels_contours_img;
	this->floor_prior = floor_prior.clone();
	this->superpixels_list = superpixels_list;
	this->algorithmType = 'p';
	calculateSobelCannyBorders();
	findLinesHough();
	fitPolynomialFloorContour();
	drawPolynomialFloorBoundary();
	findProbSuperpixelsBelowBoundary();
	calculateFloorProbability();
	drawProbabilisticFloor();
//	showImages();	
}


void HoughHorizon::doBayesianEstimation(cv::Mat frame, cv::Mat gray, cv::Mat superpixels_contours_img, cv::Mat floor_prior, vector<Superpixel> superpixels_list)
{
	this->frame = frame.clone();
	this->gray = gray.clone();
	this->superpixels_contours_img = superpixels_contours_img;
	this->floor_prior = floor_prior.clone();
	this->superpixels_list = superpixels_list;
	this->algorithmType = 'b';
	calculateSobelCannyBorders();
	findLinesHough();
	fitPolynomialFloorContour();
	drawPolynomialFloorBoundary();
	findProbSuperpixelsBelowBoundary();
	calculateFloorProbability();
	drawProbabilisticFloor();
	calculateBayesianEstimateFloor();
//	showImages();
}

