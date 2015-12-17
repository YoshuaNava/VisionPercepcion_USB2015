
#include <global.h>

#include <probabilistic_functions.h>
#include <segmentation_handler.h>


using namespace std;
using namespace cv;
using namespace ProbFloorSearch;

cv::Mat frame, gray, superpixels_contours_img, floor_prior; // Mat Declarations
cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny, borders_combined;
cv::Mat img_lines, floor_boundary_img, superpixels_below_boundary, floor_prob_map;cv::Mat bayes_prob_floor, coloured_bayes_floor;
vector<Vec4i> lines;
vector<cv::Point> lines_dataset;
int lines_history = 5;
deque<vector<cv::Point>> acc_lines_points;
vector<cv::Point> polynomial_fit;
vector<int> superpixel_is_floor;


VideoCapture cap;
vector<Superpixel> superpixels_list;

SegmentationHandler segHandler("SLIC");
int poly_degree = 3;
Eigen::VectorXd poly_coeff;

int superpixels_history = 5;
deque<vector<int>> superpixels_isfloor_samples;
vector<double> superpixels_floor_prob;


//REFERENCE: http://stackoverflow.com/questions/16796732/how-to-sort-vector-of-points-based-on-a-y-axis
struct cvPointComparator {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} cvPointComparator;



void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0, 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, gray, 1, 0);
	cv::resizeWindow("gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, borders_combined, 2, 0);
	cv::resizeWindow("borders_combined", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, img_lines, 3, 0);
	cv::resizeWindow("img_lines", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, floor_boundary_img, 4, 0);
	cv::resizeWindow("floor_boundary_img", proc_W, proc_H);
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


void calculateSobel()
{
	//cv::equalizeHist(gray,gray);
	GaussianBlur(gray, gray, Size(5, 5), 0, 0 );
	cv::Sobel(gray, temp_grad[0], gray.depth(), 2, 0, 3, 15, 0, BORDER_DEFAULT);
	cv::Sobel(gray, temp_grad[1], gray.depth(), 0, 2, 3, 15, 0, BORDER_DEFAULT);
	cv::convertScaleAbs(temp_grad[0], sobel[0]);
	cv::convertScaleAbs(temp_grad[1], sobel[1]);
	addWeighted(sobel[0], 0.5, sobel[1], 0.5, 0, borders_sobel);
	

	//REFERENCE #1: http://stackoverflow.com/questions/16665742/a-good-approach-for-detecting-lines-in-an-image
	//REFERENCE #2: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
	cv::Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	cv::Mat skel(borders_sobel.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat eroded, dilated;
	cv::erode(borders_sobel, eroded, element);
	cv::dilate(eroded, dilated, element); // temp = open(img)
	cv::subtract(borders_sobel, dilated, dilated);
	cv::bitwise_or(skel, dilated, skel);
	eroded.copyTo(borders_sobel);
}


void calculateCanny()
{
	int edgeThresh = 1;
	int lowThreshold = 3;
	int const max_lowThreshold = 100;
	int ratio = 5;
	int kernel_size = 3;
	GaussianBlur(gray, borders_canny, Size(proc_W/8+1-((int)proc_W%8), (int)proc_W/8+1-((int)proc_W%8)), 0, 0);

	/// Canny detector
	Canny(borders_canny, borders_canny, lowThreshold, lowThreshold*ratio, kernel_size, true);
}


void findLinesHough()
{
	Vec4i l;
	float line_slope;
	cv::Point aux_point;
	vector<cv::Point> lines_points;
	img_lines = frame.clone();
	
	HoughLinesP(borders_combined, lines, 1, CV_PI/180, 70, 20, 5);
	// Threshold for Sapienza dataset = 70
	// Threshold for ps3eye camera = 120
	for( size_t i = 0; i < lines.size(); i++ )
	{
		l = lines[i];

		if(abs(l[0] - l[2]) != 0)
		{
			line_slope = (float)(l[1] - l[3])/(l[0] - l[2]);

			if(abs(line_slope) < 1)
			{
				//cout << line_slope << "\n";
				if((l[0] > proc_W/2 && line_slope >= 0.0) || (l[2] < proc_W/2 && line_slope <= 0.0))
				{
					if((l[1] < 2/3*proc_H) || (l[3] < 2.0/3.0*proc_H))
					{
						aux_point = cv::Point(l[0], l[1]);
						// printf("x = %d  ;  y = %d\n", l[0],l[1]);
						// printf("x = %d  ;  y = %d\n", l[2],l[3]);
						lines_points.push_back(aux_point);
						aux_point = cv::Point(l[2], l[3]);
						lines_points.push_back(aux_point);
						cv::line(img_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
					}
				}
			}
		}
	}

	
	//cout << "Points history array length " << acc_lines_points.size() << "\n";
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



void fitPolynomialFloorContour()
{
	cv::Point aux_point;
	int j, k, l;
	for(j=0; j<acc_lines_points.size() ;j++)
	{
		for(k=0; k<acc_lines_points[j].size() ;k++)
		{
			//std::copy(acc_lines_points[j][k].begin(), acc_lines_points[j][k].end(), lines_dataset.begin());
			lines_dataset.push_back(cv::Point(acc_lines_points[j][k].x, acc_lines_points[j][k].y));
		}
	}
	std::sort(lines_dataset.begin(), lines_dataset.end(), cvPointComparator);
	lines_dataset.erase( unique( lines_dataset.begin(), lines_dataset.end() ), lines_dataset.end() );


	Eigen::MatrixXd vandemonde(lines_dataset.size(), poly_degree+1);
	// Eigen::VectorXd x(lines_points.size());

	Eigen::VectorXd y(lines_dataset.size());
	for( size_t i = 0; i < lines_dataset.size(); i++ )
	{
		aux_point = lines_dataset[i];
		// x(i) = aux_point.x;
		y(i) = aux_point.y;
		//printf("x = %d  ;  y = %d\n", aux_point.x, aux_point.y);
		for(j=0; j<poly_degree+1 ;j++)
		{
			vandemonde(i,j) = pow(aux_point.x, j);
		}
	}
	poly_coeff = vandemonde.colPivHouseholderQr().solve(y);
	// cout << "x:\n" << x << endl;
	// cout << "y:\n" << y << endl;
	// cout << "Vandemonde matrix:\n" << vandemonde << endl;
	// cout << "Vandemonde " << poly_degree << "-th degree polynomial coefficients:\n" << poly_coeff << endl;
}


void drawPolynomialFloorBoundary()
{
	floor_boundary_img = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);;
	int i, j;
	float poly_value;
	if(acc_lines_points[lines_history-1].size() > 0)
	{
		for(i=0; i<proc_W ;i++)
		{
			poly_value = 0.0;
			for(j=0; j<poly_coeff.size() ;j++)
			{
				poly_value += poly_coeff(j)*pow(i, j);
			}

			if((poly_value >= 0) && (poly_value < proc_H))
			{
				cv::circle(floor_boundary_img, cv::Point(i, poly_value), 1, CV_RGB(0,0,255), 3);
			}
		}
		for( size_t i = 0; i < lines_dataset.size(); i++ )
		{
			cv::Point aux_point;
			aux_point = lines_dataset[i];
			cv::circle(floor_boundary_img, cv::Point(aux_point.x, aux_point.y), 1, CV_RGB(0,255,0), 3);
		}
	}
	lines_dataset.clear();
}


void findSuperpixelsBelowBoundary()
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


void calculateFloorProbability()
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
		//cout << "Superpixel " << i << "  positive samples  " << superpixels_floor_prob[i] << "   number of samples  " << samples_count[i] << "\n";
		superpixels_floor_prob[i] = superpixels_floor_prob[i]/samples_count[i];
		//cout << "   probability of being floor  " << superpixels_floor_prob[i] << "\n";
	}
}


void drawProbabilisticFloor()
{
	superpixels_below_boundary = floor_boundary_img.clone();
	floor_prob_map = cv::Mat::zeros(proc_H, proc_W, CV_32FC1);
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


void calculateBayesianEstimateFloor()
{
	coloured_bayes_floor = floor_boundary_img.clone();
	bayes_prob_floor = cv::Mat::zeros(proc_H, proc_W, CV_32FC1);
	int i, j;
	point2Dvec points;
	int x_coord, y_coord;
	uchar green_tonality;
	double prob_bayes_floor = 0;
	double prob_prior = 0;
	for(i=0; i<superpixels_list.size() ;i++)
	{
		if(i<superpixels_floor_prob.size())
		{
			prob_prior = floor_prior.at<float>(superpixels_list[i].get_center().y, superpixels_list[i].get_center().x);
			prob_bayes_floor = (superpixels_floor_prob[i] * prob_prior) / (superpixels_floor_prob[i] * prob_prior + 0.7 * (1-prob_prior));
			// cout << "prior floor =  " << prob_prior << "\n";
			// cout << "horizon floor =  " << superpixels_floor_prob[i] << "\n";
			// cout << "bayes floor =  " << prob_bayes_floor << "\n";
			if(superpixels_floor_prob[i] > 0)
			{
				green_tonality = prob_bayes_floor * 255;
			}
			else
			{
				green_tonality = 0;
			}
			
			points = superpixels_list[i].get_points();
			for(j=0; j < points.size() ;j++)
			{
				x_coord = points[j].x;
				y_coord = points[j].y;
				bayes_prob_floor.at<float>(y_coord, x_coord) = prob_bayes_floor; 
				coloured_bayes_floor.at<cv::Vec3b>(y_coord, x_coord)[1] = green_tonality;
			}
		}
	}				
	addWeighted(superpixels_below_boundary, 0.7, coloured_bayes_floor, 0.3, 0.0, coloured_bayes_floor);
}




void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

  cap = VideoCapture(0);
	//cap = VideoCapture("eng_stat_obst.avi");
	//cap = VideoCapture("Laboratorio.avi");
	//cap = VideoCapture("LaboratorioMaleta.avi");
	//cap = VideoCapture("PasilloLabA.avi");
	//cap = VideoCapture("PasilloLabB.avi");
 // cap = VideoCapture("Laboratorio4.avi");
//  cap = VideoCapture("Calle1.avi");
	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	// Good reference: http://superuser.com/questions/897420/how-to-know-which-framerate-should-i-use-to-capture-webcam-with-ffmpeg
	cap.set(CV_CAP_PROP_FPS, 30);
}



/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            MAIN           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
int main( int argc, char** argv ) 
{
	ros::init(argc, argv, "Plane_Segmentation");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	cameraSetup();
	cap.read(frame);

	printf("******************************************\n");
	printf("Probabilistic Ground Plane Segmentation\n");
	printf("Authors: Rafael Colmenares and Yoshua Nava\n");
	printf("******************************************\n");
	waitKey(0);
	
    
	while (nh.ok()) 
	{
		CV_TIMER_START(X)
		//cap >> frame; // Image from Cam to Mat Variable
		if (!cap.read(frame)) 
		{
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}

		printf("#######################################\n");
		CV_TIMER_STOP(A, "Received image from camera")
		cv:resize(frame, frame, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(frame, gray, CV_BGR2GRAY);
		waitKey(1); // Wait Time

		
		superpixels_contours_img = frame.clone();
		segHandler.segmentImage(frame, gray);
		superpixels_list = segHandler.getSuperpixels();
		superpixels_contours_img = segHandler.getContoursImage();
		CV_TIMER_STOP(B, "Superpixels processed")
		floor_prior = ProbFns::getFloorPrior(frame, superpixels_list);
		CV_TIMER_STOP(C, "Prior probability calculated")
		calculateCanny();
		calculateSobel();
		addWeighted(borders_sobel, 0.7, borders_canny, 0.3, 0.0, borders_combined);
		CV_TIMER_STOP(D, "Applied Canny and Sobel edge detectors")
		findLinesHough();
		CV_TIMER_STOP(E, "Found lines using Hough Transform")
		fitPolynomialFloorContour();
		CV_TIMER_STOP(F, "Fitted a polynomial to the points given by the Hough Transform")
		drawPolynomialFloorBoundary();
		CV_TIMER_STOP(G, "Drawn polynomial floor boundary")
		findSuperpixelsBelowBoundary();
		CV_TIMER_STOP(H, "Found superpixels below boundary for present iteration")
		calculateFloorProbability();
		CV_TIMER_STOP(I, "Calculated probability of a superpixel being floor")
		drawProbabilisticFloor();
		CV_TIMER_STOP(J, "Tagged superpixels below probabilistic boundary")
		calculateBayesianEstimateFloor();
		CV_TIMER_STOP(K, "Tagged superpixels below bayesian boundary")
		showImages();


		CV_TIMER_STOP(Z, "Loop finished")
		printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
