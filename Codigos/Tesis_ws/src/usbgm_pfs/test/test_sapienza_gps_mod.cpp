
#include <global.h>



//macros for stopwatch
//REFERENCE: https://sites.google.com/site/mikesapi/downloads/ground-plane-segmentation-and-autonomous-guidance
#define CV_TIMER_START(X)       	double X = (double)cvGetTickCount();
#define CV_TIMER_STOP(Y, STRING) 	double Y = (double)cvGetTickCount() - X; \
                                            printf("Time @ [%s] = %gms\n", \
                                            STRING, Y/((double)cvGetTickFrequency()*1000.) );

#define Window_W 1.02*proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(proc_H)+20
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cv::imshow(#img, img);
using namespace std;
using namespace cv;


cv::Mat frame, seg_image, gray, prevgray, floor_prior; // Mat Declarations

cv::Mat temp_grad[3], sobel[3], borders_sobel, borders_canny, borders_combined;
cv::Mat img_lines, contoured_img, superpixels_below_boundary;
vector<Vec4i> lines;
vector<cv::Point> lines_dataset;
int lines_history = 5;
deque<vector<cv::Point>> acc_lines_points;
vector<cv::Point> polynomial_fit;
vector<int> superpixel_is_floor;


double proc_W, proc_H;
VideoCapture cap;

Slic slic;
int poly_degree = 3;
Eigen::VectorXd poly_coeff;


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
	DISPLAY_IMAGE_XY(true, contoured_img, 4, 0);
	cv::resizeWindow("contoured_img", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, superpixels_below_boundary, 0, 1);
	cv::resizeWindow("superpixels_below_boundary", proc_W, proc_H);

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
	
	HoughLinesP(borders_combined, lines, 1, CV_PI/180, 150, 20, 5);
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
//	contoured_img = frame.clone();
	contoured_img = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);;
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
				contoured_img.at<cv::Vec3b>(poly_value, i)[0] = 255;
				contoured_img.at<cv::Vec3b>(poly_value, i)[1] = 0;
				contoured_img.at<cv::Vec3b>(poly_value, i)[2] = 0;
			}
			//cout << "x = " << i << "  ;  y = " << poly_value << "\n";
		}
		for( size_t i = 0; i < lines_dataset.size(); i++ )
		{
			cv::Point aux_point;
			aux_point = lines_dataset[i];
			contoured_img.at<cv::Vec3b>(aux_point.y, aux_point.x)[0] = 0;
			contoured_img.at<cv::Vec3b>(aux_point.y, aux_point.x)[1] = 255;
			contoured_img.at<cv::Vec3b>(aux_point.y, aux_point.x)[2] = 0;
		}
	}
	lines_dataset.clear();
}


void findSuperpixelsBelowBoundary()
{
	superpixels_below_boundary = contoured_img.clone();
	superpixel_is_floor.clear();
	if(acc_lines_points[lines_history-1].size() > 0)
	{
		vector<Superpixel> superpixel_list = slic.get_superpixels();
		int i, j;
		point2Dvec points;
		float poly_value;
		int x_coord, y_coord;
		for(i=0; i<superpixel_list.size() ;i++)
		{
			cv::Point center = superpixel_list[i].get_center();
			poly_value = 0;
			for(j=0; j<poly_coeff.size() ;j++)
			{
				poly_value += poly_coeff(j)*pow(center.x, j);
			}

			if(poly_value < center.y)
			{
				superpixel_is_floor.push_back(1);
				points = superpixel_list[i].get_points();
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

		addWeighted(superpixels_below_boundary, 0.5, frame, 0.5, 0.0, superpixels_below_boundary);
	}
}

void superpixels(cv::Mat src)
{
	namedWindow( "SuperPixels", 1 ); 
	IplImage frame2 = (IplImage)src; // Reference on deallocating memory: http://stackoverflow.com/questions/12635978/memory-deallocation-of-iplimage-initialised-from-cvmat

	  /* Yield the number of superpixels and weight-factors from the user. */
	IplImage *lab_image = cvCloneImage(&frame2);
	cvCvtColor(&frame2, lab_image, CV_BGR2Lab);
	int w = lab_image->width, h = lab_image->height;
	//int nr_superpixels = atoi(argv[2]);
	int nr_superpixels = 6*proc_W;
	//int nc = atoi(argv[3]);
	int nc = 20;
	double step = sqrt((w * h) / (double) nr_superpixels)*3;

	/* Perform the SLIC superpixel algorithm. */
	slic.clear_data();
	slic.generate_superpixels(lab_image, step, nc);
	slic.create_connectivity(lab_image);
		//slic.colour_with_cluster_means(&frame2);
	slic.store_superpixels(&frame2);

	//slic.export_superpixels_to_files(&frame2);
	slic.display_contours(&frame2, CV_RGB(255,0,0));

	slic.display_number_grid(&frame2, CV_RGB(0,255,0));

	//slic.show_histograms(1,32);

	//slic.display_center_grid(frame2, CV_RGB(0,255,0));

	cvShowImage("SuperPixels", &frame2);
	cvReleaseImage(&lab_image);
	cvWaitKey(10);
}


void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

  //cap = VideoCapture(0);
	//cap = VideoCapture("eng_stat_obst.avi");
	//cap = VideoCapture("Laboratorio.avi");
	cap = VideoCapture("LaboratorioMaleta.avi");
	//cap = VideoCapture("PasilloLabA.avi");
	//cap = VideoCapture("PasilloLabB.avi");

	//VideoCapture cap(1); //Otra camara, conectada a la computadora mediante USB, por ejemplo.
	
//	proc_W = 160;//160
//	proc_H = 120;//120
	proc_W = 320;//160
	proc_H = 240;//120
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
	waitKey(10);
    
	while (nh.ok()) 
	{
		CV_TIMER_START(X)
		//cap >> frame; // Image from Cam to Mat Variable
		if (!cap.read(frame)) 
		{
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}

		CV_TIMER_STOP(A, "Received image from camera")
		cv:resize(frame, frame, Size(proc_W, proc_H), 0, 0, INTER_AREA);
		cvtColor(frame, gray, CV_BGR2GRAY);
		waitKey(1); // Wait Time

		
		seg_image = frame.clone();
		superpixels(seg_image);
		CV_TIMER_STOP(B, "Superpixels processed")
		floor_prior = ProbFns::getFloorPrior(frame, slic.get_superpixels());
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
		CV_TIMER_STOP(H, "Tagged superpixels below boundary")
		showImages();


		CV_TIMER_STOP(Z, "Loop finished")
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	//ros::spin();

	return 0;
}
