
#include "../include/global.h"
//#include "slic_superpixels/slic.h"



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

double proc_W, proc_H;
VideoCapture cap;

cv::Mat frame, seg_image, gray, prevgray; // Mat Declarations


static int int_sigma = 5;
static int int_k = 100;
static int min_size = 100;
float floatsigma;
const char * GBS 		= "seg"; //"Graph Based Segmentation";




void egbisSuperpixels()
{
	namedWindow( "SuperPixels", 1 ); 
	IplImage gray_temp = (IplImage)gray;
	IplImage* ipl_gray = &gray_temp; // Reference on deallocating memory: http://stackoverflow.com/questions/12635978/memory-deallocation-of-iplimage-initialised-from-cvmat
	IplImage* seg = cvCreateImage(cv::Size(proc_W, proc_H), 8, 1);;

    static int num_ccs;
    static float sigma;
    static float k;

            //printf("processing\n");
    floatsigma = (float)int_sigma*0.1;
    k = (float)int_k;
    SegmentImage(seg, ipl_gray, sigma, k, min_size, &num_ccs);



    seg_image = cv::cvarrToMat(seg);

	cv::imshow("SuperPixels", seg_image);

	for(int i=0; i<seg_image.rows ;i++)
	{
		for(int j=0; j<seg_image.cols ;j++)
		{
			cout << seg_image.at<uchar>(j,i) << "\n";
		}
	}
	cvReleaseImage(&seg);
}



void cameraSetup()
{
	//cap = VideoCapture(0); // Declare capture form Video: "eng_stat_obst.avi"
  

  //cap = VideoCapture(0);
	cap = VideoCapture("eng_stat_obst.avi");
	//cap = VideoCapture("Laboratorio.avi");
	// cap = VideoCapture("LaboratorioMaleta.avi");
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
		egbisSuperpixels();
		CV_TIMER_STOP(B, "Superpixels processed")


		CV_TIMER_STOP(Z, "Loop finished")
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	return 0;
}
