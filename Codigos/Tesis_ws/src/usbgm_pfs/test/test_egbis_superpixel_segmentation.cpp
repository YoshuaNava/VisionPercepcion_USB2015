
#include "../include/global.h"
#include <libsuperpixel/superpixel.h>



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

cv::Mat frame, seg_image, gray, contours_image; // Mat Declarations


static int int_sigma = 5;
static int int_k = 100;
static int min_size = 100;
float floatsigma;
const char * GBS 		= "seg"; //"Graph Based Segmentation";
vector<vector<int>> clusters;
int num_superpixels = -1;
vector<int> superpixels_centers_count;
vector<cv::Point> superpixels_centers;
vector<Superpixel> superpixels_list;
vector<vector<int>> superpixels_adjacency_matrix;
vector<vector<int>> superpixels_Gsimilarity_matrix;

void showImages()
{
	DISPLAY_IMAGE_XY(true, frame, 0, 0);
	cv::resizeWindow("frame", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, gray, 1, 0);
	cv::resizeWindow("gray", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, seg_image, 2, 0);
	cv::resizeWindow("seg_image", proc_W, proc_H);
	DISPLAY_IMAGE_XY(true, contours_image, 3, 0);
	cv::resizeWindow("contours_image", proc_W, proc_H);
}

void displayCenterGrid(cv::Mat image, CvScalar colour) 
{
    for (int i=0; i<num_superpixels ;i++) 
    {
        cv::circle(image, cvPoint(superpixels_centers[i].x, superpixels_centers[i].y), 2, colour, 2);
    }
}


void storeSuperpixels()
{
    int i, x, y, k;
    int index = 0;
    cv::Point temp_point;
    superpixels_adjacency_matrix.resize(num_superpixels);
    const int dx[8] = {-1, -1, -1, 0, +1, +1, +1, 0};
    const int dy[8] = {-1, 0, +1, +1, +1, 0, -1, -1};

    for(i = 0 ; i < num_superpixels ;++i)
    {
        //Grow Columns by n
        superpixels_adjacency_matrix[i].resize(num_superpixels);
        std::fill(superpixels_adjacency_matrix[i].begin(), superpixels_adjacency_matrix[i].end(), 0);
    }
    for (i = 0; i < num_superpixels ;i++) 
    {
        temp_point = cvPoint(superpixels_centers[i].x, superpixels_centers[i].y);
        Superpixel sp(i, temp_point);
        superpixels_list.push_back(sp);
    }

    for (x = 0; x < frame.cols; x++)
    {
        for (y = 0; y < frame.rows; y++)
        {
    		// cout << "num SuperPixels  " << superpixels_list.size() << "\n";
      //       cout << "hola " << clusters[x][y] << "\n";
            if(clusters[x][y] != -1)
            {
                index = clusters[x][y];
            }
            //CvScalar colour = cvGet2D(image, y, x);
            temp_point = cv::Point(x, y);

            //cout << index << "\n";
            superpixels_list[index].add_point(temp_point);
            for(k=0; k<8 ;k++)
            {
                if(((x+dx[k]>=0) && (x+dx[k]<frame.cols)) && ((y+dy[k]>=0) && (y+dy[k]<frame.rows)))
                {
                    if(clusters[x+dx[k]][y+dy[k]] == index)
                    {
                         superpixels_adjacency_matrix[index][index] = -1;
                    }
                    else
                    {
                        //cout << "index = " << index << "    x+dx[k] = " << x+dx[k] << "     y+dy[k] = " << y+dy[k] << "\n";
                        if(clusters[x+dx[k]][y+dy[k]] != -1)
                        {
                            superpixels_adjacency_matrix[index][clusters[x+dx[k]][y+dy[k]]] = 1;
                            superpixels_adjacency_matrix[clusters[x+dx[k]][y+dy[k]]][index] = 1;
                        }
                    }
                }
            }
        }
    }
    // cout << "bye\n";


    // for(i=0; i<num_superpixels ;++i)
    // {
    //     for(k=0; k<num_superpixels ;k++)    //Grow Columns by n
    //     {
    //         cout << (int) superpixels_adjacency_matrix[i][k] << "   ";
    //     }
    //     cout << "\n";
    // }
    // cout << "**************************************************************************************\n";

    for (i = 0; i < (int) superpixels_list.size(); i++) 
    {
  //   	vector<cv::Point> points = superpixels_list[i].get_points();
		// cout << "superpixel  " << superpixels_list[i].get_id() << "	points count  " << points.size() << "\n";
        //superpixels_list[i].calculate_bounding_rect();
        superpixels_list[i].add_pixels_information(frame, clusters);
        superpixels_list[i].calculate_histogram();
    }
    //superpixels_list[0].print_everything();
}



void calculateSuperpixelCenters()
{
	int j, k, c_id;
    /* Compute the new cluster centers. */
    for (j=0; j<frame.cols ;j++) 
    {
        for (k = 0; k<frame.rows ;k++) 
        {
            c_id = clusters[j][k];
            
            if (c_id != -1) 
            {
            	superpixels_centers[c_id].x += j;
            	superpixels_centers[c_id].y += k;    
                superpixels_centers_count[c_id] += 1;
            }
        }
    }

    /* Normalize the clusters. */
    for (j=0; j<num_superpixels ;j++) {
    	//cout << "superpixel  " << j << "	centers count  " << superpixels_centers_count[j] << "\n";
        superpixels_centers[j].x /= superpixels_centers_count[j];
        superpixels_centers[j].y /= superpixels_centers_count[j];
    }

}



void outlineSuperpixelsContours(CvScalar colour) {
    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
	
	/* Initialize the contour vector and the matrix detailing whether a pixel
	 * is already taken to be a contour. */
	vector<CvPoint> contours;
	vec2db istaken;
	for (int i = 0; i < seg_image.cols; i++) { 
        vector<bool> nb;
        for (int j = 0; j < seg_image.rows; j++) {
            nb.push_back(false);
        }
        istaken.push_back(nb);
    }
    
    /* Go through all the pixels. */
    for (int i = 0; i < seg_image.cols; i++) {
        for (int j = 0; j < seg_image.rows; j++) {
            int nr_p = 0;
            
            /* Compare the pixel to its 8 neighbours. */
            for (int k = 0; k < 8; k++) {
                int x = i + dx8[k], y = j + dy8[k];
                
                if (x >= 0 && x < seg_image.cols && y >= 0 && y < seg_image.rows) {
                    if (istaken[x][y] == false && clusters[i][j] != clusters[x][y]) {
                        nr_p += 1;
                    }
                }
            }
            
            /* Add the pixel to the contour list if desired. */
            if (nr_p >= 2) {
                contours.push_back(cvPoint(i,j));
                istaken[i][j] = true;
            }
        }
    }
    
    /* Draw the contour pixels. */
    for (int i = 0; i < (int)contours.size(); i++) {
		contours_image.at<cv::Vec3b>(contours[i].y, contours[i].x).val[0] = colour.val[0];
		contours_image.at<cv::Vec3b>(contours[i].y, contours[i].x).val[1] = colour.val[1];
		contours_image.at<cv::Vec3b>(contours[i].y, contours[i].x).val[2] = colour.val[2];
    }
}



void egbisSuperpixels()
{
	superpixels_centers_count.clear();
	superpixels_centers.clear();
	superpixels_list.clear();
	clusters.clear();

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

    seg_image = cv::cvarrToMat(seg, true, true, 0);
    cvReleaseImage(&seg);
	num_superpixels = -1;
	int i, j;
	for(i=0; i<seg_image.cols ;i++)
	{
		vector<int> seg_row;
		for(j=0; j<seg_image.rows ;j++)
		{
//			cout << "u = " << j << "  ;  v = " << i << "	;	Superpixel = " << seg_image.at<uchar>(i,j) << "\n";
			seg_row.push_back((int)seg_image.at<uchar>(j,i));
			if((int)seg_image.at<uchar>(j,i) > num_superpixels)
			{
				num_superpixels = seg_image.at<uchar>(j,i);
			}
		}
		clusters.push_back(seg_row);
	}
	num_superpixels = num_superpixels + 1;

	for(i=0; i<num_superpixels ;i++)
	{
		superpixels_centers.push_back(cv::Point(0.0, 0.0));
		superpixels_centers_count.push_back(1);
	}



	contours_image = frame.clone();
	outlineSuperpixelsContours(cv::Scalar(255,0,0));
	calculateSuperpixelCenters();
	storeSuperpixels();
	displayCenterGrid(contours_image, cv::Scalar(0,255,0));
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

		egbisSuperpixels();
		CV_TIMER_STOP(B, "Superpixels processed")

		showImages();

		CV_TIMER_STOP(Z, "Loop finished")
	 	ros::spinOnce();
	}


	loop_rate.sleep();
	cap.release(); //Destroy the Capture from webcam
	destroyAllWindows(); //Destroy the windows


	return 0;
}
