#ifndef FUNCTIONS
#define FUNCTIONS

#include "iostream"
#include "stdio.h" //standard input-output-library
#include "algorithm" //functions on sequences, like find, sort etc. 
#include "stdlib.h" //includes random number generation
#include "time.h" //get date and time information
#include "math.h"
#include "string.h" //manipulate strings and arrays
#include "iomanip"
#include "fstream"
#include "vector"
#include "thread"
#include <float.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"


using namespace std;
using namespace cv;

struct mat_variables {

IplImage *hp;
Mat hola;
};
typedef struct mat_variables Mat_Variables; 

void ShowImages2(Mat src);
void DrawSaveWindow(Mat frame,Mat& frame_out);
void CreateSaveWindow(Mat frame,Mat& frame_window);
void CreateWindowCluster(Mat Cluster,Mat& Window_Cluster);
void CompareHistograms(Mat frame,double Valor_hist,Mat& img_out);
#endif
