#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <iostream>
#include <cmath>
using namespace cv;

std::string img1_path = "../img/eval-data-gray/Army/frame10.png";
std::string img2_path = "../img/eval-data-gray/Army/frame11.png";


int main(int argc, char *argv[])
{
    cv::Mat img1 = imread(img1_path);
    cv::Mat img2 = imread(img2_path);
    cv::Mat diff_img = img2-img1;
    cv::Mat binary_diff_img = (abs(diff_img) > 0);

    namedWindow("Image difference", CV_WINDOW_AUTOSIZE );
    imshow("Image difference", binary_diff_img);
    waitKey(0);
    return 0;
}



