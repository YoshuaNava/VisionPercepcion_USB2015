#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <stdlib.h>
#include <iostream>
#include <cmath>
//using namespace cv;

bool using_grayscale_imgs = false;

std::string left_img_path = "../img/im0.png";
std::string right_img_path = "../img/im1.png";

cv::Mat left_img, right_img, left_gray, right_gray;
cv::Mat disp, disp8;

class rectifier{
    private:
        cv::Mat map_left1, map_left2, map_right1, map_right2;
        std::string path;
    public:
        rectifier(cv::Size);
        void show_rectified(cv::Size);
};

rectifier::rectifier(cv::Size image_size)
{
    cv::Mat left_cameraMatrix, left_distCoeffs, right_cameraMatrix, right_distCoeffs, R, T;
    float left_matrix[] = {3989.963, 0, 1039.414,
                        0, 3989.963, 956.875,
                        0, 0, 1};
    float right_matrix[] = {3989.963, 0, 1194.824,
                         0, 3989.963, 956.875,
                         0, 0, 1};
    left_cameraMatrix = cv::Mat(3, 3, CV_32FC1, left_matrix);
    right_cameraMatrix = cv::Mat(3, 3, CV_32FC1, right_matrix);

    std::cout << left_cameraMatrix;
}

int StereoBlockMatching()
{
    cv::StereoBM sbm;
    sbm.state->SADWindowSize = 9;
    sbm.state->numberOfDisparities = 112;
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;
    sbm.state->minDisparity = -39;
    sbm.state->textureThreshold = 507;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 1;

    sbm(left_gray, right_gray, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    return 0;
}

int StereoSemiBlockMatching()
{
    cv::StereoSGBM sgbm;
    sgbm.SADWindowSize = 5;
    sgbm.numberOfDisparities = 192;
    sgbm.preFilterCap = 4;
    sgbm.minDisparity = -64;
    sgbm.uniquenessRatio = 1;
    sgbm.speckleWindowSize = 150;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 10;
    sgbm.fullDP = false;
    sgbm.P1 = 600;
    sgbm.P2 = 2400;

    sgbm(left_gray, right_gray, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    return 0;
}

void LoadImages()
{
    if (using_grayscale_imgs)
    {
        left_gray = cv::imread(left_img_path);
        right_gray = cv::imread(right_img_path);
    }
    else
    {
        left_img = cv::imread(left_img_path);
        right_img = cv::imread(right_img_path);
        cv::cvtColor(left_img, left_gray, CV_BGR2GRAY);
        cv::cvtColor(right_img, right_gray, CV_BGR2GRAY);
        //Reference: http://stackoverflow.com/questions/20822288/python-opencv-cv2-opencv-error-assertion-failed-scn-3-scn-4-in-unkno
    }

}


int TestOpenCVLinkage()
{
    /* Reference on how to install OpenCV + Code::Blocks:
    http://jonniedub.blogspot.in/2013/01/setting-up-codeblocks-ide-for-use-with.html */
    cv::Mat img = cv::imread("lena.jpg", CV_LOAD_IMAGE_COLOR);
    if(img.empty())
       return -1;
    cv::namedWindow( "lena", CV_WINDOW_AUTOSIZE );
    cv::imshow("lena", img);
    return 0;
}

int main(int argc, char *argv[])
{
    //int result = TestOpenCVLinkage();
    //std::cout << "Hola " << result;

    LoadImages();
    //StereoBlockMatching();
    rectifier::rectifier(left_gray.size());
    //StereoSemiBlockMatching();

    //Reference: http://stackoverflow.com/questions/15901015/opencv-resizewindow-do-nothing
    namedWindow("Left", cv::WINDOW_NORMAL);
    namedWindow("Right", cv::WINDOW_NORMAL);
    namedWindow("Disparity", cv::WINDOW_NORMAL);
    imshow("Left", left_img);
    imshow("Right", right_img);
    imshow("Disparity", disp8);
    cv::resizeWindow("Left",800,600);
    cv::resizeWindow("Right",800,600);
    cv::resizeWindow("Disparity",800,600);


    cv::waitKey(0);
    return EXIT_SUCCESS;
}



