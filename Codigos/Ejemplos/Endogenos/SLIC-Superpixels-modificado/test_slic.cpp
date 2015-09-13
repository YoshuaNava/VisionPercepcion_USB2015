/*
 * test_slic.cpp.
 *
 * Written by: Pascal Mettes.
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
using namespace std;

#include "slic.h"


// Referencia sobre como configurar CodeBlocks para usar OpenCV:
// http://jonniedub.blogspot.com/2013/01/setting-up-codeblocks-ide-for-use-with.html

int main(int argc, char *argv[]) {
    /* Load the image and convert to Lab colour space. */
    //IplImage *image = cvLoadImage(argv[1], 1);
    IplImage *image = cvLoadImage("dog.png", 1);
    IplImage *lab_image = cvCloneImage(image);
    cvCvtColor(image, lab_image, CV_BGR2Lab);

    /* Yield the number of superpixels and weight-factors from the user. */
    int w = image->width, h = image->height;
    //int nr_superpixels = atoi(argv[2]);
    int nr_superpixels = 400;
    //int nc = atoi(argv[3]);
    int nc = 40;

    double step = sqrt((w * h) / (double) nr_superpixels);

    /* Perform the SLIC superpixel algorithm. */
    Slic slic;
    slic.generate_superpixels(lab_image, step, nc);
    slic.create_connectivity(lab_image);

    /* Display the contours and show the result. */
    slic.display_contours(image, CV_RGB(255,0,0));
    cvShowImage("result", image);
    cvWaitKey(0);
    //cvSaveImage(argv[4], image);
    cvSaveImage("dog_superpixelated.jpg", image);
}
