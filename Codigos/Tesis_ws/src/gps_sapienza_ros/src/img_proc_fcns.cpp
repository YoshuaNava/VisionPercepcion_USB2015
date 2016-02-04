/*
 *  Copyright (C) <2014>  <Michael Sapienza>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <algorithm>
using namespace std;

#include "img_proc_fcns.h"

#include <cstdio>
#include <cstdlib>
#include <math.h>
#include "image.h"
#include "misc.h"
#include "segment-image.h"

//Binary image zero & one

#define ZERO	0
#define ONE	255

#define LP	0.01
#define HP	0.99

#define Window_W 1.02*p.proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(p.proc_H)+20

#define CV_TIMER_START(X)       	double X = (double)cvGetTickCount();
#define CV_TIMER_STOP(X, STRING) 	X = (double)cvGetTickCount() - X; \
                                            printf("Time @ [%s] = %gms\n", \
                                            STRING, X/((double)cvGetTickFrequency()*1000.) );

#define DISPLAY_IMAGE(img)			cvNamedWindow(#img); cvShowImage(#img, img);
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(#img, img);
#define DISPLAY_IMAGE_XY_NAME(R,img,X,Y,NAME)		if(R){cvNamedWindow(NAME); cvMoveWindow(NAME, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(NAME, img);
#define DISPLAY_IMAGE_NAME(img,NAME)		cvNamedWindow(NAME); cvShowImage(NAME, img);

extern Params p;

double pi = 3.141592653589;

const char * TM_RESULT 	= "Result Window";
const char * TM_TEMPL 	= "Template Window";

const char * POST_THRES 	= "P(G|F)>T";

const char * GROUND_RESULT 	= "SuperPixel Classification Result"; // window name
const char * GBS_HIST 		= "GBS Image Histogram"; // window name
const char * GBS 		= "seg"; //"Graph Based Segmentation";
const char * LBP_HIST 		= "LBP Image Histogram"; // window name
const char * iic_HIST 		= "IIC Image Histogram"; // window name
const char * LBP_ANALYSIS 	= "LBP Analysis"; // window name
const char * IIC_ANALYSIS 	= "IIC Analysis";
const char * LBP_CUE_IMG 	= "LBP CUE"; // window name
const char * EDGE_ANG 		= "Edge Grad Ang";
const char * EDGE_MAG 		= "Edge Magnitude";
const char * HIST_HUE = "Hue Hist Model";
const char * HIST_SAT = "Sat Hist Model";
const char * HIST_VAL = "Val Hist Model";

const char * HIST_EDGE = "Edge Grad Angle";

const char * IMG_HUE = "Hue Result";
const char * IMG_SAT = "Sat Result";
const char * IMG_VAL = "Val Result";

const char * SAT_MASK       = "Saturation Mask";

const char * SOBEL_X       = "Sobel X";
const char * SOBEL_Y       = "Sobel Y";
const char * SOBEL_XY       = "Sobel XY";

const char * DEPTH       = "Depth Map";

const char * P1name[N] = {"MAG1 G","ANG1 G","HUE1 G","SAT1 G","LBP1 G","IIC1 G"};
const char * P0name[N] = {"MAG0 G","ANG0 G","HUE0 G","SAT0 G","LBP0 G","IIC0 G"};

const char * STATS       = "Statistics";

const char * POST_RATIO       = "Log Post Ratio";
const char * GSTAT       = "Gstat";

static int vmin = 10, vmax = 256, smin = 5; // min/max values for Variance/Saturation mask


static int int_sigma = 5;
static int int_k = 100;
static int min_size = 100;



IplImage *contour_image; //display final binary segmentation

IplImage *hist_img; //display histogram of GBS
CvHistogram *hist = NULL;	    // pointer to histogram object

IplImage *mask[N]; //Images for use as masks eg:superpixels

IplImage *bin[N]; //images to use as masks with colour analysis

CvHistogram *histH, *histS, *histV; //Histograms of superpixels to match against model
IplImage *HistImgH, *HistImgS, *HistImgV; //Images to display histograms

IplImage *sobel[N], *EdgeHist_img; //Images to be used with edge analysis
CvHistogram *ANG_HIST = NULL;

IplImage *inv_prob[N], *temp[N], *PG[N], *PG_prev[N], *PG1_DISP[N], *PG0_DISP[N]; //Images for use when calculating probabilities

IplImage* DEPTH_MAP;//Display depth array

IplImage* LBPhist_img, *iichist_img;
CvHistogram *LBPBOXhist = NULL;

IplImage* GhistImg;
CvHistogram *Ghist = NULL;

IplImage* GhistImg2;
CvHistogram *Ghist2 = NULL;
CvHistogram *Ghist2DISP = NULL;

CvSize HistSize;

IplImage *stats_disp, *hist_temp; //Display superpixel statistics

//CvHistogram* DISP_HIST;


static inline void POLAR2CART(CvPoint2D32f *P, CvPoint2D32f *C){
C->x = (P->x)*(sin(P->y));
C->y = (P->x)*(cos(P->y));
}

static inline void CART2DISPLAY(CvPoint2D32f *C, CvPoint *D, CvPoint *Offset, double scale){
//static const float scale_x = 0.02, scale_y = 0.02;
//D->x = -(C->x)*(scale_x) + Offset->x;
//D->y = -(C->y)*(scale_y) + Offset->y;
D->x = -(C->x)*(scale) + Offset->x;
D->y = -(C->y)*(scale) + Offset->y;


}

static inline CvScalar hue2rgb( float hue ){

    // hue to RGB conversion : coverts a given hue value to a RGB triplet for
    // display
    // parameters:
    // hue - hue value in range 0 to 180 (OpenCV implementation of HSV)
    // return value - CvScalar as RGB triple
    // taken from OpenCV 1.0 camshiftdemo.c example
    int rgb[3], p, sector;
    static const int sector_data[][3]= {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

static inline void PrintGHistogram(int hist_size, CvHistogram *Hist, IplImage* Hist_img, const char * Window, bool flag, int X, int Y){

    float max_value;
    int bin_w = cvRound((double)Hist_img->width/hist_size);
    cvNormalizeHist(Hist, (10*Hist_img->height));
    //grab the min and max values and their indeces
    //cvGetMinMaxHistValue( Hist, NULL, &max_value, NULL, NULL);
    //scale the bin values so that they will fit in the image representation
    //cvScale( Hist->bins, Hist->bins, ((double)Hist_img->height - 10.)/(max_value), 0 );
    if(flag==0){
        ( Hist_img, cvScalarAll(255), 0 );
        for(int i = 0; i < hist_size; i++ ){
            int val = cvRound( cvGetReal1D(Hist->bins,i));
            cvRectangle( Hist_img, cvPoint(i*bin_w,Hist_img->height),
                            cvPoint((i+1)*bin_w,Hist_img->height - val),
                            cvScalarAll(0), -1, 8, 0 );
        }
    }
    DISPLAY_IMAGE_XY_NAME(p.refresh,Hist_img,X,Y,Window)
    //cvShowImage(Window, Hist_img );
}

static inline void PrintHistogram(int hist_size, CvHistogram *Hist, IplImage* Hist_img, const char * Window, bool flag, int X, int Y){
    float max_value;
    int bin_w = cvRound((double)Hist_img->width/hist_size);
    cvNormalizeHist(Hist, (Hist_img->height));

    //grab the min and max values and their indeces
    //cvGetMinMaxHistValue( Hist, NULL, &max_value, NULL, NULL);
    //scale the bin values so that they will fit in the image representation
    //cvScale( Hist->bins, Hist->bins, ((double)Hist_img->height - 10.)/(max_value), 0 );

    if(flag==0){
        cvSet( Hist_img, cvScalarAll(255), 0 );
        for(int i = 0; i < hist_size; i++ ){
            int val = cvRound( cvGetReal1D(Hist->bins,i));
            cvRectangle( Hist_img, cvPoint(i*bin_w,Hist_img->height),
                            cvPoint((i+1)*bin_w,Hist_img->height - val),
                            cvScalarAll(0), -1, 8, 0 );
        }

    }

    else if(flag==1){
        cvZero( Hist_img );
        for(int i = 0; i < hist_size; i++ ){
            int val = cvRound( cvGetReal1D(Hist->bins,i));
            CvScalar color = hue2rgb(i*180.f/hist_size);
            cvRectangle( Hist_img, cvPoint(i*bin_w,Hist_img->height),
                            cvPoint((i+1)*bin_w,Hist_img->height - val),
                            color, -1, 8, 0 );
        }

    }

    if (p.disp_img){
        cvShowImage(Window, Hist_img);
        DISPLAY_IMAGE_XY_NAME(p.refresh,Hist_img,X,Y,Window)
    }
}

void init_images_img_proc(CvSize S){
    int i;
    //Draw Contours
    contour_image = cvCreateImage( S, 8, 3);

    //View_Histogram Graph Based Segmentation
    int GBShist_size = 256;			// size of histogram (number of bins)
    float range_0[]={0,256};
    float* ranges[] = { range_0 };
    hist = cvCreateHist(1, &GBShist_size, CV_HIST_ARRAY, ranges, 1);

    int LBPhist_size = 32;
    LBPBOXhist = cvCreateHist(1, &LBPhist_size, CV_HIST_ARRAY, ranges, 1);
    hist_img = cvCreateImage(cvSize(255,200), 8, 1);


    //Histogram Analysis
    HistSize = cvSize(128,100);
    HistImgH = cvCreateImage( HistSize, 8, 3 );
    HistImgS = cvCreateImage( HistSize, 8, 3 );
    HistImgV = cvCreateImage( HistSize, 8, 3 );

    EdgeHist_img = cvCreateImage(HistSize, 8, 1);

    LBPhist_img = cvCreateImage(HistSize, 8, 1);
    iichist_img = cvCreateImage(HistSize, 8, 1);
    GhistImg = cvCreateImage(cvSize(512,100), 8, 1);
    GhistImg2 = cvCreateImage(cvSize(120,100), 8, 1);

    for(i=0;i<N;++i){
        bin[i] = cvCreateImage( S, 8, 1);
        mask[i] = cvCreateImage( S, 8, 1);
        sobel[i] = cvCreateImage( S, IPL_DEPTH_8U, 1);
        inv_prob[i] = cvCreateImage( S, 32, 1);
        temp[i] = cvCreateImage( S, 32, 1);
        PG[i] = cvCreateImage( S, 32, 1);
        PG_prev[i] = cvCreateImage( S, 32, 1);
        cvZero(PG_prev[i]);
        PG1_DISP[i] = cvCreateImage( cvSize(120,100), 8, 3);
        PG0_DISP[i] = cvCreateImage( cvSize(120,100), 8, 3);
    }

    static int hdims = 32;
    static int hrange = 181;
    static int vrange = 256;
    float hranges_arr[] = {float(0),float(hrange-1)};
    float vranges_arr[] = {float(0),float(vrange-1)};
    float* hranges = hranges_arr;
    float* vranges = vranges_arr;

    static int dim_40 	= 40;
    static int range_40 	= 40;
    float range_40_arr[] = {float(0),float(range_40-1)};
    float* range_40_ptr = range_40_arr;

    histH = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
    histS = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &vranges, 1 );
    histV = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &vranges, 1 );
    static int dim_9 	= 9;
    static int dim_32 	= 32;
    static int range_32 	= 32;
    float range_2pi_arr[] = {-CV_PI,CV_PI};
    float* range_2pi_ptr = range_2pi_arr;
    float range_32_arr[] = {float(0),float(range_32-1)};
    float* range_32_ptr = range_32_arr;
    float range_log_arr[] = {-30,30};
    float* range_log_ptr = range_log_arr;
    ANG_HIST = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
    Ghist = cvCreateHist( 1, &vrange, CV_HIST_ARRAY, &range_log_ptr, 1 );
    Ghist2 = cvCreateHist( 1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1 );
    Ghist2DISP = cvCreateHist( 1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1 );

    //sobel_16S = cvCreateImage( S, IPL_DEPTH_16S, 1);

    DEPTH_MAP = cvCreateImage(cvSize(400,400), 8, 3);

    stats_disp = cvCreateImage( S, 8, 3);
    hist_temp = cvCreateImage( cvSize(120,100), 8, 3);
    //init_r(S);
}

void release_images_img_proc( void )
{
    int i;
    cvReleaseImage( &contour_image );
    cvReleaseImage( &stats_disp );

    cvReleaseImage( &hist_img );
    cvReleaseImage( &EdgeHist_img );
    cvReleaseImage( &LBPhist_img );

    cvReleaseImage( &DEPTH_MAP );

    cvReleaseHist( &hist );

    cvReleaseHist( &LBPBOXhist );

    //Histogram Analysis
    cvReleaseImage( &HistImgH );
    cvReleaseImage( &HistImgS );
    cvReleaseImage( &HistImgV );

    for(i=0;i<4;++i){
        cvReleaseImage( &bin[i]  );
        cvReleaseImage( &mask[i] );
        cvReleaseImage( &sobel[i]);
        cvReleaseImage( &inv_prob[i]);
        cvReleaseImage( &temp[i]);
    }

    cvReleaseHist( &histH );
    cvReleaseHist( &histS );
    cvReleaseHist( &histV );

    if (p.disp_img){ cvDestroyAllWindows(); }
}

CvMemStorage* storage = cvCreateMemStorage();
CvSeq* first_contour = NULL;
CvSeq* poly_result = NULL;
void Contours(IplImage* binary){

    int Nc = cvFindContours(
	binary,
	storage,
	&first_contour,
	sizeof(CvContour),
	CV_RETR_CCOMP, //Try all values to see what happens
	//CV_RETR_LIST,
	CV_CHAIN_APPROX_NONE,
	//cvPoint(-cvRound((1 - TPL_WIDTH)/2),-cvRound((1 - TPL_HEIGHT)/2))
	cvPoint(0,0)
    );

/*
    poly_result = cvApproxPoly(
	first_contour,
	sizeof(CvContour),
	storage,
	CV_POLY_APPROX_DP,
	cvContourPerimeter(first_contour)*0.005,
	0
    );
*/
    if(p.verb){ printf("Total Contours Detected: %d\n", Nc); }
}

void DrawContours(IplImage* contour, CvScalar color, CvRect S){

    cvZero(contour);
    for( ; first_contour != 0; first_contour = first_contour->h_next )
    {
        //CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
	//CvScalar color = CV_RGB( 255, 0, 0 );
        //replace CV_FILLED with 1 to see the outlines
        //cvDrawContours( dst, contour, color, color, -1, CV_FILLED, 8 );

	cvDrawContours(
	contour,
	first_contour,
	//poly_result,
	//CV_RGB(255,0,0),
	color,
	//CV_RGB(0,0,255),
	color,
	2, //try different values of max level to see what happens
	//1,
	CV_FILLED,
	8
	);


    }
    //cvMul(contour_image, const CvArr* src2, CvArr* dst, double scale=1)¶
    //cvAddWeighted( contour, 0.5, image, 1, 0, contour );
    //cvResetImageROI(contour_image);

    cvRectangle(contour, cvPoint(S.x,S.y),
    cvPoint(S.x+S.width,S.y+S.height), cvScalarAll(255), 1, 8, 0);
    /*DISPLAY_IMAGE_XY(p.refresh, contour, 6, 2);
    //cvShowImage(GROUND_RESULT,contour);
*/
}

void ReleaseContours(void){
    if (first_contour != NULL){
        cvClearSeq(first_contour);
    }
    cvClearMemStorage(storage);

}

void GraphBasedSegmentation(IplImage* seg, IplImage* gray_source){

    static int num_ccs;
    static float sigma;
    static float k;

            //printf("processing\n");
    sigma = (float)int_sigma*0.1;
    k = (float)int_k;
    SegmentImage(seg, gray_source, sigma, k, min_size, &num_ccs);

    /*
    double minval, maxval;
    cvMinMaxLoc(seg, &minval, &maxval, NULL, NULL, NULL);
    cout<<"min="<<minval<<""<<"max="<<maxval<<endl;
    */
    if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, seg, 1,1);
    if (p.refresh){
        cvCreateTrackbar( "sigma", GBS, &int_sigma , 100, NULL );
        cvCreateTrackbar( "k", GBS, &int_k, 1000, NULL );
        cvCreateTrackbar( "min_size", GBS, &min_size, 400, NULL );

    }
    //cvShowImage( GBS , seg);

}

//printf("got %d components\n", num_ccs);
//printf("done! uff...thats hard work.\n");

}

void GradMagAng32(IplImage* X,IplImage* Y,IplImage* G, IplImage* A){

    static int step       = A->widthStep/sizeof(float);
    static int channels   = A->nChannels;

    float * XPixelData = (float *)(X->imageData);
    float * YPixelData = (float *)(Y->imageData);
    float * GPixelData = (float *)(G->imageData);
    float * APixelData = (float *)(A->imageData);

    for (int y = 0; y < X->height; y++) {
        for (int x = 0; x < X->width; x++) {

            //int X_index = y*step+x*channels+0;
            float X_pixel = XPixelData[y*step+x*channels+0];

            //int Y_index = y*step+x*channels+0;
            float Y_pixel = YPixelData[y*step+x*channels+0];

            //int MAG = cvRound( sqrt( X_pixel*X_pixel + Y_pixel*Y_pixel ) );
            float MAG = abs(X_pixel) + abs(Y_pixel);
            //MAG = MAG > ONE ? ONE : MAG;
            GPixelData[y*step+x*channels+0] = ONE - MAG;
            GPixelData[y*step+x*channels+0] = MAG;
            //calculate edge orientations

            float ANG = atan2( Y_pixel , X_pixel);

            //if(ANG < 0) printf("This is smaller than zero: %0.2f\n", ANG);
            APixelData[y*step+x*channels+0] = ANG;

        }
    }

}

void combine_channels(IplImage* Cr, IplImage* Cb, IplImage* a, IplImage* iic){
    unsigned char * CrPixelData = (unsigned char *)(Cr->imageData);
    unsigned char * CbPixelData = (unsigned char *)(Cb->imageData);
    unsigned char * aPixelData = (unsigned char *)(a->imageData);
    unsigned char * iicPixelData = (unsigned char *)(iic->imageData);

    cvZero(iic);

    for (int y = 0; y < iic->height; y++) {
        for (int x = 0; x < iic->width; x++) {

            //int Cr_index = (y*Cr.width+x);
            //int Cb_index = (y*Cb.width+x);
            //int a_index = (y*a.width+x);
            int index = (y*iic->width+x);
            iicPixelData[index] = cvRound((CrPixelData[index] + CbPixelData[index] + 2*(aPixelData[index]))/4);
        }
    }

}

void getEdgeMagOri(IplImage* gray, IplImage* mag, IplImage* ang32){

    cvSobel(gray, temp[0], 1, 0, CV_SCHARR); //Find edges in x direction
    cvConvertScaleAbs(temp[0], sobel[1], 0.2, 0); //convert 16bit image to 8-bit

    cvSobel(gray, temp[1], 0, 1, CV_SCHARR);
    cvConvertScaleAbs(temp[1], sobel[2], 0.2, 0); //convert 16bit image to 8-bit

    //GradMagAng( sobel[1], sobel[2], mag, ang );

    GradMagAng32( temp[0], temp[1], temp[2], ang32 );


    cvNormalize( temp[2],temp[2], 0, 255, CV_MINMAX);
    cvNormalize(ang32,temp[3], 0, 255, CV_MINMAX);
    cvAbsDiffS(temp[2], temp[2], cvScalar(255));
    cvConvertScaleAbs(temp[2], mag, 1, 0);
    cvConvertScaleAbs(temp[3], sobel[0], 1, 0);

    if (p.disp_img){
    //DISPLAY_IMAGE_XY(p.refresh, sobel[0], 4,0);
    //DISPLAY_IMAGE_XY(p.refresh, mag, 3,0);
    }

}

double GetBit(CvSize S, unsigned char * PixelData, int p_index, int pc_index, int py, int px){
    if ( px<0 || py<0 || px>=S.width || py>=S.height) //check that image point indexes are within range
            return 0;
    else
    {
        int p_val = PixelData[p_index]; // pixel value
        int p_c = PixelData[pc_index];  // centre pixel value
        if (p_val>=p_c)
            return 1;
        else
            return 0;
    }

}

void convertGray2LBP(IplImage* gray, IplImage* LBP){

    CvSize S = cvSize(gray->width, gray->height);
    unsigned char * GrayPixelData = (unsigned char *)(gray->imageData);
    //const int * imgData = (const int *)(gray->imageData);
    unsigned char * LbpPixelData = (unsigned char *)(LBP->imageData);

    cvZero(LBP);

    for (int y = 0; y < gray->height; y++) {
        for (int x = 0; x < gray->width; x++) {

            int p0x,p1x,p2x,p3x,p4x,p5x,p6x,p7x;
            int p0y,p1y,p2y,p3y,p4y,p5y,p6y,p7y;

            p0x=x-1;
            p0y=y-1;
            p1x=x;
            p1y=y-1;
            p2x=x+1;
            p2y=y-1;
            p3x=x+1;
            p3y=y;
            p4x=x+1;
            p4y=y+1;
            p5x=x;
            p5y=y+1;
            p6x=x-1;
            p6y=y+1;
            p7x=x-1;
            p7y=y;

            int pc_index = (y*gray->width+x); //centre pixel index

            int p0_index = (p0y*gray->width+p0x); //p0 index..
            int p1_index = (p1y*gray->width+p1x);
            int p2_index = (p2y*gray->width+p2x);
            int p3_index = (p3y*gray->width+p3x);

            int p4_index = (p4y*gray->width+p4x);
            int p5_index = (p5y*gray->width+p5x);
            int p6_index = (p6y*gray->width+p6x);
            int p7_index = (p7y*gray->width+p7x); //..p7 index

            double b0 = 128*GetBit(S, GrayPixelData, pc_index, p0_index, p0y, p0x);
            double b1 = 64*GetBit(S, GrayPixelData, pc_index, p1_index, p1y, p1x);
            double b2 = 32*GetBit(S, GrayPixelData, pc_index, p2_index, p2y, p2x);
            double b3 = 16*GetBit(S, GrayPixelData, pc_index, p3_index, p3y, p3x);

            double b4 = 8*GetBit(S, GrayPixelData, pc_index, p4_index, p4y, p4x);
            double b5 = 4*GetBit(S, GrayPixelData, pc_index, p5_index, p5y, p5x);
            double b6 = 2*GetBit(S, GrayPixelData, pc_index, p6_index, p6y, p6x);
            double b7 = 1*GetBit(S, GrayPixelData, pc_index, p7_index, p7y, p7x);

            //int decimal = cvRound(b0+b1+b2+b3 + b4+b5+b6+b7);

            LbpPixelData[pc_index] = cvRound(b0+b1+b2+b3 + b4+b5+b6+b7); //pixel intensity is equal to the decimal number equivalent to the binary 8-bit pattern.

        }
    }

}

//static functions are functions that are only visible to other functions in the same file.
static inline double GetPrior(int h, CvRect* R){
    //const static int hmax = h-1;
    const static double lambda = 3./(double)h;
    double height = (double)(h - (R->y + (R->height)/2)) ;

    return exp(-lambda*height);
}

static inline double EXP_DIST_1(double z, double l, double g){
    return (1/z)*exp(-l*g);
}

static inline double EXP_DIST_0(double z, double l, double g){
    return (1/z)*exp(l*g);
}

void draw_dist(int range, double gmax, double Z, double L, IplImage* Img, bool T){
    int bin_w = cvRound((double)Img->width/(range));

    double value;
    for(int g=0; g<cvRound(gmax); g++){


        if (T==1) {
        value = 50*(EXP_DIST_1(Z, L, g));
            }
        if (T==0){
        value = 50*(EXP_DIST_0(Z, L, g));
        }

        value = value > 0 ? value : 0.001;
        value = value > Img->height ? Img->height : value;
        //double value = 100*(value1+value0)/2;

                cvRectangle( Img, cvPoint(cvRound(g)*bin_w,Img->height),
                        cvPoint((cvRound(g)+1)*bin_w,cvRound(Img->height - value)),
                        CV_RGB(200,0,0), -1, 8, 0 );

    }

}

static inline double Gstat(CvHistogram* H1, CvHistogram* H2, int size){

    double G = 0.;
    const static double SMALL = 0.000001;
    cvNormalizeHist(H1, 1);
    cvNormalizeHist(H2, 1);

    for(int i=0; i<size; i++){
        double M = cvGetReal1D(H1->bins,i); //sample
        double S = cvGetReal1D(H2->bins,i); //model

        if(M == 0) M = SMALL;
        if(S == 0) S = SMALL;
        //printf("M=%0.5f, S=0.5f\n", M, S);
        G += 2*(S*(log (S)) - S*(log (M)));
        //G += - S*(log (M));
        //printf("G=%0.5f\n",G);
        //cvWaitKey(0);
    }

    //printf("G=%0.4f\n", G);

    //return 1/exp(0.2*G);
    return G;
}

void SuperPixelStats(IplImage *gbs, IplImage *gray, Statistics *sts){

    static int hist_size = 256;			// size of histogram (number of bins)
    //float max_value = 0.f;			// max value in histogram

    //int bin_w = 0;				// initial width to draw bars
    int k = 0;

    cvCalcHist( &gbs, hist, 0, NULL );
    //cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
    //cvScale( hist->bins, hist->bins, ((double)hist_img->height)/max_value, 0 );
    //cvSet( hist_img, cvScalarAll(255), 0 );
    //bin_w = cvRound((double)hist_img->width/hist_size);

    cvSet( stats_disp, cvScalarAll(255), 0 );

    for(int i = 0; i < hist_size; i++ )
    {
        //draw histogram, where bin size indicates superpixel size
        //cvRectangle( hist_img, cvPoint(i*bin_w, hist_img->height),
        //cvPoint((i+1)*bin_w, hist_img->height - cvRound(cvGetReal1D(hist->bins,i))),
        //cvScalarAll(0), -1, 8, 0 );

        if (cvRound(cvGetReal1D(hist->bins,i)) > 0){

            sts->id[k] = k;
            sts->gray_id[k] = i;
            cvCmpS(gbs, i, mask[0], CV_CMP_EQ); //mask out image segment
            sts->size[k] = cvCountNonZero(mask[0]); //count number of pixels in current segment
            cvAvgSdv(gray, &sts->mean[k], &sts->stdDev[k], mask[0]);
            sts->box[k] = cvBoundingRect(mask[0], 0);

            sts->P_Gt[k] = GetPrior(sts->img_h, &sts->box[k]);
            sts->P_Gf[k] =  1. - sts->P_Gt[k];

            //printf("id=%d, size=%d, mean=%0.2f, stdDev=%0.2f\n", sts->id[k], sts->size[k], sts->mean[k].val[0], sts->stdDev[k].val[0]);




            //DISPLAY

            cvSet(stats_disp, cvScalarAll(sts->mean[k].val[0]), mask[0]);
            cvSet(sts->prior_img, cvScalar(sts->P_Gt[k]), mask[0]);

            cvRectangle(stats_disp,	cvPoint(sts->box[k].x,sts->box[k].y),
                        cvPoint(sts->box[k].x+sts->box[k].width,sts->box[k].y+sts->box[k].height), CV_RGB(255,0,0), 1, 8, 0);

            cvLine(stats_disp, cvPoint(sts->box[k].x + (sts->box[k].width)/2 , sts->box[k].y + (sts->box[k].height)/2 ),
                    cvPoint(sts->img_w/2,sts->img_h), CV_RGB(255,0,0), 1, 8, 0);

            //cvShowImage(GBS_HIST, hist_img );


            k++;

        }

    }
    sts->nos = k;
    if (p.disp_img){
        DISPLAY_IMAGE_XY(p.refresh, stats_disp, 7,4); //cvShowImage(STATS, stats_disp);
    }
    if (p.verb){ printf("nos=%d\n", sts->nos); }
}

void UpdatePrior(IplImage *gbs, Statistics *sts, Features* F){

    CvScalar mean1 = cvScalar(0,0,0,0);
    CvScalar mean0 = cvScalar(0,0,0,0);
    static double alpha = 0.6;
    static double beta = 1.- alpha;

    cvAvgSdv(F->post1, &mean1, 0, NULL);
    if(mean1.val[0] > 0){

        for(int i = 0; i < sts->nos; i++ )
            {
            cvCmpS(gbs, sts->gray_id[i], mask[0], CV_CMP_EQ); //mask out image segmen

            cvSmooth(F->post1, F->post1, CV_GAUSSIAN, 5, 5);
            cvSmooth(F->post0, F->post0, CV_GAUSSIAN, 5, 5);

            cvAvgSdv(F->post1, &mean1, 0, mask[0]);
            cvAvgSdv(F->post0, &mean0, 0, mask[0]);
            // /*
            if(mean1.val[0] > 0){
            double prior1 = alpha*sts->P_Gt[i] + beta*mean1.val[0];
            //sts->P_Gf[i] =  1. - sts->P_Gt[i];
            double prior0 = alpha*sts->P_Gf[i] + beta*mean0.val[0];

            sts->P_Gt[i] = prior1/(prior1+prior0);
            sts->P_Gf[i] = prior0/(prior1+prior0);
            }
            // */
            /*
            double prior = sts->P_Gt[i];
            double posterior = mean.val[0];

            double prior_update = prior*posterior;
            double prior_update_not = (1-prior)*(1-posterior);

            sts->P_Gt[i] = prior_update;
            //sts->P_Gf[i] = 1-sts->P_Gt[i];
            printf("prior=%0.5f, post=%0.5f, update=%0.5f\n", prior, posterior, prior_update);
            //sts->P_Gf[i] =  (sts->P_Gf[i]) * (1. - mean.val[0]);

            //sts->P_Gt[i] = sts->P_Gt[i] / sts->P_Gt[i] + sts->P_Gf[i];
            */

            cvSet(sts->prior_img, cvScalar(sts->P_Gt[i]), mask[0]);

        }
    }
    if (p.disp_img){
    /*cvNamedWindow("Updated Prior");
    cvShowImage("Updated Prior", sts->prior_img);*/  }

}

void GetModel(IplImage* gray, Features* F, Model* M, bool dynamic){
    int loop;
    //cvAvgSdv(gray, &M->mean[0], &M->stdDev[0], mask[0]);

    //cvCvtPixToPlane( F->hsv, F->hue, F->sat, F->val, 0 );

    //static int vmin = 10, vmax = 256, smin = 10; // min/max values for Variance/Saturation mask
    cvInRangeS( F->hsv, cvScalar(0,   smin, min(vmin,vmax), 0),
                        cvScalar(180, 256,  max(vmin,vmax), 0), bin[0] );

    cvAnd(bin[0], M->mask, bin[1]);
    if(dynamic) loop = 200;
    if(!dynamic) loop = 0;
    static bool acc =1;
    //static int loop = 1;
    static int j=0;
    if(j<loop){
        acc = 1;
        j++;
    }
    else{
        acc=0;
        j=0;
    }

    cvCalcHist( &F->mag, 	M->H_M[0], acc, M->mask );
    cvCalcHist( &F->ang32, 	M->H_M[1], acc, M->mask );
    cvCalcHist( &F->hue, 	M->H_M[2], acc, bin[1] );
    cvCalcHist( &F->sat, 	M->H_M[3], acc, M->mask );
    cvCalcHist( &F->lbp, 	M->H_M[4], acc, M->mask );
    cvCalcHist( &F->iic, 	M->H_M[5], acc, M->mask );

    /*cvAnd(F->mag, M->mask, bin[2]);
    cvCalcHist( &F->ang32, M->H_M[1], 0, bin[2] );
    cvNamedWindow("test1");
    cvShowImage("test1", M->mask);
    cvNamedWindow("test2");
    cvShowImage("test2", bin[2]);
    */


    static int dim_9 = 9;
    static int dim_32 = 32;
    //static int dim_64 = 64;

    for(int i=0;i<N;i++){
        cvCopyHist(M->H_M[i], &M->H_M_DISP[i]);
    }
    //if(!dynamic){
    if (p.disp_img){
        static int row = 1;
        PrintHistogram(dim_32, M->H_M_DISP[0], HistImgV, HIST_VAL, 1, 4, row);
        PrintHistogram(dim_9, M->H_M_DISP[1], EdgeHist_img, HIST_EDGE, 0, 5, row);
        PrintHistogram(dim_32, M->H_M_DISP[2], HistImgH, HIST_HUE, 1, 2, row);
        PrintHistogram(dim_32, M->H_M_DISP[3], HistImgS, HIST_SAT, 1, 3, row);
        PrintHistogram(dim_32, M->H_M_DISP[4], LBPhist_img, LBP_HIST, 0, 6, row);
        PrintHistogram(dim_32, M->H_M_DISP[5], iichist_img, iic_HIST, 0, 7, row);
    }
    //}

//cvWaitKey(0);
}

void UpdatePrevModel(Model* Current, Model* Prev){

    if(Current->mean[0].val[0] > 0){
        Prev->mean[0] = Current->mean[0];
        Prev->stdDev[0] = Current->stdDev[0];


        //cvCopyHist(const CvHistogram* src, CvHistogram** dst)
        for(int i=0;i<N;i++){
            cvCopyHist(Current->H_M[i], &Prev->H_M[i]);
        }
        /*
        cvClearHist(Current->H_M[2]);
        cvClearHist(Current->H_M[3]);
        cvClearHist(Current->H_M[0]);
        cvClearHist(Current->H_M[1]);
        cvClearHist(Current->H_M[4]);
        */
    }
}

static inline void UpdateHistogram(CvHistogram* H1, CvHistogram*H2, int dim){

    cvNormalizeHist(H1, 1);
    cvNormalizeHist(H2, 1);
    static double max = 0.;

    for(int i = 0; i < dim; i++ )
    {

        double val1 =  cvGetReal1D(H1->bins,i);
        double val2 =  cvGetReal1D(H2->bins,i);

        //double max = val1 > val2 ? val1 : val2;
        //if (val2 > val1){

        double update = 0.2*val1 + 0.8*val2;
        //double update = val2;
        //update  = update > 0.5 ? 0.5 : update;
        cvSetReal1D(H1->bins, i, update);
        //}

    }
}

void UpdateModel(Model* Current, Model* Prev){
    static int dim_9 = 9;
    static int dim_32 = 32;
    static int dim_64 = 64;

    for(int i=0;i<N;i++){
        UpdateHistogram(Current->H_M[i], Prev->H_M[i], Current->dim[i]);
        cvCopyHist(Current->H_M[i], &Current->H_M_DISP[i]);
    }
    if (p.disp_img){
    static int row = 1;
        PrintHistogram(dim_32, Current->H_M_DISP[2], HistImgH, HIST_HUE, 1, 2, row);
        PrintHistogram(dim_32, Current->H_M_DISP[3], HistImgS, HIST_SAT, 1, 3, row);
        PrintHistogram(dim_32, Current->H_M_DISP[0], HistImgV, HIST_VAL, 1, 4, row);
        PrintHistogram(dim_9, Current->H_M_DISP[1], EdgeHist_img, HIST_EDGE, 0, 5, row);
        PrintHistogram(dim_32, Current->H_M_DISP[4], LBPhist_img, LBP_HIST, 0, 6, row);
        PrintHistogram(dim_32, Current->H_M_DISP[5], iichist_img, iic_HIST, 0, 7, row);

    }
}

void FeatureAnalysis(Features *F, Model* M, Statistics *S, IplImage *gbs, bool dynamic){


    //static int prob = 80;
    //float p = (float)prob*0.01;

    cvInRangeS( F->hsv, cvScalar(0,   smin, min(vmin,vmax), 0),
                cvScalar(180, 256,  max(vmin,vmax), 0), bin[0] );
    if (p.disp_img){  /*DISPLAY_IMAGE_XY(p.refresh, bin[0], 0, 4);*/ //cvShowImage( SAT_MASK, bin[0] );
    }


    //const static int Ehist_size = 9;
    //const static int Chist_size = 32;
    //const static int Lhist_size = 32;

    for(int i=0;i<S->no_features;i++){
        S->L1[i] = S->L1[i] > 0 ? S->L1[i] : 0.01;
        S->L0[i] = S->L0[i] > 0 ? S->L0[i] : 0.01;
        S->gmax[i] = S->gmax[i] > 0 ? S->gmax[i] : 1.;
    }

    for(int i = 0; i < S->nos; i++ )
    {
        cvCmpS(gbs, S->gray_id[i], mask[0], CV_CMP_EQ); //mask out image segmen

        //cvAnd(F->mag, mask[0], bin[2]);
        //cvCalcHist( &F->ang32, ANG_HIST, 0, bin[2] );
        //cvNamedWindow("test3");
        //cvShowImage("test3", bin[2]);
        //cvWaitKey(100);

        cvAnd(bin[0], mask[0], bin[1]);

        cvCalcHist( &F->mag,   S->H_SF[0], 0, mask[0] );
        cvCalcHist( &F->ang32, S->H_SF[1], 0, mask[0] ); //NULL=mask
        cvCalcHist( &F->hue,   S->H_SF[2], 0, bin[1] ); //NULL=mask
        cvCalcHist( &F->sat,   S->H_SF[3], 0, mask[0] );
        cvCalcHist( &F->lbp,   S->H_SF[4], 0, mask[0] ); //NULL=mask
        cvCalcHist( &F->iic,   S->H_SF[5], 0, mask[0] );

        for(int j=0;j<S->no_features;j++){
            S->G_score[j] = Gstat(M->H_M[j], S->H_SF[j], M->dim[j]);
            S->P_FgGt[S->no_features*i+j] = EXP_DIST_1(S->Z1[j], S->L1[j], S->G_score[j]); //0.9*exp(-lambda*G_scoreV);
            S->P_FgGf[S->no_features*i+j] = EXP_DIST_0(S->Z0[j], S->L0[j], S->G_score[j]); //1. - S->P_VgGt[i];
            cvSet(PG[j], cvScalar( S->G_score[j] ), mask[0]);
            cvSet(F->P_X1[j], cvScalar(S->P_FgGt[S->no_features*i+j]/(S->P_FgGt[S->no_features*i+j]+S->P_FgGf[S->no_features*i+j])), mask[0]);
            cvSet(F->P_X0[j], cvScalar(S->P_FgGf[S->no_features*i+j]/(S->P_FgGt[S->no_features*i+j]+S->P_FgGf[S->no_features*i+j])), mask[0]);
        }


    }


    if (p.disp_img){
        static int row = 2;
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[0], 4, row); //IMG_VAL
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[1], 5, row); //SOBEL_XY
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[2], 2, row); //IMG_HUE
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[3], 3, row); //IMG_SAT
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[4], 6, row); //LBP_ANALYSIS
        DISPLAY_IMAGE_XY(p.refresh, F->P_X1[5], 7, row); //IIC_ANALYSIS

        //cvShowImage( IMG_VAL,  F->P_X1[0] );
        //cvShowImage( SOBEL_XY, F->P_X1[1]);
        //cvShowImage( IMG_HUE,  F->P_X1[2] );
        //cvShowImage( IMG_SAT,  F->P_X1[3] );
        //cvShowImage( LBP_ANALYSIS, F->P_X1[4]);
        //cvShowImage( IIC_ANALYSIS, F->P_X1[5]);
    }

    if(dynamic){
        static bool flag = 0;
        for(int i=0;i<S->no_features;i++){
        if(flag){
        cvAddWeighted(PG[0], 0.5, PG_prev[0], 0.5, 0, PG[0]);
        }
        cvCopy(PG[0], PG_prev[0], NULL);
        }
        flag =1;
    }


    //cvCopy(PG[0], inv_prob[3], NULL);
    //cvNormalize(inv_prob[3],inv_prob[3],0,1,CV_MINMAX);
    //cvNamedWindow("PG3");
    //cvShowImage("PG3", inv_prob[3]);

}

void ProbAnalysis2(Features *F, Statistics* S, IplImage* gbs){

    double t = (double)p.log_post_thres_zero_position*0.1 - 30;

    for(int i = 0; i < S->nos; i++ )
    {
        cvCmpS(gbs, S->gray_id[i], mask[0], CV_CMP_EQ); //mask out image segment

        for(int j=0;j<S->no_features;j++){
        S->P_FgGt[S->no_features*i+j] = S->P_FgGt[S->no_features*i+j] > 0.00001 ? S->P_FgGt[S->no_features*i+j] : LP;

        }
        /*
        S->P_EgGt[i] = S->P_EgGt[i] > 0 ? S->P_EgGt[i] : LP;
        S->P_HgGt[i] = S->P_HgGt[i] > 0 ? S->P_HgGt[i] : LP;
        S->P_SgGt[i] = S->P_SgGt[i] > 0 ? S->P_SgGt[i] : LP;
        S->P_VgGt[i] = S->P_VgGt[i] > 0 ? S->P_VgGt[i] : LP;
        S->P_LgGt[i] = S->P_LgGt[i] > 0 ? S->P_LgGt[i] : LP;

        S->P_EgGf[i] = S->P_EgGf[i] > 0 ? S->P_EgGf[i] : LP;
        S->P_HgGf[i] = S->P_HgGf[i] > 0 ? S->P_HgGf[i] : LP;
        S->P_SgGf[i] = S->P_SgGf[i] > 0 ? S->P_SgGf[i] : LP;
        S->P_VgGf[i] = S->P_VgGf[i] > 0 ? S->P_VgGf[i] : LP;
        S->P_LgGf[i] = S->P_LgGf[i] > 0 ? S->P_LgGf[i] : LP;
        */


        double post1 =
        (S->P_Gt[i])*(S->P_FgGt[S->no_features*i+0])*(S->P_FgGt[S->no_features*i+1])*(S->P_FgGt[S->no_features*i+2])*(S->P_FgGt[S->
        no_features*i+3])*(S->P_FgGt[S->no_features*i+4])*(S->P_FgGt[S->no_features*i+5]);
        double post0 =
        (S->P_Gf[i])*(S->P_FgGf[S->no_features*i+0])*(S->P_FgGf[S->no_features*i+1])*(S->P_FgGf[S->no_features*i+2])*(S->P_FgGf[S->
        no_features*i+3])*(S->P_FgGf[S->no_features*i+4])*(S->P_FgGf[S->no_features*i+5]);


        S->P_GtgF[i] = post1 / (post1 + post0);
        S->P_GfgF[i] = post0 / (post1 + post0);

        cvSet(F->post1, cvScalar(S->P_GtgF[i]), mask[0]);
        cvSet(F->post0, cvScalar(S->P_GfgF[i]), mask[0]);


        cvSet(F->post_ratio, cvScalar(log(post1/post0)), mask[0]);
    }


    cvCalcHist( &F->post_ratio, Ghist, 0, 0);
    if (p.disp_img){ PrintHistogram(256, Ghist, GhistImg, POST_RATIO, 0, 0, 3);
        if(p.refresh){
            cvCreateTrackbar("T", POST_RATIO, &p.log_post_thres_zero_position, 600, NULL );
        }
    }
    cvThreshold(F->post_ratio, F->bin_class_result, t, ONE, CV_THRESH_BINARY);
    cvNormalize(F->post_ratio,inv_prob[0], 0, 1, CV_MINMAX);

    if (p.disp_img){
        /*DISPLAY_IMAGE_XY(p.refresh, F->bin_class_result, 7, 4);*/ //cvShowImage( POST_THRES, F->bin_class_result );
        /*cvNamedWindow("post_ratio");
        cvShowImage("post_ratio", inv_prob[0]);*/
    }
}

void GetMeanHist(int dim, CvHistogram* H, double mean, double max){

    cvNormalizeHist(H,1);

    for(int i = 0; i < dim; i++ )
    {
        float* val = cvGetHistValue_1D(H,i);
        //increment the mean value
        mean += i*val[0];
        if(val[0]>0.){
            max = i;
        }
    }

}

void UpdateParams(IplImage* T, Statistics *S, Features *F, bool dynamic){

    //initializations
    static int loop;
    static int GmaxInt;
    static int L1Int;
    static int L0Int;
    static int dim_40 = 40;
    static bool weighted_mean=0;
    static double mean_trial;
    static bool truncated = 1;

    if(dynamic) loop = 200;
    else loop = 0;

    static bool acc = 1;
    static int j=0;
    static double gmax=5;
    static double mean1=0.;
    static double mean0=0.;
    static CvScalar mean;
    const static double min_mean1 = 0.025;
    //const static double min_mean1 = 0.05;
    const static double max_mean1 = 10;
    //const static double min_mean1 = 0.01;
    //const static double max_mean1 = 1000;
    //const static double min_mean0 = 0.01;
    //const static double max_mean1 = 100;
    const static double min_mean0 = 0.25;




    cvCopy(T, bin[4],0);
    cvNot(T,bin[3]);
    cvNormalize(F->post_ratio,inv_prob[0], 0, 1, CV_MINMAX);
    cvAbsDiffS(inv_prob[0], inv_prob[3], cvScalar(1));

    if(j<loop){
    acc = 1;
    j++;
    }else{
    acc=0;
    j=0;
    }

    //cvNamedWindow("PX3");
    //cvShowImage("PX3",F->P_X1[3]);

    for(int i=0;i<S->no_features;i++){

        //cvCopyHist(PG[i], &PG_DISP[i]);
        //cvNormalizeHist
        //if(i==5){
        cvCopy(PG[i],temp[5],0);
        cvNormalize(temp[5],temp[5],0,39,CV_MINMAX);
        cvCalcHist( &temp[5], Ghist2, 1, NULL);
        cvCopyHist(Ghist2, &Ghist2DISP);
        if (p.disp_img){ PrintGHistogram(40, Ghist2DISP, GhistImg2, GSTAT, 0, 7, 3); }
        //}
        //cvWaitKey();

        cvMinMaxLoc( PG[i], NULL, &gmax, NULL, NULL, NULL);
        //printf("gmax = %0.2f",gmax);
        //smooth the change of gmax and limit minimum gmax
        if(dynamic){
            S->gmax[i] = (0.5*gmax + 0.5*S->gmax[i]) > 0 ? (0.5*gmax + 0.5*S->gmax[i]) : 0.5;
        }else{
            S->gmax[i] = gmax > 0 ? gmax : 0.5;
        }

        if(weighted_mean){
            //cvMul(F->post1,PG[i],inv_prob[1],1);
            cvMul(inv_prob[0],PG[i],inv_prob[1],1);
            cvAvgSdv(inv_prob[1], &mean, NULL, NULL);
        }else {
            cvAvgSdv(PG[i], &mean, NULL, bin[4]);
        }
        mean.val[0] = mean.val[0] > min_mean1 ? mean.val[0] : min_mean1;
        mean.val[0] = mean.val[0] > max_mean1 ? max_mean1 : mean.val[0];
        if (mean1==0.) mean1 = mean.val[0];
        if(dynamic){
            mean1 = 0.5*mean1 + 0.5*mean.val[0];
        }else{
            mean1 = mean.val[0];
        }
        if(truncated){
            if(mean1<(gmax/2)) mean1 = mean1 - gmax/(exp(gmax/mean1)-1);
            if(mean1>=(gmax/2)) mean1 = (gmax/2);
        }

        S->L1[i] = (1./(mean1)) > 0 ? (1./(mean1)) : 1;

        //printf("%0.4f\n", mean.val[0]);
        cvSet( hist_temp, cvScalarAll(255), 0 );
        cvCalcHist( &PG[i], S->H_G1[i], acc, bin[4]);

        cvCopyHist(S->H_G1[i], &S->H_G1_DISP[i]);
        if (p.disp_img){
            PrintHistogram(dim_40, S->H_G1_DISP[i], PG1_DISP[i], P1name[i], 0, i+2, 3);
        }

        /*
        GetMeanHist(dim_40, S->H_G0_DISP[i], mean_trial, gmax);
        S->gmax[i] = cvRound(0.5*gmax + 0.5*S->gmax[i]) > 0 ? cvRound(0.5*gmax + 0.5*S->gmax[i]) : 0.5;
        mean_trial = mean_trial > min_mean1 ? mean_trial : min_mean1;
        mean_trial = mean_trial > max_mean1 ? max_mean1 : mean_trial;
        S->L1[i] = cvRound( ((1./(mean_trial))*1000.) ) > 0 ? cvRound( ((1./(mean_trial))*1000.) ) : 1;
        //draw_dist(40, S->gmax[i], S->Z1[i], S->Z0[i], I2D(S->L1[i]), I2D(S->L0[i]), PG1_DISP[i], 1);
        */
        draw_dist(40, S->gmax[i], S->Z1[i], S->L1[i], hist_temp, 1);
        cvAddWeighted( hist_temp, 0.5, PG1_DISP[i], 0.5, 0, PG1_DISP[i] );
        if (p.disp_img){
            //cvShowImage( P1name[i], PG1_DISP[i]);
            DISPLAY_IMAGE_NAME(PG1_DISP[i], P1name[i]);
        }


    if (p.refresh){
    for(int i=0;i<S->no_features;i++){
        GmaxInt = cvRound(S->gmax[i]);
        L1Int = cvRound(S->L1[i]*1000);
        L0Int = cvRound(S->L0[i]*1000);

        //if(p.debug){
        if (p.refresh){
            cvCreateTrackbar( "L1", P1name[i], &L1Int, 40000, NULL );
            cvCreateTrackbar( "L0", P1name[i], &L0Int, 4000, NULL );
            cvCreateTrackbar( "gmax", P1name[i], &GmaxInt, 60, NULL );
        }
        //}
    }
    }


        cvSet( hist_temp, cvScalarAll(255), 0 );

        if(weighted_mean){
            //cvMul(F->post0,PG[i],inv_prob[2],1);
            cvMul(inv_prob[3],PG[i],inv_prob[2],1);
            cvAvgSdv(inv_prob[2], &mean, NULL, NULL);
        }else{
            cvAvgSdv(PG[i], &mean, NULL, bin[3]);
        }

        mean.val[0] = gmax-mean.val[0];
        mean.val[0] = mean.val[0] > min_mean0 ? mean.val[0] : min_mean0;

        if (mean0==0.) mean0 = mean.val[0];
        if(dynamic){
            mean0 = 0.5*mean0 + 0.5*mean.val[0];
        }else{
            mean0 = mean.val[0];
        }
        if(truncated){
            if(mean0<(gmax/2)) mean0 = mean0 - gmax/(exp(gmax/mean0)-1);
            if(mean0>=(gmax/2)) mean0 = (gmax/2);
        }
        //S->L0[i] = cvRound( ((1./(mean0))*1000.) ) > 0 ? cvRound( ((1./(mean0))*1000.) ) : 1;
        S->L0[i] = (1./(mean0)) > 0 ? (1./(mean0)) : 1 ;
        //printf("%d\n", S->L0[i]);
        cvCalcHist( &PG[i], S->H_G0[i], acc, bin[3]);

        cvCopyHist(S->H_G0[i], &S->H_G0_DISP[i]);
        if (p.disp_img){ //PrintHistogram(dim_40, S->H_G0_DISP[i], PG0_DISP[i], P0name[i], 0, 5, 5);
        }
        /*
        GetMeanHist(dim_40, S->H_G0_DISP[i],mean_trial, gmax);
        S->gmax[i] = cvRound(0.5*gmax + 0.5*S->gmax[i]) > 0 ? cvRound(0.5*gmax + 0.5*S->gmax[i]) : 0.5;
        mean_trial = mean_trial > min_mean0 ? mean_trial : min_mean0;
        S->L0[i] = cvRound( ((1./(mean_trial))*1000.) ) > 0 ? cvRound( ((1./(mean_trial))*1000.) ) : 1;
        */
        //printf("mean=%0.4f\n", mean_trial);

        draw_dist(40, S->gmax[i], S->Z0[i], S->L0[i], hist_temp, 0);

        cvAddWeighted( hist_temp, 0.5, PG0_DISP[i], 0.5, 0, PG0_DISP[i] );

        if (p.disp_img){ /*cvShowImage( P0name[i], PG0_DISP[i]);*/ }

        S->Z1[i] = (1/S->L1[i])*(1-exp(-S->L1[i]*S->gmax[i]));
        S->Z0[i] = (1/S->L0[i])*(exp(S->L0[i]*S->gmax[i])-1);
            std::cout << "feature #" << i << std::endl;
			std::cout << mean1 << std::endl;
			std::cout << mean0 << std::endl;
			std::cout << gmax/(exp(gmax/mean0)-1) << std::endl;
			std::cout << gmax << std::endl;	
    }





}

bool CheckConvergence(Statistics* S, int em){

    static double * prev1;
    static double * prev0;
    //static bool * converged;
    const static double tolerance = 0.005;
    int count = 0;

    if(em == 0){
        prev1 = new double[S->no_features];
        prev0 = new double[S->no_features];
        for (int i=0; i<S->no_features; i++){
        prev1[i] = 0.;
        prev0[i] = 0.;
        //converged[i] = 0;
        }

    }
    else{

        for (int i=0; i<S->no_features; i++){

            //if ( S->L1[i] == prev1[i] && S->L0[i] == prev0[i]) count ++;
            if( ((S->L1[i] <= prev1[i] + tolerance) && (S->L1[i] >= prev1[i] - tolerance)) &&
            ((S->L0[i] <= prev0[i] + tolerance) && (S->L0[i] >= prev0[i] - tolerance))
            ) count++;

            prev1[i] = S->L1[i];
            prev0[i] = S->L0[i];
        }

    }
    if(count == S->no_features) return 1;
        else return 0;
}

void FindObstacleBoundary(IplImage* Out){
    //Out = cvCloneImage(In);

    //unsigned char * InPixelData = (unsigned char *)(In->imageData);
    unsigned char * OutPixelData = (unsigned char *)(Out->imageData);

    static int y = (Out->height) -1;

    bool flag0 = 0, flag1 = 0;


    for (int x = 0; x < cvRound((Out->width)/2); x++) {

        int x0 = cvRound((Out->width)/2) - (x+1);
        int x1 = cvRound((Out->width)/2) + (x+1);

        //		int In_index_0 	= (y*In->width+x0)*In->nChannels;
        //		int In_index_1 	= (y*In->width+x1)*In->nChannels;
        int Out_index_0 = (y*Out->width+x0)*Out->nChannels;
        int Out_index_1 = (y*Out->width+x1)*Out->nChannels;

        if(flag0 == 1 || OutPixelData[Out_index_0+0] != ONE){
            flag0 = 1;
            OutPixelData[Out_index_0+0] = ZERO;
        }
        if(flag1 == 1 || OutPixelData[Out_index_1+0] != ONE){
            flag1 = 1;
            OutPixelData[Out_index_1+0] = ZERO;
        }

    }


    for (int x = 0; x < Out->width; x++) {
        int flag = 0;
        for (int y = 0; y < Out->height; y++) {

            int y1 = (Out->height) - (y+1);

            //		int In_index = (y1*In->width+x)*In->nChannels;
            int Out_index = (y1*Out->width+x)*Out->nChannels;

            if(flag == 1 || OutPixelData[Out_index+0] != 255){
                flag = 1;
                OutPixelData[Out_index+0] = 0;
            }

        }
    }

}

void ExtractBoundary(CvSize S, Boundary *B){

    unsigned char * OutPixelData = (unsigned char *)(B->Bimg->imageData);

    for (int x = 0; x < S.width; x++) {
        for (int y = 0; y < S.height; y++) {

            int y1 = (S.height) - (y+1);

            int Out_index = (y1*S.width+x);//*B->Bimg->nChannels;

            if(OutPixelData[Out_index+0] != 255){
            B->BOUND[x] = cvPoint(x,y);
                y = S.height;
            }

        }
    }
    /*
    for(int i=0; i< S.width; i++){
    printf("%d, %d\n", B->BOUND[i].x, B->BOUND[i].y);
    }
    */
}

void CalculateDistances(CvSize S, Boundary *B, CamCalib camera, BotCalib bot){

    cvZero(DEPTH_MAP);

    for(int i=0; i< S.width; i++){

        double px = (double)(B->BOUND[i].x);
        double py = (double)(B->BOUND[i].y);
        //double beta = camera.pan_angle + atan( (camera.cx - px) / camera.fx );
        double beta = camera.pan_angle + atan( (camera.cx - px)*camera.pix_size_x / camera.fx );
        //double alpha = atan( (camera.cy - py) / camera.fy );
        double alpha = atan( (camera.cy - py)*camera.pix_size_y / camera.fy );
        alpha = alpha > 0 ? alpha : 0.0001;
        double depth = camera.height_from_ground / tan(camera.tilt_angle + alpha);
        //depth = depth < 500000 ? depth : 500000;
        depth = depth < 300000 ? depth : 300000;
        //printf("debug: px = %0.1f, py = %0.1f, beta = %0.3f, alpha = %0.3f, depth = %0.3f\n", px, py, beta, alpha, depth);


        //float temp1 = tan( ((float)((S.height)/2)-(float)(B->BOUND[i].y)) / camera.fy  );
        //if(temp1 <= 0) temp1 = 0.0001;

        ////B->POLAR[i].x = abs( B->Robot_height /  temp1  );
        //B->POLAR[i].x = abs( camera.height_from_ground / temp1  );
        //if(B->POLAR[i].x > 500000) B->POLAR[i].x = 500000;
        //B->POLAR[i].y = atan( ((float)((S.width)/2) - (float)(B->BOUND[i].x)) / camera.fy    );

        B->POLAR[i].x = (float) depth;
        B->POLAR[i].y = beta;
        //printf("debug: polar.x= %0.3f, polar.y = %0.3f\n", B->POLAR[i].x, B->POLAR[i].y);

        POLAR2CART(&B->POLAR[i], &B->CART[i]);

        //fit representation to 400x400 pixel image
        //CART2DISPLAY(&B->CART[i], &B->DISPLAY_CART[i], &B->Robot, B->display_scale_factor);
        CART2DISPLAY(&B->CART[i], &B->DISPLAY_CART[i], &B->Robot, bot.display_scale_factor);

        cvCircle(DEPTH_MAP, B->DISPLAY_CART[i], 1, CV_RGB(255,0,0), 1, 8, 0);

    //printf("%d, %d\n", B[i].x, B[i].y);
    //printf("%0.2f, %0.2f, %0.2f, %0.2f\n", B->POLAR[i].x, B->POLAR[i].y, B->CART[i].x, B->CART[i].y);
    //printf("%d, %d\n", CART[i].x, CART[i].y);

    }

    cvCircle(DEPTH_MAP, B->Robot, 2, CV_RGB(255,0,0), 2, 8, 0); //draw robot
    //cvCircle(DEPTH_MAP, B->Robot, B->min_distance*0.02, CV_RGB(255,255,0), 2, 8, 0); //draw robot
    //cvCircle(DEPTH_MAP, B->Robot, bot.min_dist_to_obst*0.02, CV_RGB(255,255,0), 2, 8, 0); //draw robot
    //cvCircle(DEPTH_MAP, B->Robot, bot.min_dist_to_obst*B->display_scale_factor, CV_RGB(255,255,0), 2, 8, 0); //draw robot
    cvCircle(DEPTH_MAP, B->Robot, bot.min_dist_to_obst*bot.display_scale_factor, CV_RGB(255,255,0), 2, 8, 0); //draw robot


    cvLine(DEPTH_MAP, B->Robot, B->DISPLAY_CART[0], CV_RGB(0,200,0),1,8,0);
    cvLine(DEPTH_MAP, B->Robot, B->DISPLAY_CART[S.width-1], CV_RGB(0,200,0),1,8,0);


    for(int i=0; i<10; i++){
        cvCircle(DEPTH_MAP,  B->Robot, i*30, CV_RGB(0,0,200), 1, 8, 0);
    }

}

void InterpretDepthArray(CvSize S, Boundary *B, BotCalib bot){

    B->widest_angle=0;
    B->best_segment=0;

    int i, T = 500000, seg_no =1, j=0, s=0, start_index=0;
    bool flag1=0, flag2=0;
    int count[15] = {0};

    for( i=0; i< S.width; i++){

        //if( abs( B->POLAR[i].x ) > B->min_distance ){
        if( abs( B->POLAR[i].x ) > bot.min_dist_to_obst ){
            flag1 = 1;
            B->FRONTIERS[j].x = B->POLAR[i].x;
            B->FRONTIERS[j].y = B->POLAR[i].y;
            B->FRONTIERS[j].z = seg_no;
            j++;
        }else{
            flag1 = 0;
        }


        if(flag1==0 && flag2 ==1){
            seg_no ++;
        }

        flag2=flag1;

    }

    //printf("no of seg=%d\n", seg_no);

    int FRONTIERS_LENGTH = j;

    for( s=1; s< seg_no + 1 ; s++){

        for( i=0; i< FRONTIERS_LENGTH ; i++){

            if(B->FRONTIERS[i].z == s){
            count[s] += 1;
            }
            //printf("%0.2f, %0.2f\n", TEMP[i].x, (TEMP[i].y)*(180/pi));
        }
        //printf("s=%d, count=%d\n", s, count[s]);

    }

    for( s=1; s< seg_no + 1 ; s++){
        if(count[s]>1){
            start_index += count[s-1];
            B->Median_Angle[s].x = B->FRONTIERS[cvRound(count[s]/2.0) + start_index - 1].y;
            B->Median_Angle[s].z = B->FRONTIERS[cvRound(count[s]/2.0) + start_index - 1].x;
            B->Median_Angle[s].y = B->FRONTIERS[count[s] + start_index - 1].y - B->FRONTIERS[start_index].y;
            //printf("count=%d, startindex=%d, median[%d] = %0.2f, angular separation[%d]= %0.2f\n",count[s-1], start_index, s, B->Median_Angle[s].x*(180/pi), s, B->Median_Angle[s].y*(180/pi));
            //printf("%0.2f, %0.2f\n", TEMP[14].y, TEMP[37].y);
            //printf("widest_angle=%0.2f, angsep=%0.2f\n", B->widest_angle, abs( B->Median_Angle[s].y ) );
            if( B->widest_angle < abs( B->Median_Angle[s].y ) ){
                B->widest_angle =  abs(B->Median_Angle[s].y);
                B->best_segment = s;
                //printf("largest_angle=%0.2f, best_s=%d\n", B->widest_angle, B->best_segment);
            }
        }
    }
    //Theta = Median_AngularSeparation[best_s].x;
    B->D_Theta.x = B->Median_Angle[B->best_segment].z;
    B->D_Theta.y = B->Median_Angle[B->best_segment].x;

    //printf("%0.2f, %0.2f\n\n", B->D_Theta.x, B->D_Theta.y);


    static CvPoint2D32f pt1;
    pt1.x=0;
    pt1.y=0;
    static CvPoint pt2 = cvPoint(0,0);

    if (p.disp_img){
        POLAR2CART(&B->D_Theta, &pt1);
        //CART2DISPLAY(&pt1, &pt2, &B->Robot, B->display_scale_factor);
        CART2DISPLAY(&pt1, &pt2, &B->Robot, bot.display_scale_factor);

        cvLine(DEPTH_MAP, B->Robot, pt2, CV_RGB(0,200,200),1,8,0);

        //cvLine(DEPTH_MAP, cvPoint(200,390), cvPoint(cvRound(     ((-B->D_Theta.x*0.03)*sin(B->D_Theta.y))    ) + 200 , cvRound(	   -(B->D_Theta.x*0.01)*cos(B->D_Theta.y)   )  + 390), CV_RGB(0,200,200),1,8,0);
        //cvShowImage(DEPTH,DEPTH_MAP);
        DISPLAY_IMAGE_XY(p.refresh, DEPTH_MAP,5,5);
    }
    //printf("Theta=%0.2f\n",Theta*(180/pi));

}
