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

//opencv header files
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <stdio.h> //standard input-output-library
#include <algorithm> //functions on sequences, like find, sort etc.
#include <stdlib.h> //includes random number generation
#include <time.h> //get date and time information
#include <string.h> //manipulate strings and arrays

#include "init_structures.h" //initialise structures used in this algorithm
#include "img_proc_fcns.h" //various image processing/computer vision functions
#include "capture.h" //image capture
//#include "ms_overwrite_safe_buffer.h"
//#include "ms_communications_loop.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>

#include "Net.h" //header for UDP transmission

//#include <libplayerc/playerc.h> include library to interface with robot

using namespace std; //standard c++ library

//definitions and macros
#define ZERO	0 //Binary image zero=0 & one=255
#define ONE	255

#define LP	0.0001 //low probability
#define HP	0.9999 //high probability
#define PI	3.141592653589

#define Window_W 1.02*p.proc_W //appriximate wht window width and hight as a function of the frame size
#define Window_H 1.3*(p.proc_H)+20

#define RAD2DEG(X)	(double) X*(180.0/PI)

//macros for stopwatch
#define CV_TIMER_START(X)       	double X = (double)cvGetTickCount();
#define CV_TIMER_STOP(Y, STRING) 	double Y = (double)cvGetTickCount() - X; \
                                            if(p.verb){ printf("Time @ [%s] = %gms\n", \
                                            STRING, Y/((double)cvGetTickFrequency()*1000.) );}

#define I2D(L) 	((double)L*0.001)
#define DISPLAY_IMAGE(img)			cvNamedWindow(#img); cvShowImage(#img, img);
#define DISPLAY_IMAGE_XY(R,img,X,Y)		if(R){cvNamedWindow(#img); cvMoveWindow(#img, int(round(X*Window_W)), int(round(Y*Window_H))) ;} cvShowImage(#img, img);


int loop_through(const int j, const int keypress_id, const int size_of_collection = 99);
void exitProgram(int code);
int initAll();
void init_images_main(CvSize S);
void release(void);
void init_safe_region(void);
double randdouble();
double randdouble(double min, double max);

//initialise 2D array of strings
char name[15][30] =
{{"Cloudy_Dry"},{"Cloudy_Wet"},{"Cloudy_Muddy"},{"Sunny_Wet"},{"Complex_Scene"},{"Fence"},{"Shadows"},{"Snow"},{"DS1A"},{"DS1B"},{"DS2A"},{"DS2B"},{"DS3A"},{"DS3B"},{"internet_indoor"}};
int id = 1; //int to denote the video id number in a dataset

Boundary Bound; //Bound is a variable of type Boundary
Boundary * BoundPtr = &Bound; //BoundPtr is a pointer to the variable Bound - the & operator returns the address of its operand

Statistics Stats;
Statistics * StatsPtr = &Stats;

//feature names: egm [0], ego[1], hue[2], sat[3], lbp[4];
Features F;
Features *FPtr = &F;

Model StatModel;
Model *StatModelPtr = &StatModel;

Model PrevModel;
Model *PrevModelPtr = &PrevModel;

Model CurrentModel;
Model *CurrentModelPtr = &CurrentModel;

Params p; //class parameters
Params *pPtr = &p;

Robot Bot;
Robot *BotPtr = &Bot;

//Window Names
/*
const char * SOURCE 		= "Source Image";
const char * GRAY_SRC 		= "Gray Image";
const char * SAFE_AREA 		= "Safe Region";
const char * LBP_IMG 		= "LBP Image";
const char * POSTERIOR 		= "Posterior";
const char * PRIOR 		= "Prior Prob";
const char * OBSTACLE_BOUND 	= "Obstacle Boundary";
*/

//extern const char * POST_RATIO;
static CvRect SafeRegion;
static CvSize FrameSize;

IplImage *input_image; // = NULL;

IplImage *safe_region_mask;

//source colour, hsv, hue, and gray images
IplImage *source_img, *gray_img, *result_img;

//Graph Based Segmentation
IplImage *gbs_img;

//Ground cue images
IplImage *contour[2], *bin_img, *prior, *prior_rgb, *prior_prob;

net::Socket soc1;
net::Socket * socketPtr = &soc1;

OwSafeBuffer<string> commbuf(5);

int main(int argc, char** argv)
{
// CargarParametros();
    //parse command line options
    if (argc > 1){
        if( p.parse_cmd_options(argc,argv) != 0 ) return 0;
    }

    static int key=0; //holds user-input keystroke
    static int i=0,j=0;

    //LOOPS for generating results on datasets
    //for (int id=14;id<=14;id++){ // do for all the 15 video sequences [0-14]
    //for (int id=1;id<=1;id++){ //do once!
    //for (int T = -14;T<=14;T+=2){ // do for a range of Log posterior thresholds to generate a ROC curve
    //for (int run = 1; run<=400; run++){ do for several runs on the static traversability dataset
    //for (int T = 0;T<=1;T+=2){// do once!

    //local variables
    int T=0;
    if(p.verb){ printf("T=%d\n",T); }



    //j=-1; //image index number for image sequences without stopping
    j=0;

    p.log_post_thres_zero_position = T*10 + 300; //thres represents the log posterior threshold on a opencv `trackbar', 0 => 300

    //INITIALISATIONS
    FrameSize = cvSize(cvRound(p.proc_W), cvRound(p.proc_H)); //define image size
    source_img = cvCreateImage(FrameSize, 8, 3);


// VerificarOrigenImagenes();
    if (p.video_data){
        if (p.webcam){
            if( !initCapture() )            return 0;
        } //capture from webcam
    else{
        if( initVideoCapture() != 0)    return 1;} //capture video from file or network stream
    }

// InicializarContenedoresImagenesYalgoritmosEstimacion();
    input_image = cvCreateImage(p.capture_size,8,3);

    init_images_main(FrameSize);
    init_images_img_proc(FrameSize);
    //if (p.disp_img){ init_windows_img_proc(FrameSize); }

    init_safe_region();

    init_boundary(FrameSize, BoundPtr);
    init_model(FrameSize, SafeRegion, PrevModelPtr);
    init_model(FrameSize, SafeRegion, StatModelPtr);
    init_stats(FrameSize, StatsPtr, 1);
    init_features(FrameSize, FPtr);

    printf( "\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n"
    "This software runs a vision-based autonomous guidance algorithm\n"
    "for a mobile robot equipped with a low-quality monocular camera.\n"
    "The vision system allows a mobile robot to autonomously guide itself\n"
    "past static or dynamic obstacles in both indoor or outdoor natural environments\n"
    "in a real-time, reactive manner.\n\n"
    "To begin, make sure all parameters are correct and press 'Enter'\n\n"
    "To manually reinitialise the system press '*TODO*'\n\n"
    "To exit, click inside one of the display windows,\n"
    "then press the 'q' key\n\n\n"
    "***WARNING***\n"
    "--Prolonged use of this software may cause injuries--"
    "\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n");
    //fgetc(stdin);
    cvWaitKey(0);



    //MAIN PROCESSING LOOP
    //j = 363;
    int flag=1;

    while(1)
    {
        //SELECTION OF INPUT//
        static int frame_no = 0;
        //static int Im_ID_max = 53; //58

// GrabFrame();
        //--START-->CAPTURE FRAME//
        if(p.video_data){
            NextFrame(&input_image); //captures a video frame from webcam or video file

        }else if(!p.video_data){ //gets an image from file
            j = loop_through(j, key); //control test image sequence

            char buffer [250];
            sprintf(buffer,"static_traversability_dataset/source_%d.bmp", j);

            input_image = cvLoadImage(buffer);
            if (input_image == NULL){
                    fprintf(stderr, "Did not load image:\n");
                    fprintf(stderr, "%s\n", buffer);
                    cvWaitKey(0);
                    input_image = cvLoadImage("corridor.jpg");
            }
        }// end else if(!video_data)
        //--END-->CAPTURE FRAME//

        //if(!p.dynamic) init_stats(FrameSize, StatsPtr, 0);
        //--START-->IMAGE PROCESSING//
        //if(!finished){ //finished == 1 if something goes wrong with capture
        CV_TIMER_START(X)  //start timer
        if(p.verb){ printf("\nSTART OF LOOP\n"); }

// PreprocesamientoImagen();
        //IMAGE PRE-PROCESSING//
        cvResize(input_image, source_img, CV_INTER_AREA); 			//Resize
        //cvSmooth(source_img, source_img, CV_GAUSSIAN, 3, 3);//Smooth image
        if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, source_img, 0 , 0); }

        if (p.write_output_to_disk){
        char buffer1[150];
        sprintf(buffer1,"results/source_%d.bmp",frame_no);
        cv::Mat image(source_img);
        cv::imwrite(buffer1, image); //at time of writing imwrite was better/faster at saving images
        }

        cvCvtColor( source_img, gray_img, CV_BGR2GRAY);	//Convert Color to gray RGB[A]->Gray: Y<-0.299*R + 0.587*G + 0.114*B
        if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, gray_img, 1, 0); }

        cvCvtColor( source_img, F.hsv, 	CV_BGR2HSV );	//Convert to HSV
        cvCvtColor( source_img, F.YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
        cvCvtColor( source_img, F.lab, 	CV_BGR2Lab );	//Convert to Lab

        cvCvtPixToPlane( F.hsv, F.hue, F.sat, 0, 0 ); //cvCvtPixToPlane Divides a multi-channel array into separate single-channel arrays
        cvCvtPixToPlane( F.YCrCb, 0, F.Cr, F.Cb, 0 );
        cvCvtPixToPlane( F.lab, 0, F.a, 0, 0 );

        //view iamges separately
        if (p.disp_img){	DISPLAY_IMAGE_XY(p.refresh, F.hue, 2, 0);
                            DISPLAY_IMAGE_XY(p.refresh, F.sat, 3, 0); }

        combine_channels(F.Cr, F.Cb, F.a, F.iic); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
        //cvEqualizeHist(F.iic, F.iic);
        cvNormalize(F.iic, F.iic, 0, 255, CV_MINMAX);
        if (p.disp_img){ 	DISPLAY_IMAGE_XY(p.refresh, F.iic, 4, 0); }

        convertGray2LBP(gray_img, F.lbp); //Convert to LBP

        //getEdgeMagOri(gray_img, F.mag32, F.ang32); //Edge Detection
        getEdgeMagOri(F.sat, F.mag, F.ang32); //Extract edge magnitudes and orientation
        if (p.disp_img){ 	DISPLAY_IMAGE_XY(p.refresh, F.mag, 5, 0);
                            DISPLAY_IMAGE_XY(p.refresh, F.ang32, 6, 0);
        }
        CV_TIMER_STOP(A, "Resize, Smooth, CvtColor, LBP, EDGE, IIC");

        //DISPLAY IMAGES
        if (p.disp_img){
            //DISPLAY_IMAGE_XY(p.refresh, source_img, 0 , 0);
            //DISPLAY_IMAGE_XY(p.refresh, gray_img, 1 , 0);
            DISPLAY_IMAGE_XY(p.refresh, F.lbp, 7, 0);
        //cvShowImage( GRAY_SRC, gray_img);
        //cvShowImage( LBP_IMG,  F.lbp);
        }

// SegmentacionGrafos();
        //SPLIT IMAGE INTO SUPERPIXELS USING FELZENSZWALB GRAPH-BASED SEGMENTATION
        GraphBasedSegmentation( gbs_img, gray_img);
        CV_TIMER_STOP(B, "Felzenszwalb Graph Based Segmentation")

        //Superpixel Statistics
        SuperPixelStats(gbs_img, gray_img, StatsPtr);
        if(p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, Stats.prior_img, 0 , 2); //cvShowImage(PRIOR, Stats.prior_img);
        }
        CV_TIMER_STOP(C, "SuperPixel Statistics (mean, stdDev, size, position)")


// ActualizarModeloProbabilistico();
        //if(p.move){
        if(!p.dynamic){
            GetModel(gray_img, FPtr, StatModelPtr, p.dynamic);
            CV_TIMER_STOP(D, "Get Model Statistics")

        }else if(p.dynamic){
            if( flag == 1 ){

                UpdatePrior(gbs_img, StatsPtr, FPtr);
                //UpdatePrevModel(StatModelPtr, PrevModelPtr);//

                //Get Ground Statistical Model
                GetModel(gray_img, FPtr, StatModelPtr, p.dynamic);
                CV_TIMER_STOP(D, "Get Model Statistics")

                //UpdateModel(StatModelPtr, PrevModelPtr);//
                flag=0;
            }
            flag++;
        }
        //}

        //for(int em=0; em<10;em++){
        //static bool converged = 0;
        FeatureAnalysis(FPtr, StatModelPtr, StatsPtr, gbs_img, p.dynamic);
        CV_TIMER_STOP(G, "Feature analysis")

        ProbAnalysis2(FPtr, StatsPtr, gbs_img);
        if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, F.post1, 0 , 4);
        //cvShowImage(POSTERIOR, F.post1);

        }
        CV_TIMER_STOP(H, "Prob Analysis")

        UpdateParams(F.bin_class_result, StatsPtr, FPtr, p.dynamic);
        CV_TIMER_STOP(I, "EM Analysis")


// EstimarFronteraObstaculo();
        //FIND OBSTACLE BOUNDARY AND COMPUTE STEERING DIRECTION
        //cvCopy(gbs_img,Bound.Bimg);
        cvMerge(F.bin_class_result, F.bin_class_result, F.bin_class_result, NULL, result_img);
        cvCopy(F.bin_class_result,Bound.Bimg);

        //cvSmooth(Bound.Bimg, Bound.Bimg, CV_MEDIAN, 3, 3);//Smooth image
        cvDilate(Bound.Bimg,Bound.Bimg, NULL, 1);
        cvErode(Bound.Bimg,Bound.Bimg, NULL, 3);
        cvSmooth( Bound.Bimg,  Bound.Bimg, CV_MEDIAN, 3, 3);
        FindObstacleBoundary(Bound.Bimg);
        if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, Bound.Bimg, 1 , 4); //cvShowImage(OBSTACLE_BOUND, Bound.Bimg);

        }
        cvCopy(Bound.Bimg, bin_img);

        //UpdateParams(Bound.Bimg, StatsPtr, FPtr, p.dynamic);

        ExtractBoundary(FrameSize, BoundPtr);
        CalculateDistances(FrameSize, BoundPtr, p.camera, p.bot);

        InterpretDepthArray(FrameSize, BoundPtr, p.bot);
        CV_TIMER_STOP(J, "Path analysis")

        Contours(F.bin_class_result); //Find contours
        DrawContours(contour[0], CV_RGB( 0, 150, 0 ), SafeRegion); //Draw contours
        Contours(Bound.Bimg); //Find contours
        DrawContours(contour[1], CV_RGB( 255, 0, 0 ), SafeRegion); //Draw contours

        cvAddWeighted( contour[0], 0.5, contour[1], 1, 0, contour[0] );
        cvAddWeighted( contour[0], 0.5, source_img, 1, 0, source_img );
        // //checking for memory leaks
        DISPLAY_IMAGE(source_img)
        //cvNamedWindow("test");
        //cvShowImage("test", source_img);



        if(frame_no%25 == 0 && frame_no>0){
            frame_no=j;
                char buffer1 [150];
                char buffer2 [150];
                //char buffer3 [150];
                //cvNamedWindow("result");
                //cvShowImage("result", result_img);
            sprintf(buffer1,"results/%s/%d/%s_source_%d.bmp", name[id-1],T,name[id-1],frame_no);
            sprintf(buffer2,"results/%s/%d/%s_result_%d.bmp", name[id-1],T,name[id-1],frame_no);
            //sprintf(buffer3,"results/%s/%d/%d/%s_bound_%d_%d.bmp", name[id-1],T,frame_no,name[id-1],frame_no,run);
            cvSaveImage(buffer1, source_img);
            cvSaveImage(buffer2, result_img);
            //cvSaveImage(buffer3, bin_img);
        }

        ReleaseContours();

        ////cvReleaseImage(&input_image);


        frame_no++;
        CV_TIMER_STOP(K, "End of Loop")
        if(p.verb){ printf("\n\n"); }

        //}
        //printf("key = %d\n", key);

        if(!p.debug) 	key = cvWaitKey(10);
        else 		key = cvWaitKey(p.debug_delay_ms);

        //j++; //loop through image sequences automatically
        //if(j>100) finished=1;
        //if(j>Im_ID_max) finished=1;
        //if(j>Im_ID_max) j=0;
        //static int delay = 10;
        //cvWaitKey(delay);
        //	cvCreateTrackbar("D", POST_RATIO, &delay, 1500, NULL );
        //	cvCreateTrackbar("I", POST_RATIO, &j, Im_ID_max, NULL );
        //}// end if !finished
        if(key == 'q' || key == 'x' || key == 1048689 || key == 1048603)
        break;
        p.refresh=0;
    }


    //}// end for T
    //} //end for id
    //release_images
//    comm_thread.join();

    exitProgram(EXIT_SUCCESS);

}//end main



double randdouble()
{
    return rand()/(double(RAND_MAX)+1);
}

//generates a psuedo-random double between min and max
double randdouble(double min, double max)
{
    if (min>max)
    {
        return randdouble()*(min-max)+max;
    }
    else
    {
        return randdouble()*(max-min)+min;
    }
}

void init_images_main(CvSize S){

    safe_region_mask 	= cvCreateImage( S, 8, 1);
    source_img 		= cvCreateImage( S, 8, 3);
    result_img 		= cvCreateImage( S, 8, 3);
    gray_img 		= cvCreateImage( S, 8, 1);
    gbs_img 		= cvCreateImage( S, 8, 1);
    contour[0] 		= cvCreateImage( S, 8, 3);
    contour[1] 		= cvCreateImage( S, 8, 3);
    prior_prob	 	= cvCreateImage( S, 32, 1);
    prior_rgb	 	= cvCreateImage( S, 8, 3);
    prior	 	= cvCreateImage( S, 8, 1);
    bin_img	 	= cvCreateImage( S, 8, 1);

}

void release(void)
{

    cvDestroyAllWindows();
    release_images_img_proc();

    input_image = cvCreateImage(p.capture_size,8,3); //needed to avoid segmentation fault!
    cvReleaseImage( &input_image );

    cvReleaseImage( &source_img );
    cvReleaseImage( &result_img );
    cvReleaseImage( &gray_img );

    cvReleaseImage( &bin_img );
    cvReleaseImage( &gbs_img );
    cvReleaseImage( &safe_region_mask );

    cvReleaseImage( &Bound.Bimg );
    cvReleaseImage( &prior );
    cvReleaseImage( &prior_prob );
    cvReleaseImage( &prior_rgb );


    cvReleaseImage( &F.mag );
    cvReleaseImage( &F.ang32 );
    cvReleaseImage( &F.P_ang );

    cvReleaseImage( &F.hsv );
    cvReleaseImage( &F.hue );
    cvReleaseImage( &F.sat );
    cvReleaseImage( &F.val );
    cvReleaseImage( &F.P_hue );
    cvReleaseImage( &F.P_sat );
    cvReleaseImage( &F.P_val );

    cvReleaseImage( &F.lbp );
    cvReleaseImage( &F.P_lbp );

    cvReleaseImage( &F.post0 );
    cvReleaseImage( &F.post1 );
    cvReleaseImage( &F.post_ratio );
    cvReleaseImage( &F.bin_class_result );

    cvReleaseImage( &contour[0] );
    cvReleaseImage( &contour[1] );

}

void init_safe_region(void)
{

     SafeRegion = cvRect( cvRound(source_img->width/3),//+25,
                            (source_img->height)-cvRound((source_img->height)/7),
                            cvRound(source_img->width/3),
                            cvRound((source_img->height)/8) );

/*    SafeRegion = cvRect( cvRound(source_img->width/4),//+25,
                           (source_img->height) -cvRound((source_img->height)/4) -3,
                           cvRound(source_img->width/2),
                           cvRound((source_img->height)/4) );
*/
    cvSetImageROI( safe_region_mask, SafeRegion );
    cvSet(safe_region_mask,cvScalarAll(ONE),0);
    cvResetImageROI( safe_region_mask );

    if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, safe_region_mask, 0 , 1);
    //cvShowImage( SAFE_AREA , safe_region_mask);
    }
}

// exit Program
void exitProgram(int code)
{
    release();

    if(p.robot){
    Bot.shutdown(pPtr);
    if (p.network) net::ShutdownSockets(); // shutdown socket layer
    }

    exit(code);
}

int loop_through(int j, const int keypress_id, const int size_of_collection)
{
    static int change=0;
    //control test image sequence

    if (keypress_id=='w' || keypress_id == 1048695){
        j--;
        j = j < 0 ? size_of_collection : j;

    }
        else if(keypress_id == 'e' || keypress_id == 1048677){
        j++;
        j = j > size_of_collection ? 0 : j;

    }//*/

    if(change != j){
    init_stats(FrameSize, StatsPtr,0);
    change =j;
    }
    //*/

    /*
    int k = (j/100)%10;
    int l = (j/10)%10;
    int m = (j/1)%10;
    char buffer [250];
    sprintf(buffer,"/home/mikesapi/Videos/msc_experiments/Datasets/DARPA/%s/rgbframe%d%d%d.bmp", name[id-1],k,l,m);
    printf("/home/mikesapi/Videos/DARPA/%s/rgbframe%d%d%d.bmp", name[id-1],k,l,m);
    //cvWaitKey();
    */

    return j;
}
