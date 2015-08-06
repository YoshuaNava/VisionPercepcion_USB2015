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
#include <math.h>

#include <string>
using std::string;

#include "init_structures.h"

#define DUMMY 0
#define QUAD 1
#define ROVER 2
#define VISAR 3
#define EMPTY 4

#define ZERO	0 //Binary image zero=0 & one=255
#define ONE	255

#include "Net.h" //header for UDP transmission


void init_stats(CvSize Img_Size, Statistics * S, bool init)
{
    static int range = 256;
    S->no_features = 6;

    if(init) S->id = new int[range];
    for (int i=0; i<range; i++){
    S->id[i] = 0;
    }

    if(init) S->size = new int[range];
    for (int i=0; i<range; i++){
    S->size[i] = 0;
    }

    if(init) S->gray_id = new int[range];
    for (int i=0; i<range; i++){
    S->gray_id[i] = 0;
    }

    if(init) S->mean = new CvScalar[range];
    for (int i=0; i<range; i++){
    S->mean[i] = cvScalar(0,0,0,0);
    }

    if(init) S->stdDev = new CvScalar[range];
    for (int i=0; i<range; i++){
    S->stdDev[i] = cvScalar(0,0,0,0);
    }

    if(init) S->box = new CvRect[range];
    for (int i=0; i<range; i++){
    S->box[i] = cvRect(0,0,0,0);
    }

    if(init) S->P_Gt = new double[range];
    if(init) S->P_Gf = new double[range];
    /*
    S->P_EgGt = new double[range];
    S->P_EgGf = new double[range];
    S->P_HgGt = new double[range];
    S->P_HgGf = new double[range];
    S->P_SgGt = new double[range];
    S->P_SgGf = new double[range];
    S->P_VgGt = new double[range];
    S->P_VgGf = new double[range];
    S->P_LgGt = new double[range];
    S->P_LgGf = new double[range];
    */
    if(init) S->P_GtgF = new double[range];
    if(init) S->P_GfgF = new double[range];
    for (int i=0; i<range; i++){
    S->P_Gt[i] = 0.;
    S->P_Gf[i] = 0.;
    /*
    S->P_EgGt[i] = 0.;
    S->P_EgGf[i] = 0.;
    S->P_HgGt[i] = 0.;
    S->P_HgGf[i] = 0.;
    S->P_SgGt[i] = 0.;
    S->P_SgGf[i] = 0.;
    S->P_VgGt[i] = 0.;
    S->P_VgGf[i] = 0.;
    S->P_LgGt[i] = 0.;
    S->P_LgGf[i] = 0.;
    */
    S->P_GtgF[i] = 0.;
    S->P_GfgF[i] = 0.;
    }

    if(init) S->P_FgGt = new double[range*S->no_features];
    if(init) S->P_FgGf = new double[range*S->no_features];

    for (int i=0; i<range; i++){
        for (int j=0; j<S->no_features; j++){
            S->P_FgGt[S->no_features*i + j] = 0.;
            S->P_FgGf[S->no_features*i + j] = 0.;
        }
    }


    if(init)  S->prior_img =  cvCreateImage(Img_Size,32,1);
    if(!init) cvZero(S->prior_img);

    S->nos = 0; //number of segments
    S->img_w = Img_Size.width;
    S->img_h = Img_Size.height;

    unsigned int iseed = (unsigned int)time(NULL);
    srand (iseed);
    //double min = 0.001;
    double min = 0.01;
    double max = 40.01;

    if(init) S->L1 = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->L1[i] = 0.01;
        //S->L1[i] = randdouble(min, max);
        //printf("L1[%d] = %0.5f\n", i, S->L1[i]);
    }

    if(init) S->L0 = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->L0[i] = 0.01;
        //S->L0[i] = randdouble(min, max*0.1);
        //printf("L0[%d] = %0.5f\n", i, S->L0[i]);
    }

    //double gmax_random = randdouble(1, 10);
    if(init) S->gmax = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->gmax[i] = 2.;
        //S->gmax[i] = randdouble(1, 10);
        //S->gmax[i] = gmax_random;
        //printf("gmax[%d] = %0.5f\n", i, S->gmax[i]);
    }

    if(init) S->Z1 = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->Z1[i] = (1/S->L1[i])*(1-exp(-S->L1[i]*S->gmax[i]));
    }

    if(init) S->Z0 = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->Z0[i] = (1/S->L0[i])*(exp(S->L0[i]*S->gmax[i])-1);
    }

    if(init) S->G_score = new double[S->no_features];
    for (int i=0; i<S->no_features; i++){
        S->G_score[i] = 0;
    }

    //Histogram Initialization
    static int dim_40 	= 40;
    static int range_40 	= 40;
    float range_40_arr[] = {float(0.),float(range_40-1)};
    float* range_40_ptr = range_40_arr;

    if(init){
    for(int i=0;i<S->no_features;i++){
    S->H_G1[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
    S->H_G0[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
    S->H_G1_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
    S->H_G0_DISP[i] = cvCreateHist(1, &dim_40, CV_HIST_ARRAY, &range_40_ptr, 1);
    }
    }
    if(!init){
        for(int i=0;i< S->no_features; i++)
        {
            cvClearHist(S->H_G1[i]);
            cvClearHist(S->H_G0[i]);
            cvClearHist(S->H_G1_DISP[i]);
            cvClearHist(S->H_G0_DISP[i]);
        }
    }

    //Histogram Initialization
    static int dim_9	= 9;
    static int dim_32 	= 32;

    static int range_256 	= 256;
    static int range_181 	= 181;

    float range_256_arr[] = {float(0.),float(range_256-1)};
    float range_181_arr[] = {float(0.),float(range_181-1)};
    float range_2pi_arr[] = {-CV_PI,CV_PI};

    float* range_256_ptr = range_256_arr;
    float* range_181_ptr = range_181_arr;
    float* range_2pi_ptr = range_2pi_arr;

    if(init){
        S->H_SF[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
        S->H_SF[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
        S->H_SF[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
        S->H_SF[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
        S->H_SF[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
        S->H_SF[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
    }
    if(!init){
        for(int i=0;i<S->no_features;i++){
            cvClearHist(S->H_SF[i]);
        }
    }

}


void init_boundary(CvSize S, Boundary * B)
{
    B->Bimg = cvCreateImage(S,8,1);

    B->BOUND = new CvPoint[S.width];
    for (int i=0; i<S.width; i++){
        B->BOUND[i] = cvPoint(0,0);
    }

    B->POLAR = new CvPoint2D32f[S.width];
    for (int i=0; i<S.width; i++){
        B->POLAR[i].x = 0.;
        B->POLAR[i].y = 0.;
    }

    B->CART = new CvPoint2D32f[S.width];
    for (int i=0; i<S.width; i++){
        B->CART[i].x = 0.;
        B->CART[i].y = 0.;
    }

    B->DISPLAY_CART = new CvPoint[S.width];
    for (int i=0; i<S.width; i++){
        B->DISPLAY_CART[i].x = 0.;
        B->DISPLAY_CART[i].y = 0.;
    }

    //delete [] B->BOUND;
    //B->BOUND = NULL;
    B->FRONTIERS = new CvPoint3D32f[S.width];
    for (int i=0; i<S.width; i++){
        B->FRONTIERS[i].x = 0.;
        B->FRONTIERS[i].y = 0.;
        B->FRONTIERS[i].z = 0.;
    }

    B->Robot = cvPoint(200,390);
    //B->Robot_height = 1600.0; //1600
    //B->F.x = 190.0;
    //B->F.y = 190.0;

    //B->min_distance = 12000; //18000; // for quad
    //B->min_angular_separation = 15.0; //25 for quad
    B->display_scale_factor = 0.08; //was 0.02!

    B->Median_Angle = new CvPoint3D32f[15];
    for (int i=0; i<15; i++){
        B->Median_Angle[i].x = 0.;
        B->Median_Angle[i].y = 0.;
        B->Median_Angle[i].z = 0.;
    }

    B->best_segment = 0;
    B->widest_angle = 0;

    B->D_Theta.x = 0.;
    B->D_Theta.y = 0.;
}

void init_model(CvSize S, CvRect SafeRegion, Model *M)
{
    int n = 1; //number of model regions

    M->box = &SafeRegion;

    M->mask = cvCreateImage(S,8,1);

    cvSetImageROI( M->mask, SafeRegion );
    cvSet(M->mask,cvScalarAll(ONE),0);
    cvResetImageROI( M->mask );

    M->mean = new CvScalar[n];
    for (int i=0; i<n; i++){
    M->mean[i] = cvScalar(0,0,0,0);
    }

    M->stdDev = new CvScalar[n];
    for (int i=0; i<n; i++){
    M->stdDev[i] = cvScalar(0,0,0,0);
    }

    //Histogram Initialization
    static int dim_9	= 9;
    static int dim_16 	= 16;
    static int dim_32 	= 32;
    static int dim_64 	= 64;
    static int dim_128 	= 128;

    static int range_256 	= 256;
    static int range_181 	= 181;
    static int range_91 	= 91;

    float range_256_arr[] = {float(0),float(range_256-1)};
    float range_181_arr[] = {float(0),float(range_181-1)};
    float range_91_arr[] = {float(0),float(range_91-1)};
    float range_2pi_arr[] = {-CV_PI,CV_PI};

    float* range_256_ptr = range_256_arr;
    float* range_181_ptr = range_181_arr;
    float* range_91_ptr = range_91_arr;
    float* range_2pi_ptr = range_2pi_arr;

    M->dim = new int[N];
    M->dim[0] = 32;
    M->dim[1] = 9;
    for (int i=2; i<N; i++){
        M->dim[i] = 32;
    }

    M->H_M[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
    M->H_M[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
    M->H_M[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
    M->H_M[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
    M->H_M[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
    M->H_M[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);

    M->H_M_DISP[0] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
    M->H_M_DISP[1] = cvCreateHist( 1, &dim_9, CV_HIST_ARRAY, &range_2pi_ptr, 1 );
    M->H_M_DISP[2] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_181_ptr, 1 );
    M->H_M_DISP[3] = cvCreateHist( 1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1 );
    M->H_M_DISP[4] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
    M->H_M_DISP[5] = cvCreateHist(1, &dim_32, CV_HIST_ARRAY, &range_256_ptr, 1);
}


void init_features(CvSize S, Features * F)
{

    F->mag = cvCreateImage(S,8,1);
    F->ang32 = cvCreateImage(S,32,1);
    F->P_ang = cvCreateImage(S,32,1);

    F->hsv   = cvCreateImage( S, 8, 3);
    F->lab   = cvCreateImage( S, 8, 3);
    F->YCrCb   = cvCreateImage( S, 8, 3);
    F->hue   = cvCreateImage( S, 8, 1);
    F->sat   = cvCreateImage( S, 8, 1);
    F->val   = cvCreateImage( S, 8, 1);
    F->Cr   = cvCreateImage( S, 8, 1);
    F->a   = cvCreateImage( S, 8, 1);
    F->Cb   = cvCreateImage( S, 8, 1);
    F->iic   = cvCreateImage( S, 8, 1);
    F->P_hue   = cvCreateImage( S, 32, 1);
    F->P_sat   = cvCreateImage( S, 32, 1);
    F->P_val   = cvCreateImage( S, 32, 1);

    F->lbp  = cvCreateImage( S, 8, 1);
    F->P_lbp = cvCreateImage( S, 32, 1);

    F->post0 = cvCreateImage( S, 32, 1);
    F->post1 = cvCreateImage( S, 32, 1);
    F->post_ratio = cvCreateImage( S, 32, 1);

    F->bin_class_result = cvCreateImage( S, 8, 1);

    for(int i=0;i<5;i++){
        F->P_X1[i] = cvCreateImage( S, 32, 1);
        F->P_X0[i] = cvCreateImage( S, 32, 1);
    }

}

//float Params::scalingFactor = 42.0;

#define dummy	0 //Binary image zero=0 & one=255
#define quad	1
#define rober	2 //Binary image zero=0 & one=255
#define visar	3
#define empty  	4

//Params constructor initialises parameters to default values
const string Params::robot_names[5] = {"dummy","quad","rover","visar","empty"};
Params::Params()
{
    //defauly capture properties:
    sprintf(video_capture_source,"eng_stat_obst.avi");
    sprintf(image_capture_source,"static_traversability_dataset/");
    capture_W = 160;
    capture_H = 120;

    proc_H = 120;
    proc_W = proc_H*1.33333;

    //bool flags
    video_data = 1; 	//flag to indicate capture from video file or webcam vs image files
    refresh = 1;		//refresh window names and positions
    webcam = 0;		//flag to indicate capture from video file vs webcam
    disp_img = 1;   	//flag to show all windows
    verb = 1; 	  	//verbose
    debug = 0; 		//debug
    debug_delay_ms = 1000;
    write_output_to_disk = 0;
    robot = 0; 		//use a robotic platform
    move = 0; 		//monitor robot movement to check when it stops moving
    network = 0; 		//use network
    robot_name.assign("dummy");
    robot_id = 0;

    dynamic = 1;

    net.port = 8888;
    net.ip1 = 127;
    net.ip2 = 0;
    net.ip3 = 0;
    net.ip4 = 1;

    //camera.height_from_ground = 1600.0;
    //camera.fy = 190.0; //pi 3.6mm
    //camera.fx = 190.0; //pi 3.6mm
    camera.cx = proc_W/2;
    camera.cy = proc_H/2;

    camera.height_from_ground = 400.0; //mm
    camera.fx = 3.6; //mm
    camera.fy = 3.6; //mm
    camera.pix_size_x = 0.0014*(2592/proc_W); //mm
    camera.pix_size_y = 0.0014*(1944/proc_H); //mm
    camera.tilt_angle = 0.0;
    camera.pan_angle = 0.0;


    //bot.min_dist_to_obst = 6000;
    bot.min_dist_to_obst = 4000; //mm
    bot.min_ang_between_obst = 15;
    bot.display_scale_factor = 0.08;
    bot.max_depth_threshold = 1e6;

    log_post_thres_zero_position = 300;

}

//initialise the parameters of a specific robot
int Params::init_specific_robot()
{
    int i;
    int listsize = sizeof robot_names / sizeof robot_names[0];
    //printf("listsize=%d\n",listsize);
    for (i=0;i<listsize;i++)
    {
        //if(std::strcmp(robot_names[i].c_str(),robot_name[1].c_str()) == 0)
        //if(robot_names[i].compare(robot_name[0])==0)
        if(robot_names[i].compare(robot_name)==0)
        break;
    }
    //printf("i=%d\n",i);
    robot_id=i;

    if (robot_id == DUMMY){
        sprintf(video_capture_source,"eng_stat_obst.avi");
        sprintf(image_capture_source,"static_traversability_dataset/");

        robot=1; move=0; network=0;

        return 0;
    }
    if (robot_id == QUAD){
        if (debug==1){
            sprintf(video_capture_source,"eng_stat_obst.avi");
            sprintf(image_capture_source,"static_traversability_dataset/");
            debug_delay_ms = 10;
        }else{
            sprintf(video_capture_source,"tcpclientsrc host=192.168.1.101 port=5000  ! gdpdepay !  rtph264depay ! ffdec_h264 ! ffmpegcolorspace ! appsink");
        }

        robot=1; move=1; network=1;

        net.port = 8888; net.ip1 = 192; net.ip2 = 168; net.ip3 = 1; net.ip4 = 100;

        camera.height_from_ground = 1600.0;
        camera.fx = 190.0;
        camera.fy = 190.0;
        camera.pix_size_x = 1; //0.0014*(2592/proc_W); //mm
        camera.pix_size_y = 1; //0.0014*(1944/proc_H); //mm
        camera.tilt_angle = 0.0;
        camera.pan_angle = 0.0;


        //bot.min_dist_to_obst = 6000;
        //bot.min_dist_to_obst = 18000; //mm
        bot.min_dist_to_obst = 18000; //mm
        bot.min_ang_between_obst = 15;
        bot.display_scale_factor = 0.02;
        bot.max_depth_threshold = 5e6;
        /*
        bot.min_dist_to_obst = 18000;
        bot.min_ang_between_obst = 25;

        camera.height_from_ground = 1600.0;
        camera.fy = 190.0;
        camera.fx = 190.0;
        camera.tilt_angle = 0.0;
        camera.pan_angle = 0.0;
        */
        return 0;
    }
    if (robot_id == ROVER){
        if (debug==1){
            sprintf(video_capture_source,"eng_stat_obst.avi");
            sprintf(image_capture_source,"static_traversability_dataset/");
            debug_delay_ms = 10;
        }else{
            sprintf(video_capture_source,"tcpclientsrc host=192.168.2.127 port=5000  ! gdpdepay !  rtph264depay ! ffdec_h264 ! ffmpegcolorspace ! appsink");
        }

        robot=1; move=1; network=1;

        net.port = 8888; net.ip1 = 192; net.ip2 = 168; net.ip3 = 2; net.ip4 = 6;


        camera.height_from_ground = 240.0; //mm
        camera.fx = 3.6; //mm
        camera.fy = 3.6; //mm
        camera.pix_size_x = 0.0014*(2592/proc_W); //mm
        camera.pix_size_y = 0.0014*(1944/proc_H); //mm
        camera.tilt_angle = 0.0;
        camera.pan_angle = 0.0;


        //bot.min_dist_to_obst = 6000;
        bot.min_dist_to_obst = 2000; //mm
        bot.min_ang_between_obst = 15;
        bot.display_scale_factor = 0.08;
        /*
        bot.min_dist_to_obst = 6000;
        bot.min_ang_between_obst = 15;

        camera.height_from_ground = 1600.0;
        camera.fy = 190.0;
        camera.fx = 190.0;
        camera.tilt_angle = 0.0;
        camera.pan_angle = 0.0;
        */
        return 0;
    }
    else{
        printf("Warning: the robot you selected has no initialisations yet!\n");
        return 1;
    }

}

int Params::parse_cmd_options(int argc, char **argv)
{

if(cmdOptionExists(argv, argv+argc, "-db"))
{
    debug = 1;// Do stuff
    printf("debug set to: %d\n",debug);
}

char * filename = getCmdOption(argv, argv + argc, "-r");

if (filename)
{
    printf("robot_name=%s\n",filename);
    robot_name = filename;
    //cout << "robot -> " << p.robot_name[1] << "\n";
    cvWaitKey();
        // Do interesting things
        // ...
}

return 0;
}

char* Params::getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool Params::cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}


//class Robot definitions
Robot::Robot()
{
//net::Socket *socPtr = &soc;
}



int Robot::send2robot(Params *p, net::Socket *socPtr, double angle, double speed)
{
    if (p->robot_id == DUMMY){
        if (p->verb) printf("PRETENDING TO SEND DATA: angle=%0.4f, speed=%0.4f\n", angle, speed);
    }
    else if (p->robot_id == ROVER){
        sprintf(rover_data,"%ds%dt", angle2rover(angle), speed2rover(speed));
        if (p->network) socPtr->Send( net::Address(p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port), rover_data, sizeof(rover_data) );
        if (p->verb) printf("SEND: angle=%0.4f, speed=%0.4f as UDP DATA: %s, on address: %d.%d.%d.%d:%d\n",angle, speed, rover_data,p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port);

    }
    else if (p->robot_id == QUAD){
        if (p->debug) printf("PRETENDING TO SEND DATA: angle=%d, speed=%d\n", angle2quad(angle),speed2quad(speed));
        char first = 0x01;
        sprintf(quad_data,"%c%c%c", first,(char) speed2quad(speed), (char) angle2quad(angle));
        if (p->network) socPtr->Send( net::Address(p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port), quad_data, sizeof(rover_data) );
        if (p->verb) printf("SEND: angle=%0.4f, speed=%0.4f as UDP DATA: %s, on address: %d.%d.%d.%d:%d\n",angle, speed, quad_data,p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port);
        if (p->debug) write_to_file(fptr, quad_data);
    }
    else{
        printf("Warning: the robot you selected has no sending functions yet!\n");
        exit(EXIT_FAILURE);
    }

    return 0;
}

int Robot::send2robot(Params *p, OwSafeBuffer<string> *commbuf, double angle, double speed)
{
    if (p->robot_id == DUMMY){
        if (p->verb) printf("PRETENDING TO SEND DATA: angle=%0.4f, speed=%0.4f\n", angle, speed);
        char tmpbuf[200];
        sprintf(tmpbuf,"angle=%0.2f, speed=%0.2f", angle, speed);
        commbuf->Reset(string(tmpbuf));
    }
    else if (p->robot_id == ROVER){
        sprintf(rover_data,"%ds%dt", angle2rover(angle), speed2rover(speed));
        commbuf->Reset(string(rover_data));
        if (p->verb) printf("SEND: angle=%0.4f, speed=%0.4f as UDP DATA: %s, on address: %d.%d.%d.%d:%d\n",angle, speed, rover_data,p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port);
    }
    else if (p->robot_id == QUAD){
        if (p->debug) printf("PRETENDING TO SEND DATA: angle=%d, speed=%d\n", angle2quad(angle),speed2quad(speed));
        char first = 0x01;
        sprintf(quad_data,"%c%c%c", first,(char) speed2quad(speed), (char) angle2quad(angle));
        commbuf->Reset(string(quad_data));
        if (p->verb) printf("SEND: angle=%0.4f, speed=%0.4f as UDP DATA: %s, on address: %d.%d.%d.%d:%d\n",angle, speed, quad_data,p->net.ip1,p->net.ip2,p->net.ip3,p->net.ip4,p->net.port);
        if (p->debug) write_to_file(fptr, quad_data);
    }
    else{
        printf("Warning: the robot you selected has no sending functions yet!\n");
        exit(EXIT_FAILURE);
    }

    return 0;
}

int Robot::init_socket_udp(net::Socket *socPtr, int port)
{
    printf( "creating socket on port %d\n", port );

    if ( !socPtr->Open( port ) )
    {	printf( "failed to create socket!\n" );
	    return 1;
    }

  return 0;
}


/*int Robot::init_comms(Params *p)
{
    if (p->robot_id == ROVER){
        printf( "creating socket on port %d\n", p->net.port );

        if ( !socPtr->Open( p->net.port ) )
        {	printf( "failed to create socket!\n" );
                return 1; 	}
    }
    else{
        printf("Warning: the robot you selected has no sending functions yet!\n");
        return 1;
    }
    //p->network=1;

    return 0;
}*/

int Robot::shutdown(Params *p)
{
    if (p->robot_id == DUMMY){
        printf("BYE FROM DUMMY!\n");
    }
    else if (p->robot_id == ROVER){
        printf("BYE FROM ROVER!\n");
        net::ShutdownSockets();
    }
    else if (p->robot_id == QUAD){
        printf("BYE FROM QUAD!\n");
        net::ShutdownSockets();
    }
    else if (p->robot_id == VISAR){
    /*
    // Shutdown and tidy up FOR VISAR with player OS
    playerc_position2d_unsubscribe(position2d);
    playerc_position2d_destroy(position2d);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);
    */
    }
    else{
        printf("Warning: the robot you selected has no SHUTDOWN functions yet!\n");
        exit(EXIT_FAILURE);
    }
}

int Robot::write_to_file(FILE *fptr, char *line)
{
    fptr = fopen("log.txt", "a");
    if (fptr == NULL) {
        printf("I couldn't open results.dat for writing.\n");
        exit(0);
    }

    fprintf(fptr, "%s\n", line);

    fclose(fptr);

    return 0;
    //return int(round(angle*(180/3.14159)+50));
}

//speed range -1-+1
//steering range = degrees -90to90
//scaled = ((mtx - MIN)./RANGE).*(MAXVAL-MINVAL) + MINVAL;

int Robot::angle2quad(double angle)
{
    int exaggeration_multiplier=3;
    return int(round( (( (angle*exaggeration_multiplier) -(-90))/180)*(0-(127))+(127) ) );
    //return int(round(angle*(180/3.14159)+50));
}

int Robot::speed2quad(double speed)
{
    //scaled = ((mtx - MIN)./RANGE).*(MAXVAL-MINVAL) + MINVAL;
    int offset = 0;
    int scale = 5;
    return int(round((((speed-(-1))/2)*(94-(-94))+(-94))*scale + offset) );
}

int Robot::angle2rover(double angle)
{
    return int(round(50-angle));
}

int Robot::speed2rover(double speed)
{
    //scaled = ((mtx - MIN)./RANGE).*(MAXVAL-MINVAL) + MINVAL;
    int offset = 0;
    int scale = 7;
    return int(round(((( speed * scale + offset)-(-1))/2)*(75-25)+25));
}
