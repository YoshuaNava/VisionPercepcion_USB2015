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

#ifndef INIT_STRUCTURES
#define INIT_STRUCTURES

#include <iostream>
#include <string>
#include "Net.h" //header for UDP transmission
#include "ms_overwrite_safe_buffer.h"

using namespace std;

# define N 6
/* boundary structure definition */
struct boundary {
  
    IplImage *Bimg;
    CvPoint *BOUND;
    CvPoint2D32f *POLAR;
    CvPoint2D32f *CART;
    CvPoint *DISPLAY_CART;

    CvPoint3D32f *FRONTIERS;
    
    CvPoint Robot;
    CvPoint2D32f F;
    
    //steering direction
    int min_distance;
    double min_angular_separation;
    CvPoint3D32f *Median_Angle; 
    
    double display_scale_factor;
    
    int best_segment;
    double widest_angle;
    
    CvPoint2D32f D_Theta; 

};
typedef struct boundary Boundary; 
//typedef is used here to be able to declare a "struct boundary" by just using "Boundary"


struct statistics {
  
    int *id;
    int *size;
    int *gray_id;
    CvScalar* mean;
    CvScalar* stdDev;
    CvRect* box; 
    int no_features;


    double *P_Gt;
    double *P_Gf;
    IplImage* prior_img;

    double *P_FgGt, *P_FgGf;

    double *P_GtgF;//posterior
    double *P_GfgF;

    double *G_score;

    int nos; //nuber of segments
    int img_w;
    int img_h;

    double *L1, *L0;
    double *Z1, *Z0, *gmax;

    CvHistogram *H_SF[N];
    CvHistogram *H_G1[N], *H_G1_DISP[N];
    CvHistogram *H_G0[N], *H_G0_DISP[N];

};
typedef struct statistics Statistics;


struct model {

    CvRect* box; //safe area
    IplImage* mask; //safe area
    
    //Basic Stats
    CvScalar* mean;
    CvScalar* stdDev;

    //Edge Stats
    //CvHistogram *HegoM, *HegmM;
    
    //Colour Stats
    //CvHistogram *HhueM, *HsatM;
    
    //LBP Stats
    //CvHistogram *HlbpM;
    CvHistogram *H_M[N],*H_M_DISP[N];
    int *dim;

};
typedef struct model Model;


struct features {

    //Edges
    IplImage *mag, *ang32, *P_ang;

    //Colour
    IplImage *hsv, *lab, *YCrCb;
    IplImage *hue, *sat, *val; 
    IplImage *Cr, *a, *Cb, *iic;
    IplImage *P_hue, *P_sat, *P_val;

    //LBP
    IplImage *lbp, *P_lbp;

    //Posterior
    IplImage *post0, *post1, *post_ratio, *P_X1[5], *P_X0[5];

    //Classification
    IplImage *bin_class_result;

};

typedef struct features Features;


struct socketAdd {
  
    //networking
    int port;
    int ip1, ip2, ip3, ip4;
  
};
typedef struct socketAdd SocketAdd;


struct camCalib {
  
    //structure collecting the parameters related to the robot camera configuration
    double height_from_ground;
    double fy, fx;
    double tilt_angle, pan_angle;
    double cx, cy;
    double pix_size_x, pix_size_y;
  
};
typedef struct camCalib CamCalib;


struct botCalib {
  
    //structure collecting the parameters related to the robot configuration
    double min_dist_to_obst;
    double min_ang_between_obst;
    double max_depth_threshold;
    double display_scale_factor;
};
typedef struct botCalib BotCalib;

class Params
{ 
public:
  
    //FUNCTIONS
    Params(); //default constructor  
    int init_specific_robot(); //initialise variables depending on user input
    int parse_cmd_options(int argc, char **argv); 

    //VARIABLES
    //capture
    char video_capture_source[150];
    char image_capture_source[150];
    int capture_W; //capture resolution
    int capture_H;
    CvSize capture_size;

    double proc_W; //processing resolution
    double proc_H;

    //bool flags
    bool disp_img; //flag to show all windows
    bool refresh; //refresh the window names and positions
    bool verb; //verboinitiase
    bool debug; //debug 
    int debug_delay_ms;
    bool move; //robot use motors (used for debugging)
    bool write_output_to_disk; //variable to control writing intermediate images to disk 
    bool robot; //using robot or not
    bool network;
    bool video_data; 
    bool dynamic; 
    bool webcam; 
    //networking
    SocketAdd net;
    CamCalib camera; 
    BotCalib bot;
    //visual output
    int log_post_thres_zero_position;

    const static string robot_names[5];
    //static string robot_name[1];
    string robot_name;
    int robot_id;
    //[15][30] =
    //{{"Cloudy_Dry"},{"Cloudy_Wet"},{"Cloudy_Muddy"},{"Sunny_Wet"},{"Complex_Scene"},{"Fence"},{"Shadows"},{"Snow"},{"DS1A"},{"DS1B"},{"DS2A"},{"DS2B"},{"DS3A"},{"DS3B"},{"internet_indoor"}};	

private:
  
  //http://stackoverflow.com/questions/865668/parse-command-line-arguments
  char* getCmdOption(char ** begin, char ** end, const std::string & option); //parse command line options
  bool cmdOptionExists(char** begin, char** end, const std::string& option); //parse command line option
  
  
};


class Robot
{
  
public:
    Robot(); //default constructor
    int send2robot(Params *p, net::Socket *socPtr, double angle, double speed);
    int send2robot(Params *p, OwSafeBuffer<string> *commbuf, double angle, double speed);
    //int init_comms(Params *p); //seg fault
    int shutdown(Params *p);
    int init_socket_udp(net::Socket *socPtr, int port);

    //net::Socket *socketPtr, int port);
  
private: 
    //functions for rover
    int angle2rover(double angle);
    int speed2rover(double speed);
    char rover_data[7]; // = "19s20t"; //variable to hold string to control speed and orientation of robot platform
    
    //functions for quad
    int angle2quad(double angle);
    int speed2quad(double speed);
    char quad_data[4];
    //net::Socket soc;
    //net::Socket *socPtr;
    
    int write_to_file(FILE *fptr, char *line);
    FILE *fptr;
};

void init_stats(CvSize Img_Size, Statistics * S, bool init);
void init_boundary(CvSize S, Boundary * B);
void init_model(CvSize S, CvRect SafeRegion, Model *M);
void init_features(CvSize S, Features * F);

#endif