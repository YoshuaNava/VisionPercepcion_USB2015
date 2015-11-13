#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>

#include <string>
using std::string;
#include "../src/functions.h"
#include "../slic_mod/slic.h"

Slic slic;

using namespace std;
using namespace cv;

void ShowImages2(Mat src)
{
	imshow("Prueba_Prueba",src);  
}

void DrawSaveWindow(Mat frame,Mat& frame_out)
{
  frame_out= frame.clone();
  for(int y = 0; y < frame.rows; y=y+1)
  {
      for(int x = 0; x < frame.cols; x=x+1)
    	{
          if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows/4)*3)
          {
              Vec3b color = frame.at<Vec3b>(y, x);  
              color.val[0]=75;
              color.val[1]=248;
              color.val[2]=251;      
              frame_out.at<Vec3b>(y, x) = color;
          } 
         
          if (x>=(frame.cols/4) && x<=(frame.cols/4)*3 && y==(frame.rows-(frame.rows/12)))
          {
              Vec3b color = frame.at<Vec3b>(y, x);  
              color.val[0]=75;
              color.val[1]=248;
              color.val[2]=251;      
              frame_out.at<Vec3b>(y, x) = color;                  
          } 
          
          if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4))
          {
              Vec3b color = frame.at<Vec3b>(y, x);  
              color.val[0]=75;
              color.val[1]=248;
              color.val[2]=251;      
              frame_out.at<Vec3b>(y, x) = color;                              
          }
          
          if (y>=(frame.rows/4)*3 && y<=(frame.rows-(frame.rows/12)) && x==(frame.cols/4)*3)
          {
              Vec3b color = frame.at<Vec3b>(y, x);  
              color.val[0]=75;
              color.val[1]=248;
              color.val[2]=251;      
              frame_out.at<Vec3b>(y, x) = color;                            
          }
      }                               
  } 
}

void CreateSaveWindow(Mat frame,Mat& frame_window)
{
  int xx=0;
  int yy=0;
  int A=(((frame.cols/4)*3)-(frame.cols/4))-1;
  int B=((frame.rows-(frame.rows/12))-((frame.rows/4)*3))-1;
  frame_window=frame.clone();
  Size size(A,B);//the dst image size,e.g.AxB
  resize(frame_window,frame_window,size);//resize image
  for(int y = 0; y < frame.rows; y=y+1)
  {
      for(int x = 0; x < frame.cols; x=x+1)
    	{
          
          if (x>=(frame.cols/4)+1 && x<=(frame.cols/4)*3-1 && y>=((frame.rows/4)*3)+1 && y<=(frame.rows-(frame.rows/12)-1))
          {
           Vec3b color = frame.at<Vec3b>(y, x); 
           frame_window.at<Vec3b>(yy, xx) = color;
           xx=xx+1;
           if(xx>((frame.cols/4)*3)-(frame.cols/4)-2)
           {
           xx=0;
           yy=yy+1;
           } 
          }                          
      }                               
  } 
}

void CreateWindowCluster(Mat Cluster,Mat& Window_Cluster)
{
  int xx=0;
  int yy=0;
  int A=(((Cluster.cols/4)*3)-(Cluster.cols/4))-1;
  int B=((Cluster.rows-(Cluster.rows/12))-((Cluster.rows/4)*3))-1;
  Window_Cluster=Cluster.clone();
  Size size(A,B);//the dst image size,e.g.AxB
  resize(Window_Cluster,Window_Cluster,size);//resize image


  for(int y = 0; y < Cluster.rows; y=y+1)
  {
      for(int x = 0; x < Cluster.cols; x=x+1)
	    {
          
          if (x>=(Cluster.cols/4)+1 && x<=(Cluster.cols/4)*3-1 && y>=((Cluster.rows/4)*3)+1 && y<=(Cluster.rows-(Cluster.rows/12)-1))
          {
             
             Vec3b color = Cluster.at<Vec3b>(y, x); 
             Window_Cluster.at<Vec3b>(yy, xx) = color;
             xx=xx+1;
             if(xx>((Cluster.cols/4)*3)-(Cluster.cols/4)-2)
             {
                 xx=0;
                 yy=yy+1;
             } 
          } 
          
       
      }                               
  } 
}

void Histogram(cv::Mat src, cv::Mat& histred, cv::Mat& histgreen, cv::Mat& histblue,float& R_max_x,float& R_max_y,float& G_max_x,float& G_max_y,float& B_max_x,float& B_max_y){
             
  Mat dst;
  float b_max[2];
  float g_max[2];
  float r_max[2];
  
  b_max[1]=400;
  
  g_max[1]=400;
  
  r_max[1]=400;
  
  vector<Mat> bgr_planes;
  split( src, bgr_planes );

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
  Mat histBlue( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
  Mat histGreen( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
  Mat histRed( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  



  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      line( histBlue, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
                       
     if (i>1)
       {
         if ((hist_h - cvRound(b_hist.at<float>(i))) < b_max[1]) 
          {
             b_max[0]=i;
             b_max[1]=hist_h - cvRound(b_hist.at<float>(i));
          }
        }
                       
      line( histGreen, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
                       
      if (i>1)
       {
         if ((hist_h - cvRound(g_hist.at<float>(i))) < g_max[1]) 
          {
             g_max[0]=i;
             g_max[1]=hist_h - cvRound(g_hist.at<float>(i));
          }
        }
        
      line( histRed, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
                       
      if (i>1)
       {
         if ((hist_h - cvRound(r_hist.at<float>(i))) < r_max[1]) 
          {
             r_max[0]=i;
             r_max[1]=hist_h - cvRound(r_hist.at<float>(i));
          }
        }
                 
  }
       line( histBlue, Point( bin_w*b_max[0], 400)  ,
                           Point( bin_w*b_max[0], b_max[1] ),
                           Scalar( 13, 230, 250), 2, 8, 0  );
                           //imshow("Blue_hist",histBlue);
                          
       line( histGreen, Point( bin_w*g_max[0], 400)  ,
                           Point( bin_w*g_max[0], g_max[1] ),
                           Scalar( 13, 230, 250), 2, 8, 0  );
                           //imshow("Green_hist",histGreen);
                           
       line( histRed, Point( bin_w*r_max[0], 400)  ,
                           Point( bin_w*r_max[0], r_max[1] ),
                           Scalar( 13, 230, 250), 2, 8, 0  );
                           //imshow("Red_hist",histRed);                                             
  //cout << "b_max: " << b_max[1] << endl;
  histred=histRed;
  histblue=histBlue;
  histgreen=histGreen;
  
  B_max_x=b_max[0];
  B_max_y=b_max[1];
  G_max_x=g_max[0];
  G_max_y=g_max[1];
  R_max_x=r_max[0];
  R_max_y=r_max[1];     
 
}

void CompareCluster(cv::Mat src,cv::Mat Cluster,cv::Mat& result)
{

  result=src.clone();
  
  Mat img_out1,img_out2,img_out_pixels;
  int xx,yy,r,g,b,bb,rr,gg,r_seed,g_seed,b_seed;
  int superpixels_x, superpixels_y;  
  Vec3b color,color_seed;
  Mat img_out_seed,img_out_seed2;
  
  slic.export_superpixels_data(0,Cluster,img_out1,img_out_pixels,superpixels_x,superpixels_y,img_out_seed); 
  int init_val_y=(superpixels_y-(img_out_pixels.rows/2))+2;
  int final_val_y=(superpixels_y+(img_out_pixels.rows/2))+2;
  int init_val_x=(superpixels_x-(img_out_pixels.cols/2))+2;
  int final_val_x=(superpixels_x+(img_out_pixels.cols/2))+2;
  
  if (init_val_x<0)
  {
  init_val_x=0;
  }
  if (final_val_x>=result.cols)
  {
  final_val_x=final_val_x-3;
  }
  if (init_val_y<0)
  {
  init_val_y=0;
  }
  if (final_val_y>=result.rows)
  {
  final_val_y=final_val_y-3;
  }

 
  for(int i = 1; i < 35; i=i+1)
  {
      color = img_out1.at<Vec3b>(5, 5); 
      b=color.val[0];
      g=color.val[1];
      r=color.val[2];
      
      color_seed = img_out_seed.at<Vec3b>(img_out_seed.rows/2, img_out_seed.cols/2); 
      b_seed=color_seed.val[0];
      g_seed=color_seed.val[1];
      r_seed=color_seed.val[2];
      
      //imshow("Seed_Image",img_out_seed);
      //cout << b << ";" << g << ";" << r << ";" << b_seed << ";" << g_seed << ";" << r_seed << endl;
      
      if((b<=b_seed+30 && b>=b_seed-30) && (g<=g_seed+30 && g>=g_seed-30) && (r<=r_seed+30 && b>=r_seed-30))
      {
          xx=0;
          yy=0;
  
          for(int y = init_val_y; y < final_val_y; y=y+1)
          {
              for(int x = init_val_x; x < final_val_x; x=x+1)
              {
         
                      Vec3b color = img_out_pixels.at<Vec3b>(yy, xx); 
                 
                       int bb=color.val[0] ;
                       int gg=color.val[1] ;
                       int rr=color.val[2]; 
                       
                      //if(y>(result.rows/2)+15)
                       //{
                          if(bb==0 && gg==0 && rr==0)
                          {
                          } 
                          else
                          {
                                        
                               color.val[0]=0;
                               color.val[1]=0;
                               color.val[2]=0; 
                               result.at<Vec3b>(y, x) = color; 
                               
                               if(y+1<result.rows && x+1<result.cols && x-1>0 && y-1>0)
                               {
                                   for(int hy=y-1;hy<(y-1)+3;hy=hy+1)
                                   {
                                       for(int hx=x-1;hx<(x-1)+3;hx=hx+1)
                                       {
                                       result.at<Vec3b>(hy, hx) = color;
                                        }  
                                    }
                                
                               }
                                                  
                           }
                       // }
                               
                        xx=xx+1 ;
                                   
              }
              xx=0;
              yy=yy+1;                          
          }
      }
      
      slic.export_superpixels_data(i,Cluster,img_out1,img_out_pixels,superpixels_x,superpixels_y,img_out_seed2);
      init_val_y=(superpixels_y-(img_out_pixels.rows/2))+2;
      final_val_y=(superpixels_y+(img_out_pixels.rows/2))+2;
      init_val_x=(superpixels_x-(img_out_pixels.cols/2))+2;
      final_val_x=(superpixels_x+(img_out_pixels.cols/2))+2;
      
      if (init_val_x<0)
      {
      init_val_x=0;
      }
      if (final_val_x>=result.cols)
      {
      final_val_x=final_val_x-5;
      }
      if (init_val_y<0)
      {
      init_val_y=0;
      }
      if (final_val_y>=result.rows)
      {
      final_val_y=final_val_y-5;
      }

    }
}

/*
void CompareHistograms(Mat frame,Double Valor_hist,Mat& img_out)
{
 
  Mat ventana=frame;
  Mat img_out=frame;
  Size size(79,19);//the dst image size,e.g.100x100
  resize(ventana,ventana,size);//resize image
  
  int yy=0;
  int xx=0;
  int vent_x=0;
  int vent_y=0;
  
  for(int y = 0; y < frame.rows; y=y+20)
      {
        for(int x = 0; x < frame.cols; x=x+80)
      	{ 
              for(int yy = y; yy < y+19; yy=yy+1)
              {
                for(int xx = x; xx < x+79; xx=xx+1)
              	{
         
                           Vec3b color = frame.at<Vec3b>(yy, xx);           
                            ventana.at<Vec3b>(vent_y, vent_x) = color;
                            
                            vent_x=vent_x+1;
                            
                                                         
                       
                }
                vent_x=0;   
                vent_y=vent_y+1;                           
              }
              
              vent_y=0; 
              vent_x=0;
              Mat hist_save_window_1=ventana;
              Mat hist_save_window_2;
              Histogram(hist_save_window_1,hist_save_window_2);
              Comparison(model,img1,hist_save_window_1);

              cout << valor_hist << endl;
            
              if(valor_hist>0.1)              
              {
                for(int yy = y; yy < y+19; yy=yy+1)
                  {
                for(int xx = x; xx < x+79; xx=xx+1)
              	   {
         
                           Vec3b color = frame.at<Vec3b>(yy, xx);  
                          color.val[0]=255;
                          color.val[1]=255;
                           color.val[2]=255;      
                            img_out.at<Vec3b>(yy, xx) = color;
                            
                           
                            
                                                         
                       
                     }                          
                  }
              }
              
      }                               
    }
}
*/

