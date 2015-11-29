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


void LookingforGround(cv::Mat src,cv::Mat Cluster,cv::Mat green_line,double points_1[80],double points_2[80],double points_3[80],cv::Mat& result)
{

  result=src.clone();
  
  Mat img_out1,img_out2,img_out_pixels;
  int xx,yy,r,g,b,bb,rr,gg,r_seed,g_seed,b_seed;
  int superpixels_x, superpixels_y;  
  Vec3b color,red_base;
  Mat img_out_seed,img_out_seed2;
  int base_1,base_2,base_3;
  int value_y=0;
  
  slic.export_superpixels_data(0,img_out_pixels,superpixels_x,superpixels_y); 
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

 
  for(int i = 1; i < 80; i=i+1)
  {
            for(int loop_y = 0; loop_y < green_line.rows; loop_y=loop_y+1)
                	   {
                      red_base=green_line.at<Vec3b>(loop_y, superpixels_x); 
                      base_1=red_base[0];
                      base_2=red_base[1];
                      base_3=red_base[2];
 
                       
                       if ((base_1==0) && (base_2==255) && (base_3==0))
                        {
                       value_y=loop_y;
                     
                         }
                        
                          } 
                          if (superpixels_y>value_y)
                           {
      if( (points_1[i-1]<30) && (points_2[i-1]<30) && (points_3[i-1]<30) )
      
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
                       
                   
                          if(bb==0 && gg==0 && rr==0)
                          {
                          } 
                          else
                          {
                          
                          
        
               
         
 
                            
 
                        
                                    
                               color.val[0]=0;
                               color.val[1]=0;
                               color.val[2]=0; 
                               result.at<Vec3b>(y, x) = color; 
                              
                       
                                                  
                        }
                      xx=xx+1 ;
                                   
              }
                          xx=0;
              yy=yy+1;                       
          }
      }
      }
      slic.export_superpixels_data(i,img_out_pixels,superpixels_x,superpixels_y);
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

void CreateGreenLine(cv::Mat src,cv::Mat cdst,cv::Mat& blue_img)
{

 Mat red_line=src.clone();
 int init_point_x=0;
 int init_point_y=(src.rows/2)+20;
 int final_point_x,final_point_y;
 double rect_distance;
 double rect_distance_base=1000;
 Vec3b red_base;
 int base_1,base_2,base_3;

             for(int x = 12 ; x < cdst.cols ; x=x+1)
              {
                 
                  for(int y = 0; y < cdst.rows; y=y+1)
                	   {
                        red_base=cdst.at<Vec3b>(y, x); 
                        base_1=red_base[0];
                        base_2=red_base[1];
                        base_3=red_base[2];
                        
                        if ((base_1==0) && (base_2==0) && (base_3==255))
                          {
                              double value_a=(x-init_point_x)*(x-init_point_x);
                              double value_b=(y-init_point_y)*(y-init_point_y);
                              rect_distance=sqrt(abs(value_a)+abs(value_b));
                            
                              if(rect_distance<rect_distance_base)
                                {
                                    rect_distance_base=rect_distance;
                                    final_point_x=x;
                                    final_point_y=y;
                           
                                } 
                           }
                             
                       } 
                       
              line( red_line, Point(init_point_x, init_point_y),
              Point(final_point_x, final_point_y), Scalar(0,255,0), 3, 8 ); 
              init_point_x=final_point_x;
              init_point_y=final_point_y;  
              rect_distance_base=1000; 
                  } 
                  
             
  
    blue_img=red_line.clone();  
    bool flag=false;

           for(int x = 0 ; x < cdst.cols ; x=x+1)
              {
                 
                  for(int y = 0; y < cdst.rows; y=y+1)
                	   {
                        red_base=red_line.at<Vec3b>(y, x); 
                        base_1=red_base[0];
                        base_2=red_base[1];
                        base_3=red_base[2];
   
                          if (flag==true)
                            {
                                red_base=src.at<Vec3b>(y, x); 
                                red_base[2]=255;
                                blue_img.at<Vec3b>(y, x)=red_base;
                            }
                            
                          if ((base_1==0) && (base_2==255) && (base_3==0))
                             {
                                flag=true;                     
                             }
                          
                      }
                 flag=false;

              } 

} 
