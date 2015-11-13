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

#include <string>
using std::string;
#include "../src/functions2.h"



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

