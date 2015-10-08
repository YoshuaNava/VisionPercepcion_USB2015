 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "image_transport/image_transport.h"
 #include "sensor_msgs/image_encodings.h"
 #include "cv_bridge/cv_bridge.h"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/video/tracking.hpp"
 #include "opencv2/core/core.hpp"
 #include "iostream"
 #include "stdio.h"
 #include "stdlib.h"
 #include "math.h"
 #include "string.h"
 #include "iomanip"


using namespace std;
using namespace cv;

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%d
%%%%%%%%%% VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Variables Globales %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

Mat src,nueva,image_pub;


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% FUNCION DE DRAWING PARA DIBUJAR LOS MOVIMIENTOS   %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    /*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% DECLARACION DE VARIABLES A UTILIZAR EN LA FUNCION %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    */
    system("cls");

    /*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%    LOOP PRINCIPAL   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    */
    for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
        {
           const Point2f& fxy = flow.at<Point2f>(y, x);
           line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x*4), cvRound(y+fxy.y*4)),color);
           circle(cflowmap, Point(x,y), 2, color, -1);
        } 
    system("pause");
}


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%      TRACKBAR       %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/ 
int cvCreateTrackbar
(
   const char* trackbar_name,
   const char* window_name,
   int* posicion,
   int count,
   CvTrackbarCallback on_change
);
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/ 

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_pub=cv_ptr->image;
        //image_pub=cv_bridge::toCvShare(msg, "bgr8")->image;
        //imshow("view",image_pub);
        // imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //  waitKey(30);       

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            MAIN           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
int main( int argc, char** argv ) 
{
    ros::init(argc, argv, "Object_Detection_Publisher");
    ros::NodeHandle nh;
    namedWindow("view");
    startWindowThread();
    image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
    CvScalar s; 


      namedWindow( "Imagen", 1 );
      namedWindow( "Controles_W", 1 );
      namedWindow( "Colores", 1 );
      namedWindow( "template", 1 );

    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
    Mat prevgray, gray, flow, cflow, frame;
      VideoCapture cap(0);
    while(nh.ok())
    {
        //cout << a << endl;
        
        if (image_pub.rows==0)
        {
 
        }
        else
        {
            imshow("view",image_pub);
            // imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
            waitKey(30);   
   
    
    //cap >> frame;
    
    frame=image_pub;
    IplImage* nueva;
    nueva=cvCreateImage(cvSize(frame.cols,frame.rows),8,3);
    IplImage ipltemp=frame;
    cvCopy(&ipltemp,nueva);
     //cap >> frame;
    frame=image_pub;
     
    
    cvtColor(frame, gray, CV_BGR2GRAY);

    waitKey(1);


    int input=cvWaitKey(40);
    if ((char)input==32)
    {
         std::swap(prevgray, gray);
    }

      

          
            
     if( prevgray.data )
    {
      
        calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 30, 3, 5, 1.2, 0);     
        cvtColor(prevgray, cflow, CV_GRAY2BGR);
        drawOptFlowMap(flow, cflow, 14, 1.5, CV_RGB(0, 255, 0));
        imshow("Controles_W", cflow);
        waitKey(2);
                
    }
    

  
   IplImage* colores=NULL;//inicializo imagen

   colores=cvCreateImage(cvSize(frame.cols,frame.rows),IPL_DEPTH_8U,3);

     for(int y = 0; y < flow.rows; y=y+1)
    {
          for(int x = 0; x < flow.cols; x=x+1)
        	{
               const Point2f& fxy = flow.at<Point2f>(y, x);
               s=cvGet2D(nueva,y,x);            
               if (abs(fxy.x) > 2 || abs(fxy.y) > 2)
               {
                    s.val[0]=0;
                    s.val[1]=0;
                    s.val[2]=255;
                    //	cout << fxy.x << endl;
                    
                    
                }        
                
                cvSet2D(colores,y,x,s);
                /*
                s=cvGet2D(colores,y,x);
               if (abs(fxy.x) > 5)
               {
                    cout << "*****" << s.val[2] << endl;
                    ES BGR
                }  
                */
        	}
     }

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%   Object Detection  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/ 
int valora,valorb,valorc;
int matriz[flow.cols][flow.rows];
      for(int y = 0; y < flow.rows; y=y+1)
    {
          for(int x = 0; x < flow.cols; x=x+1)
        	{
             
               s=cvGet2D(colores,y,x); 	
               	      //cout << s.val[2] << endl;  
               	      valora=s.val[0]; 
               	      valorb=s.val[1]; 
               	      valorc=s.val[2]; 
               if (valora==0 && valorb==0 && valorc==255)
               {
               matriz[x][y]=1;
               }  
               else
              {
               matriz[x][y]=0;
               }              
                       
        	}
     }
     
   IplImage* datos_del_objeto=NULL;//inicializo imagen

   datos_del_objeto=cvCreateImage(cvSize(frame.cols,frame.rows),IPL_DEPTH_8U,3);
      for(int y = 0; y < flow.rows; y=y+1)
    {
          for(int x = 0; x < flow.cols; x=x+1)
        	{
               
               s=cvGet2D(nueva,y,x);            
               if (matriz[x][y]==1)
               {
                    s.val[0]=0;
                    s.val[1]=0;
                    s.val[2]=255;
                    }
                    else
                    {
                    
                    s.val[0]=0;
                    s.val[1]=0;
                    s.val[2]=0;
                }        
                
                cvSet2D(datos_del_objeto,y,x,s);
                             
        	}
     }
         
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%      Ventana        %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/   

int x_derecha=0, x_izquierda=3000, y_superior=0, y_inferior=0,bandera_y_superior=0;


      for(int y = 0; y < flow.rows; y=y+1)
    {
          for(int x = 0; x < flow.cols; x=x+1)
        	{
               
                        
               if (matriz[x][y]==1)
               {
               
                  if(bandera_y_superior==0)
                  {
                      y_superior=y;
                       bandera_y_superior=1;
                  }
                
                  if(x_izquierda>x)
                  {
                       x_izquierda=x;
                  }
                  
                  if(x_derecha<x)
                  {
                       x_derecha=x;
                  }
                    
                  if(y_inferior<y)
                  {
                       y_inferior=y;
                  }
                
                }        
                                             
        	}
     }

      for(int y = 0; y < flow.rows; y=y+1)
    {
          for(int x = 0; x < flow.cols; x=x+1)
        	{
               
               s=cvGet2D(nueva,y,x);            
               if (x>=x_izquierda && x<=x_derecha && y_superior==y)
               {
                    s.val[0]=75;
                    s.val[1]=248;
                    s.val[2]=251;
                    cvSet2D(datos_del_objeto,y,x,s);
                    }
                   
                 if (x>=x_izquierda && x<=x_derecha && y_inferior==y)
               {
                    s.val[0]=75;
                    s.val[1]=248;
                    s.val[2]=251;
                    cvSet2D(datos_del_objeto,y,x,s);
                    }
                  
                  if (y>=y_superior && y<=y_inferior && x_derecha==x)
               {
                    s.val[0]=75;
                    s.val[1]=248;
                    s.val[2]=251;
                    cvSet2D(datos_del_objeto,y,x,s);
                  }
                    
                  if (y>=y_superior && y<=y_inferior && x_izquierda==x)
               {
                    s.val[0]=75;
                    s.val[1]=248;
                    s.val[2]=251;
                    cvSet2D(datos_del_objeto,y,x,s);
                    }  
                
                    
                }       
                
                
                             
        	}
     

     waitKey(1);

     
     if(waitKey(30)>=0)
     break;
     std::swap(prevgray, gray);
     waitKey(1);
     cvShowImage("Colores",colores);
     cvShowImage("template",datos_del_objeto);
 
   //cvShowImage("Colores",nueva);
}
    
    ros::spinOnce();
    } 
    
    cap.release();
    destroyAllWindows();
    cvDestroyWindow( "Imagen" );
   //ros::spin();

    return 0;
}
