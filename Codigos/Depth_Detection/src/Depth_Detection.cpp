#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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
double pz2=1000;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 
  void callback(const PointCloud::ConstPtr& msg)
   {
   
    //
   //  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
   // if (pt.x==200)
    if (pz2>pt.z)
    {
    pz2=pt.z;
    cout << pt.z << endl;
    waitKey(100);
    }
  
    
  //    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
      
   }
  
   int main(int argc, char** argv)
   {
  ros::init(argc, argv, "sub_pcl");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
   ros::spin();
  }
