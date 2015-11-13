/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dimitrios Kanoulas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dimitrios Kanoulas nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pcl/common/time.h>
#include "patch.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

/** \brief Main */
int
main (int argc, char** argv)
{
  VectorXd d(2), k(2), r(3), c(3);
  d << 1.5, 1.0;
  k << -1.0, -2.0;
  r << 0.0, 0.0, 0.0;
  Patch p ("elliptic paraboloid", Patch::s_e, Patch::b_e,
           d, k, r, c);

  cout << "Creating x,y,z" << endl;
  VectorXd x(61), y(61), z(61);
  x << 1.9045, 1.4386, 1.3764, 1.3604, 1.3745,  1.0866,  0.9881,  0.9465,  0.9376,  0.9395,  0.9761,  0.6765,  0.5987,  0.5617,  0.5517,  0.5524,  0.5740,  0.6896,  0.3116,  0.2423,  0.2011,  0.1823,  0.1829,  0.2094,  0.2946, -0.0269, -0.0928, -0.1412, -0.1710, -0.1710, -0.1335, -0.0514, -0.3492, -0.4127, -0.4666, -0.5039, -0.5034, -0.4566, -0.3719, -0.6624, -0.7222, -0.7772, -0.8159, -0.8131, -0.7614, -0.6764, -0.9730, -1.0263, -1.0775, -1.1120, -1.1054, -1.0534, -0.9732, -1.3310, -1.3741, -1.4008, -1.3904, -1.3418, -1.6762, -1.6934, -1.6803;
  
  y <<  0.1190, -0.8047, -0.4509,    -0.0380,    0.4291,    -1.1826,    -0.8742,    -0.5352,    -0.1292,    0.3340,    0.8109,    -1.2148,    -0.9178,    -0.5897,    -0.1883,    0.2799,    0.7434,    1.2510,    -1.2324,    -0.9415,    -0.6196,    -0.2221,    0.2486,    0.7086,    1.1730,    -1.2382,    -0.9483,    -0.6276,    -0.2329,    0.2339,    0.6910,    1.1411,    -1.2339,    -0.9405,    -0.6166,    -0.2239,    0.2338,    0.6862,    1.1356,    -1.2210,    -0.9206,    -0.5903,    -0.1988,   0.2478,    0.6953,    1.1535,    -1.2008,    -0.8904,    -0.5520,    -0.1605,    0.2763,    0.7218,    1.1996,    -0.8513,    -0.5033,    -0.1103,    0.3210,    0.7708,    -0.4443,    -0.0468,    0.3857;
  
  z << -0.8396,    -0.6373,    -0.2910,    -0.1542,    -0.2774,    -0.7802,    -0.2865,    0.0313,    0.1890,    0.1353,    -0.2114,    -0.5419,    -0.0988,    0.2066,    0.3790,    0.3563,    0.0848,    -0.6087,    -0.4270,    -0.0123,    0.2846,    0.4640,    0.4576,    0.2176,    -0.3297,    -0.3981,    -0.0012,    0.2864,    0.4628,    0.4626,    0.2408,    -0.2494,    -0.4393,    -0.0533,    0.2235,    0.3887,    0.3865,    0.1759,    -0.2943,    -0.5445,    -0.1640,    0.1020,    0.2517,    0.2415,    0.0329,    -0.4463,    -0.7152,    -0.3340,    -0.0771,    0.0562,    0.0328,    -0.1874,    -0.7135,    -0.5702,    -0.3191,    -0.2019,    -0.2451,    -0.4979,    -0.6379,    -0.5377,    -0.6124;

  PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(x.size());
  for (int i=0; i<x.size(); i++)
  {
    cloud->points[i].x = x(i);
    cloud->points[i].y = y(i);
    cloud->points[i].z = z(i);
  }
  cloud->width = x.size();
  cloud->height = 1;
  
  cout << "calling computeResidual" << endl;
  p.computeResidual (cloud);
}
