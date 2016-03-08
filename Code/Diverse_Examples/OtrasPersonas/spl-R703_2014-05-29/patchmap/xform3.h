/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013, Dimitrios Kanoulas
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#ifndef PCL_XFORM3_H_
#define PCL_XFORM3_H_

#include <pcl/common/eigen.h>

using namespace Eigen;

namespace pcl 
{
  /** \brief applies the rigid body transform (r,t) to 3D points
   *  \author Dimitrios Kanoulas
   */
  class Xform3
  {
    public:
      /** \brief Empty constructor */
      Xform3 () {};

      /** \brief Empty destructor */
      ~Xform3 () {};

      inline void
      transform(MatrixXd &x, MatrixXd &y, MatrixXd &z,
                Matrix3d R, Vector3d t, bool invert, bool rotonly)
      {
        //Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
        Vector3d pt;
        for (int i=0; i<x.rows(); i++)
        {
          for (int j=0; j<x.cols(); j++)
          {
            pt << x(i,j), y(i,j), z(i,j);
            
            if (rotonly)
            {
              if (invert)
                pt = R.adjoint()*pt;
              else
                pt = R*pt;
            }
            else
            {
              if (invert)
                pt = R.adjoint()*(pt-t);
              else
                pt = R*pt+t;
            }
            
            x(i,j) = pt(0);
            y(i,j) = pt(1);
            z(i,j) = pt(2);
          }
        }
      }
  };
}

#endif // PCL_XFORM3_H_
