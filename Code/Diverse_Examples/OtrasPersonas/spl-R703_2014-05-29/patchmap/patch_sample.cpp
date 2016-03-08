/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-, Dimitrios Kanoulas
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
 */

// SPL headers
#include "patch_sample.h"

typedef Matrix<float, Dynamic, Dynamic> MatFlt;
typedef Matrix<float, Dynamic, Dynamic, RowMajor> MatFltRow;

///////////////////////////////////////////////////////////////////////////////
void
PatchSample::findSamples(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
                         double cx, double cy, double cz)
{
  // reset the cloud
  cloud->points.resize(0);

  // transform input data to local frame if necessary
  if (world_frame_)
  {
    pcl::Xform3 pose;
    pose.transform(mx, my, mz, p_.rexp(p_.getR()), p_.getC(), true, true); // invert, rotonly
    MatrixXd cxMat(1,1), cyMat(1,1), czMat(1,1);
    cxMat << cx;
    cyMat << cy;
    czMat << cz;
    //TBD: wrong p.c or p.r
    pose.transform(cxMat, cyMat, czMat, p_.rexp(p_.getR()), p_.getC(), true, false); // invert
    cx = cxMat(0,0);
    cy = cyMat(0,0);
    cz = czMat(0,0);
  }
  // calculate ray intersections
  r_  = p_.ri(mx, my, mz, cx, cy, cz);
  
  // create cloud
  MatrixXd x (mx.rows(),mx.cols());
  x = (mx.array() * r_.array()) + cx;
  MatrixXd y (my.rows(),my.cols());
  y = (my.array() * r_.array()) + cy;
  MatrixXd z (mz.rows(),mz.cols());
  z = (mz.array() * r_.array()) + cz;
  
  // clean them up
  MatrixXd bl_mat;
  p_.bl(x, y, bl_mat);
  
  for (int i=0; i<x.rows(); i++)
  {
    for (int j=0; j<x.cols(); j++)
    {
      if (bl_mat(i,j)>0.0)
      {
        x(i,j) = std::numeric_limits<float>::quiet_NaN();
        y(i,j) = std::numeric_limits<float>::quiet_NaN();
        z(i,j) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  pcl::PointXYZ point;
  for (int i=0; i<x.rows(); i++)
  {
    for (int j=0; j<x.cols(); j++)
    {
      point.x = x(i,j);
      point.y = y(i,j);
      point.z = z(i,j);
      cloud->points.push_back(point);
    }
  }
  cloud->width = x.cols();
  cloud->height = x.rows();
  cloud->is_dense = false;
}

///////////////////////////////////////////////////////////////////////////////
RowVectorXd
PatchSample::vectorizeColWise (MatrixXd &m)
{
  RowVectorXd v(m.rows()*m.cols());
  for (int i=0; i<m.cols(); i++)
  {
    for (int j=0; j<m.rows(); j++)
    {
      v(i*m.rows()+j) = m(j,i);
    }
  }

  return v;
}


///////////////////////////////////////////////////////////////////////////////
void
PatchSample::sample_frustum(double hfov, double vfov, int nh, int nv,
                            Vector3d p, Vector3d u,
                            MatrixXd &mx, MatrixXd &my, MatrixXd &mz)
{
  //TBD: check input args

  // pointing, up, left
  p = p/p.norm();
  u = u/u.norm();
  Vector3d l = u.cross(p);

  // sample like a camera would
  double ll = (tan(hfov/2.0)*((double)(nh-1)))/nh;
  double uu = (tan(vfov/2.0)*((double)(nv-1)))/nv;
  RowVectorXd y;
  y.setLinSpaced(nh,-ll,ll);
  MatrixXd yy;
  yy = y.replicate(nv,1);
  VectorXd z;
  z.setLinSpaced(nv,-uu,uu);
  MatrixXd zz;
  zz = z.replicate(1,nh);
  MatrixXd xx = MatrixXd::Ones(nv,nh);
  MatrixXd nn = (xx.array().square() + yy.array().square() + zz.array().square()).cwiseSqrt();
  xx = xx.array() / nn.array();
  yy = yy.array() / nn.array();
  zz = zz.array() / nn.array();

  // rotation matrix
  Matrix3d rr;
  rr << p, l, u;

  // rotate points
  MatrixXd xyz;
  MatrixXd cam (3,xx.rows()*xx.cols());
  cam.row(0) = vectorizeColWise(xx);
  cam.row(1) = vectorizeColWise(yy);
  cam.row(2) = vectorizeColWise(zz);
  xyz = rr*cam;

  // extract coordinates
  xx = xyz.row(0);
  yy = xyz.row(1);
  zz = xyz.row(2);
  mx = Map<MatrixXd>(xx.data(),nv,nh);
  my = Map<MatrixXd>(yy.data(),nv,nh);
  mz = Map<MatrixXd>(zz.data(),nv,nh);
}
