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
#include "patch_fit.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

#define EPS std::numeric_limits<double>::epsilon()

////////////////////////////////////////////////////////////////////////////////
bool
sign(double x, double y)
{
  return ((x < 0.0 && y < 0.0) || (x > 0.0 && y > 0.0));
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setCloud (const PointCloudInPtr cloud)
{
  this->cloud_ = cloud;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setBt (Patch::b_type bt)
{
  this->bt_ = bt;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setBcp (double bcp)
{
  if (bcp>0.0 && bcp<=1.0)
  {
    this->bcp_ = bcp;
  }
  else
  {
    cerr << "bcp should be in (0,1]" << endl;
    this->bcp_ = 0.95;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setSSmax (int ssmax)
{
  if (ssmax >= 0)
    this->ssmax_ = ssmax;
  else
    cerr << "ssmax should be possitive." << endl;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setKtol (double ktol)
{
  if (ktol >= 0.0)
  {
    this->ktol_ = ktol;
  }
  else
  {
    cerr << "ktol must be nonnegative, o/w 0.1." << endl;
    this->ktol_ = fabs(ktol);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setCcon (bool ccon)
{
  this->ccon_ = ccon;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setMaxi (int maxi)
{
  if (maxi>=0)
    this->maxi_ = maxi;
  else
    cerr << "maxi should be nonnegative" << endl;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchFit::setMinrd (double minrd)
{
  if (minrd > 0)
    this->minrd_ = minrd;
  else
    cerr << "minrd should be positive" << endl;
}

////////////////////////////////////////////////////////////////////////////////
int
PatchFit::fit()
{
  // first check whether cloud subsampling is required
  RandomSample<PointXYZ> *rs;
  rs = new RandomSample<PointXYZ>();
  if (this->ssmax_)
  {
    rs->setInputCloud(cloud_);
    rs->setSample(ssmax_);
    rs->setSeed(rand()); 
    rs->filter(*cloud_); 
  }
  delete (rs);

  //remove NAN points from the cloud
  std::vector<int> indices;
  removeNaNFromPointCloud(*cloud_, *cloud_, indices); //is_dense should be false
 
  
  // Step 1: fit plane by lls
  Vector4d centroid;
  compute3DCentroid(*cloud_, centroid);
  p_->setC(centroid.head(3));

  cloud_vec.resize(cloud_->points.size(),3);
  fit_cloud_vec.resize(cloud_->points.size(),3);
  int vec_counter = 0;
  for (size_t i = 0; i<cloud_->points.size (); i++)
  {
    cloud_vec.row(vec_counter) = cloud_->points[i].getVector3fMap ().cast<double>();

    // for fitting sub the centroid
    fit_cloud_vec.row(vec_counter) = cloud_vec.row(vec_counter) - centroid.head(3).transpose();
    vec_counter++;
  }
  
  cloud_vec.conservativeResize(vec_counter-1, 3); //normal resize does not keep old values
  
  VectorXd b(fit_cloud_vec.rows());
  b.fill(0.0);

  Vector3d zl = lls(fit_cloud_vec,b);
  p_->setR(zh.cross(zl));
  double ss = p_->getR().norm();
  double cc = zh.adjoint()*zl;
  double t = atan2(ss,cc);
  double th = sqrt(sqrt(EPS))*10.0;
  double aa;
  if (t>th)
    aa = t/ss;
  else
    aa = 6.0/(6.0-t*t);
  p_->setR(aa*p_->getR());

  // Save the zl and the centroid p.c, for patch center constraint's use
  plane_n = zl;
  plane_c = p_->getC();


  // Step 2: continue surface fitting if not plane
  VectorXd a;

  if (st_a) // general paraboloid
  {
    if (ccon_)
    {
      a.resize(6);
      a << 0.0, 0.0, p_->getR(), 0.0;
    }
    else
    {
      a.resize(8);
      a << 0.0, 0.0, p_->getR(), p_->getC();
    }
    a = wlm(cloud_vec, a);

    // update the patch
    p_->setR(a.segment(2,3));
    if (ccon_)
      p_->setC(plane_c + a(5)*plane_n);
    else
      p_->setC(a.segment(5,3));
    Vector2d k;
    k << a(0), a(1);

    // refine into a specific paraboloid type
    if (k.cwiseAbs().maxCoeff() < this->ktol_)
    {
      // fitting planar paraboloid
      p_->setName("planar paraboloid");
      p_->setS(Patch::s_p);
      Matrix<double,1,1> k_p;
      k_p << 0.0;
      p_->setK(k_p);
      p_->setR(p_->rcanon2(p_->getR(),2,3)); //TBD not updating correctly
    }
    else if (k.cwiseAbs().minCoeff() < this->ktol_)
    {
      // fitting cylindric paraboloid
      p_->setName("cylindric paraboloid");
      p_->setS(Patch::s_y);
      p_->setB(Patch::b_r);
      kx = k(0);
      ky = k(1);

      if (fabs(kx) > fabs(ky))
      {
        // swap curvatures
        kx = k(1);
        ky = k(0);
        Matrix3d ww;
        ww << yh, -xh, zh;
        Matrix3d rr;
        rr = p_->rexp(p_->getR());
        rr = rr*ww;
        p_->setR(p_->rlog(rr));
      }
        
      Eigen::Matrix<double,1,1> k_c_p;
      k_c_p << ky;
      p_->setK(k_c_p);
    }
    else if (fabs(k(0)-k(1)) < this->ktol_)
    {
      // fitting circular paraboloid
      p_->setName("circular paraboloid");
      p_->setS(Patch::s_o);
      p_->setB(Patch::b_c);
      Matrix<double,1,1> k_c_p;
      k_c_p << k.mean();
      p_->setK(k_c_p);
      p_->setR(p_->rcanon2(p_->getR(),2,3));
    }
    else
    {
      if (sign(k(0),k(1)))
      {
        // fitting elliptic paraboloid
        p_->setName("elliptic paraboloid");
        p_->setS(Patch::s_e);
      }
      else
      {
        // fitting hyperbolic paraboloid
        p_->setName("hyperbolic paraboloid");
        p_->setS(Patch::s_h);
      }

      p_->setB(Patch::b_e);
      p_->setK(k);
      kx = k(0);
      ky = k(1);
      if (fabs(kx) > fabs(ky))
      {
        // swap curvatures
        Vector2d k_swap;
        k_swap << ky, kx;
        p_->setK(k_swap);

        Matrix3d ww;
        ww << yh, -xh, zh;
        Matrix3d rr;
        rr = p_->rexp(p_->getR());
        rr = rr*ww;
        p_->setR(p_->rlog(rr));
      }
    }
  }
  else
  {
    // plane fitting
    p_->setS(Patch::s_p);
    p_->setB(this->bt_);
    Matrix<double,1,1> k_p;
    k_p << 0.0;
    p_->setK(k_p);
  }

  
  // Step 5: fit boundary; project to local frame XY plane
  Xform3 proj;
  Matrix3d rrinv;
  rrinv = p_->rexp(-p_->getR());
  MatrixXd ux(cloud_vec.rows(),1), uy(cloud_vec.rows(),1), uz(cloud_vec.rows(),1);
  ux = cloud_vec.col(0);
  uy = cloud_vec.col(1);
  uz = cloud_vec.col(2);
  proj.transform(ux,uy,uz,p_->rexp(-p_->getR()), (-rrinv*p_->getC()),0,0);

  // (normalized) moments
  double mx = ux.mean();
  double my = uy.mean();
  double vx = (ux.array().square()).mean();
  double vy = (uy.array().square()).mean();
  double vxy = (ux.array() * uy.array()).mean();

  
  // Step 5
  lambda = sqrt(2.0) * boost::math::erf_inv(this->bcp_); 

  
  //STEP 6: cylindric parab: aa rect bound.  This step also sets p.c as the 1D
  // data centroid along the local frame x axis, which is the symmetry axis of
  // the cylinder.
  if (p_->getS() == Patch::s_y)
  {
    // fitting boundary: cyl parab, aa rect
    VectorXd d(2);
    d << lambda * sqrt(vx-mx*mx) , lambda * (sqrt(vy));
    p_->setD(d);
    
    p_->setC(p_->rexp(p_->getR())*mx*xh + p_->getC());
  }


  // STEP 7: circ parab: circ bound
  if (p_->getS() == Patch::s_o)
  {
    // fitting boundary: cir parab, circ
    VectorXd d(1);
    d << lambda * max(sqrt(vx),sqrt(vy));
    p_->setD(d);
  }


  // STEP 8: ell or hyp parab: ell bound
  if (p_->getS() == Patch::s_e || p_->getS() == Patch::s_h)
  {
    // fitting boundary: ell/hyb parab, ell
    VectorXd d(2);
    d << lambda * sqrt(vx), lambda * sqrt(vy);
    p_->setD(d);
  }


  // STEP 9: plane bounds
  if (p_->getS() == Patch::s_p)
  {
    // fitting boundary: plane
    Matrix3d rr = p_->rexp(p_->getR());
    p_->setC(rr*(mx*xh+my*yh) + p_->getC());

    double a = vx-mx*mx;
    double b = 2.0*(vxy-mx*my);
    double c = vy-my*my;

    lambda = -log(1.0-this->bcp_);
    double d = sqrt(b*b+(a-c)*(a-c));
    double wp = a+c+d;
    double wn = a+c-d;

    Vector2d l;
    l << sqrt(lambda*wp), sqrt(lambda*wn);

    VectorXd d_p(1);
    switch (p_->getB())
    {
      case Patch::b_c:
        d_p << l.maxCoeff();
        p_->setD(d_p);
        break;
      case Patch::b_e:
        p_->setD(l);
        break;
      case Patch::b_r:
        p_->setD(l);
        break;
      default:
        cerr << "Invalid pach boundary for plane" << endl;
    }
    
    if (p_->getB() != Patch::b_c)
    {
      double t = 0.5 * atan2(b,a-c);
      Matrix3d rr = p_->rexp(p_->getR());
      Vector3d xl = rr.col(0);
      Vector3d yl = rr.col(1);
      Vector3d zl = rr.col(2);
      xl = (cos(t)*xl) + (sin(t)*yl);
      yl = zl.cross(xl);
      Matrix3d xyzl;
      xyzl << xl, yl, zl;
      p_->setR(p_->rlog(xyzl));
    }
  } //plane boundary

  return (1);
}

////////////////////////////////////////////////////////////////////////////////
MatrixXd
PatchFit::parab (Matrix<double, Dynamic, 3> q, VectorXd a)
{   
  Vector3d k3, rv; 
  k3 << a(0), a(1), 0.0;
  rv << a.segment(2,3);
  
  MatrixXd rr; 
  rr = p_->rexp(rv);
  
  Vector3d c;
  if (ccon_)
    c = plane_c+a(5)*plane_n;
  else
    c = a.segment(5,3);
  MatrixXd qc(q.rows(),q.cols());

  for (int i=0; i<q.rows(); i++)
    qc.row(i) = q.row(i) - c.transpose();
  
  MatrixXd ql(q.rows(),3);
  ql = qc*rr;
  
  MatrixXd ql_sq(q.rows(),3);
  ql_sq = ql.array().square();

  return ((ql_sq * k3) - (2.0 * ql * zh));

  //TBD: enmo
}

////////////////////////////////////////////////////////////////////////////////
MatrixXd
PatchFit::parab_dfda (VectorXd a)
{
  Vector3d k3, rv;
  k3 << a(0), a(1), 0.0;
  rv = a.segment(2,3);
  
  MatrixXd rr;
  rr = p_->rexp(rv);

  vector<Matrix3d> drr (3);
  drr = p_->drexp(rv);

  Vector3d c;
  if (ccon_)
    c = plane_c + a(5)*plane_n;
  else
    c = a.segment(5,3);

  MatrixXd qc(cloud_vec.rows(),cloud_vec.cols());
  for (size_t i = 0; i<cloud_vec.rows(); i++)
    qc.row(i) = cloud_vec.row(i)-c.transpose();

  MatrixXd ql (qc.rows(),3);
  ql = qc*rr;

  MatrixXd j_full (cloud_vec.rows(),9);

  j_full = general_dfda(ql,rr,dfdql(ql,k3),dqldr(qc,drr));

  MatrixXd j (cloud_vec.rows(),8);
  for (int i=0; i<2; i++)
    j.col(i) = j_full.col(i);

  for (int i=3; i<9; i++)
    j.col(i-1) = j_full.col(i);

  MatrixXd m(8,6);
  m = Matrix<double, 8, 6>::Identity();
  m.block(5,5,3,1) = plane_n;
  if (ccon_)
    j = j*m;

  return j;
}

////////////////////////////////////////////////////////////////////////////////
Matrix<double,Dynamic,9>
PatchFit::general_dfda (MatrixXd ql, Matrix3d rr, MatrixXd dfdql, vector<MatrixXd> dqldr)
{
  MatrixXd dfdk (ql.rows(), ql.cols());
  dfdk = ql.array()*ql.array();

  MatrixXd dfdr (ql.rows(), ql.cols());
  dfdr.col(0) = (dfdql.array()*dqldr[0].array()).rowwise().sum();
  dfdr.col(1) = (dfdql.array()*dqldr[1].array()).rowwise().sum();
  dfdr.col(2) = (dfdql.array()*dqldr[2].array()).rowwise().sum();

  MatrixXd dfdc (ql.rows(), ql.cols());
  dfdc = -dfdql*rr.transpose();

  MatrixXd j (ql.rows(), 9);
  j << dfdk, dfdr, dfdc;

  return j;
}

////////////////////////////////////////////////////////////////////////////////
Matrix<double,Dynamic,3>
PatchFit::dfdql (MatrixXd &ql, Vector3d &k3)
{
  MatrixXd zhMat(ql.rows(),3);
  zhMat << MatrixXd::Zero(ql.rows(),3);
  zhMat.col(2).fill(1.0);

  return (2.0*((ql*k3.asDiagonal()) - zhMat));
}

////////////////////////////////////////////////////////////////////////////////
vector<MatrixXd>
PatchFit::dqldr (MatrixXd &qc, vector<Matrix3d> &drr)
{
  vector<MatrixXd> j(3);
  for (int i=0; i<3; i++)
    j[i] = qc*drr[i];

  return (j);
}

////////////////////////////////////////////////////////////////////////////////
VectorXd
PatchFit::lls(MatrixXd J, VectorXd b)
{
  // check input dimensions
  if (J.rows() != b.rows())
    cerr << "Wrong dimensions" << endl;

  VectorXd a (J.cols()); //output parameters

  JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
  VectorXd sv = svd.singularValues();
  MatrixXd V = svd.matrixV();

  VectorXd pv = sv.array().inverse();
  VectorXd pvSq = pv.array()*pv.array();

  if (b.any())
    a = V * pvSq.asDiagonal() * V.adjoint() * J.adjoint() * b;
  else
    a = V.rightCols(1);
  
  return a;
}

//TBD: J -> adfda, e should take data as input too.
//PatchFit::lm(MatrixXd (*J)(VectorXd) ,VectorXd (*e)(VectorXd), VectorXd a_Init)
VectorXd
PatchFit::lm(VectorXd a_Init)
{
  // Options
  bool normalize (false);
  double l_Init = 0.001;
  double nu = 10;

  VectorXd a (a_Init.rows(), a_Init.cols());
  a = a_Init;
  
  double l;
  l = l_Init;

  int i=0; //iteration
  //bool cc = false; // whether the last iteration committed a change to a

  double r; //residual
  
  //calculate initial error and residual
  if (normalize)
  {
    a = a/a.norm();
    //cc = true;
  }

  VectorXd ee;
  ee = PatchFit::parab(cloud_vec,a);
  r = ee.norm();
  //double r_Init = r;

  double dr = 0.0;
  MatrixXd jj;
  
  double lastr;

  // define matrices
  MatrixXd jtj, jtj_diag, aa;
  VectorXd b, da, lasta, lastee;
  while (1)
  {
    // terminating due to 0 residual at iteration i
    if (fabs(r) < EPS)
      break;

    // terminating due to minad at iteration i
    //TBD

    // terminating due to minrd at iteration i
    if ((i>0) && (this->minrd_>0.0) && (dr<0.0) && (fabs(dr)<this->minrd_*lastr))
      break;

    // terminating due to minar at iteration i
    //TBD

    // terminating due to minrr at iteration i
    //TBD
   
    // terminating due to minada at iteration i
    //TBD

    // terminating due to minrda at iteration i
    //TBD

    // terminating due to max iterations
    if (i==this->maxi_)
      break;

    i = i+1;

    // update Jacobian if necessary
    if ((i==1)||(dr<0.0))
      jj = PatchFit::parab_dfda(a);

    // calculate da
    jtj = jj.adjoint()*jj;
    jtj_diag = jtj.diagonal().asDiagonal();
    aa = jtj + (l*jtj_diag);
    b = -jj.adjoint()*ee;

    // solve LLS system
    da = lls(aa,b);
    
    // update a (will back out the update if residual increased)
    lasta = a;
    a = a+da;

    if (normalize)
      a = a/a.norm();
    
    // update error and residual
    lastr = r;
    lastee = ee;
    ee = PatchFit::parab(cloud_vec,a);
    r = ee.norm();
    dr = r-lastr;

    if (dr<0) // converging
    {
      l = l/nu;
      //cc = true;
    }
    else // diverging
    {
      l = l*nu;
      a = lasta;
      ee = lastee;
      r = lastr;
      //cc = false;
    }
  }

  if (normalize && (i==0))
    a = a/a.norm();

  return (a);
}

////////////////////////////////////////////////////////////////////////////////
VectorXd
PatchFit::wlm(MatrixXd d, VectorXd a_init)
{
  //TBD: for now just call lm algo

  //TBD: check input
  //int nd = d.rows();
  //int dd = d.cols();
  //int na = a_init.size();

  return (lm(a_init));
}
