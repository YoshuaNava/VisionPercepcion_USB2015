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
#include "patch.h"

#define EPS std::numeric_limits<double>::epsilon()
#define dNAN std::numeric_limits<double>::quiet_NaN()

#include <Eigen/StdVector>
#define EIGEN_ALIGNED_VECTOR(T) std::vector<T, Eigen::aligned_allocator<T> >

using namespace std;
using namespace pcl;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
void
Patch::setID (int id)
{
  this->id_ = id;
}

////////////////////////////////////////////////////////////////////////////////
int
Patch::getID ()
{
  return (this->id_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setName (string name)
{
  this->name_ = name;
}

////////////////////////////////////////////////////////////////////////////////
string
Patch::getName ()
{
  return (this->name_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setS (Patch::s_type s)
{
  this->s_ = s;
}

////////////////////////////////////////////////////////////////////////////////
Patch::s_type
Patch::getS ()
{
  return (this->s_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setB (Patch::b_type b)
{
  this->b_ = b;
}

////////////////////////////////////////////////////////////////////////////////
Patch::b_type
Patch::getB ()
{
  return (this->b_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setD (VectorXd d)
{
  // find the number of the parameters
  int d_size = d.size();

  if (d_size > 2) // parameters should be at most 2
  {
    cerr << "Boundary params should be at most 2." << endl;
    d.resize(2);
  }
  else if (d_size < 1) // if no parameters set it to 2
  {
    cerr << "Boundary params are required. They are set to d=[1 1]" << endl;
    d = Vector2d::Ones();
  }

  // check bounding curve parameters
  for (int i=0; i<d.size(); i++)
  {
    if (d(i) <= 0) // make all params possitive
      d(i) = 1.0;
  }

  this->d_ = d; // set params
  this->nd_ = d.size(); // set number of params
}

////////////////////////////////////////////////////////////////////////////////
VectorXd
Patch::getD ()
{
  return (this->d_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setK (VectorXd k)
{
  if (k.size()>2)
  {   
    cerr << "Curvature can have only up to 2 constants" << endl;
    this->nk_ = 2; // set the number of curvatures
    this->k_ = k.head(2);
  }   
  else
  {   
    this->nk_ = k.size(); // set the number of curvatures
    this->k_ = k;
  }   

  // set the standard curvature
  switch (getS())
  {   
    case s_e:
      this->sk_ << this->k_(0), this->k_(1);
      break;
    case s_h:
      this->sk_ << this->k_(0), this->k_(1);
      break;
    case s_y:
      this->sk_ << 0.0, this->k_(0);
      break;
    case s_o:
      this->sk_ << this->k_(0), this->k_(0);
      break;
    case s_p:
      this->k_ = VectorXd::Zero(1); // reset k just in case
      this->sk_ << 0.0, 0.0;
      break;
    default:
      cerr << "Param set: Unknown surface type" << endl;
  }   
}   

////////////////////////////////////////////////////////////////////////////////
VectorXd
Patch::getK()
{
  return (this->k_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setR (Vector3d r)
{
  this->r_ = r;
}

////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getR ()
{
  return (this->r_);
}

////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getR (Affine3d pose)
{
  return (rlog(pose.linear() * rexp(getR())));
}

////////////////////////////////////////////////////////////////////////////////
Matrix3d
Patch::getRot ()
{
  return (rexp (getR ()));
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setC (Vector3d c)
{
  this->c_ = c;
}

////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getC ()
{
  return (this->c_);
}


////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getC (Affine3d pose)
{
  return (pose*getC());
}

////////////////////////////////////////////////////////////////////////////////
Matrix3d
Patch::getRM ()
{
  return (this->rm_);
}

////////////////////////////////////////////////////////////////////////////////
Matrix4d
Patch::getPM ()
{
  return (this->pm_);
}


////////////////////////////////////////////////////////////////////////////////
Affine3d
Patch::getPose()
{
  Affine3d aff;
  aff.linear() = rexp(getR());
  aff.translation() = getC();

  this->pose_ = aff;

  return (this->pose_);
}

////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getNormal ()
{
  this->normal_ = getRot().col(2);
  return (this->normal_);
}

////////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::getYFrameAxis ()
{
  return (getRot().col(1));
}

////////////////////////////////////////////////////////////////////////////////
Vector2d
Patch::getSK ()
{
  return (this->sk_);
}

////////////////////////////////////////////////////////////////////////////////
int
Patch::getNK ()
{
  return (this->nk_);
}

////////////////////////////////////////////////////////////////////////////////
int
Patch::getND ()
{
  return (this->nd_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setSS (double ss)
{
  // check passed sample spacing
  if (ss < 0.0)
  {
    cerr << "Invalid sampling space.  ss = 1." << endl;
    this->ss_ = 1.0;
  }
  else
  {
    this->ss_ = ss;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setAutoSS ()
{
  this->ss_ = (getD().maxCoeff())/4;
}

////////////////////////////////////////////////////////////////////////////////
double
Patch::getSS ()
{
  return (this->ss_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setGD (int gd)
{
  // check passed decimation factor
  if (gd < 0)
  {
    cerr << "Decimation factor must be possitive integer.  gd = |gd|." << endl;
    this->gd_ = abs(gd);
  }
  else
  {
    this->gd_ = gd;
  }
}

////////////////////////////////////////////////////////////////////////////////
int
Patch::getGD ()
{
  return (this->gd_);
}

////////////////////////////////////////////////////////////////////////////////
Matrix2d
Patch::getBB ()
{
  return (this->bb_);
}

////////////////////////////////////////////////////////////////////////////////
double
Patch::getBA ()
{
  return (this->ba_);
}

////////////////////////////////////////////////////////////////////////////////
double
Patch::getResidual ()
{
  return (this->residual_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setCnn (double cnn)
{
  this->cnn_ = cnn;
}

////////////////////////////////////////////////////////////////////////////////
double
Patch::getCnn ()
{
  return (this->cnn_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::setCng (double cng)
{
  this->cng_ = cng;
}

////////////////////////////////////////////////////////////////////////////////
double
Patch::getCng ()
{
  return (this->cng_);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::updatePose (Affine3d pose)
{
  setR (rlog(pose.linear()*rexp(getR())));
  setC (pose*getC());
}

////////////////////////////////////////////////////////////////////////////////
vector<Patch::PointCloudConstPtr>
Patch::getGV()
{
  return (this->gv_);
}


////////////////////////////////////////////////////////////////////////////////
Matrix3d
Patch::rexp(Vector3d r)
{
  Matrix3d m;
  float t = r.norm();

  if (t>0)
  {
    Matrix3d rx;
    rx << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0;

    double ath = sqrt(sqrt(std::numeric_limits<double>::epsilon()))*10.0;
    double bth = sqrt(sqrt(std::numeric_limits<double>::epsilon()/2.0))*10.0;

    double a = 1.0;
    if(t>ath)
      a = sin(t)/t;
    else if(t>0.0)
      a = 1.0-pow(t,2)/6.0;
  
    double b = 1.0/2.0;
    if (t>bth)
      b = (1.0-cos(t))/pow(t,2);
    else if (t>0)
      b = (1/2-pow(t,2)/24.0);
    
    m = (Matrix<double, 3, 3>::Identity() + (rx*a) + (rx*rx*b));
  }
  else
  {
    m = Matrix3d::Identity();
  }
  
  return m;
}

///////////////////////////////////////////////////////////////////////////////
vector<Matrix3d>
Patch::drexp(Vector3d r)
{
  vector<Matrix3d> j (3); // return vector
  
  double t = r.norm();
  
  Matrix3d rx;
  rx << 0.0, -r(2), r(1),
        r(2), 0.0, -r(0),
        -r(1), r(0), 0.0;
  
  double a = 1.0;
  double b = 1.0/2.0;
  double da = -1.0/3.0;
  double db = -1.0/12.0;

  if (t > EPS)
    a = sin(t)/t;
  else if (t > 0.0)
    a = 1.0-(t*t)/6.0;

  if (t >= EPS)
    b = (1.0-cos(t))/(t*t);
  else if (t > 0.0)
    b = ((1.0/2.0)-((t*t)/24.0));
  
  if (t >= EPS)
    da = (t*cos(t)-sin(t))/(t*t*t);
  else if (t > 0.0)
    da = (t*t)/30.0-1.0/3.0;

  if (t >= EPS)
    db = (t*sin(t)+2.0*cos(t)-2.0)/(t*t*t*t);
  else if (t > 0.0)
    db = (t*t)/180.0-1.0/12.0;

  Matrix3d rx2;
  rx2 = rx*rx;

  vector<Matrix3d> drx (3);
  drx[0] = Matrix3d::Zero();
  drx[0](1,2) = -1.0;
  drx[0](2,1) =  1.0;
  
  drx[1] = Matrix3d::Zero();
  drx[1](0,2) =  1.0;
  drx[1](2,0) = -1.0;
  
  drx[2] = Matrix3d::Zero();
  drx[2](0,1) = -1.0;
  drx[2](1,0) =  1.0;
  

  for (int i=0; i<3; i++)
    j[i] = (rx*da*r(i)) + (drx[i]*a) + (rx2*db*r(i)) + ((rx*drx[i] + drx[i]*rx)*b);
  
  return j;
}

///////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::rlog(Matrix3d m)
{
  Vector3d r; // output vector

  Vector3d v;
  v << (m(2,1)-m(1,2)), (m(0,2)-m(2,0)), (m(1,0)-m(0,1));
  
  double cc = 0.5*(m.trace()-1);
  if (cc>1.0)
    cc = 1.0;
  else if(cc<-1.0)
    cc = -1.0;
  
  double ss = 0.5*v.norm();
  if (ss<0.0)
    ss = 0.0;
  else if (ss>1.0)
    ss = 1.0;

  double t = atan2(ss,cc);

  //ijk = 012, 120, or 201 st m(i,i) is max in diag(m)
  std::ptrdiff_t i_coeff;
  m.diagonal().maxCoeff(&i_coeff);
  int i = (int)i_coeff;
  int j = (i+1)%3;
  int k = (j+1)%3;

  double d2 = 1.0 + m(i,i) - m(j,j) - m(k,k);
  double th = 1e-4;

  if (d2>th)
  {
    double d = sqrt(d2);

    th = sqrt(std::numeric_limits<double>::epsilon())*100.0;
    double gg;
    if (t>th)
      gg = t/sqrt(3.0-m.trace());
    else
      gg = sqrt(12.0/(12.0-t*t));
  
    r = Vector3d::Zero(3,1);
    r(i) = gg*d;
    float ggd = gg/d;
    r(j) = ggd*(m(j,i)+m(i,j));
    r(k) = ggd*(m(k,i)+m(i,k));

    th = sqrt(std::numeric_limits<double>::epsilon())*100.0;
    double f = 1.0;
    Vector3d p;
    if (t<(M_PI-th))
    {
      p = r.cross(Vector3d::UnitZ());
      if (p.cwiseProduct(p).sum() < 0.25)
        p = r.cross(Vector3d::UnitY());
      if ((m*p).adjoint() * r.cross(p) < 0.0)
        f = -1.0;
    }
    else if ((rreparam(r).cwiseProduct(r)).sum() < 0.0)
      f = -1.0;
    
    d = f*d;
    ggd = f*ggd;
    r = r*f;
  }
  else
  {
    th = sqrt(std::numeric_limits<double>::epsilon())*100.0;
    double aa;
    if (t>th)
      aa = t/ss;
    else
      aa = 6.0/(6.0-(t*t));

    r = 0.5*aa*v;
  }// if t small

  return (r);
}

///////////////////////////////////////////////////////////////////////////////
Vector3d
Patch::rreparam(Vector3d r)
{
  // always map r to at most pi
  double t = r.norm ();
  double nt = t;
  double k = floor (nt/(2.0*M_PI));

  if (fabs(k)>EPS)
    nt = nt - (k*2.0*M_PI);
  if (nt>M_PI) // negative
    nt = nt - (2.0*M_PI);
  if (nt!=t)
    r = r*(nt/t);

  // apply sign policy to resolve ambiguity when t~=pi
  if (fabs(fabs(t)-M_PI) < (100.0*sqrt(EPS)))
  {
    if (r(0)<EPS)
    {
      r(0) = -r(0);
      r(1) = -r(1);
      r(2) = -r(2);
    }
    else
    {
      if (fabs(r(0)<EPS)) // on circle in yz plane
      {
        if (r(1) < EPS)
        {
          r(0) = -r(0);
          r(1) = -r(1);
          r(2) = -r(2);
        }
        else
        {
          if (fabs(r(1))<EPS)
          {
            if (r(2)<EPS)
            {
              r(0) = -r(0);
              r(1) = -r(1);
              r(2) = -r(2);
            }
          }
        }
      }
    }
  }

  return (r);
}

///////////////////////////////////////////////////////////////////////////////
VectorXd
Patch::rcanon2(Vector3d r, int p, int d)
{
  if (d!=2 && d!=3)
  {
    cout << "rcanon2: d=2" << endl;
    d = 2;
  }

  if (p!=0 && p!=1 && p!=2)
  {
    cout << "rcanon2: p=2" << endl;
    p = 2;
  }
  
  Vector3d r2;
  Vector3d a = Vector3d::Zero();
  a(p) = 1.0;

  r = Patch::rreparam(r);
  Matrix3d R = Patch::rexp(r);
  Vector3d v = R.col(p);
  
  r2 = a.cross(v);

  double ss = r2.norm();
  double cc = v(p);
  double t = atan2(ss,cc);
  double th = sqrt(sqrt(EPS))*10.0;
  double aa;
  
  if (t>th)
    aa = t/ss;
  else
    aa = 6.0/(6.0-(t*t));

  th = sqrt(EPS)*10.0;
  
  if (t<(M_PI-th))
    r2 = aa*r2;
  else
    r2 = r;

  VectorXd final_r2(d);
  int j = 0;
  if (d!=3)
  {
    for (int i=0; i<3; i++)
    {
      if (i!=d)
      {
        final_r2[j] = r2[i];
        j++;
      }
    }
  }
  else
  {
    final_r2 = r2;
  }

  return (final_r2);
}

///////////////////////////////////////////////////////////////////////////////
void
Patch::infer_params()
{
  Vector2d rr;

  switch (Patch::getB ())
  {
    case Patch::b_c:
      rr << this->d_(0), this->d_(0);
      this->bb_ = bbaaquad(rr);
      this->ba_ = baellipse(rr);
      break;
    case Patch::b_e:
      this->bb_ = bbaaquad((Vector2d)d_);
      this->ba_ = baellipse((Vector2d)d_);
      break;
    case Patch::b_r:
      this->bb_ = bbaaquad((Vector2d)d_);
      this->ba_ = baaaquad((Vector2d)d_);
      break;
    default:
      cout << "No bounds param given." << endl;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::gp()
{
  (this->rm_) = rexp(this->r_);

  this->pm_.block<3,3>(0,0) = this->rm_;
  this->pm_.block<3,1>(0,3) = this->c_;
  this->pm_.block<1,4>(3,0) << 0.0,0.0,0.0,1.0;
}

///////////////////////////////////////////////////////////////////////////////
void
Patch::gs()
{
  // x (vert) and y (horiz) gridline abscissas as Nx1 mat
  RowVectorXd gu[2]; 

  // corresponding ordinates along gridline in the other dim as NxVAR cell
  // array of 1xVAR row matrices, where VAR is the number of samples along
  // that gridline (may vary from line to line)
  VectorXd *gvv[2];

  for (int i=0; i<2; i++) //i=0 x gridlines (vertical), i=1 y gridlines (horizontal)
  {
    Matrix<double,1,2> b;
    b = this->bb_.row(i); //[min max] bounds in dimension i
    
    // abscissas in this dim as Nx1 matrix
    gu[i] = samples(b(0),b(1),this->ss_,0,0); //sample open interval

    // [min, max] ordinates in other dim as Nx2 matrix
    Matrix<double,Dynamic,2> mm;
    bi(i, gu[i], mm);

    //TBD: can the intersection be imaginary number?
    // valid gridline in this dim
    std::vector<int> index;
    //removeRowwiseNaN(mm,mm,index);
    //TBD: gv = gv(index,:);

    for (int j=0; j<gu[i].size(); j++)
    {
      if (isnan(mm(i)))
      {
        // NaN
        //cout << "NaN" << endl;
      }
    }
    
    // Generate itermediate ordinates in the other dim. Can skip if zero
    // curvature in other dim, but still must convert to cell array.
    int n = gu[i].size();
    int j = (int) ((i+1)%2); //other dim
 
    // corresponding ordinates along gridline in the other dim as NxVAR cell
    // array of 1xVAR row matrices, where VAR is the number of samples along
    // that gridline (may vary from line to line)
    gvv[i] = new Matrix<double,Eigen::Dynamic,1> [n];

    if (sk_(j)!=0) // other dim curvature not zero
    {
      // sample closed interval
      for (int l=0; l<n; l++)
        gvv[i][l] = samples(mm(l,0),mm(l,1),this->ss_,1,1);
    }
    else
    {
      for (int l=0; l<n; l++)
        gvv[i][l] = mm.row(l);
    }
  } //i=0:1

  // number of gridlines
  int ngx = gu[0].size();
  int ngy = gu[1].size();
  int ng = ngx+ngy;

  // Nx1 arrays of vertex coords, one triple per gridline
  typedef Matrix<double,Dynamic,3> MatNx3;
  typedef Matrix<double,Dynamic,1> MatNx1;
  EIGEN_ALIGNED_VECTOR(MatNx3) g(ng);
  EIGEN_ALIGNED_VECTOR(MatNx1) gx(ng), gy(ng), gz(ng);

  int l = 0;
  int mv = 0;
  MatrixXd m(1,1);
  for (int ii=0; ii<2; ii++) //i=0 x gridlines (vertical), i=1 y gridlines (horizontal)
  {
    for (int jj=0; jj<gu[ii].size(); jj++)// for each abscissa
    {
      g[l].resize(gvv[ii][jj].size(),3);
      
      MatrixXd u(1,gvv[ii][jj].size());
      m(0,0) = gu[ii][jj];
      u = m.replicate(1,gvv[ii][jj].size());

      MatrixXd v(1,gvv[ii][jj].size());
      v = gvv[ii][jj].adjoint();
      
      if (ii==0)
      {
        g[l].col(0) = u;
        g[l].col(1) = v;
        gx[l] = u;
        gy[l] = v;
      }
      else
      {
        g[l].col(1) = u;
        g[l].col(0) = v;
        gx[l] = v;
        gy[l] = u;
      }
      g[l].col(2) = zl(gx[l],gy[l]);
      gz[l] = zl(gx[l],gy[l]);

      mv = max(mv,(int)gx[l].size());
      l = l+1;
    }
  }

  /*
     if (this->gd_<2); gdi = 1:ng;
     else % decimate grid
     gss = ss*this->gd_;
     for i=1:2
     ngui = length(gu{i});
     first = 1+ceil(gu{i}(1)/gss)*this->gd_-round(gu{i}(1)/ss);
     last = ngui-(round(gu{i}(ngui)/ss)-floor(gu{i}(ngui)/gss)*this->gd_);
     gdii{i} = first:this->gd_:last;
     end
     gdi = [gdii{1}, ngx+gdii{2}];
     ng = length(gdi);
     end
   */
  

  // Create the cloud
  for (int j=0; j<ng; j++)
  {
    PointCloud< PointIn >::Ptr point_cloud (new PointCloud<PointIn>);

    for (int i=0; i<g[j].rows(); i++)
    {
      PointIn basic_point;
      basic_point.x = g[j](i,0);
      basic_point.y = g[j](i,1);
      basic_point.z = g[j](i,2);
      point_cloud->points.push_back(basic_point);
    }
    this->gv_.push_back(point_cloud);
  }
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::computeResidual (PointCloud<PointXYZ>::Ptr &cloud)
{
  double res; // final residual
  //double tol = sqrt (EPS); // tolerance for denominator zero checking //TBD: unused
  int nd = cloud->points.size ();
  MatrixXd x(nd,1), y(nd,1), z(nd,1);

  for (int i=0; i<nd; i++)
  {
    x(i,0) = cloud->points[i].x;
    y(i,0) = cloud->points[i].y;
    z(i,0) = cloud->points[i].z;
  }

  // transform data to local frame
  Xform3 proj;
  Matrix3d rrinv = rexp (-getR ());
  proj.transform (x, y, z, rexp(-getR ()), (-rrinv*getC ()), 0, 0);
  
  // If plane
  if (getS() == s_p)
  {
    // plane
    res = sqrt((z.array().square()).sum()/nd);
    return (res);
  }

  // If paraboloid

  // determine parameters
  double kx, ky, kxx, kyy, kxy;
  kx = sk_(0);
  ky = sk_(1);
  kxx = kx*kx;
  kyy = ky*ky;
  kxy = kx*ky;
  
  // sum squared distance using exact method for paraboloids
  VectorXd d2(nd); // dist squared
  VectorXd vi(nd); // bool index of valid (non-degenerate) samples
  vi = VectorXd::Ones(nd);
  
  // handle degenerate cases
  bool cp (false), ypx (false), ypy (false), pl (false);
  if (abs(kx-ky)<EPS)
    cp = true;
  if (!cp && (abs(kx)<EPS))
    ypx = true;
  if (!cp && (abs(ky)<EPS))
    ypy = true;
  if (ypx&&ypy)
    pl = true; //plane

  VectorXd zz(nd);
  if (cp||ypx||ypy)
    zz = z.array().square();
  
  VectorXd yz(nd);
  yz = VectorXd::Zero(nd);
  if (~pl&&(cp||ypx))
  {
    //yz = abs(y)<eps;
    for (int i=0; i<nd; i++)
      if (y(i,0) < EPS)
        yz(i) = 1;
  }

  VectorXd xz(nd);
  xz = VectorXd::Zero(nd);
  if (~pl&&(cp||ypy))
  {
    //xz = abs(x)<eps;
    for (int i=0; i<nd; i++)
      if (x(i,0) < EPS)
        xz(i) = 1;
  }

  if (pl) // plane
  {
    d2 = zz;
    vi.setZero();
  }
  else if (cp) // circular paraboloid
  {
    // circular degenerated
    double ii, jj;
    for (int i=0; i<nd; i++)
    {
      ii = xz(i) && yz(i);
      jj = (z(i)*kx<0.0) || (z(i)<2.0/kx);

      if (ii && jj)
      {
        d2(i) = zz(i);
        vi(i) = 0;
      }
      if (ii && !jj)
      {
        d2(i) = abs(2.0*z(i)/kx);
        vi(i) = 0;
      }
    }
  }
  else if (ypx) // cylindric paraboloid, x axis
  {
    // cylindric-x degenerated
    for (int i=0; i<nd; i++)
    {
      if (yz(i) && (abs(z(i,0)-1.0/ky) < EPS))
      {
        d2(i) = zz(i);
        vi(i) = 0;
      }
    }
  }
  else if (ypy) // cylindric paraboloid, y axis
  {
    // cylindric-y degenerated
    for (int i=0; i<nd; i++)
    {
      if (xz(i) && (abs(z(i,0)-1.0/kx) < EPS))
      {
        d2(i) = zz(i);
        vi(i) = 0;
      }
    }
  }

  // solve polynomial for non-degenerate points
  double g, h;
  g = -4.0*((kxx*ky)+(kx*kyy));
  h = -2.0*(kxx+(4.0*kxy)+kyy);
  
  VectorXd xx(nd), yy(nd);
  xx = x.array().square();
  yy = y.array().square();
  
  VectorXd aa(nd), bb(nd), cc(nd), dd(nd), ee(nd), ff(nd);
  aa = (-2.0*kxx*kyy) * VectorXd::Ones(nd);
  bb = (aa*z).array() + g;
  cc = (g*z).array() + h;
  dd = ((h*z) + (kxy*((ky*xx) + (kx*yy)) )).array() - (4.0*(kx+ky));
  ee = ((2.0*kxy*(xx+yy)) - (4.0*(kx+ky)*z)).array() - 2.0;
  ff = (kx*xx) + (ky*yy) - (2.0*z);
 
  VectorXd coeff(6); 
  for (int i=0; i<nd; i++)
  {
    if (vi(i))
    {
      coeff << ff(i), ee(i), dd(i), cc(i), bb(i), aa(i);
      d2(i) = Patch::polysolve(coeff, x(i), y(i), z(i));
    }
  }

  res = sqrt(d2.sum()/nd);
  this->residual_ = res; // geometrical residual
  
  return (res);
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::sl (double x, double y, double z)
{
  //TBD
  cerr << "TBD: sl impl" << endl;
  return (dNAN);
}

///////////////////////////////////////////////////////////////////////////////
Matrix<double,Dynamic,3>
Patch::sg (Matrix<double,Dynamic,3> xyz)
{
  //TBD
  cerr << "TBD: sg impl" << endl;
  Matrix<double,Dynamic,3> g;

  return (g);
}

///////////////////////////////////////////////////////////////////////////////
RowVectorXd
Patch::zl(RowVectorXd x, RowVectorXd y)
{
  MatrixXd z (1,x.cols());
  if ((fabs(this->sk_(0))<EPS) && (fabs(this->sk_(1))<EPS)) //plane
  {
    z.fill(0.0);
  }
  else //paraboloid
  {
    z = (1.0/2.0)*((sk_(0,0)*(x.array().square())) + (sk_(1,0)*(y.array().square())));
  }

  return z;
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::ri(MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
          double cx, double cy, double cz)
{
  if ((fabs(sk_(0))<EPS) && (fabs(sk_(1))<EPS)) //plane
  {
    return (riplane(mx, my, mz, cx, cy, cz));
  }
  else //paraboloid
  {
    return (riquadratic(mx, my, mz, cx, cy, cz));
  }
}

///////////////////////////////////////////////////////////////////////////////
int
Patch::bi(int i, Matrix<double,1,Dynamic> u, Matrix<double,Dynamic,2> &o)
{
  if ((i!=0) && (i!=1))
  {
    cerr << "i should be either 0 or 1" << endl;
    return -1;
  }

  // check for valid boundary type
  if ((Patch::getB () != Patch::b_c) &&
      (Patch::getB () != Patch::b_e) &&
      (Patch::getB () != Patch::b_r))
  {
    cerr << "bi requires valid boundary type" << endl;
    return -1;
  }

  switch (Patch::getB ())
  {
    case Patch::b_c:
      o = Patch::biellipse(i,u);
      break;
    case Patch::b_e:
      o = Patch::biellipse(i,u);
      break;
    case Patch::b_r:
      o = Patch::biaaquad(i,u);
      break;
    default:
      return -1;
  }

  return 1;
}

///////////////////////////////////////////////////////////////////////////////
int
Patch::bl(MatrixXd x, MatrixXd y, MatrixXd &o)
{
  // check for valid boundary
  if ((Patch::getB () != Patch::b_c) &&
      (Patch::getB () != Patch::b_e) &&
      (Patch::getB () != Patch::b_r))
  {
    cerr << "bi requires valid boundary type" << endl;
    return -1;
  }
  
  switch (Patch::getB ())
  {
    case Patch::b_c:
      o = Patch::blellipse(x,y);
      break;
    case Patch::b_e:
      o = Patch::blellipse(x,y);
      break;
    case Patch::b_r:
      o = Patch::blaaquad(x,y);
      break;
    default:
      return -1;
  }

  return 1;
}

///////////////////////////////////////////////////////////////////////////////
void
Patch::flipPatchFrameTowardsViewpoint (float vp_x, float vp_y, float vp_z)
{
  // rotate pi around the x-axis
  Matrix3d rot = -Matrix3d::Identity();
  rot (0,0) = 1.0;

  Vector3f c = getC().cast<float>();
  Vector3f vp (vp_x-c(0), vp_y-c(1), vp_z-c(2));
  
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = vp.dot (getNormal().cast<float>());

  // Flip the frame
  if (cos_theta < 0)
  {
    setR (rlog (getRot() * rot));

    // flip curvatures
    k_ = - k_;
    sk_ = -sk_;
  }
}

///////////////////////////////////////////////////////////////////////////////
bool
Patch::isSimilar (Patch &p, double d_thres, double k_thres, double t_thres,
                  double r_thres)
{
  // Test 1: check the patch isurface and boundary type
  //if ((this->s_ != p.getS()) || (this->b_ != p.getB()))
  //  return (false);

  // Test 2: check boundary params
  int d_size = (this->d_).size();
  
  if (d_size != (p.getD()).size())
  {
    //cout << "d_size failed" << endl;
    return (false);
  }
  
  VectorXd d_thres_vec (d_size);
  d_thres_vec.fill (d_thres);
  if ((((this->d_ - p.getD()).cwiseAbs()).array() > d_thres_vec.array()).any())
  {
    //cout << "d: " << ((this->d_ - p.getD()).cwiseAbs()).array() << ">" << d_thres_vec.array() << endl;
    return (false);
  }

  // Test 3: check curvature
  int k_size = (this->k_).size();
  
  if (k_size != (p.getK()).size())
  {
    //cout << "k_size failed" << endl;
    return (false);
  }
  
  VectorXd k_thres_vec (k_size);
  k_thres_vec.fill (k_thres);
  if ((((this->k_ - p.getK()).cwiseAbs()).array() > k_thres_vec.array()).any())
  {
    //cout << "k: " << ((this->k_ - p.getK()).cwiseAbs()).array() << ">" << k_thres_vec.array() << endl;
    return (false);
  }

  // Test 4: check pose (r,c)
  if ((this->getC()-p.getC()).norm() > r_thres)
  {
    //cout << "c failed" << endl;
    return (false);
  }
  
  // check the rotation only if needed taking care of symmetries
  // note that normals should be oriented to the viewpoint if any
  if (findAngle (this->getNormal(), p.getNormal()) > deg2rad(t_thres))
  {
    //cout << "Normal failed" << endl;
    return (false);
  }

  // plane and circular paraboloid don't need to be checked for symmetries
  if ((this->s_ == s_p) || (this->s_ == s_o))
    return (true);

  // rotate pi around the z-axis
  Matrix3d rot = -Matrix3d::Identity();
  rot (2,2) = 1.0;
  Vector3d y = (getRot()*rot).col(1);

  //cout << "angle 0: " << findAngle (this->getYFrameAxis(), p.getYFrameAxis()) << endl;
  //cout << "angle 0: " << findAngle (y, p.getYFrameAxis()) << endl;
  if (((findAngle (this->getYFrameAxis(), p.getYFrameAxis())) > deg2rad(t_thres))
      && (findAngle (y, p.getYFrameAxis()) > deg2rad(t_thres)))
  {
    //cout << "x-axis angle failed" << endl;
    return (false);
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////
void
Patch::printInfo (Affine3d pose)
{
  // var_name var_size var_args
  cout << "# s b d k r c" << endl;

  cout << "1" << " ";
  cout << this->getS() << endl;
  
  cout << "1" << " ";
  cout << this->getB() << endl;
  
  cout << (this->getD()).size() << " ";
  cout << this->getD().transpose() << endl;
  
  cout << (this->getK()).size() << " ";
  cout << this->getK().transpose() << endl;

  cout << (this->getR()).size() << " ";
  cout << this->getR(pose).transpose() << endl;
  
  cout << (this->getC()).size() << " ";
  cout << this->getC(pose).transpose() << endl;
}

////////////////////////////////////////////////////////////////////////////////
bool
Patch::loadPatch (const string &fn)
{
  std::ifstream ifs (fn.c_str());
  if (!ifs.is_open())
  { cerr << "unable to open \"" << fn << "\"\n"; return false; }

  std::string line;
  double num_args, num;
  VectorXd k,d;

  while (ifs.good())
  {
    // get next line
    getline(ifs, line);
    
    // erase comments
    line.erase(find(line.begin(), line.end(), '#'), line.end());
  
    // read number of args
    int arg=0;
    while (ifs >> num_args)
    {
      // change sizes
      switch (arg)
      {
        case 2:
          d.resize (num_args);
          d_.resize (num_args);
          break;
        case 3:
          k.resize (num_args);
          k_.resize (num_args);
          break;
        case 4:
          r_.resize (num_args);
          break;
        case 5:
          c_.resize (num_args);
          break;
      }
      
      //add values
      for (int i=0; i<num_args; i++)
      {
        ifs >> num;
        switch (arg)
        {
          case 0:
            this->setS (static_cast<s_type>(num));
            break;
          case 1:
            this->setB (static_cast<b_type>(num));
            break;
          case 2:
            d(i) = num;
            break;
          case 3:
            k(i) = num;
            break;
          case 4:
            r_(i) = num;
            break;
          case 5:
            c_(i) = num;
            break;
        }
      }
      arg++;
    }
    this->setD (d);
    this->setK (k);

  }
  return (true);
}

////////////////////////////////////////////////////////////////////////////////
int
Patch::bc (MatrixXi x, MatrixXi y, double w, MatrixXd &o)
{
  // check for valid boundary
  if ((Patch::getB () != Patch::b_c) &&
      (Patch::getB () != Patch::b_e) &&
      (Patch::getB () != Patch::b_r))
  {
    cerr << "bc requires valid boundary type" << endl;
    return (-1);
  }

  // check for valid input
  if ((x.rows() != y.rows()) || (x.cols() != y.cols()))
  {
    cerr << "bc input should be of same size" << endl;
    return (-1);
  }
  
  switch (Patch::getB ())
  {
    case Patch::b_c:
      o = Patch::bcellipse (x,y,w);
      break;
    case Patch::b_e:
      o = Patch::bcellipse (x,y,w);
      break;
    case Patch::b_r:
      o = Patch::bcaaquad (x,y,w);
      break;
    default:
      return (-1);
  }

  return (1);
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::polysolve (VectorXd &coeffs, double qx, double qy, double qz)
{
  double sol = numeric_limits<double>::infinity ();
  
  // Removing 0 coeffs from the end of the polynomial because PolynomialSolve
  // throws an error in these cases.
  int coeffs_counter = coeffs.size();
  for (int i=coeffs.size()-1; i>-1; i--)
  {
    if (coeffs(i))
      break;
    else
      coeffs_counter--;
  }
  VectorXd new_coeffs (coeffs_counter);
  new_coeffs = coeffs.head(coeffs_counter);

  // TBD: Dynamic is not working correctly.  By giving magic number 5 returns
  // correct real solution but wrong imaginary.
  //PolynomialSolver<double,Dynamic> psolve (new_coeffs);
  PolynomialSolver<double,5> psolve;
  psolve.compute(new_coeffs);

  // complex roots
  //cout << "Complex roots: " << psolve.roots().transpose() << endl;
  vector<double> real_roots;
  psolve.realRoots (real_roots);
  int rr_size = real_roots.size();
  Map< Matrix< double, Dynamic, 1 > > mapRR(&real_roots[0], rr_size);
  VectorXd ll (rr_size);
  ll = mapRR.transpose ();
  //cout << "ll: " << ll << " ll size: " << ll.size() << endl;

  // if there are real roots
  double kx, ky;
  kx = sk_(0);
  ky = sk_(1);
  
  double lr, s;
  for (int i=0; i<ll.size(); i++)
  {
    lr = ll(i);
    s = lr * lr * (pow((kx*qx)/(1.0+lr*kx), 2) + pow((ky*qy)/(1.0+lr*ky), 2) + 1.0);
    sol = min(sol,s);
  }

  if (isinf (sol))
  {
    cout << "new_coeffs: " << new_coeffs << endl;
    cout << "Complex roots: " << psolve.roots().transpose() << endl;
    cout << "ll: " << ll << " ll size: " << ll.size() << endl;
  }

  return (sol);
}


///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::riplane (MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
                double cx, double cy, double cz)
{
  MatrixXd czMat(mx.rows(), mx.cols());
  czMat.fill(-cz);
  
  return (czMat.array()/mz.array());
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::riquadratic (MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
                    double cx, double cy, double cz)
{
  MatrixXd r(mx.rows(),mx.cols());
  double r1, r2;
  double delta;
  double a,b,c,q;
  double kx = sk_(0);
  double ky = sk_(1);
  //double kz = 0; //TBD: unused
  
  for (int i=0; i<mx.rows(); i++)
  {
    for(int j=0; j<mx.cols(); j++)
    {
      a = (kx*mx(i,j)*mx(i,j)) + (ky*my(i,j)*my(i,j));
      b = (2*kx*cx*mx(i,j)) + (2*ky*cy*my(i,j) + (-2*mz(i,j)));
      c = (cx*kx*cx) + (cy*ky*cy) + (-2*cz);
      
      delta = (b*b)-(4.0*a*c);
      if (delta<0)
      {
        r1 = std::numeric_limits<double>::quiet_NaN();
        r2 =  std::numeric_limits<double>::quiet_NaN();
        r(i,j) =std::numeric_limits<double>::quiet_NaN();
        // TBD: maybe it should be std::numeric_limits<double>::infinity();
      }
      else
      {
        if (b>0)
          q = -0.5*(b+sqrt(delta));
        else
          q = -0.5*(b-sqrt(delta));
        r1 = q/a;
        r2 = c/q;


        if (a==0)
        {
          r(i,j) = r2;
        }
        else
        {
          // if both solutions are nonnegative, select the smaller one
          if (r1>0 && r2>0)
            r(i,j) = min(r1,r2);
          else
            r(i,j) = max(r1,r2);
        }
      }
    }
  }

  return r;
}

///////////////////////////////////////////////////////////////////////////////
Matrix<double,Dynamic,2>
Patch::biellipse (int i, RowVectorXd &u)
{
  RowVector2d rr;
  if (this->nd_ == 1)
    rr << this->d_(0), this->d_(0);
  else if(this->nd_ == 2)
    rr << this->d_(0), this->d_(1);

  MatrixXd o(u.size(),2);
  o.fill(*( double* )nan);

  int j = (int) ((i+1)%2); //other dim
  
  for (int k=0; k<u.size() ; k++)
  {
    if (abs(u(k)) <= rr(i))
    {
      o(k,1) = (rr(j)/rr(i))*sqrt((rr(i)*rr(i))-(u(k)*u(k)));
      o(k,0) = -o(k,1);
    }
  }
  return (o);
}

///////////////////////////////////////////////////////////////////////////////
Matrix<double,Dynamic,2>
Patch::biaaquad (int i, RowVectorXd &u)
{
  MatrixXd o(u.size(),2);
  o.fill(*( double* )nan);

  int j = (int) ((i+1)%2); // other dim

  for (int k=0; k<u.size() ; k++)
  {
    if (fabs(u(k)) <= this->d_(i))
    {
      o(k,0) = -this->d_(j);
      o(k,1) = this->d_(j); // always symmetric
    }
  }

  return (o);
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::blellipse (MatrixXd &x, MatrixXd &y)
{
  double rx2 = this->d_(0)*this->d_(0);
  double ry2 = this->d_(1)*this->d_(1);
  return ((ry2*x.array().square()) + (rx2*y.array().square()) - (rx2*ry2));
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::blaaquad (MatrixXd &x, MatrixXd &y)
{
  Matrix<double,4,2> r;
  r << this->d_(0),  this->d_(1),
      -this->d_(0),  this->d_(1),
      -this->d_(0), -this->d_(1),
       this->d_(0), -this->d_(1);

  return (blquad(x,y,r));
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::blquad (MatrixXd &x, MatrixXd &y, Matrix<double,4,2> &v)
{
  // edge direction vectors
  Matrix<double,4,2> d;
  d << v.row(1)-v.row(0), v.row(2)-v.row(1), v.row(3)-v.row(2), v.row(0)-v.row(3);
  Vector4d l;
  l << d.row(0).norm(), d.row(1).norm(), d.row(2).norm(), d.row(3).norm();

  int ns = 0;
  for (int i=0; i<l.rows(); i++)
  {
    if (l(i) > EPS)
      ns++;
  }
 
  // degenerate cases
  if (ns < 3)
    return ((x.array().square() + y.array().square()).cwiseSqrt());

  // make s and d the start points and unit direction vectors of each
  // non-degenerate side
  MatrixXd s(ns,2), dd(ns,2);
  int j=0;
  for (int i=0; i<4; i++)
  {
    if (l(i) > EPS)
    {
      s.row(j) = v.row(i);
      dd.row(j) = d.row(i).array()/l(i);
      j++;
    }
  }
  dd.conservativeResize(j,2);

  Vector3d nn, dn;
  MatrixXd n(ns,2);
  for (int i=0; i<ns; i++)
  {
    dn << dd(i,0), dd(i,1), 0.0;
    nn =  dn.cross(Vector3d::UnitZ());
    n.row(i) = nn.block(0,0,2,1).transpose();
  }

  VectorXd c(ns); // perp dist to origin
  for (int i=0; i<ns; i++)
  {
    c(i) = -n.row(i)*s.row(i).transpose();
  }
  
  //int nd = x.rows()*x.cols(); //TBD: unused
  vector<MatrixXd> dist (ns);
  
  for (int i=0; i<ns; i++)
  {
    dist[i].resize(x.rows(),x.cols());
    // distance from each pt (cols) to each line (rows)
    dist[i] = (x*n(i,0)) + (y*n(i,1));
    dist[i] = dist[i].array() + c(i);
  }
  
  MatrixXd bl(x.rows(),x.cols());
  MatrixXd dist_max(x.rows(),x.cols());
  dist_max = dist[0];
  VectorXd dist_tmp(ns);

  for (int id=0; id<x.rows(); id++)
  {
    for (int jd=0; jd<x.cols(); jd++)
    {
      for (int kd=0; kd<ns; kd++)
        dist_tmp(kd) = dist[kd](id,jd);

      // for pts inside, return closest negative distance
      // for pts outside, return closest positive distance
      if ((dist_tmp.array()<0.0).all())
        dist_max(id,jd) = dist_tmp.minCoeff();
      else
        dist_max(id,jd) = dist_tmp.maxCoeff();
    }
  }
  return (dist_max);
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::bcellipse (MatrixXi x, MatrixXi y, double w)
{
  double a = d_(0);
  double b = d_(1);

  int rows = x.rows ();
  int cols = x.cols ();
  
  // Output matrix
  MatrixXd o = MatrixXd::Zero (rows, cols);
  
  // flip quadrants II, III, IV to I
  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      if (x(ii,jj)<0)
        x(ii,jj) = -x(ii,jj)-1;
    
      if (y(ii,jj)<0)
        y(ii,jj) = -y(ii,jj)-1;
    }
  }

  // convert from units of w
  MatrixXd dx = x.cast<double>() * w;
  MatrixXd dy = y.cast<double>() * w;
  MatrixXd xw = dx.array() + w;
  MatrixXd yw = dy.array() + w;

  // intersection area by case analysis, see Kanoulas, Vona ICRA2013
  complex<double> dXy, dYx, dXyw, dYxw, dYXyw;
  complex<double> temp;
  MatrixXd a1(rows,cols), a2(rows,cols), a3(rows,cols), a4(rows,cols);
  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      // intersection area by case analysis, see Kanoulas, Vona ICRA2013
      temp = 1.0 - ((dy(ii,jj)*dy(ii,jj))/(b*b));
      dXy = a * sqrt(temp);
      
      temp = 1.0 - ((dx(ii,jj)*dx(ii,jj))/(a*a));
      dYx = b * sqrt(temp);

      temp = 1.0 - ((yw(ii,jj)*yw(ii,jj))/(b*b));
      dXyw = a * sqrt(temp);

      temp = 1.0 - ((xw(ii,jj)*xw(ii,jj))/(a*a));
      dYxw = b * sqrt(temp);

      dYXyw = b * sqrt(1.0 - ((dXyw*dXyw)/(a*a)));

      //a1, a2, a3, a4
      a1(ii,jj) = (((dXyw - dx(ii,jj)) * w) + 
                   ((xw(ii,jj)-dXyw) * (dYXyw-dy(ii,jj))) +
                   ((xw(ii,jj)-dXyw) * (dy(ii,jj)+w-dYXyw))/2.0).real();
  
      a2(ii,jj) = (((xw(ii,jj)-dx(ii,jj)) * (dYxw-dy(ii,jj))) +
                   (((xw(ii,jj)-dx(ii,jj)) * (dYx-dYxw)))/2.0).real();
  
  
      a3(ii,jj) = (((yw(ii,jj)-dy(ii,jj)) * (dXyw-dx(ii,jj))) +
                   ((yw(ii,jj)-dy(ii,jj)) * (dXy-dXyw))/2.0).real();
  
  
      a4(ii,jj) = ((dXy-dx(ii,jj)) * (dYx-dy(ii,jj))/2.0).real();
    }
  }

  // bool indices of square corners inside the ellipse
  MatrixXd llci, lrci, urci, ulci;
  bl (dx, dy, llci);
  bl (xw, dy, lrci);
  bl (xw, yw, urci);
  bl (dx, yw, ulci);
  
  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      if (urci(ii,jj)<=0)
        o(ii,jj) = w*w;

      if (llci(ii,jj)<=0 && urci(ii,jj)>0 && lrci(ii,jj)<=0 && ulci(ii,jj)<=0)
        o(ii,jj) = a1(ii,jj);
          
      if (llci(ii,jj)<=0 && lrci(ii,jj)<=0 && ulci(ii,jj)>0)
        o(ii,jj) = a2(ii,jj);
      
      if (llci(ii,jj)<=0 && ulci(ii,jj)<=0 && lrci(ii,jj)>0)
        o(ii,jj) = a3(ii,jj);
      
      if (llci(ii,jj)<=0 && lrci(ii,jj)>0 && ulci(ii,jj)>0)
        o(ii,jj) = a4(ii,jj);
    }
  }

  return (o);
}

///////////////////////////////////////////////////////////////////////////////
MatrixXd
Patch::bcaaquad (MatrixXi x, MatrixXi y, double w)
{
  double a = d_(0);
  double b = d_(1);

  int rows = x.rows ();
  int cols = x.cols ();
  
  // output matrix
  MatrixXd o = MatrixXd::Zero (rows, cols);

  // flip quadrants II, III, IV to I
  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      if (x(ii,jj)<0)
        x(ii,jj) = -x(ii,jj)-1;
      
      if (y(ii,jj)<0)
        y(ii,jj) = -y(ii,jj)-1;
    }
  }

  // convert from units of w
  MatrixXd dx = x.cast<double>() * w;
  MatrixXd dy = y.cast<double>() * w;
  MatrixXd xw = dx.array() + w;
  MatrixXd yw = dy.array() + w;

  
  //intersection area by case analysis, see Kanoulas, Vona ICRA2013
  MatrixXd a5 = w*(b-dy.array());
  MatrixXd a6 = w*(a-dx.array());
  MatrixXd a7 = (a-dx.array()).cwiseProduct(b-dy.array());

  //bool indices of square corners inside the quad
  bool llci, lrci, urci, ulci;
  bool xi, yi, xwi, ywi;

  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      xi = (abs(dx(ii,jj))<=a);
      yi = (abs(dy(ii,jj))<=b);
      xwi = (abs(xw(ii,jj))<=a);
      ywi = (abs(yw(ii,jj))<=b);
      
      llci = xi&yi;
      lrci = xwi&yi;
      urci = xwi&ywi;
      ulci = xi&ywi;

      if (urci) {o(ii,jj) = w*w;}
      if (llci & lrci & !ulci) {o(ii,jj) = a5(ii,jj);}
      if (llci & ulci & !lrci) {o(ii,jj) = a6(ii,jj);}
      if (llci & !lrci & !ulci) {o(ii,jj) = a7(ii,jj);}
    }
  }
  
  return (o);
}

///////////////////////////////////////////////////////////////////////////////
Matrix<double,2,2>
Patch::bbaaquad (Vector2d d)
{
  Matrix<double,2,2> bb;
  bb << -d(0,0), d(0,0), -d(1,0), d(1,0);
  
  return (bb);
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::baellipse (Vector2d d)
{
  return (M_PI*d(0)*d(1));
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::baaaquad (Vector2d d)
{
  return (4.0*d(0)*d(1));
}

///////////////////////////////////////////////////////////////////////////////
double
fix (double n)
{
  // round towards zero
  return (n > 0.0) ? std::floor(n) : std::ceil(n);
}

///////////////////////////////////////////////////////////////////////////////
Matrix<double,1,Dynamic> 
Patch::samples(double s, double e, double ss, double cs, double ce)
{
  double first = fix(s/ss) * ss; // first sample generated by colon
  double last = first + fix((e-first)/ss)*ss; // last sample generated by colon
  
  if (!cs && (fabs(first-s)<EPS)) //i.e. first==s
    first = first+ss;
  if (!ce && (fabs(last-e)<EPS)) //i.e. last==e
    last = last-ss;

  int p_size = 0;
  if ((last-first)>EPS) //first!=last
    p_size = static_cast<int> (((last-first)/ss)) + 1;
  
  if(cs && (fabs(first-s)>EPS)) //i.e. first!=s
    p_size++;
  if(ce && (fabs(last-e)>EPS)) //i.e. last!=e
    p_size++;
 
  MatrixXd p(1,p_size);

  int j=0;
  
  if (cs && (fabs(first-s)>EPS))
  {
    p(j) = s;
    j++;
  }
    
  // Note: 0 is not represented exactly
  if (fabs(last-first)>=EPS)
  {
    for (double i=first; (i-last)<=EPS; i = i+ss)
    {
      p(j) = i;
      j++;
    }
  }

  if (ce && (fabs(last-e)>EPS))
  {
    p(j) = e;
    j++;
  }

  return p;
}

///////////////////////////////////////////////////////////////////////////////
void
Patch::removeRowwiseNaN (Matrix<double,Dynamic,2> &m_in,
                         Matrix<double,Dynamic,2> &m_out,
                         std::vector<int> &index)
{
  bool isNaN = false;

  // If the matrices are not the same, prepare the output
  if (&m_in != &m_out)
    MatrixXd m_out (m_in.rows(), m_in.cols());

  // Reserve enough space for the indices
  index.resize (m_in.rows());
  size_t j = 0;

  for (size_t r=0; r<m_in.rows(); r++)
  {
    for (size_t c=0; c<m_in.cols(); c++)
      if (!pcl_isfinite (m_in(r,c)))
        isNaN = true;
    
    if (isNaN)
      continue;

    m_out.row(j) = m_in.row(r);
    index[j] = static_cast<int>(r); // index the row
    j++;
  }

  if (j != m_in.rows())
  {
    // Resize to the correct size
    m_out.resize (j,m_in.cols());
    index.resize (j);
  }
}

///////////////////////////////////////////////////////////////////////////////
double
Patch::findAngle (Vector3d a, Vector3d b)
{
  return (atan2((a.cross(b)).norm(), a.dot(b)));
}
