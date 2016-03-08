/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-, Dimitrios Kanoulas
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

#include "patch_coverage.h"

////////////////////////////////////////////////////////////////////////////////
PatchCoverage::PatchCoverage (Patch p, PointInCloudPtr nn_cloud, bool iw):
  nn_cloud_ (new PointInCloud)
{
  this->p_ = p;
  if (iw)
    world2local (nn_cloud);
  else
    this->nn_cloud_ = nn_cloud;
  this->iw_ = iw;

  // protected params
  this->wc_ = 0.01;
  this->nc_ = 0;
  this->inc_ = false;
  this->ti_ = this->to_ = 0.0;
  this->iti_ = this->ito_ = false;
  this->zi_ = this->zo_ = 0.5;
  this->ne_ = 0;
  this->ine_ = false;

  // private params
  this->nd_ = nn_cloud_->points.size();
}

////////////////////////////////////////////////////////////////////////////////
void
PatchCoverage::setNc (int nc)
{
  this->nc_ = nc;
  this->inc_ = true;
}

////////////////////////////////////////////////////////////////////////////////
int
PatchCoverage::colon (int j, int i, int k, RowVectorXi &o)
{
  if ((i==0) || (i>0 && j>k) || (i<0 && j<k))
    return (0);
  
  int size = static_cast<int>((k-j)/i) + 1;
  o.resize(size);
  
  int it = j;
  int it_is = j+(size*i);
  if (it_is>0)
    it_is--;
  else
    it_is++;

  int iter = 0;
  do
  {
    o (iter) = it;
    
    if ((it >= it_is && it_is>0) || (it <= it_is && it_is<=0))
      break;
    
    it = it+i;
    iter++;
  } while (1);

  return (size);
}

////////////////////////////////////////////////////////////////////////////////
void
PatchCoverage::meshgrid (RowVectorXi xgv, VectorXi ygv, MatrixXi &X, MatrixXi &Y)
{
  X = xgv.replicate(ygv.size(),1);
  Y = ygv.replicate(1,xgv.size());
}

////////////////////////////////////////////////////////////////////////////////
MatrixXi
PatchCoverage::rot90 (MatrixXi A)
{
  return (A.transpose().colwise().reverse());
}

////////////////////////////////////////////////////////////////////////////////
MatrixXi
PatchCoverage::fliplr (MatrixXi A)
{
  return (A.rowwise().reverse().eval());
}

////////////////////////////////////////////////////////////////////////////////
MatrixXd
PatchCoverage::flipud (MatrixXd A)
{
  return (A.rowwise().reverse().eval());
}

////////////////////////////////////////////////////////////////////////////////
void
PatchCoverage::world2local (PointInCloudPtr nn_cloud)
{
  Affine3d aff;
  Matrix3d rrinv = p_.rexp(-p_.getR());
  aff.linear() = rrinv;
  aff.translation() = -rrinv * p_.getC();
  //Affine3f aff =  Affine3f::Identity();
  transformPointCloud (*nn_cloud, *nn_cloud_, aff);
}

////////////////////////////////////////////////////////////////////////////////
double
PatchCoverage::findCoverage ()
{
  // patch bounding box
  Matrix2f bb = (p_.getBB ()).cast<float>();
  float xmin = bb (0,0);
  float xmax = bb (0,1);
  float ymin = bb (1,0);
  float ymax = bb (1,1);

  // patch boundary area
  float ba = static_cast<float>(p_.getBA ());

  // expanding bounding box to fit all data
  PointIn minPt, maxPt;
  getMinMax3D(*nn_cloud_, minPt, maxPt);
  xmin = min (xmin, minPt.x);
  xmax = max (xmax, maxPt.x);
  ymin = min (ymin, minPt.y);
  ymax = max (ymax, maxPt.y);
  assert ((xmin<=0.0) && (0.0<=xmax));
  assert ((ymin<=0.0) && (0.0<=ymax));

  // infer wc from nc
  if (inc_ && (nc_>0))
  {
    wc_ = sqrt((xmax-xmin)*(ymax-ymin)/nc_);
  }
  
  assert(wc_>0);

  // cell grid left, right, top, bottom extents
  lc_ = abs(floor(xmin/wc_));
  rc_ = ceil(xmax/wc_);
  bc_ = abs(floor(ymin/wc_));
  tc_ = ceil(ymax/wc_);
  
  // num cell rows, cols
  int cr = tc_ + bc_;
  int cc = lc_ + rc_;

  // actual number of cells
  nc_ = cr*cc;

  // infer expected number of points per in-bounds cell
  if (!ine_)
    ne_ = (nd_*wc_*wc_)/ba;
  assert (ne_>=0);
  
  if (!iti_)
    ti_ = zi_*ne_;
  assert (ti_>=0);
  
  if (!ito_)
    to_ = zo_*ne_;
  assert (to_>=0);


  // Create Cell Matrices

  // cell at coords (i,j)=(0,0) has its lower left corner at local frame origin
  RowVectorXi ccx, ccy;
  colon (-lc_, 1, (rc_-1), ccx);
  colon ((tc_-1), -1, -bc_, ccy);
  MatrixXi cx,cy;
  meshgrid(ccx, ccy, cx, cy);

  // index of in-bounds points
  int nn_cloud_size = nn_cloud_->points.size();
  MatrixXd x_mat (nn_cloud_size,1);
  MatrixXd y_mat (nn_cloud_size,1);
  MatrixXd patch_bl (nn_cloud_size,1);
  for (int i=0; i<nn_cloud_->points.size(); i++)
  {
    x_mat (i,0) = static_cast<double>(nn_cloud_->points[i].x);
    y_mat (i,0) = static_cast<double>(nn_cloud_->points[i].y);
  }

  p_.bl (x_mat, y_mat, patch_bl);

  // centers of grid cells
  //TBD: no need to compute all of them
  RowVectorXd ctrsx = (ccx.cast<double>()*wc_).array() + (wc_/2.0);
  RowVectorXd ctrsy = (flipud(ccy.cast<double>())*wc_).array() + (wc_/2.0);
  double ctrsx_min = ctrsx(0)-(wc_/2.0);
  double ctrsy_min = ctrsy(0)-(wc_/2.0);

  int rows = patch_bl.rows();
  int cols = patch_bl.cols();
  int ni, no;
  ni = no = 0;
  
  MatrixXi ic = MatrixXi::Zero(cc,cr);
  MatrixXi oc = MatrixXi::Zero(cc,cr);
  for (int ii=0; ii<rows; ii++)
  {
    for (int jj=0; jj<cols; jj++)
    {
      if (patch_bl(ii,jj) <= 0.0) // inbound
      {
        int r = static_cast<int>((x_mat(ii,jj) - ctrsx_min)/wc_);
        int c = static_cast<int>((y_mat(ii,jj) - ctrsy_min)/wc_);
        
        ic(r,c) = ic(r,c) + 1;
        
        ni++;
      }
      else //outbound
      {
        int r = static_cast<int>((x_mat(ii,jj) - ctrsx_min)/wc_);
        int c = static_cast<int>((y_mat(ii,jj) - ctrsy_min)/wc_);
        
        oc(r,c) = oc(r,c) + 1;
        
        no++;
      }
    }
  }
  //TBD: assert no = nd_ - ni;

  MatrixXi rot_ic = MatrixXi::Zero(cr,cc);
  MatrixXi rot_oc = MatrixXi::Zero(cr,cc);
  rot_ic = rot90 (ic);
  rot_oc = rot90 (oc);

  // normalized area of intersection
  MatrixXd bc_mat;
  p_.bc (cx, cy, wc_, bc_mat);
  
  int bc_rows = bc_mat.rows();
  int bc_cols = bc_mat.cols();

  MatrixXd ai(bc_rows,bc_cols), ao (bc_rows,bc_cols);
  MatrixXi igd(bc_rows,bc_cols), ogd(bc_rows,bc_cols), gd(bc_rows,bc_cols);
  for (int ii=0; ii<bc_rows; ii++)
  {
    for (int jj=0; jj<bc_cols; jj++)
    {
      ai(ii,jj) = min (bc_mat(ii,jj)/(wc_*wc_), 1.0);
      ao(ii,jj) = 1.0 - ai(ii,jj);

      // bool index of good cells
      if (rot_ic(ii,jj) >= round(ai(ii,jj)*ti_))
        igd(ii,jj) = 1;
      else
        igd(ii,jj) = 0;
      
      if (rot_oc(ii,jj) <= round(ao(ii,jj)*to_))
        ogd(ii,jj) = 1;
      else
        ogd(ii,jj) = 0;

      gd(ii,jj) = igd(ii,jj) & ogd(ii,jj);
    }
  }

  // Generate Outputs
  int ng = gd.sum();
  
  // num empty outside cells
  int eo = 0;
  for (int ii=0; ii<bc_mat.rows(); ii++)
    for (int jj=0; jj<bc_mat.cols(); jj++)
      if ((ao(ii,jj) == 1) && (rot_ic(ii,jj)+rot_oc(ii,jj)==0))
        eo++;

  int num = ng-eo;
  int den = nc_-eo;

  assert(den>=0); // because nc>=eo
  assert(num<=den); // because ng<=nc

  if (num<0)
  {
    cerr << "(ng= " << ng << ")<(eo=" << eo << ")" << endl;
    num = 0;
  }

  pct_ = static_cast<double>(num) / static_cast<double>(den);
  
  return (pct_);
}
