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

// SPL libraries
#include "filter_nms.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
int
NMS::applyNMS ()
{
  if (n_ == 1) // scan-line
    return (NMS::nms3x3 (im_, om_, *id_om_));
  else if (n_ < 9) // scan-line with spiral indexing
    return (NMS::nmsScanline (im_, om_, *id_om_));
  else // quarter-block partitioning
    return (NMS::nmsQuarterblock (im_, om_, *id_om_));
}

///////////////////////////////////////////////////////////////////////////////
int
NMS::nms3x3 (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_)
{
  //Sizes
  int h_ = im_.rows();
  int w_ = im_.cols();

  lmn_ = 0;
  om_.fill (0); // binary output image
  MatrixXi skip (h_,2);
  skip.fill (0);
  int cur = 0;
  int next = 1;
  int r, tmp;
  
  for (int c=1; c<w_-1; c++)
  {
    r = 1;
    while (r<h_-1)
    {
      if (skip(r,cur))
      {
        // skip current pixel
        r=r+1;
        continue;
      }
      
      if (im_(r,c) <= im_(r+1,c)) // compare to pixel on the left
      {
        r=r+1;
        while ((r<h_-1) && (im_(r,c) <= im_(r+1,c)))
          r=r+1; //rising
        
        if (r==h_-1)
          break; // reach scanline's local maximum
      }
      else // compare to pixel on the right
      {
        if (im_(r,c) <= im_(r-1,c))
        {
          r=r+1;
          continue;
        }
      }
            
      skip(r+1,cur) = 1; // skip next pixel in the scanline
      
      //compare to 3 future then 3 past neighbors
      if (im_(r,c) <= im_(r-1,c+1))
      {
        r=r+1;
        continue;
      }
      skip(r-1,next) = 1; // skip future neighbors only
                
      if (im_(r,c) <= im_(r ,c+1))
      {
        r=r+1;
        continue;
      }
      skip(r ,next) = 1;
      
      if (im_(r,c) <= im_(r+1,c+1))
      {
        r=r+1;
        continue;
      }
      skip(r+1,next) = 1;
      
      if (im_(r,c) <= im_(r-1,c-1))
      {
        r=r+1;
        continue;
      }
      
      if (im_(r,c) <= im_(r ,c-1))
      {
        r=r+1;
        continue;
      }
        
      if (im_(r,c) <= im_(r+1,c-1))
      {
        r=r+1;
        continue;
      }
      // a new local maximum is found
      om_(r,c) = 1;
      id_om_.push_back (r * im_.cols () + c);
      lmn_++;
      r=r+1;
    } 
    
    // swap mask indices
    tmp = cur;
    cur = next;
    next = tmp;

    // reset next scanline mask
    skip.col(next).fill(0);
  }

  return (lmn_);
}

///////////////////////////////////////////////////////////////////////////////
void
NMS::spiralindex (VectorXi &r, VectorXi &c)
{
  int index_rc = 0;
  r(0) = 0;
  c(0) = 0;

  int run = 0;
  int dr, dc;
  for (int ii=0; ii<n_; ii++)
  {
    run++;
    dr = -1;
    dc = 0;
    for (int jj=0; jj<run; jj++)
    {
      index_rc++;
      r(index_rc) = r(index_rc-1) + dr; 
      c(index_rc) = c(index_rc-1) + dc; 
    }
    dr = 0;
    dc = 1;
    for (int jj=0; jj<run; jj++)
    {
      index_rc++;
      r(index_rc) = r(index_rc-1) + dr; 
      c(index_rc) = c(index_rc-1) + dc; 
    }

    run++;
    dr = 1;
    dc = 0;
    for (int jj=0; jj<run; jj++)
    {
      index_rc++;
      r(index_rc) = r(index_rc-1) + dr; 
      c(index_rc) = c(index_rc-1) + dc; 
    }
    dr = 0;
    dc = -1;
    for (int jj=0; jj<run; jj++)
    {
      index_rc++;
      r(index_rc) = r(index_rc-1) + dr; 
      c(index_rc) = c(index_rc-1) + dc; 
    }
  }

  dr = -1;
  dc = 0;
  for (int jj=0; jj<run; jj++)
  {
    index_rc++;
    r(index_rc) = r(index_rc-1) + dr; 
    c(index_rc) = c(index_rc-1) + dc; 
  }

}

double
sign(double i)
{
  if (i>0)
    return 1;
  if (i<0)
    return -1;

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
int
NMS::nmsScanline (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_)
{
  lmn_ = 0;
  
  // binary output image
  om_.fill(0);
  
  // scanline mask
  VectorXd scanline(h_);
  VectorXi skip(h_);

  int rc_size = (2*n_+1)*(2*n_+1);
  VectorXi dr(rc_size), dr_tmp(rc_size);
  VectorXi dc(rc_size), dc_tmp(rc_size);
  NMS::spiralindex (dr_tmp, dc_tmp);

  int j = 0;
  for (int i=0; i<dc_tmp.rows(); i++)
  {
    if (dc_tmp(i) != 0)
    {
      dr(j) = dr_tmp(i);
      dc(j) = dc_tmp(i);
      j++;
    }
  }
  dr.conservativeResize(j);
  dc.conservativeResize(j);

  VectorXd resp (h_);
  resp(0) = 0;
  resp(h_-1) = 0;

  VectorXd diff1 (h_-1);
  VectorXd diff2 (h_-1);
  for (int c=n_; c<w_-n_; c++)
  {
    // current scanline
    scanline = (im_.col(c)).transpose();
    skip = VectorXi::Zero(h_);

    // discrete Hessian
    diff1 = scanline.segment(1,h_) - scanline.segment(0,h_-1);
    diff2 = diff1.unaryExpr(std::ptr_fun(sign));
    resp.segment(1,h_-1) = diff2.segment(1,h_-1) - diff2.segment(0,h_-2);
    resp(h_-1) = 0;

    // peak
    double curPix;
    bool whoops;
    for (int r=n_; r<h_-n_; r++)
    {
      // Skip if the point is invalid
      if (isnan (im_(r,c)))
        continue;

      if (resp(r) == -2.0)
      {
        if (skip(r))
          continue;
        curPix = scanline(r);
        whoops = false;
        
        int jj;
        int jjo = 0;
        for (jj=r+1; jj<r+n_+1; jj++)
        {
          if (resp(jj)!=0.0)
          {
            jjo = jj;
            break; // downhill
          }
          else
          {
            jjo = jj;
          }
        }

        for (int jjj=jjo; jjj<r+n_+1; jjj++)
        {
          if (curPix <= scanline(jjj))
          {
            whoops=true;
            break;
          }
          skip(jjj) = 1; // skip future pixels if < current one
        }
        
        if (whoops)
          continue; //skip to next scanline peak

        for (int jj=r-1; jj > r-n_-1; jj--)
        {
          if (resp(jj)!=0.0)
          {
            jjo = jj;
            break;
          }
          else
          {
            jjo = jj;
          }
        }
       
        for (int jjj=jjo; jjj>r-n_-1; jjj--)
        {
          if (curPix <= scanline(jjj))
          {
            whoops=true;
            break;
          }
        }
         
        if (whoops)
          continue; // skip to next scanline peak

        // if reach here, current pixel is a (2n+1)−maximum
        jjo = -1;
        for (int jj=0; jj<dr.size(); jj++) // visit neighborhood in spiral order
        {
          // Skip invalid point
          if (isnan (im_(r+dr(jj),c+dc(jj))))
            continue;

          if (im_(r,c) <= im_(r+dr(jj),c+dc(jj)))
          {
            jjo = jj;
            break;
          }
          else
          {
            jjo = jj;
          }
        }
       
        // TBD: check if this solves the seg fault prob
        if (jjo==-1)
          break;

        if (jjo>=dr.size()-1 && im_(r,c)>im_(r+dr(jjo),c+dc(jjo)))
        {
          om_(r,c) = 1; // a new (2n+1)x(2n+1)−maximum is found
          id_om_.push_back (r * im_.cols () + c);
          lmn_++;
        }
      }
    }
  }
 
  return (lmn_);
}

///////////////////////////////////////////////////////////////////////////////
int
NMS::nmsQuarterblock (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_)
{
  om_.fill(0);
  int m = (int) (floor(((double)n_+1.0)/2.0)); 
  
  // hh x ww = number of m x m blocks
  int hh = (int) (floor((double)h_/(double)m));

  // starting from (m,m) offset
  int ww = (int) (floor((double)w_/(double)m));

  MatrixXd val (hh*m,ww*m);
  MatrixXd max_val (hh,ww); //canditates
  
  // Indices in the full val matrix
  MatrixXd R (hh,ww);
  int r;
  MatrixXd C (hh,ww);
  int c;

  val = im_.block(0,0,(hh*m),(ww*m));
  for (int i=0; i<hh; i++)
  {
    for (int j=0; j<ww; j++)
    {
      max_val(i,j) = (val.block(i*m,j*m,m,m)).maxCoeff(&r,&c);
      R(i,j) = i*m + r;
      C(i,j) = j*m + c;
    }
  }

  MatrixXd mask0 (hh,ww);
  vector<int> id_mask0;
  NMS::nms3x3(max_val, mask0, id_mask0);

  lmn_ = 0;
  MatrixXd cmp(2*n_+1, 2*n_+1);
  for (int i=0; i<hh; i++)
  {
    for (int j=0; j<ww; j++)
    {
      if (mask0(i,j)==0)
        continue;

      // out of bound
      if (R(i,j)<n_ || C(i,j)<n_ || R(i,j)>h_-n_-1 || C(i,j)>w_-n_-1)
        continue;

      // compare to full (2n+1)x(2n+1) block for code simplicity
      cmp = im_.block(R(i,j)-n_, C(i,j)-n_, 2*n_+1, 2*n_+1);

      if (cmp.maxCoeff() <= max_val(i,j))
      {
        om_(R(i,j),C(i,j)) = 1;
        lmn_++;
      }
    }
  }

  return (lmn_);
}
