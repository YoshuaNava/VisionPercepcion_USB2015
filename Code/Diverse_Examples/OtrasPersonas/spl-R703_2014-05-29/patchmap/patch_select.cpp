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

#include "patch_select.h"

////////////////////////////////////////////////////////////////////////////////
PatchSelect::PatchSelect ()
{
  vp_x_ = vp_y_ = vp_z_ = 0.0;
  use_pt_ = use_curv_ = use_slope_ = false;
}

////////////////////////////////////////////////////////////////////////////////
PatchSelect::~PatchSelect () {};

////////////////////////////////////////////////////////////////////////////////
void
PatchSelect::setInputMapPatches (vector<MapPatch>* input_map_patches)
{
  this->input_map_patches_ = input_map_patches;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSelect::setPt (Vector3d pt, double pt_r)
{
  this->pt_ = pt;

  if (pt_r < 0)
  {
    print_warn ("[PatchSelect::setPt] Radius should be positive.");
    pt_r = abs (pt_r);
  }

  this->pt_r_ = pt_r;

  // set the flag
  use_pt_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSelect::setCurv (double min_curv, double max_curv)
{
  this->min_curv_ = min_curv;
  this->max_curv_ = max_curv;

  // set the flag
  use_curv_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSelect::setVp (float vp_x, float vp_y, float vp_z)
{
  this->vp_x_ = vp_x;
  this->vp_y_ = vp_y;
  this->vp_z_ = vp_z;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSelect::setSlope (Vector3f slope_vec, double min_slope, double max_slope)
{
  this->slope_vec_ = slope_vec;
  this->min_slope_ = min_slope;
  this->max_slope_ = max_slope;

  // set the flag
  use_slope_ = true;
}

////////////////////////////////////////////////////////////////////////////////
vector<int>
PatchSelect::getOutputMapPatches ()
{
  return (output_map_patches_);
}

////////////////////////////////////////////////////////////////////////////////
double
PatchSelect::findPtDist (Vector3d c)
{
  // euclidean (l2) distance
  return ((pt_-c).norm());
}

////////////////////////////////////////////////////////////////////////////////
double
PatchSelect::findMaxCurv (VectorXd curv)
{
  return (curv.maxCoeff());
}

////////////////////////////////////////////////////////////////////////////////
double
PatchSelect::findMinCurv (VectorXd curv)
{
  return (curv.minCoeff());
}

////////////////////////////////////////////////////////////////////////////////
double
PatchSelect::findSlope (Vector3f vec)
{
  return (findAngle(vec,slope_vec_));
}

////////////////////////////////////////////////////////////////////////////////
double
PatchSelect::findAngle (Vector3f a, Vector3f b)
{
  double angle;

  angle = atan2((a.cross(b)).norm(), a.dot(b));

  return (angle);
}

////////////////////////////////////////////////////////////////////////////////
int
PatchSelect::filter ()
{
  vector<MapPatch>& imp = *input_map_patches_; // not copied

  output_map_patches_.clear();
  for (int i=0; i<imp.size(); i++)
  {
    // proceed only if the patch is valid
    if (!imp[i].getIsValid())
      continue;

    // get the patch
    Patch p = imp[i].getP();
   
    // filter wrt to the point
    double dist = findPtDist (p.getC());
    
    if (use_pt_)
    {
      if (dist > pt_r_)
        continue;
    }

    // filter wrt the curvature
    if (use_curv_)
    {
      double min_k = findMinCurv(p.getK());
      double max_k = findMaxCurv(p.getK());
      if ((max_k > max_curv_) || (min_k < min_curv_))
        continue;
    }

    // filter wrt the slope
    if (use_slope_)
    {
      // find normal and flip it towards viewpoint
      Vector3f normal = ((p.getRot()).col(2)).cast<float>();
      Vector3f nb = p.getC().cast<float>();
      flipNormalTowardsViewpoint (PointXYZ(nb.x(),nb.y(),nb.z()),
                                  vp_x_, vp_y_, vp_z_, normal);

      double slope_diff = findSlope(normal);
      if (slope_diff < M_PI)
      {
        if (slope_diff > max_slope_)
          continue;
      }
      else
      {
        if ((M_PI-slope_diff) > max_slope_)
          continue;
      }
    }

    // set the weight
    imp[i].setWeight (dist);

    // the output vector
    output_map_patches_.push_back (i);
  }

  if (!output_map_patches_.empty())
    return (output_map_patches_[0]);
  else
    return (-1);
}
