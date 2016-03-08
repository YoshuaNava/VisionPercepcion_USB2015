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
#include "patch_set.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
vector<Patch>
PatchSet::getPatches ()
{
  return (this->patches_);
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::create_patches ()
{
  // rotation and translation matrices (unset)
  Vector3d r = MatrixXd::Zero(3,1);
  Vector3d c = MatrixXd::Zero(3,1);
  
  // elliptic paraboloid
  Vector2d d_e_e (1.5, 1.0);
  Vector2d k_e_e (-1.0, -2.0);
  
  Patch e_e_patch("ell parab", Patch::s_e, Patch::b_e,
                  d_e_e, k_e_e, r, c);
  e_e_patch.setSS (0.5*dss_);
  e_e_patch.setGD (2);
  patches_.push_back(e_e_patch);
  
  // hyperbolic paraboloid
  Vector2d d_h_e (1.5, 1.0);
  Vector2d k_h_e (0.5, -1.0);
  
  Patch h_e_patch("hyp parab", Patch::s_h, Patch::b_e,
                  d_h_e, k_h_e, r, c);
  h_e_patch.setSS (0.5*dss_);
  h_e_patch.setGD (2);
  patches_.push_back(h_e_patch);
  
  // cylindric paraboloid
  Vector2d d_y_r (2.0, 0.8);
  Matrix<double,1,1> k_y_r;
  k_y_r << -2.0;
  
  Patch y_r_patch("cyl parab", Patch::s_y, Patch::b_r,
                  d_y_r, k_y_r, r, c);
  y_r_patch.setSS (0.5*dss_);
  y_r_patch.setGD (4);
  patches_.push_back(y_r_patch);

  // circular paraboloid
  Matrix<double,1,1> d_o_c;
  d_o_c << 1.8;
  Matrix<double,1,1> k_o_c;
  k_o_c << -1.0;
  
  Patch o_c_patch("circular paraboloid", Patch::s_o, Patch::b_c,
                  d_o_c, k_o_c, r, c);
  o_c_patch.setSS (0.5*dss_);
  o_c_patch.setGD (4);
  patches_.push_back(o_c_patch);
  
  // plane (circle)
  Matrix<double,1,1> k_p_c;
  k_p_c << 0.0;
  Matrix<double,1,1> d_p_c;
  d_p_c << 2.0;
  
  Patch p_c_patch("plane (circle)", Patch::s_p, Patch::b_c,
                  d_p_c, k_p_c, r, c);
  p_c_patch.setSS (0.5*dss_);
  p_c_patch.setGD (2);
  patches_.push_back(p_c_patch);
  
  // plane (ellipse)
  Matrix<double,1,1> k_p_e;
  k_p_e << 0.0;
  Vector2d d_p_e (1.7, 2.5);
  
  Patch p_e_patch("plane (ellipse)", Patch::s_p, Patch::b_e,
                  d_p_e, k_p_e, r, c);
  p_e_patch.setSS (0.5*dss_);
  p_e_patch.setGD (2);
  patches_.push_back(p_e_patch);
  
  // plane (aa rect)
  Matrix<double,1,1> k_p_r (0.0);
  Vector2d d_p_r (2.0, 2.5);
  
  Patch p_r_patch("plane (aa rect)", Patch::s_p, Patch::b_r,
                  d_p_r, k_p_r, r, c);
  p_r_patch.setSS (0.5*dss_);
  p_r_patch.setGD (4);
  patches_.push_back(p_r_patch);

  return;
}

////////////////////////////////////////////////////////////////////////////////
int
PatchSet::run_tests ()
{
  // for timing
  StopWatch timer_;

  // run tests on patches
  for (int i=0; i<patches_.size(); i++)
  {
    cout << "Patch " << i << "/" << patches_.size()-1 << endl;
    
    // elaborate patch's parameters
    //patches_.at(i).gs();
   
    // patch_plot test
    cout << "showPatch" << endl;

    double patch_plot_ms_ = timer_.getTime ();
    if (do_patch_plot_)
      showPatch (patches_.at(i));
    cerr << "plot time: " << timer_.getTime() - patch_plot_ms_ << "ms" << endl;

    cout << "patchSample" << endl;
    // patch_sample test
    if (do_patch_sample_ || do_patch_fit_)
    {
      patchSample (patches_.at(i));
      
      // show samples
      if (do_sample_plot_)
        showSample ();
    }

    cout << "Do patch fit" << endl;
    // patch fit test
    if (do_patch_fit_)
    {
      // patch fit
      doPatchFit ();
      
      // elaborate patch's parameters
      //fitted_p_.gs();
      showPatch (fitted_p_);
    }
  
    // show the patch
    if (do_patch_plot_ || do_sample_plot_ || do_patch_fit_)
      showViewer();
  
    // clear the viewer for the next patch
    patch_viewer_->removeAllShapes ();
    patch_viewer_->removeAllPointClouds ();
    next_patch_ = false;
  }

  return (1);
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::showViewer ()
{
  // viewer axis
  patch_viewer_->addCoordinateSystem (1.0,0);
  
  // show the patches in one fig
  while (!patch_viewer_->wasStopped () && !next_patch_)
  {
    patch_viewer_->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000)); 
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::showPatch (Patch p)
{
  // set patch parameteres
  //p.setSS (0.2);
  p.setAutoSS ();
  p.infer_params ();
  p.gs();
  
  // plot the patch
  PatchPlot patch_plot (p);
  patch_plot.showPatch (patch_viewer_, 0, 1, 0,  boost::to_string(p.getID()) +
                        "_patch");
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::patchSample (Patch p)
{
  MatrixXd mx, my, mz, r;
  double cx (-1.0), cy (-1.0), cz (6.0); // center of projection

  PatchSample patch_sample (p);

  // frustum
  double fh = M_PI/3.0;
  double fv = M_PI/3.0;
  Vector3d fp;
  fp << -cx, -cy, -cz;
  fp = fp/fp.norm();
  Vector3d fu;
  fu << 0.0, 1.0, 0.0;
  int nh = 20;
  int nv = 20;
  patch_sample.sample_frustum(fh, fv, nh, nv, fp, fu, mx, my, mz);

  // samples
  patch_sample.findSamples(sample_cloud_, mx, my, mz, cx, cy, cz);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::showSample ()
{
  patch_viewer_->addPointCloud<PointXYZ> (sample_cloud_, "sample cloud");
  patch_viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE,
                                                   10, "sample cloud");
}

////////////////////////////////////////////////////////////////////////////////
void
PatchSet::doPatchFit ()
{
  // patch fit
  PatchFit patchFit (sample_cloud_);
  patchFit.fit ();

  // set the output patch
  fitted_p_ = *patchFit.p_;
}

////////////////////////////////////////////////////////////////////////////////
int
PatchSet::test_patches (int (*tfun)(Patch p))
{
  return (1);
}
