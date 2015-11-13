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

// SPL headers
#include "patch.h"
#include "patch_set.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

/** \brief Tests the isSimilar patch method to different types of patches.
  *
  * \author Dimitrios Kanoulas
  */
int
main (int argc, char** argv)
{
  // create a set of patches
  PatchSet *patch_set;
  patch_set = new PatchSet;
  vector<Patch> patches = patch_set->getPatches ();

  double dss_ = 0.4;
  Vector3d r = MatrixXd::Zero(3,1);
  Vector3d c = MatrixXd::Zero(3,1);
  Vector2d d_e_e (1.5, 1.0);
  Vector2d k_e_e (-1.0, -2.0);
  Patch patch_a("ell parab", Patch::s_e, Patch::b_e,
                  d_e_e, k_e_e, r, c);
  patch_a.setSS (0.5*dss_);
  patch_a.setGD (2);

  //Matrix<double,1,1> d_b (1.5);
  Matrix3d rot = Matrix3d::Identity();
  rot (0,0) = cos(deg2rad(175.0));
  rot (0,1) = -sin(deg2rad(175.0));
  rot (1,0) = sin(deg2rad(175.0));
  rot (1,1) = cos(deg2rad(175.0));
  
  c << 0.0, 0.0, 0.1;
  
  Vector2d d_b (1.5, 1.1);
  Vector2d k_b (-1.0, -2.1);
  Patch patch_b ("ell parab", Patch::s_e, Patch::b_e,
                  d_b, k_b, r, c);
  patch_b.setR (patch_b.rlog (patch_b.getRot()*rot));
  patch_b.setSS (0.5*dss_);
  patch_b.setGD (2);

  cout << "similar: " << patch_a.isSimilar (patch_b, 0.11, 0.11, 11.0, 0.11) << endl; 
  delete (patch_set);
}
