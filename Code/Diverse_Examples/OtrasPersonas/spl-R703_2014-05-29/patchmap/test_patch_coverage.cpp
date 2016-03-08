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
#include "patch_coverage.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

/** \brief Main */
int
main (int argc, char** argv)
{
  VectorXd d(2), k(1), r(3), c(3);
  //d << 1.5, 1.0;
  d << 2, 0.8;
  //k << -1.0, -2.0;
  k << -2.0;
  r << 0.0, 0.0, 0.0;
  c << 0.0, 0.0, 0.0;
  //Patch p ("elliptic paraboloid", Patch::s_e, Patch::b_e, d, k, r, c);
  Patch p ("cylindric paraboloid", Patch::s_y, Patch::b_r, d, k, r, c);
  p.infer_params();
  p.gs();

  cout << "Creating x,y,z" << endl;
  VectorXd x(100), y(100);
  x << -2.238079, -2.414758, -1.329011, 0.281289, 1.801683, -0.469432, -2.785730, 1.072411, 1.546441, -0.646638, 0.932867, -1.972880, 1.236277, -1.338462, -1.097403, -0.710649, 1.593101, 1.771199, -1.878764, -0.326483, 1.256189, 1.528120, -1.343850, -0.009816, 0.511607, -1.657128, 1.507602, -1.469429, 0.035742, 1.194460, -1.474307, 1.885709, -0.900097, -1.820428, -1.493497, 0.696268, -0.160267, -0.890043, 1.984972, 0.511585, 0.298342, 2.503162, -1.284966, 1.543201, 1.522375, -0.717325, 0.184785, 1.675003, 0.412942, -0.183656, -0.977264, -2.745413, 0.129899, 1.908891, 0.957632, 0.111570, 0.893949, 1.801983, -0.277214, -0.405651, 1.951883, -1.959668, -0.654373, 1.820186, -0.604453, 0.161255, -0.499203, 0.941159, -1.248096, -1.996990, -0.765542, -1.811290, -0.061874, -1.385283, -0.462986, 0.287225, -0.493535, 0.998033, 0.234759, 1.188633, 0.999167, -1.931205, -2.804395, 0.367199, 2.291199, -1.857400, -0.786501, -0.235644, 0.868587, -0.742367, -1.854458, -0.430482, -2.276330, 0.537045, -1.642874, -0.692285, 0.497918, -1.489163, -1.257356, 0.702545;

  y << -0.453084, 0.244757, -0.568869, 0.369790, -0.834293, 0.092022, 1.108555, 0.761528, 0.884867, -0.240522, -0.576311, 0.720164, -0.164607, -0.763567, 0.191291, 0.292932, -0.357714, 0.031799, -0.235661, -0.624201, -0.758621, -0.624114, -0.198559, -0.025794, -0.313808, -0.933113, 0.672605, -0.264627, -0.619941, -0.230611, -0.352419, 0.770866, -0.794424, 0.357877, 0.556134, 0.354590, -0.117783, 0.112821, -0.488830, 0.587263, -0.746508, 0.448261, -0.759573, -0.315637, 0.301485, 0.672546, -0.031700, -0.153939, 0.020421, 0.025852, 0.707595, 0.236457, 0.470278, 0.332474, 0.074074, 0.370670, 0.767955, 0.524061, 1.124758, 0.075201, -0.419650, 0.669125, -0.183713, -0.560468, -0.525587, -0.143796, 0.065143, -0.102182, 0.043325, -0.622303, -0.506245, 0.412340, 0.468337, -0.373290, 0.673247, 0.420797, 0.245209, -0.090122, -0.181562, -0.093801, 0.648383, -0.426068, -0.777901, 0.532219, -0.063634, -0.381301, 0.257734, -0.739811, -0.554252, 0.637200, -0.747211, -0.510004, 0.182903, 0.440072, 0.111823, -0.178251, 0.346663, 0.354282, 0.429640, 0.325888;
  
  PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(x.size());
  for (int i=0; i<x.size(); i++)
  {
    cloud->points[i].x = x(i);
    cloud->points[i].y = y(i);
  }
  cloud->width = x.size();
  cloud->height = 1;
  
  cout << "Computing Coverage" << endl;
  
  StopWatch timer_;
  double cov_ms_ = timer_.getTime ();
  PatchCoverage pc (p, cloud, false);
  pc.setNc (50);
  double pct = pc.findCoverage ();
  cerr << "cov pct: " << pct << endl;
  cerr << "cov time: " << timer_.getTime() - cov_ms_ << "ms" << endl;
}
