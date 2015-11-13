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

#include "patch.h"

using namespace std;
using namespace Eigen;

/** \brief Main */
int
main (int argc, char** argv)
{
  Patch p ("test patch",Patch::s_e,Patch::b_e,
           MatrixXd::Zero(1,1), MatrixXd::Zero(1,1),
           MatrixXd::Zero(3,1), MatrixXd::Zero(3,1));
  Eigen::Vector3d r;
  r << 1.0, 2.0, 3.0;
  cout << "rreparam(r): " << p.rreparam(r) << endl;
  cout << "rexp(r): " << endl << p.rexp(r) << endl;
  cout << "drexp(r): " << endl << p.drexp(r)[0] << endl
       << p.drexp(r)[1] << endl << p.drexp(r)[2] << endl;
  cout << "rcanon2(r,2,3): " << endl << p.rcanon2(r,2,3) << endl;

  Eigen::Matrix3d m;
  m << -0.6949,  0.7135, 0.0893,
       -0.1920, -0.3038, 0.9332,
       -0.6930,  0.6313, 0.3481;
  r = p.rlog(m);
  cout << "r: " << endl << r << endl;
  return (1);
}
