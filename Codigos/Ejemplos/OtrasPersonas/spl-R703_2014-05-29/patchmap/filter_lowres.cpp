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
#include "filter_lowres.h"

void
LowerResolutionFilter::applyFilter (PointCloud &output)
{
  if (!input_->isOrganized ())
  {
    cerr << "Input cloud needs to be organized.\n" << endl;
    return;
  }

  output.points.resize ((input_->width/lowres_factor_) * (input_->height/lowres_factor_));

  float val;
  int pt_i=0;

  for (int y = 0; y < input_->height; y = y+lowres_factor_)
  {
    for (int x = 0; x < input_->width; x = x+lowres_factor_)
    {
      val = 0.0f;
      for (int i=0; i<lowres_factor_; i++)
      {
        for (int j=0; j<lowres_factor_; j++)
        {
          val = val + (*input_)(x, y).z;
        }
      }
      val = val/(lowres_factor_*lowres_factor_);
      output.points[pt_i].x = (*input_)(x, y).x;
      output.points[pt_i].y = (*input_)(x, y).y;
      output.points[pt_i].z = val;
      pt_i++;
    }
  }

  // Fill the rest of the fields
  output.width = static_cast<uint32_t> (input_->width/lowres_factor_);
  output.height = static_cast<uint32_t> (input_->height/lowres_factor_);
  output.is_dense = input_->is_dense;
  output.sensor_origin_ = input_->sensor_origin_;
  output.sensor_orientation_ = input_->sensor_orientation_;
}
