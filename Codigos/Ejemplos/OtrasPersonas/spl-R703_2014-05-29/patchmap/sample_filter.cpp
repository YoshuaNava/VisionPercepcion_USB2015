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
#include "sample_filter.h"

using namespace std;
using namespace pcl;

///////////////////////////////////////////////////////////////////////////////
void
SampleFilter::filter ()
{
  // Deep copy the point cloud to a filtered one
  cloud_filtered_ = cloud_->makeShared();

  // Passthrough filter
  if (do_pt_filter_)
  {
    PassThrough<PointXYZ> pt;
    pt.setInputCloud (cloud_filtered_);
    pt.setFilterFieldName ("z");
    pt.setFilterLimits (static_cast<float> (min_pt_limit_),
                        static_cast<float> (max_pt_limit_));
    pt.setKeepOrganized (true);
    pt.filter (*cloud_filtered_);
  }

  // Statistical Outlier Removal filter
  if (do_sor_filter_)
  {
    // Create the filtering object
    StatisticalOutlierRemoval<PointXYZ> sor;
    sor.setInputCloud (cloud_filtered_);
    sor.setMeanK (10);
    sor.setStddevMulThresh (1.0);
    sor.setKeepOrganized (true);
    sor.filter (*cloud_filtered_);
  }

  // Radius Outlier Removal filter
  if (do_ror_filter_)
  {
    RadiusOutlierRemoval<PointXYZ> ror;
    ror.setInputCloud (cloud_filtered_);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius (2);
    ror.setKeepOrganized (true);
    ror.filter (*cloud_filtered_);
  }
  
  // Fast Bilateral filter
  if (do_fb_filter_)
  {
    FastBilateralFilterOMP<PointXYZ> fb;
    fb.setInputCloud (cloud_filtered_);
    fb.filter (*cloud_filtered_);
  }

  // Median filter
  if (do_m_filter_)
  {
    MedianFilter<PointXYZ> m;
    m.setInputCloud (cloud_filtered_);
    m.setWindowSize (2);
    m.filter (*cloud_filtered_);
  }

  // Decimation filter
  if (do_d_filter_)
  {
    DecimationFilter d;
    d.setInputCloud (cloud_filtered_);
    d.setDecimateFactor (2);
    d.filter (*cloud_filtered_);
  }
  
  // Lower Resolution filter
  if (do_lr_filter_)
  {
    LowerResolutionFilter lr;
    lr.setInputCloud (cloud_filtered_);
    lr.setLowResFactor (2);
    lr.filter (*cloud_filtered_);
  }
  
  // Approximate Voxel-Grid filter
  if (do_avg_filter_)
  {
    ApproximateVoxelGridOrganized avg;
    avg.setLeafSize (0.01f, 0.01f, 0.01f);
    avg.setInputCloud (cloud_filtered_);
    avg.setKeepOrganized (false);
    avg.setUseCentroid (false);
    avg.filter (*cloud_filtered_);
    avg.organizeOutput(*cloud_filtered_, *cloud_temp_);
    cloud_filtered_ = cloud_temp_;
  }
}
