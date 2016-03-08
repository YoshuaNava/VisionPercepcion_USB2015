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

#ifndef SAMPLE_FILTER_H_
#define SAMPLE_FILTER_H_

// STD headers
#include <float.h>

// SPL headers
#include "filter_dec.h"
#include "filter_lowres.h"
#include "filter_approx_voxel_grid_organized.h"

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/median_filter.h>

using namespace std;
using namespace pcl;

class SampleFilter
{
  /** \brief Filters the point cloud samples.
    *
    * Given an organized point cloud it applies various filters (in a
    * particular order), keeping the cloud organized when possible.
    *
    * \author Dimitrios Kanoulas
    */

  public:
    typedef PointCloud<PointXYZ> PointCloudIn;
    typedef PointCloudIn::Ptr PointCloudInPtr;
    typedef PointCloudIn::ConstPtr PointCloudInConstPtr;
    
    /** Contructor. */
    SampleFilter (const PointCloudInConstPtr &cloud) :
      cloud_filtered_ (new PointCloudIn),
      cloud_temp_ (new PointCloudIn),
      cloud_ (cloud),
      do_pt_filter_ (false),
      min_pt_limit_ (FLT_MIN),
      max_pt_limit_ (FLT_MAX),
      do_sor_filter_ (false),
      do_ror_filter_ (false),
      do_fb_filter_ (true),
      do_m_filter_ (false),
      do_d_filter_ (false),
      do_lr_filter_ (true),
      do_avg_filter_ (false)
    {};

    /** Destructor. */
    virtual
    ~SampleFilter () {};

    /** Filter the input point cloud. */
    void
    filter();

    /** \brief Filtered point cloud. */
    PointCloudInPtr cloud_filtered_;

    /** \brief Temp point cloud. */
    PointCloudInPtr cloud_temp_;

    /** \brief Set whether to apply Passthrough filter. */
    inline void
    setDoPtFilter (bool flag)
    {
      this->do_pt_filter_ = flag;
    }

    /** \brief Set limit_min for Passthrough */
    inline void
    setMinPtLimit (double lm_arg)
    {
      this->min_pt_limit_ = lm_arg;
    }

    /** \brief Set limit_max for Passthrough */
    inline void
    setMaxPtLimit (double lm_arg)
    {
      this->max_pt_limit_ = lm_arg;
    }
    
    /** \brief Set whether to apply Statistical Outlier Removal filter. */
    inline void
    setDoSorFilter (bool flag)
    {
      this->do_sor_filter_ = flag;
    }

    /** \brief Set whether to apply Radius Outlier Removal filter. */
    inline void
    setDoRorFilter (bool flag)
    {
      this->do_ror_filter_ = flag;
    }
    
    /** \brief Set whether to apply Fast Bilateral filter. */
    inline void
    setDoFbFilter (bool flag)
    {
      this->do_fb_filter_ = flag;
    }
    
    /** \brief Set whether to apply Median filter. */
    inline void
    setDoMFilter (bool flag)
    {
      this->do_m_filter_ = flag;
    }
    
    /** \brief Set whether to apply Decimation filter. */
    inline void
    setDoDFilter (bool flag)
    {
      this->do_d_filter_ = flag;
    }
    
    /** \brief Set whether to apply Lower Resolution filter. */
    inline void
    setDoLrFilter (bool flag)
    {
      this->do_lr_filter_ = flag;
    }

    /** \brief Set whether to apply Approximate Voxel Grid filter. */
    inline void
    setDoAvgFilter (bool flag)
    {
      this->do_avg_filter_ = flag;
    }

  private:
    /** \brief Pointer to the input point cloud. */
    PointCloudInConstPtr cloud_;

    /** Whether to apply Passthrough filter. */
    bool do_pt_filter_;
    
    /** \brief Min field (Passthrough) */
    double min_pt_limit_;
    
    /** \brief Max field (Passthrough) */
    double max_pt_limit_;
    
    /** Whether to apply Statistical Outlier Removal filter.
      *
      * \note It is not working in real time.
      */
    bool do_sor_filter_;
    
    /** Whether to apply Radius Outlier Removal filter.
      *
      * \note It is not working in real time.
      */
    bool do_ror_filter_;

    /** Whether to apply Fast Bilateral filter.
      *
      * \note When using NAN points, it changes z-value to valid!
      */
    bool do_fb_filter_;
    
    /** Whether to apply Median filter. */
    bool do_m_filter_;
    
    /** Whether to apply Decimation filter. */
    bool do_d_filter_;
    
    /** Whether to apply Lower Resolution filter. */
    bool do_lr_filter_;
    
    /** Whether to apply Approximate Voxel Grid filter. */
    bool do_avg_filter_;
};

#endif // #ifndef SURFACE_PATCH_H_
