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

#ifndef FILTER_DEC_H_
#define FILTER_DEC_H_

// PCL headers
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>

using namespace std;
using namespace pcl;

/** \brief Decimates the data by the given factor.  E.g. decimate=2 will discard
  * rows and columns 2,4,6,etc.; decimate=3 will discard rows and columns 2,3,5,
  * etc.
  *
  * \author Dimitrios Kanoulas
  */
class DecimationFilter : public pcl::Filter<PointXYZ>
{
  using pcl::Filter<PointXYZ>::input_;
  typedef pcl::Filter<PointXYZ>::PointCloud PointCloud;

  public:
  /** \brief Empty constructor. */
  DecimationFilter () :
    decimate_factor_ (2)
  {};

  /** \brief Set the decimation factor. */
  inline void
  setDecimateFactor (int decimate_factor)
  {
    if (decimate_factor > 0)
      this->decimate_factor_ = decimate_factor;
  }

  /** \brief Get the decimation factor. */
  inline int
  getDecimateFactor ()
  {
    return (this->decimate_factor_);
  }

  /** \brief Filter the input data and store the results into output.
    *
    * \param[out] output the result point cloud
    */
  void
  applyFilter (PointCloud &output);

  protected:
  /** \brief Factor of decimation. */
  int decimate_factor_;
};

#endif // FILTER_DEC_H_
