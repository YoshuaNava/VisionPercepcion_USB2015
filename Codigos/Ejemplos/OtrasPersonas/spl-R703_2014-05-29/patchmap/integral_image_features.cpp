/* Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
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
#include "integral_image_features.h"

using namespace std;
using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::initData ()
{
  initCovarianceMatrixMethod ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::setRectSize (const int width, const int height)
{
  rect_width_      = width;
  rect_width_2_    = width/2;
  rect_width_4_    = width/4;
  rect_height_     = height;
  rect_height_2_   = height/2;
  rect_height_4_   = height/4;
}

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::setRectSize (const float rect_size)
{
  if (rect_size >= 0)
    rect_size_ = rect_size;
  else
    rect_size_ = fabs(rect_size);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::initCovarianceMatrixMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (PointXYZ) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride
  // number of elements per row)
  int row_stride     = element_stride * input_->width;

  const float *data_ = reinterpret_cast<const float*> (&input_->points[0]);

  integral_image_XYZ_.setSecondOrderComputation (true);
  integral_image_XYZ_.setInput (data_, input_->width, input_->height, element_stride, row_stride);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::computePointNormal (
    const int pos_x, const int pos_y, const unsigned point_index, Normal &normal)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  unsigned count = integral_image_XYZ_.getFiniteElementsCount (pos_x - (rect_width_2_), pos_y - (rect_height_2_), rect_width_, rect_height_);

  // no valid points within the rectangular reagion?
  if (count == 0)
  {
    normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = bad_point;
    return;
  }

  if (count > rect_width_*rect_height_) {
    static bool printed_wtf = false;
    if (!printed_wtf) {
      cerr << "WTF count=" << count
           << " x=" << pos_x << " y=" << pos_y
           << " w=" << rect_width_ << " h=" << rect_height_
           << " w*h=" << (rect_width_*rect_height_) << endl;
      printed_wtf = true;
    }
    normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = bad_point;
    return;
  }
   
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector3f center;
  IntegralImage2D<float, 3>::SecondOrderType so_elements;
  //TBD segfault on the next line sometimes
  //center = integral_image_XYZ_.getFirstOrderSum(pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_).template cast<float> ();
  center = integral_image_XYZ_.getFirstOrderSum(pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_). cast<float> ();
  //TBD segfault on the next line sometimes
  so_elements = integral_image_XYZ_.getSecondOrderSum(pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);

  covariance_matrix.coeffRef (0) = static_cast<float> (so_elements [0]);
  covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = static_cast<float> (so_elements [1]);
  covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = static_cast<float> (so_elements [2]);
  covariance_matrix.coeffRef (4) = static_cast<float> (so_elements [3]);
  covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = static_cast<float> (so_elements [4]);
  covariance_matrix.coeffRef (8) = static_cast<float> (so_elements [5]);
  covariance_matrix -= (center * center.transpose ()) / static_cast<float> (count);
  
  // Debug
  float smallest_eigen_value;
  Eigen::Vector3f smallest_eigen_vector;
  pcl::eigen33 (covariance_matrix, smallest_eigen_value, smallest_eigen_vector);
  flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, smallest_eigen_vector[0], smallest_eigen_vector[1], smallest_eigen_vector[2]);
  normal.getNormalVector3fMap () = smallest_eigen_vector;
  if (smallest_eigen_value > 0.0)
    normal.curvature = fabsf (smallest_eigen_value / (covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8)));
  else
    normal.curvature = 0;

  // Alternative way for the eigen-decomposition on covariance matrix
  /*
  Eigen::Matrix3f eigen_vector;
  Eigen::Vector3f sm_eigen_vector;
  Eigen::Vector3f eigen_value;

  // compute eigen values (in ascending order) and corresponding eigenvectors
  pcl::eigen33 (covariance_matrix, eigen_vector, eigen_value);
  if (eigen_value[0] >0)
  {
    sm_eigen_vector = eigen_vector.col(0);

    // flip the normal towards the camera
    flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, sm_eigen_vector[0], sm_eigen_vector[1], sm_eigen_vector[2]);
  
    // get the normal vector
    normal.getNormalVector3fMap () = sm_eigen_vector;
    normal.curvature = fabsf (eigen_value[0] / (covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8)));
  }
  */
  
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
void
IntegralImageFeatures::computeFeature (PointCloudOut &output)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  float smoothing, smoothing_2;
  unsigned index;
  
  output.is_dense = false;
  output.sensor_origin_ = input_->sensor_origin_;
  output.sensor_orientation_ = input_->sensor_orientation_;

  // for each point in the cloud
  for (unsigned ri = 0; ri < input_->height; ++ri)
  {
    for (unsigned ci = 0; ci < input_->width; ++ci)
    {
      index = ri * input_->width + ci;

      const float depth = input_->points[index].z;
      
      if (!pcl_isfinite (input_->points[index].x) || 
          !pcl_isfinite (input_->points[index].y) ||
          !pcl_isfinite (depth))
      {
        output[index].getNormalVector3fMap ().setConstant (bad_point);
        output[index].curvature = bad_point;
        continue;
      }

      // Calculate the size in pixels
      if (use_pixel_size_)
        smoothing = rect_size_;
      else // pixels = size * focal_length / depth
        smoothing = (rect_size_ * fl_)/static_cast<float>(depth);
     
      // Half of smoothing distance
      smoothing_2 = static_cast<unsigned> (smoothing/2.0);

      // For rect that exceeds the borders minimize its size
      //if (ri-smoothing_2 < 0 || ci-smoothing_2 < 0)
      //  smoothing = static_cast<float> (min (ri+1,ci+1));
      //
      //if (ri+smoothing_2 >  input_->height || ci+smoothing_2 > input_->width)
      //  smoothing = min (smoothing, static_cast<float> (min (input_->height-ri+1, input_->width-ci+1)));
      
      // Uncomment only if you want absolute rect size to match
      if ((ri-smoothing_2 < 0) || (ci-smoothing_2 < 0) || 
          (ri+smoothing_2 >  input_->height) || 
          (ci+smoothing_2 > input_->width))
      {
        output[index].getNormalVector3fMap ().setConstant (bad_point);
        output[index].curvature = bad_point;
        continue;
      }
     
      if (smoothing > 2.0f) // at least a 3px wide rect is required
      {
        setRectSize (static_cast<int> (smoothing), static_cast<int> (smoothing));
        computePointNormal (ci, ri, index, output [index]);
      }
      else // set a bad point (NAN)
      {
        output[index].getNormalVector3fMap ().setConstant (bad_point);
        output[index].curvature = bad_point;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
IntegralImageFeatures::initCompute ()
{
  if (!input_->isOrganized ())
  {
    cerr << "Input dataset is not organized (height = 1)." << endl;
    return (false);
  }
  return (Feature<PointXYZ, Normal>::initCompute ());
}
