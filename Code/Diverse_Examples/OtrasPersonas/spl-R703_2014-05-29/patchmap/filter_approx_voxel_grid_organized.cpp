/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-, Willow Garage, Inc.
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
#include "filter_approx_voxel_grid_organized.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
void
ApproximateVoxelGridOrganized::flush (PointCloud &output, size_t op, he *hhe,
                                      int rgba_index, int centroid_size)
{
  if (use_centroid_)
  {
    hhe->centroid /= static_cast<float> (hhe->count);
  }
  else
  {
    // Find distances from centroid and use the one with smallest distance
    double dist = FLT_MAX;
    double new_dist;
    int centroid_id;
    for (int i=0; i<hhe->points.size (); i++)
    {
      new_dist = (hhe->centroid - input_points_.row(hhe->points[i])).norm ();
      if (new_dist < dist)
      {
        dist = new_dist;
        hhe->centroid = input_points_.row(hhe->points[i]);
        centroid_id = hhe->points[i];
      }
    }
    filtered_cloud_id_.push_back (centroid_id);
  }

  for_each_type <FieldList> (xNdCopyEigenPointFunctorOrganized (hhe->centroid, output.points[op]));
  // ---[ RGB special case
  if (rgba_index >= 0)
  {
    // pack r/g/b into rgb
    float r = hhe->centroid[centroid_size-3],
          g = hhe->centroid[centroid_size-2],
          b = hhe->centroid[centroid_size-1];
    int rgb = (static_cast<int> (r)) << 16 | (static_cast<int> (g)) << 8 | (static_cast<int> (b));
    memcpy (reinterpret_cast<char*> (&output.points[op]) + rgba_index, &rgb, sizeof (float));
  }
}

////////////////////////////////////////////////////////////////////////////////
void
ApproximateVoxelGridOrganized::applyFilter (PointCloud &output)
{
  // input dimensions
  input_width_ = input_->width;
  input_height_ = input_->height;

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  vector<PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = getFieldIndex (*input_, "rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  for (size_t i = 0; i < histsize_; i++)
  {
    history_[i].count = 0;
    history_[i].centroid = VectorXf::Zero (centroid_size);
  }
  VectorXf scratch = VectorXf::Zero (centroid_size);

  output.points.resize (input_->points.size ());   // size output for worst case
  input_points_ = MatrixXf::Zero (input_->points.size (), centroid_size);

  filtered_cloud_id_.resize(0);

  size_t op = 0;    // output pointer
  for (size_t cp = 0; cp < input_->points.size (); cp++)
  {
    // In which leaf the point falls
    int ix = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]));
    int iy = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]));
    int iz = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]));
  
    // Keep the min and max coordinates of the x and y coordinates
    if(pcl_isfinite(input_->points[cp].x))
    {
      if (ix > ix_max)
        ix_max = ix;

      if (ix < ix_min)
        ix_min = ix;
    }

    if(pcl_isfinite(input_->points[cp].y))
    {
      if (iy > iy_max)
        iy_max = iy;
      if (iy < iy_min)
        iy_min = iy;
    }

    // Apply hash function
    unsigned int hash = static_cast<unsigned int> ((ix * 7171 + iy * 3079 + iz * 4231) & (histsize_ - 1));
    he *hhe = &history_[hash]; // find the bin
    // If there were already elements or there is new coordinate, flush the current data and empty the bin
    if (hhe->count && ((ix != hhe->ix) || (iy != hhe->iy) || (iz != hhe->iz)))
    {
      flush (output, op++, hhe, rgba_index, centroid_size);
      hhe->count = 0;
      hhe->centroid.setZero ();// = Eigen::VectorXf::Zero (centroid_size);
      hhe->points.resize(0);
    }
    hhe->ix = ix;
    hhe->iy = iy;
    hhe->iz = iz;
    hhe->count++;

    // Unpack the point into scratch, then accumulate
    // ---[ RGB special case
    if (rgba_index >= 0)
    {
      // fill r/g/b data
      pcl::RGB rgb;
      memcpy (&rgb, (reinterpret_cast<const char *> (&input_->points[cp])) + rgba_index, sizeof (RGB));
      scratch[centroid_size-3] = rgb.r;
      scratch[centroid_size-2] = rgb.g;
      scratch[centroid_size-1] = rgb.b;
    }
    for_each_type <FieldList> (xNdCopyPointEigenFunctorOrganized (input_->points[cp], scratch));
    hhe->centroid += scratch;

    // If point closer to centroid will be used, then we need all points as Eigen vectors.
    if (!use_centroid_)
    {
      input_points_.row (cp) = scratch;
      hhe->points.push_back(cp);
    }

  } // for each input point

  // Flush point from the hash to the output cloud
  for (size_t i = 0; i < histsize_; i++)
  {
    he *hhe = &history_[i];
    if (hhe->count)
      flush (output, op++, hhe, rgba_index, centroid_size);
  }

  // Fill the rest cloud fields
  output.points.resize (op);
  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1; // downsampling breaks the organized structure
  output.is_dense = false; // we filter out invalid points
  output.sensor_origin_ = input_->sensor_origin_;
  output.sensor_orientation_ = input_->sensor_orientation_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ApproximateVoxelGridOrganized::organizeOutput (PointCloud &output, PointCloud &organized_output)
{
  //float bad_point = std::numeric_limits<float>::quiet_NaN ();

  int ix_diff = ix_max-ix_min; //width
  int iy_diff = iy_max-iy_min;
 
  if (use_centroid_)
  {
    organized_output.points.resize (ix_diff*iy_diff);
    organized_output.width = static_cast<uint32_t> (ix_diff);
    organized_output.height = static_cast<uint32_t> (iy_diff);
  }
  else
  {
    organized_output.points.resize (input_width_*input_height_); //(input_->points.size());
    organized_output.width = input_width_;
    organized_output.height = input_height_;
  }
  organized_output.is_dense = false;
  organized_output.sensor_origin_ = output.sensor_origin_;
  organized_output.sensor_orientation_ = output.sensor_orientation_;

  // Initialize organized cloud with NAN
  for (size_t cp = 0; cp < organized_output.points.size (); cp++)
  {
    organized_output.points[cp].x = NAN; //bad_point; 
    organized_output.points[cp].y = NAN;
    organized_output.points[cp].z = NAN;
  }
  
  for (size_t cp = 0; cp < output.points.size (); cp++)
  {
    // Replace only if the point is not NAN
    if(!pcl_isfinite(output.points[cp].x) ||
       !pcl_isfinite(output.points[cp].y) ||
       !pcl_isfinite(output.points[cp].z))
      continue;
    
    if (!use_centroid_)
    {
      organized_output.points[filtered_cloud_id_[cp]] = output.points[cp];
    }
    else
    {
      // Recalculate voxel's id
      int ix = static_cast<int> (floor (output.points[cp].x * inverse_leaf_size_[0]));
      int iy = static_cast<int> (floor (output.points[cp].y * inverse_leaf_size_[1]));
      //int iz = static_cast<int> (floor (output.points[cp].z * inverse_leaf_size_[2]));

      // Replace the point only if the z if smaller or the point was NAN
      int indx = ((iy+abs(iy_min))*ix_diff) + (ix+abs(ix_min));
      if ((output.points[cp].z < organized_output.points[indx].z) || !pcl_isfinite(organized_output.points[indx].z))
      {
        organized_output.points[indx] = output.points[cp];
        //TBD: color from output cloud
      }
    }
  }
}
