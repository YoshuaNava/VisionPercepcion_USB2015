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

#ifndef FILTER_APPROX_VOXEL_GRID_ORGANIZED_MAP_H_
#define FILTER_APPROX_VOXEL_GRID_ORGANIZED_MAP_H_

// PCL headers
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
 
using namespace std;
using namespace pcl;
using namespace pcl::traits;
using namespace Eigen;

/** \brief Helper functor structure for copying data between an VectorXf and 
  * a PointT.
  */
struct xNdCopyEigenPointFunctorOrganized
{
  typedef POD<PointXYZ>::type Pod;

  xNdCopyEigenPointFunctorOrganized (const VectorXf &p1, PointXYZ &p2) :
    p1_ (p1), p2_ (reinterpret_cast<Pod&>(p2)), f_idx_ (0) { }

  template<typename Key> inline void operator() ()
  {
    typedef typename datatype<PointXYZ, Key>::type T;
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&p2_) + offset<PointXYZ, Key>::value;
    *reinterpret_cast<T*>(data_ptr) = static_cast<T> (p1_[f_idx_++]);
  }

  private:
  const VectorXf &p1_;
  Pod &p2_;
  int f_idx_;
};

/** \brief Helper functor structure for copying data between an VectorXf and a
  * PointT.
  */
struct xNdCopyPointEigenFunctorOrganized
{
  typedef POD<PointXYZ>::type Pod;

  xNdCopyPointEigenFunctorOrganized (const PointXYZ &p1, VectorXf &p2) :
    p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

  template<typename Key> inline void operator() ()
  {
    typedef typename datatype<PointXYZ, Key>::type T;
    const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&p1_) + offset<PointXYZ, Key>::value;
    p2_[f_idx_++] = static_cast<float> (*reinterpret_cast<const T*>(data_ptr));
  }

  private:
  const Pod &p1_;
  Eigen::VectorXf &p2_;
  int f_idx_;
};

/** \brief ApproximateVoxelGridOrganized creates a *3D voxel grid* over the
  * input point cloud data.  Then it downsamples the data with the following 
  * way:
  * -- If keep_organized is set then the filtered cloud will have the same 
  *    dimensions as the input cloud, o/w the filtered cloud will only include
  *    the unorganized dense points.
  * -- If use_centroid is set and keep_organized is not then the points present
  *    in each *voxel* will be approximated (i.e., *downsampled*) with their 
  *    centroid.  Otherwise if keep_organized is set then the closest original
  *    point to the centroid will be kept.  In the latter case the cloud will 
  *    be organized with the dimension of the xy-voxel grids, although in this
  *    case the original projection matrix is not preserved, and also a new 
  *    projective matrix is not guaranteed.
  *
  * \author Dimitrios Kanoulas
  *
  * \note similar to PCL ApproximateVoxelGrid
  */
class ApproximateVoxelGridOrganized: public Filter<PointXYZ>
{
  using Filter<PointXYZ>::input_;
  using Filter<PointXYZ>::indices_;
  using Filter<PointXYZ>::filter_name_;
  using Filter<PointXYZ>::getClassName;

  typedef Filter<PointXYZ>::PointCloud PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;

  private:
  struct he
  {
    he ():
      ix (), iy (), iz (), count (0), centroid (), points () { }

    int ix, iy, iz;
    int count;
    VectorXf centroid;
    vector<int> points; // for saving the points in the particular leaf
  };

  int ix_max, ix_min, iy_max, iy_min;

  public:

  typedef boost::shared_ptr< ApproximateVoxelGridOrganized > Ptr;
  typedef boost::shared_ptr< const ApproximateVoxelGridOrganized > ConstPtr;


  /** \brief Empty constructor. */
  ApproximateVoxelGridOrganized () :
    Filter<PointXYZ> (),
    leaf_size_ (Vector3f::Ones ()),
    inverse_leaf_size_ (Array3f::Ones ()),
    downsample_all_data_ (true),
    histsize_ (512),
    history_ (new he[histsize_]),
    keep_organized_ (false),
    use_centroid_ (true)
  {
    filter_name_ = "ApproximateVoxelGridOrganized";
    ix_max = 0;
    iy_max = 0;
    ix_min = 0;
    iy_min = 0;
  }

  /** \brief Copy constructor. 
   *
   * \param[in] src the approximate voxel grid to copy into this. 
   */
  ApproximateVoxelGridOrganized (const ApproximateVoxelGridOrganized &src):
    Filter<PointXYZ> (),
    leaf_size_ (src.leaf_size_),
    inverse_leaf_size_ (src.inverse_leaf_size_),
    downsample_all_data_ (src.downsample_all_data_),
    histsize_ (src.histsize_),
    history_ (),
    keep_organized_ (src.keep_organized_),
    use_centroid_ (src.use_centroid_)
  {
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
    ix_max = 0;
    iy_max = 0;
    ix_min = 0;
    iy_min = 0;
  }


  /** \brief Destructor. */
  ~ApproximateVoxelGridOrganized ()
  {
    delete [] history_;
  }


  /** \brief Copy operator. 
    *
    * \param[in] src the approximate voxel grid to copy into this. 
    */
  inline ApproximateVoxelGridOrganized& 
  operator = (const ApproximateVoxelGridOrganized &src)
  {
    leaf_size_ = src.leaf_size_;
    inverse_leaf_size_ = src.inverse_leaf_size_;
    downsample_all_data_ = src.downsample_all_data_;
    histsize_ = src.histsize_;
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
    ix_max = src.ix_max;
    iy_max = src.iy_max;
    ix_min = src.ix_min;
    iy_min = src.iy_min;
    keep_organized_ = src.keep_organized_;
    use_centroid_ = src.use_centroid_;
    return (*this);
  }

  /** \brief Set the voxel grid leaf size.
    *
    * \param[in] leaf_size the voxel grid leaf size
    */
  inline void 
  setLeafSize (const Eigen::Vector3f &leaf_size) 
  { 
    leaf_size_ = leaf_size; 
    inverse_leaf_size_ = Eigen::Array3f::Ones () / leaf_size_.array ();
  }

  /** \brief Set the voxel grid leaf size.
    *
    * \param[in] lx the leaf size for X
    * \param[in] ly the leaf size for Y
    * \param[in] lz the leaf size for Z
    */
  inline void
  setLeafSize (float lx, float ly, float lz)
  {
    setLeafSize (Eigen::Vector3f (lx, ly, lz));
  }

  /** \brief Get the voxel grid leaf size. */
  inline Eigen::Vector3f 
  getLeafSize () const { return (leaf_size_); }

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
    *
    * \param downsample the new value (true/false)
    */
  inline void 
  setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

  /** \brief Get the state of the internal downsampling parameter (true if
    * all fields need to be downsampled, false if just XYZ). 
    */
  inline bool 
  getDownsampleAllData () const { return (downsample_all_data_); }

  /** \brief Set to true if points sound remain organized in a grid-fashion.
    * See keep_centroid to see the two different types of organization.
    */
  inline void
  setKeepOrganized (bool keep_organized)
  {
    this->keep_organized_ = keep_organized;
  }

  /** \brief Get whether the filtered points should be organized.
    *
    * \return The value of the \a keep_organized_ parameter.
    */
  inline bool
  getKeepOrganized ()
  {
    return (this->keep_organized_);
  }

  /** \brief Set whether to use the centroid or not (i.e. the points close to
    * the centroid) in each voxel.
   */
  inline void
  setUseCentroid (bool use_centroid)
  {
    this->use_centroid_ = use_centroid;
  }

  /** \brief Organize the vector data in output to voxel xy-frame orgniazed points. */
  void
  organizeOutput (PointCloud &output, PointCloud &organized_output);


  protected:
  /** \brief The size of a leaf. */
  Vector3f leaf_size_;

  /** \brief Compute 1/leaf_size_ to avoid division later */ 
  Array3f inverse_leaf_size_;

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_;

  /** \brief history buffer size, power of 2 */
  size_t histsize_;

  /** \brief history buffer */
  struct he* history_;

  typedef fieldList<PointXYZ>::type FieldList;

  /** \brief Downsample a Point Cloud using a voxelized grid approach.
    *
    * \param output the resultant point cloud message
    */
  void 
  applyFilter (PointCloud &output);

  /** \brief Write a single point from the hash to the output cloud. */
  void 
    flush (PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size);

  /** \brief Whether to keep the original points that are close to the
    * centroid in each leaf, instead of the centroid itself.
    *
    * \note in this way you make sure that the projected matrix can be
    * recreated.
    */
  bool keep_organized_;

  /** \brief Whether to use the centroid for each voxel, or the point closest
    * to the centroid.
    */
  bool use_centroid_;

  /** For storing the points in each leaf. */
  MatrixXf input_points_;

  /** Original input width and height. */
  int input_width_, input_height_;

  /** Filtered points id to original cloud. */
  vector<int> filtered_cloud_id_;
};

#endif  // FILTER_APPROX_VOXEL_GRID_ORGANIZED_MAP_H_
