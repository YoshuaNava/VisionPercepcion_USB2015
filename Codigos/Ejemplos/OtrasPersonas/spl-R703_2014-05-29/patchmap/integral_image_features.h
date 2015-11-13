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

#ifndef INTEGRAL_IMAGE_FEATURES_H_
#define INTEGRAL_IMAGE_FEATURES_H_

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/integral_image2D.h>

using namespace std;
using namespace pcl;

/** \brief Surface normal estimation on organized data using integral images.  
  * The features will be estimated by creating 9 integral images to compute the
  * normal for a specific point from the covariance matrix of its local 
  * neighborhood.
  *
  * \note adapted from PCL IntegralImageNormalEstimation class.
  *
  * \author Stefan Holzer, modified by Dimitrios Kanoulas
  */
  class IntegralImageFeatures: public Feature<PointXYZ, Normal>
  {
    using Feature<PointXYZ, Normal>::input_;
    using Feature<PointXYZ, Normal>::feature_name_;
    using Feature<PointXYZ, Normal>::tree_;
    using Feature<PointXYZ, Normal>::k_;

    public:
      typedef boost::shared_ptr<IntegralImageFeatures > Ptr;
      typedef boost::shared_ptr<const IntegralImageFeatures > ConstPtr;
      typedef Feature<PointXYZ, Normal>::PointCloudIn  PointCloudIn;
      typedef Feature<PointXYZ, Normal>::PointCloudOut PointCloudOut;

    /** \brief Constructor. */
    IntegralImageFeatures () :
      rect_width_ (0), rect_width_2_ (0), rect_width_4_ (0),
      rect_height_ (0), rect_height_2_ (0), rect_height_4_ (0),
      rect_size_ (10), use_pixel_size_ (true), fl_ (580.0f),
      integral_image_XYZ_ (true),
      vpx_ (0.0f), vpy_ (0.0f), vpz_ (0.0f),
      use_sensor_origin_ (true)
      {
        feature_name_ = "IntegralImagesFeaturesEstimation";
        tree_.reset ();
        k_ = 1;
      }

    /** \brief Destructor. **/
    virtual ~IntegralImageFeatures ()
    {};

    /** \brief Set the regions size which is considered for normal estimation.
      *
      * \param[in] width the width of the search rectangle.
      * \param[in] height the height of the search rectangle.
      */
    void
    setRectSize (const int width, const int height);
    
    /** \brief Set the regions size which is considered for normal estimation
      * in m (if use_pixel_size is NOT set), in pixels o/w.
      *
      * \param[in] rect_size the length of the search rectangle side.
      */
    void
    setRectSize (const float rect_size);

    /** Set focal length. */
    inline void
    setFocalLength (const float fl)
    {
      this->fl_ = fl;
    }

    /** \brief Get rect_width. */
    int
    getRectWidth()
    {
      return (this->rect_width_);
    }

    /** \brief Get rect_height. */
    int
    getRectHeight()
    {
      return (this->rect_height_);
    }

    /** \brief Computes the normal at the specified position.
      *
      * \param[in] pos_x x position (pixel)
      * \param[in] pos_y y position (pixel)
      * \param[in] point_index the position index of the point
      * \param[out] normal the output estimated normal
      */
    void
    computePointNormal (const int pos_x, const int pos_y,
                        const unsigned point_index, Normal &normal);


    /** \brief Set whether to use depth depending smoothing or not.
      *
      * \param[in] use_pixel_size decides whether the smoothing
      * is depth dependent.
      */
    inline void
    setUsePixelSize (bool use_pixel_size)
    {
      this->use_pixel_size_ = use_pixel_size;
    }

    /** \brief Provide a pointer to the input dataset (overwrites the
      * PCLBase::setInputCloud method).
      *
      * \param[in] cloud the const boost shared pointer to a PointCloud message.
      */
    virtual inline void
    setInputCloud (const PointCloudIn::ConstPtr &cloud)
    {
      input_ = cloud;
      if (!cloud->isOrganized ())
      {
        PCL_ERROR ("[pcl::IntegralImageFeatures::setInputCloud] Input dataset is not organized (height = 1).\n");
        return;
      }

      if (use_sensor_origin_)
      {
        vpx_ = input_->sensor_origin_.coeff (0);
        vpy_ = input_->sensor_origin_.coeff (1);
        vpz_ = input_->sensor_origin_.coeff (2);
      }

      // Initialize the correct data structure based on the normal estimation method chosen
      initData ();
    }

    /** \brief Set the viewpoint.
      *
      * \param vpx the X coordinate of the viewpoint
      * \param vpy the Y coordinate of the viewpoint
      * \param vpz the Z coordinate of the viewpoint
      */
    inline void
    setViewPoint (float vpx, float vpy, float vpz)
    {
      vpx_ = vpx;
      vpy_ = vpy;
      vpz_ = vpz;
      use_sensor_origin_ = false;
    }

    /** \brief Get the viewpoint.
      *
      * \param[out] vpx x-coordinate of the view point
      * \param[out] vpy y-coordinate of the view point
      * \param[out] vpz z-coordinate of the view point
      *
      * \note this method returns the currently used viewpoint for normal
      * flipping.  If the viewpoint is set manually using the setViewPoint 
      * method, this method will return the set view point coordinates.  If an
      * input cloud is set, it will return the sensor origin otherwise it will
      * return the origin (0, 0, 0).
      */
    inline void
    getViewPoint (float &vpx, float &vpy, float &vpz)
    {
      vpx = vpx_;
      vpy = vpy_;
      vpz = vpz_;
    }

    /** \brief Sets whether the sensor origin or a user given viewpoint should
      * be used. After this method, the normal estimation method uses the sensor
      * origin of the input cloud.  To use a user defined view point, use the 
      * method setViewPoint.
     */
    inline void
    useSensorOriginAsViewPoint ()
    {
      use_sensor_origin_ = true;
      if (input_)
      {
        vpx_ = input_->sensor_origin_.coeff (0);
        vpy_ = input_->sensor_origin_.coeff (1);
        vpz_ = input_->sensor_origin_.coeff (2);
      }
      else
      {
        vpx_ = 0;
        vpy_ = 0;
        vpz_ = 0;
      }
    }

    protected:
    /** \brief Computes the normal for the complete cloud.
      *
      * \param[out] output the resultant normals
      */
    void
    computeFeature (PointCloudOut &output);

    /** \brief Initialize the data structures, based on the normal estimation
      * method chosen.
      */
    void
    initData ();

    private:
    /** \brief Flip (in place) the estimated normal of a point towards a given
      * viewpoint.
      *
      * \param point a given point
      * \param vp_x the X coordinate of the viewpoint
      * \param vp_y the X coordinate of the viewpoint
      * \param vp_z the X coordinate of the viewpoint
      * \param nx the resultant X component of the plane normal
      * \param ny the resultant Y component of the plane normal
      * \param nz the resultant Z component of the plane normal
      * 
      * \ingroup features
      */
    inline void
    flipNormalTowardsViewpoint (const PointXYZ &point,
                                float vp_x, float vp_y, float vp_z,
                                float &nx, float &ny, float &nz)
    {
      // See if we need to flip any plane normals
      vp_x -= point.x;
      vp_y -= point.y;
      vp_z -= point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

      // Flip the plane normal
      if (cos_theta < 0)
      {
        nx *= -1;
        ny *= -1;
        nz *= -1;
      }
    }

    /** The width of the neighborhood region used for computing the normal. */
    int rect_width_;
    int rect_width_2_;
    int rect_width_4_;
    
    /** The height of the neighborhood region used for computing the normal. */
    int rect_height_;
    int rect_height_2_;
    int rect_height_4_;

    /** The size of the neighborhood area in cm */
    float rect_size_;
    
    /** \brief Whether the rect_size_ is in pixels (true) or in meters (false). */
    bool use_pixel_size_;

    /** \brief Camera's focal length. */
    float fl_;

    /** integral image xyz */
    IntegralImage2D<float, 3> integral_image_XYZ_;

    /** \brief Values describing the viewpoint ("pinhole" camera model assumed).
      * For per point viewpoints, inherit from Features and provide your own 
      * computeFeature (). By default, the viewpoint is set to 0,0,0.
      */
    float vpx_, vpy_, vpz_;

    /** whether the sensor origin of the input cloud or a user given viewpoint 
      * should be used.
      */
    bool use_sensor_origin_;

    /** \brief This method should get called before starting the actual 
      * computation.
      */
    bool
    initCompute ();

    /** \brief Internal initialization method for COVARIANCE_MATRIX estimation. */
    void
    initCovarianceMatrixMethod ();

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic warning "-Weffc++"
#endif

#endif // INTEGRAL_IMAGE_FEATURES_H_
