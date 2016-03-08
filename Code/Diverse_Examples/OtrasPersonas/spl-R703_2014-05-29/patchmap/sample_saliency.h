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

#ifndef SAMPLE_SALIENCY_H_
#define SAMPLE_SALIENCY_H_

// SPL headers
#include "filter_nms.h"
#include "integral_image_features.h"

// STD headers
#include <math.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace Eigen;

class SampleSaliency
{
  /** \brief Extracts salient samples from a point cloud with respect to a
    * specific task.  The saliency uses the curvature and the normal
    * vectors around points.
    *
    * \author Dimitrios Kanoulas
    */

  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointCloudIn;
    typedef PointCloud<PointIn>::Ptr PointCloudPtr;
    typedef PointCloud<PointIn>::ConstPtr PointCloudConstPtr;
    typedef PointCloud<Normal> PointCloudNormal; 
    typedef PointCloud<Normal>::Ptr PointCloudNormalPtr;
    typedef PointCloud<PointNormal> PointCloudPointNormal;
    typedef PointCloud<PointNormal>::Ptr PointCloudPointNormalPtr;
    typedef IntegralImageFeatures IIFeatures;
      
    /** \brief Constructor. */
    SampleSaliency ();

    /** \brief Destructor. */
    ~SampleSaliency ();

    /** \brief Set input cloud.
      *
      * \param[in] input_cloud the input cloud.
      */
    void
    setInputCloud (const PointCloudConstPtr &input_cloud);
  
    /** \brief Set whether to use Difference of Normals (DoN) saliency. */
    void
    setUseDoN (bool use_DoN);
    
    /** \brief Set whether to use Difference of Normal-Gravity (DoNG)
      * saliency.
      */
    void
    setUseDoNG (bool use_DoNG);
    
    /** \brief Set whether to use Distance to Fixation Point (DtFP) saliency. */
    void
    setUseDtFP (bool use_DtFP);
    
    /** \brief Set whether to use Min/Max Principal Curvature (MinMaxPC) 
      * saliency.
      */
    void
    setUseMinMaxPC (bool use_MinMaxPC);

    /** \brief Set the distance to the ground for the fixation point. */
    void
    setFixDistGround (double fix_dist_ground);

    /** \brief Set the two steps distance on flat ground fo the fixation
      * point.
      */
    void
    setFixTwoStepsDist (double fix_two_steps_dist);

    /** \brief Set the distance of the sensor from the human body for the
      * fixation point.
      */
    void
    setFixDistFromBody (double fix_dist_from_body);
    
    /** \brief Set all fixation points properties. */
    void
    setFPProps (double dist_ground, double two_steps_dist,
                double dist_from_body);

    /** \brief Set the maximum distance from the fixation point to be used for
      * saliency.
      */
    void
    setMaxDistFromFixation (double max_dist_from_fixation);
   
    /** \brief Set neighborhood size and focal length.
      *
      * \param[in] nn_size the size (width) of the rect (in m).
      * \param[in] fl focal length.
      */
    void
    setNNSize (double nn_size, float fl);
    
    /** \brief Get the index of the salient points in the input point cloud.
      *
      * \return the indx_salient_ vector.
      */
    boost::shared_ptr<vector<int> >
    getIndxSalient ();

    /** \brief Set the viewer.
      *
      * \param[in] viewer the viewer for visualization.
      */
    void
    setViewer (boost::shared_ptr<PCLVisualizer> viewer);

    /** \brief Set the camera pose wrt the world frame.  The default setting is
      * that camera frame is the world frame at any time.
      */
    void
    setCamPose (Vector3f cam_pose);

    /** \brief Set the gravity vector.
      *
      * \param[in] g the 3x1 gravity vector.
      */
    void
    setG (Vector3f g);

    /** \brief Set min curvature. */
    void
    setMinCurv (double min_curv);
    
    /** \brief Set max curvature. */
    void
    setMaxCurv (double max_curv);

    /** \brief Set cos(angle) threshold between normals (of full and half
      * neighborhood size).  Should be in [-1,1].
      */
    void
    setNNCAThres (double nnca_thres);

    /** \brief Set cos(angle) between the normal of full size and the negative
      * gravity.  Should be in [-1,1].
      */
    void
    setNGCAThres (double ngca_thres);
    
    /** \brief Set whether to do NMS. */
    void
    setNMS (bool nms);

    /** \brief Get the i-th normal-normal value. */
    float
    getNormalNormalAngle (int i);

    /** \brief Get the i-th normal-gravity value. */
    float
    getNormalGravityAngle (int i);
    
    /** \brief Copy only salient point to a new point cloud */
    int
    extractSalientPoints ();

    /** Add normal vectors in the viewer.
      *
      * \param[in] sn whether to show the normals
      */
    void
    showNormals (bool sn);
    
    /** Show curvature colors in the viewer.
      *
      * \param[in] sn whether to show the curvature
      */
    void
    showCurvature (bool sc);

    /** Show the fixation point as a small sphere. */
    
    void
    showFixation ();

    /** Show the filtered cloud wrt the Fixation Point. */
    void
    showDtFPCloud ();
    
    /** Show only the salient points in the viewer.
      *
      * \param[in] ssp whether to show the salient point
      */
    void
    showSalPoints (bool ssp);

  private:
    /** \brief Calculates the curvature and the normal vectors in an area of
      * radius nn_size.  The method sets the normals (full radius) and 
      * normals_2 (half radius) clouds.
      */
    void
    computeFeatures ();

    /** \brief Computing the fixation point, as the point in 3D space which is
      * the sum of the heading and negative-gravity vector, considering the 
      * distance the sensor was from the ground, the distance the sensor was 
      * from the body, and the distance of the desired step.
      */
    void
    computeFixationPoint (Vector3f &g, float dist_to_ground, float dist_step,
                          float dist_from_body);

    /** \brief Calculate the cos(angle) between the normals at every point. */
    void
    calculateNormalsAngle (PointCloudNormalPtr &normals1,
                           PointCloudNormalPtr &normals2);

    /** \brief Calculate the cos(angle) between the normal and the gravity
      * vector at every point.
      */
    void
    calculateNormalGravityAngle(PointCloudNormalPtr &normals, Vector3f &g);

    /** \brief Calculate the cos angle between 2 vectors.
      *
      * \param[in] v1 first vector
      * \param[in] v2 second vector
      *
      * \return the cos(angle) between v1 and v2
      */
    double
    calculateVectorAngle (Vector3f &v1, Vector3f &v2);
  
    /** \brief Calculate distance between fixation point and input cloud. */
    void
    calculateDistFromFix (PointCloudPtr &fixation_cloud_);

    /** \brief Euclidean distance between two points. */
    float
    euclideanDist (const PointIn &p1, const PointIn &p2);
  
  protected:
    /** \brief Input point cloud. */
    PointCloudConstPtr input_cloud_;
    
    /** \brief Whether the input point cloud was initialized. */
    bool input_cloud_set_;
    
    /** \brief Whether to use the Difference of Normals (DoN), the Difference of
      * Normal-Gravity (DoNG), the Distance to Fixation Point (DtFP), or the 
      * Min/Max Principal Curvature (MinMaxPC).
      */
    bool use_DoN_, use_DoNG_, use_DtFP_, use_MinMaxPC_;
   
    /** \brief vertical dist of camera from ground in m */
    double fix_dist_ground_;

    /** horizontal dist of two human steps in m */
    double fix_two_steps_dist_;

    /** horizontal dist of camera from human's body in m*/
    double fix_dist_from_body_;

    /** max allowed distance from fixation point */
    double max_dist_from_fixation_;

    /** \brief Neighborhood radius size. */
    double nn_size_;

    /** \brief Whether the nn size was initialized. */
    bool nn_size_set_;
    
    /** \brief Output salient points index in the input point cloud. */
    boost::shared_ptr<vector<int> > indx_salient_;
    
    /** \brief The filtered point cloud. */
    PointCloudPtr filtered_cloud_;

    /** \brief Input point cloud embended with curvature.
      *
      * \note Required only for visualizing the curvature.
      */
    PointCloudPointNormalPtr curv_cloud_;
    
    /** \brief Salient point cloud.
      *
      * \note Required only for visualizing saliency.
      */
    PointCloudPtr sal_cloud_;
    
    /** \brief Fixation point cloud. */
    PointCloudPtr fixation_cloud_;
   
    /** \brief The viewer. */
    boost::shared_ptr<PCLVisualizer> viewer_;

    /** \brief Camera pose wrt the world frame.  The default is that the camera
      * frame is the world frame.
      */
    Vector3f cam_pose_;

    /** \brief Gravity vector. */
    Vector3f g_;
    
    /** \brief Whether gravity vector is initialized and can be used for
      * saliency.
      */
    bool g_set_;

    /** \brief Normal estimation object. */
    IntegralImageFeatures *iif_;

    /** \brief Point cloud normals in radius nn_size and nn_size/2 resp. */
    PointCloudNormalPtr normals_, normals_2_;
      
    /** \brief Min and max curvature. */
    double min_curv_, max_curv_;
    
    /** Allowed threshold for the cos(angle) between the normals of full and
      * half neighborhood size.  Should be a number in [-1,1].
      */
    float nnca_thres_;
    
    /** \brief Vector with all angles (cos) between normals in a sequence.  It
      * includes NAN if normal vector is not valid in a specific point.
      */
    vector <float> normal_cos_angles;
    
    /** \brief Threshold between normal and gravity vector cos(angles). */
    float ngca_thres_;

    /** \brief Angles between normals and gravity.
     * NAN if normal does not exist.
     */
    vector <float> normal_gravity_cos_angles;
    
    /** \brioef Point to NMS object. */
    boost::shared_ptr<NMS> nms_;
    
    /** \brief Whether to apply NMS on the data. */
    bool do_nms_;

    /** \brief Weight for the nms between normal-normal and normal-gravity 
      * cos(angles).  The weight nms_w_ is given to normal-normal and 
      * (1-nms_w_) to normal-gravity.
      */
    double nms_w_;

    /** \brief Fixation point. */
    Vector3f fix_pt_;

    /** \brief Distance to fixation point. */
    vector <float> dist_from_fixation_;
    
    /** \brief The cos(angle) between the normals of full and half neighborhood
      * size.
      */
    float normal_cos_angle;
};

#endif // #ifndef SAMPLE_SALIENCY_H_
