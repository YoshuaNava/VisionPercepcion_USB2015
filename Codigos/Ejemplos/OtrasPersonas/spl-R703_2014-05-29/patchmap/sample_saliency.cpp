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
#include "sample_saliency.h"

#define NMS_WINDOW_SIZE 1 //window size for NMS filtering

////////////////////////////////////////////////////////////////////////////////
SampleSaliency::SampleSaliency ():
  input_cloud_set_ (false),
  nn_size_set_ (false),
  indx_salient_ (new vector<int>),
  filtered_cloud_ (new PointCloudIn),
  curv_cloud_ (new PointCloudPointNormal),
  sal_cloud_ (new PointCloudIn),
  fixation_cloud_ (new PointCloudIn),
  g_set_ (false),
  iif_ (new IIFeatures),
  normals_ (new PointCloud<Normal>),
  normals_2_ (new PointCloud<Normal>),
  min_curv_ (0.0), max_curv_ (0.0),
  nnca_thres_ (-1.0), //180 degrees angle
  ngca_thres_ (1.0), // 0 degrees angle
  nms_ (new NMS),
  do_nms_(false),
  nms_w_ (0.7)
{
  // saliency measures to be used
  use_DoN_ = use_DoNG_ = use_DtFP_ = use_MinMaxPC_ = true;

  // camera pose setup
  cam_pose_ = Vector3f::Zero();

  // fixation point (DtFP) setup
  fix_dist_ground_ = 1.0;
  fix_two_steps_dist_ = 1.5;
  fix_dist_from_body_ = 0.3;
  max_dist_from_fixation_ = 0.7;

  // set size of neighborhood;
  iif_->setUsePixelSize (true);
  setNNSize (10.0, 580.0f);
}

////////////////////////////////////////////////////////////////////////////////
SampleSaliency::~SampleSaliency ()
{   
  delete (iif_);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setInputCloud (const PointCloudConstPtr &input_cloud)
{
  // set the input cloud
  this->input_cloud_ = input_cloud;
  input_cloud_set_ = true;

  iif_->setInputCloud (this->input_cloud_);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setUseDoN (bool use_DoN)
{
  this->use_DoN_ = use_DoN;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setUseDoNG (bool use_DoNG)
{
  this->use_DoNG_ = use_DoNG;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setUseDtFP (bool use_DtFP)
{
  this->use_DtFP_ = use_DtFP;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setUseMinMaxPC (bool use_MinMaxPC)
{
  this->use_MinMaxPC_ = use_MinMaxPC;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setFixDistGround (double fix_dist_ground)
{
  this->fix_dist_ground_ = fix_dist_ground;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setFixTwoStepsDist (double fix_two_steps_dist)
{
  this->fix_two_steps_dist_ = fix_two_steps_dist;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setFixDistFromBody (double fix_dist_from_body)
{
  this->fix_dist_from_body_ = fix_dist_from_body;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setFPProps (double dist_ground, double two_steps_dist,
                            double dist_from_body)
{
  setFixDistGround (dist_ground);
  setFixTwoStepsDist (two_steps_dist);
  setFixDistFromBody (dist_from_body);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setMaxDistFromFixation (double max_dist_from_fixation)
{
  this->max_dist_from_fixation_ = max_dist_from_fixation;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setNNSize (double nn_size, float fl)
{
  this->nn_size_ = nn_size;
  this->nn_size_set_ = true;
  
  iif_->setFocalLength (fl);
  iif_->setUsePixelSize (false);
  iif_->setRectSize (this->nn_size_);
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<vector<int> >
SampleSaliency::getIndxSalient ()
{
  return (this->indx_salient_);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setViewer (boost::shared_ptr<PCLVisualizer> viewer)
{
  this->viewer_ = viewer;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setCamPose (Vector3f cam_pose)
{
  this->cam_pose_ = cam_pose;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setG (Vector3f g)
{
  this->g_ = g;
  this->g_set_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setMinCurv (double min_curv)
{
  this->min_curv_ = min_curv;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setMaxCurv (double max_curv)
{
  this->max_curv_ = max_curv;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setNNCAThres (double nnca_thres)
{
  if (nnca_thres>=-1.0 && nnca_thres<=1.0)
    this->nnca_thres_ = nnca_thres;
  else
    cerr << "nnca_thres should be in [-1,1]." << endl;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setNGCAThres (double ngca_thres)
{
  if (ngca_thres>=-1.0 && ngca_thres<=1.0)
    this->ngca_thres_ = ngca_thres;
  else
    cerr << "ngca_thres should be in [-1,1]." << endl;
}

////////////////////////////////////////////////////////////////////////////////
float
SampleSaliency::getNormalNormalAngle (int i)
{
  return (this->normal_cos_angles[i]);
}

////////////////////////////////////////////////////////////////////////////////
float
SampleSaliency::getNormalGravityAngle (int i)
{
  return (this->normal_gravity_cos_angles[i]);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::setNMS (bool do_nms) 
{
  this->do_nms_ = do_nms;
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::computeFeatures ()
{
  // full radius normal estimation
  iif_->setRectSize (nn_size_);
  iif_->compute(*normals_);

  // half radius normal estimation
  iif_->setRectSize (nn_size_/2.0);
  iif_->compute(*normals_2_);
  
  // reset neighborhood size
  iif_->setRectSize (nn_size_);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::computeFixationPoint (Vector3f &g, float dist_to_ground,
                                      float dist_step, float dist_from_body)
{
  // heading vector
  Vector3f h (0.0, -g(2), g(1));

  // fixation point
  fix_pt_ = cam_pose_ + (dist_to_ground*g_) + ((dist_step-dist_from_body)*h);
  
  // for visualizing the fixation point
  fixation_cloud_->points.resize (1);
  fixation_cloud_->points[0].x = fix_pt_(0);
  fixation_cloud_->points[0].y = fix_pt_(1);
  fixation_cloud_->points[0].z = fix_pt_(2);
}

////////////////////////////////////////////////////////////////////////////////
int
SampleSaliency::extractSalientPoints() 
{
  // check whether the input point cloud has been set
  if (!this->input_cloud_set_ || !this->nn_size_set_)
  {
    cerr << typeid(this).name() << ": Input cloud not set." << endl;
    return (-1);
  }

  if (use_DoN_ || use_DoNG_)
  {
    // compute the neighborhood features (i.e. normal vectors)
    computeFeatures ();
  }
  
  if (use_DoN_)
  {
    // compute the normal-normal angle
    calculateNormalsAngle (normals_, normals_2_);
  }

  // compute the fixation point and the normal-gravity angles
  if (this->g_set_ && use_DtFP_)
  {
    computeFixationPoint (this->g_, fix_dist_ground_, fix_two_steps_dist_,
                          fix_dist_from_body_);
    calculateDistFromFix (fixation_cloud_);
  }
  
  if (this->g_set_ && use_DoNG_)
    calculateNormalGravityAngle (normals_, g_);

  // for all points
  indx_salient_->clear (); // clear the output index vector
  for (int i = 0 ; i < normals_->points.size () ; i++)
  {
    // saliency wrt angles between normals
    if (!pcl_isfinite (normal_cos_angles[i]) && use_DoN_)
      continue;

    if ((normal_cos_angles[i] < nnca_thres_) && use_DoN_)
      continue;
    
    //saliency wrt gravity vector
    if (g_set_ && use_DoNG_)
      if (!pcl_isfinite (normal_gravity_cos_angles[i]))
        continue;
    
    if (g_set_ && use_DoNG_)
      if (normal_gravity_cos_angles[i] < this->ngca_thres_)
        continue;

    if (g_set_ && use_DtFP_)
      if (!pcl_isfinite (dist_from_fixation_[i]))
        continue;

    //TBD: problem in test_autoseg
    //if (dist_from_fixation_[i] > max_dist_from_fixation_)
    //  continue;

    // saliency wrt curvature
    //if (!pcl_isfinite (normals->points[i].curvature))
    //  continue;
    
    //if (normals->points[i].curvature > 1.0/30.0)
    //  continue;

    (*indx_salient_).push_back (i);
  }
    
  // NMS
  if (this->do_nms_)
  {
    MatrixXd sal_mat(normals_->height, normals_->width);
    sal_mat.fill (-1.0f); // do NOT fill with NAN (nms is not working)
    int id;
    for (int i=0; i<(*indx_salient_).size (); i++)
    {
      id = (*indx_salient_)[i];
      if (g_set_)
      {
        // All values should be positive
        sal_mat(id/normals_->width,id%normals_->width) = 
          (nms_w_ * normal_cos_angles[id]) + ((1.0-nms_w_) * normal_gravity_cos_angles[id]);
      }
      else
      {
        sal_mat(id/normals_->width,id%normals_->width) = normal_cos_angles[id];
      }
    }
    nms_->setIM (sal_mat);
    nms_->setN (NMS_WINDOW_SIZE);
    nms_->applyNMS ();
    indx_salient_ = nms_->getIdOm ();
  }
  
  return (1);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::calculateNormalsAngle(PointCloudNormalPtr &normals1,
                                      PointCloudNormalPtr &normals2)
{
  int counter = 0;
  Vector4f normal_vec1, normal_vec2;

  // Check if both clouds have same number of normals
  if (normals1->points.size() != normals2->points.size())
    print_error ("Not same number of normals.\n");

  // Clear the list
  //normal_cos_angles.resize(0);
  normal_cos_angles.clear();
  for (int i = 0; i < normals1->points.size() ; i++)
  {
    // Check if the point is invalid
    if (!pcl_isfinite (normals1->points[i].normal_x) ||  
        !pcl_isfinite (normals1->points[i].normal_y) ||  
        !pcl_isfinite (normals1->points[i].normal_z) ||
        !pcl_isfinite (normals2->points[i].normal_x) ||  
        !pcl_isfinite (normals2->points[i].normal_y) ||  
        !pcl_isfinite (normals2->points[i].normal_z))
    {
      // So that we don't loose the structure
      normal_cos_angle = NAN;
      counter ++;
    }
    else
    {
      // Calculate the angle as a dot product
      normal_vec1 = normals1->points[i].getNormalVector4fMap (); 
      normal_vec2 = normals2->points[i].getNormalVector4fMap (); 
      
      //TBD: why do I have this?
      if (normal_vec1 == Vector4f::Zero () || normal_vec2 == Vector4f::Zero ())
      {
        normal_cos_angle = NAN;
        counter ++;
      }
      else
      {
        normal_cos_angle = static_cast<float> (normal_vec1.dot (normal_vec2));
      }
    }

    // Store the angle.  Note that the values should be in [-1,1].
    normal_cos_angles.push_back(normal_cos_angle);
  }
}


////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::calculateNormalGravityAngle(PointCloudNormalPtr &normal, Vector3f &g)
{
  float normal_gravity_cos_angle;
  int counter = 0;
  Vector4f normal_vec;
  Vector3f normal_resized_vec;

  // Clear the list
  //normal_gravity_cos_angles.resize(0);
  normal_gravity_cos_angles.clear();

  for (int i = 0; i < normal->points.size() ; i++)
  {   
    // Check if the point is invalid
    if (!pcl_isfinite (normal->points[i].normal_x) ||  
        !pcl_isfinite (normal->points[i].normal_y) ||  
        !pcl_isfinite (normal->points[i].normal_z))
    {   
      // So that we don't loose the structure
      normal_gravity_cos_angle = NAN;
      counter ++;
    }
    else
    {
      // Calculate the angle as a dot product
      normal_vec = normal->points[i].getNormalVector4fMap (); 
      
      //TBD: why do I have this?
      if (normal_vec == Vector4f::Zero())
      {
        normal_gravity_cos_angle = NAN;
        counter ++;
      }
      else
      {
        // Note we consider the negative gravity
        normal_resized_vec = normal_vec.head(3);
        normal_gravity_cos_angle = static_cast<float> (normal_resized_vec.dot (-1.0*g));
      }
    }   

    // Store the angle.  Note that the values should be in [-1,1].
    normal_gravity_cos_angles.push_back(normal_gravity_cos_angle);
  }
}

////////////////////////////////////////////////////////////////////////////////
double
SampleSaliency::calculateVectorAngle (Vector3f &v1, Vector3f &v2)
{
  return (v1.dot (v2));
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::calculateDistFromFix (PointCloudPtr &fixation_cloud_)
{
  float dist;
  
  // compute distances
  dist_from_fixation_.clear();
  for (int i=0; i<input_cloud_->points.size (); i++)
  {
    dist = static_cast<float> (euclideanDist(input_cloud_->points[i], fixation_cloud_->points[0]));
    dist_from_fixation_.push_back (dist);
  }
}

////////////////////////////////////////////////////////////////////////////////
float
SampleSaliency::euclideanDist (const PointIn &p1, const PointIn &p2)
{
  float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
  return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::showNormals (bool sn)
{
  if (sn)
  {
    // Full radius normal
    viewer_->removeShape ("normals");
    viewer_->addPointCloudNormals<PointXYZ, Normal> (input_cloud_, normals_,
        100, 0.05, "normals");
    viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0,
        0.0, "normals");

    // Half radius normal
    viewer_->removeShape ("normals_2");
    viewer_->addPointCloudNormals<PointXYZ, Normal> (input_cloud_, normals_2_,
        100, 0.05, "normals_2");
    viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_COLOR, 0.0, 1.0,
        0.0, "normals_2");
  }
  else
  {
    //TBD: remove normals  
  }
}


////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::showCurvature (bool sc)
{
  if (sc)
  {
    concatenateFields (*input_cloud_, *normals_, *curv_cloud_);
    PointCloudColorHandlerGenericField<PointNormal> rgb (curv_cloud_, "curvature");
    viewer_->addPointCloud(curv_cloud_, rgb, "curvature_cloud");
  }
  else
  {
    //TBD: remove curvatures
  }
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::showFixation ()
{
  viewer_->removeShape("fixation");
  viewer_->addSphere<PointXYZ> (fixation_cloud_->points[0], 0.03, 0.0, 0.0, 1.0, "fixation");
  viewer_->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 20, "fixation");
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::showDtFPCloud ()
{
  float bad_pt = numeric_limits<float>::quiet_NaN ();

  // check if distances from fixation point have been computed
  if (dist_from_fixation_.size () <= 0)
    return;
  
  // create the filtered cloud
  copyPointCloud<PointXYZ,PointXYZ> (*input_cloud_, *filtered_cloud_);
  for (int i=0; i<dist_from_fixation_.size (); i++)
  {
    if (!pcl_isfinite (dist_from_fixation_[i]))
      continue;

    if (dist_from_fixation_[i] > max_dist_from_fixation_)
    {
      filtered_cloud_->points[i].x = bad_pt;
      filtered_cloud_->points[i].y = bad_pt;
      filtered_cloud_->points[i].z = bad_pt;
    }
  }

  viewer_->removePointCloud("DtFP_cloud");
  PointCloudColorHandlerCustom<PointXYZ> seagrn (filtered_cloud_, 64,166,131);
  viewer_->addPointCloud<PointXYZ> (filtered_cloud_, seagrn, "DtFP_cloud");
  viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "DtFP_cloud");
}

////////////////////////////////////////////////////////////////////////////////
void
SampleSaliency::showSalPoints (bool ssp)
{
  ExtractIndices<PointIn> ei_filter (false);
  ei_filter.setInputCloud (input_cloud_);
  ei_filter.setIndices (indx_salient_);

  if (ssp)
  {
    ei_filter.filter (*sal_cloud_);
  
    viewer_->removePointCloud("sal_cloud");
    PointCloudColorHandlerCustom<PointXYZ> red (sal_cloud_, 255, 0, 0);
    viewer_->addPointCloud<PointXYZ> (sal_cloud_, red, "sal_cloud");
    viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "sal_cloud");
  }
  else
  {
    //TBD: vis original point cloud
  }
}
