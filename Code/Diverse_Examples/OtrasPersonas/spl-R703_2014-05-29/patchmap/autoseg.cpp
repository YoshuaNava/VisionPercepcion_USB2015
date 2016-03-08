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

// SPL libraries
#include "autoseg.h"

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace boost;
using namespace Eigen;

#define MAX_SAL_POINTS 100 //max number of salient points
#define MAX_DON_COS_ANGLE 0.9645f //max cos(angle) between two level normals
#define MAX_DONG_COS_ANGLE 0.8195f //max cos(angle) between normals and -gravity

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::autoSegment()
{
  // Total timing
  double last = getTime ();

  // Step 1: filter the input cloud
  //double last_f = pcl::getTime ();
  SampleFilter filter (input_cloud_);
  filter.setMaxPtLimit (input_task_->getMaxPatchDistance());
  filter.filter();
  filtered_cloud_ = filter.cloud_filtered_;
  //double now_f = pcl::getTime ();
  //cerr << "Time for filter: " << now_f-last_f << endl;

  // Return if no points are valid
  if (!AutoSeg::numValidPoints(filtered_cloud_))
    return;
  
  // Update the viewer
  if (do_vis_ && show_filtered_ && viewer_)
    AutoSeg::showFilteredCloud();

  // Decimation ratio
  float dec_ratio = static_cast<float> (nd) / 
                    (2.0*static_cast<float> (filtered_cloud_->points.size ()));

  // Step 2: find salient points on the filtered cloud
  //double last_sal = pcl::getTime ();
  sal_ = new SampleSaliency;
  
  sal_->setInputCloud (filtered_cloud_);
  sal_->setNNSize (2.0f*radius_, 580.0f/dec_ratio);
  sal_->setNNCAThres (MAX_DON_COS_ANGLE);
  if (use_gravity_) {sal_->setG (g_);}
  if (use_gravity_) {sal_->setNGCAThres (MAX_DONG_COS_ANGLE);}
  sal_->setNMS(do_nms_);
  
  sal_->extractSalientPoints ();
  
  // Keep only MAX_SAL_POINTS random points
  // TBD: do it in sal_
  if ((sal_->getIndxSalient())->size() > MAX_SAL_POINTS)
  {
    random_shuffle((sal_->getIndxSalient())->begin(), (sal_->getIndxSalient())->end());
    (sal_->getIndxSalient())->resize (MAX_SAL_POINTS);
  }
  
  //double now_sal = pcl::getTime ();
  //cerr << "Time for sal: " << now_sal-last_sal << endl;

  // Update viewer with normals
  if (do_vis_ && viewer_)
  {
    sal_->setViewer (viewer_);
    if (show_sal_filtered_) sal_->showDtFPCloud ();
    if (show_fixation_) sal_->showFixation ();
    if (show_salient_) sal_->showSalPoints (true);
    if (show_sal_curvatures_) sal_->showCurvature (true);
    if (show_sal_normals_) sal_->showNormals (true);
  }

  if (do_vis_ && show_patches_ && viewer_) removePatchPlots();

  // Create nn object
  search::OrganizedNeighbor<PointXYZ> search;
  search.setInputCloud(filtered_cloud_);
  
  // For each seed point
  ns = 0; // set number of current seeds to zero
  int num_patches = 0;
  for (int si=0; si<(sal_->getIndxSalient())->size (); si++)
  {
    // generate seeds until some termination condition holds
    //if (AutoSeg::terminate())
    //  break;

    // Step 3: generate and validate a new seed
    seed = (sal_->getIndxSalient())->at(si);
    if (!isSeedValid())
      continue;
    seeds.push_back(seed);
    ns++;

    // Step 4: fetch and validate neighborhood
    //double last_nn = pcl::getTime ();
    search.radiusSearch(filtered_cloud_->points[seed], radius_, nn_indices, nn_sqr_distances);
    nn_cloud_->points.resize (0);
    for (int i=0; i<nn_indices.size(); i++)
      nn_cloud_->points.push_back(filtered_cloud_->points[nn_indices[i]]);
    //double now_nn = pcl::getTime ();
    //cerr << "Time for nn: " << now_nn-last_nn << endl;
   
    if (do_vis_ && show_nn_ && viewer_)
      AutoSeg::showNN();

    // Step 5: fit the patch
    //double last_pf = pcl::getTime ();
    PatchFit *patchFit;
    patchFit = new PatchFit(nn_cloud_);
    patchFit->setSSmax (50);
    patchFit->fit();
    //double now_pf = pcl::getTime ();
    //cerr << "Time for fitting " << nn_cloud_->points.size() <<
    //        " points: " << now_pf-last_pf << endl;
  

    // Step 6: validate patch
    //double last_val = pcl::getTime ();
    if (do_validation_)
    {
      (*patchFit->p_).computeResidual (nn_cloud_);
      if ((*patchFit->p_).getResidual() > t_residual_)
        continue;

      VectorXd k;
      k = (*patchFit->p_).getK ();
      if (k.minCoeff () < t_min_curv_ || k.maxCoeff ()>t_max_curv_)
        continue;
    }
    num_patches++;
    //double now_val = pcl::getTime ();
    //cerr << "Time for validation: " << now_val-last_val << endl;

    // Visualize patch
    //double last_pp = pcl::getTime ();
    if (do_vis_ && show_patches_ && viewer_)
    {
      PatchPlot *patch_plot;
      (*patchFit->p_).setID (ns);
      (*patchFit->p_).setSS (0.025);
      (*patchFit->p_).infer_params();
      (*patchFit->p_).gs();

      //PatchPlot patch_plot (*patchFit->p_);
      patch_plot = new PatchPlot (*patchFit->p_);
    
      patch_plot->showPatch (viewer_, 0, 1, 0, boost::to_string(ns) + "_patch");
    
      delete (patch_plot);
    }
    //double now_pp = pcl::getTime ();
    //cerr << "Time for ploting patch: " << now_pp-last_pp << endl;

    if (!no_stats_)
    {
      // Save normal vector
      (*patchFit->p_).setCnn (sal_->getNormalNormalAngle (seed));
      if (use_gravity_)
        (*patchFit->p_).setCng (sal_->getNormalGravityAngle (seed));

      // Step 7: save statistics
      AutoSeg::saveStat (*patchFit->p_);
    
      // Step 8: print statistics
      Vector2d sk = (*patchFit->p_).getSK();
      cout << "patch stats" << " "
           << sk (0) << " "
           << sk (1) << " "
           << (*patchFit->p_).getCnn () << " "
           << (*patchFit->p_).getCng () << endl;
    }
    // delete objects
    delete (patchFit);

  } // for each seed

  if(do_vis_ && show_patches_) max_patch_plot_id_ = ns-1;
  else max_patch_plot_id_ = 0;

  // Total timing
  double now = getTime();
  cerr << "Total autoseg for " << num_patches << " patches out of " << ns <<
          " seed points: " << now-last << " sec." << endl;

  // destroy objects
  delete (sal_);
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::manSegment (const vector<int> &indices)
{
  double last = getTime (); //timing

  // init nn, normal estimation, and patch fit
  search::OrganizedNeighbor<PointXYZ> search;
  search.setInputCloud(input_cloud_);
  PatchFit *patchFit;
  PatchPlot *patch_plot;
  normal_cloud_->points.resize (input_cloud_->points.size());
  double fitting_radius;

  PointCloudNormalPtr normals_2(new pcl::PointCloud<pcl::Normal>);
  normals_2->points.resize (input_cloud_->points.size());

  // for each point in input cloud create a normal point
  float bad_pt = numeric_limits<float>::quiet_NaN ();
  for (int i=0; i<input_cloud_->points.size(); i++)
  {
    normal_cloud_->points[i].normal_x = bad_pt;
    normal_cloud_->points[i].normal_y = bad_pt;
    normal_cloud_->points[i].normal_z = bad_pt;

    normals_2->points[i].normal_x = bad_pt;
    normals_2->points[i].normal_y = bad_pt;
    normals_2->points[i].normal_z = bad_pt;
  }
  normal_cloud_->width = input_cloud_->width;
  normal_cloud_->height = input_cloud_->height;
  normal_cloud_->is_dense = input_cloud_->is_dense;

  normals_2->width = input_cloud_->width;
  normals_2->height = input_cloud_->height;
  normals_2->is_dense = input_cloud_->is_dense;

  Vector3f normal, normal_2;
  float vpx, vpy, vpz;
  vpx = input_cloud_->sensor_origin_.coeff (0);
  vpy = input_cloud_->sensor_origin_.coeff (1);
  vpz = input_cloud_->sensor_origin_.coeff (2);
  
  if (do_vis_ && show_patches_ && viewer_) removePatchPlots();

  // for each seed point
  for (int i=0; i<indices.size(); i++)
  {
    for (int j=0; j<2; j++) // for radius and radius/2
    {
      // fitting radius
      fitting_radius = radius_/static_cast<float> (j+1);
      
      // Step 1: find neighborhood
      search.radiusSearch(input_cloud_->points[indices[i]], fitting_radius,
                          nn_indices, nn_sqr_distances);
      nn_cloud_->points.resize (0); 
      for (int k=0; k<nn_indices.size (); k++)
        nn_cloud_->points.push_back (input_cloud_->points[nn_indices[k]]);

      // Step 2: fit a patch
      patchFit = new PatchFit (nn_cloud_);
      patchFit->fit ();
      (*patchFit->p_).computeResidual (nn_cloud_);

      // visualize the patch
      if (do_vis_ && show_patches_ && viewer_)
      {
        (*patchFit->p_).setID (2*i+j);
        (*patchFit->p_).setSS (0.03);
        (*patchFit->p_).infer_params();
        (*patchFit->p_).gs();
        patch_plot = new PatchPlot(*patchFit->p_);
        patch_plot->showPatch (viewer_, j, 1, 0, boost::to_string(2*i+j) + "_patch");

        // Destroy objects
        delete (patch_plot);
      }
      
      // Step 3: find normal vector
      if (j==0)
      {
        normal = (((*patchFit->p_).rexp((*patchFit->p_).getR ())).col(2)).cast<float>();
        flipNormalTowardsViewpoint (input_cloud_->points[indices[i]], vpx, vpy, vpz, normal);
      }
      else
      {
        normal_2 = (((*patchFit->p_).rexp((*patchFit->p_).getR ())).col(2)).cast<float>();
        flipNormalTowardsViewpoint (input_cloud_->points[indices[i]], vpx, vpy, vpz, normal_2);
      }
    }

    if(do_vis_ && show_patches_) max_patch_plot_id_ = 2*indices.size()-1;
    else max_patch_plot_id_ = 0;
    
    // Step 4: find the cos(angle) between normal-gravity vector
    normal_cloud_->points[indices[i]].normal_x = normal (0);
    normal_cloud_->points[indices[i]].normal_y = normal (1);
    normal_cloud_->points[indices[i]].normal_z = normal (2);
    (*patchFit->p_).setCnn (normal.dot (normal_2));
    if (use_gravity_)
      (*patchFit->p_).setCng (normal.dot (-1.0f*g_));

    normals_2->points[indices[i]].normal_x = normal_2 (0);
    normals_2->points[indices[i]].normal_y = normal_2 (1);
    normals_2->points[indices[i]].normal_z = normal_2 (2);

    // Step 5: save statistics
    AutoSeg::saveStat (*patchFit->p_);
    
    // Step 6: print patch info
    Vector2d sk = (*patchFit->p_).getSK();
    cout << "patch stats" << " "
         << sk (0) << " "
         << sk (1) << " "
         << (*patchFit->p_).getCnn () << " "
         << (*patchFit->p_).getCng () << endl;

    // Destroy objects
    delete (patchFit);
  } //for each seed point

  // Visualize normal vectors
  if (do_vis_ && show_patch_normals_ && viewer_)
  {
    const std::string normal_id = "patch_normal";
    viewer_->removePointCloud (normal_id);
    viewer_->addPointCloudNormals<PointXYZ, Normal>(input_cloud_, normal_cloud_,
                                                    1, 0.15, normal_id);
    viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0,
                                               0.0, normal_id);

    const std::string normal_2_id = "patch_normal_2";
    viewer_->removePointCloud (normal_2_id);
    viewer_->addPointCloudNormals<PointXYZ, Normal>(input_cloud_, normals_2, 1,
                                                    0.15, normal_2_id);
    viewer_->setPointCloudRenderingProperties (PCL_VISUALIZER_COLOR, 0.0, 1.0,
                                               0.0, normal_2_id);
  }
  
  // timing
  double now = getTime ();
  cout << "Manseg in: " << now-last << "sec." << endl;
}

////////////////////////////////////////////////////////////////////////////////
bool
AutoSeg::terminate ()
{
  // generated maximum seeds
  if ((maxs>0) && (ns>=maxs))
    return (true);

  return (false);
}

////////////////////////////////////////////////////////////////////////////////
bool
AutoSeg::isSeedValid()
{
  return (pcl_isfinite(filtered_cloud_->points[seed].x) &&
          pcl_isfinite(filtered_cloud_->points[seed].y) &&
          pcl_isfinite(filtered_cloud_->points[seed].z));
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::showFilteredCloud ()
{
  // Draw the cloud
  viewer_->removePointCloud("filtered_cloud");
  PointCloudColorHandlerCustom<PointXYZ> umarine (filtered_cloud_, 128,163,201);
  viewer_->addPointCloud<PointXYZ> (filtered_cloud_, umarine, "filtered_cloud");
  viewer_->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1,
                                            "filtered_cloud");
}
  
////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::showSeed ()
{
  seed_cloud_->points.push_back(input_cloud_->points[seed]);
  PointCloudColorHandlerCustom<PointXYZ> red (seed_cloud_, 255, 0, 0);
  viewer_->removePointCloud("seed_cloud");
  viewer_->addPointCloud(seed_cloud_, red, "seed_cloud");
  viewer_->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5,
                                            "seed_cloud");
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::showNN ()
{
  
  PointCloudColorHandlerCustom<PointXYZ> blue (nn_cloud_, 0, 0, 255);
  viewer_->removePointCloud("nn_cloud");
  viewer_->addPointCloud(nn_cloud_, blue, "nn_cloud");
  viewer_->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5,
                                            "nn_cloud");
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::saveStat (Patch &p)
{
  total_cos_angles_.push_back (p.getCng ());

  Vector2d sk;
  sk = p.getSK();
  total_kx_.push_back (sk(0));
  total_ky_.push_back (sk(1));
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::printStat ()
{
  double min_angle, max_angle;
  min_angle = VectorXd::Map(&total_cos_angles_[0],
                            total_cos_angles_.size()).minCoeff();
  max_angle = VectorXd::Map(&total_cos_angles_[0],
                            total_cos_angles_.size()).maxCoeff();

  cout << "Min normal-gravity cos(angle): " << min_angle  << endl;
  cout << "Max normal-gravity cos(angle): " << max_angle  << endl;

  double min_kx_curv, max_kx_curv;
  min_kx_curv = VectorXd::Map(&total_kx_[0], total_kx_.size()).minCoeff();
  max_kx_curv = VectorXd::Map(&total_kx_[0], total_kx_.size()).maxCoeff();

  cout << "Min kx curv: " << min_kx_curv << endl;
  cout << "Max kx curv: " << max_kx_curv << endl;
  
  double min_ky_curv, max_ky_curv;
  min_ky_curv = VectorXd::Map(&total_ky_[0], total_ky_.size()).minCoeff();
  max_ky_curv = VectorXd::Map(&total_ky_[0], total_ky_.size()).maxCoeff();

  cout << "Min ky curv: " << min_ky_curv << endl;
  cout << "Max ky curv: " << max_ky_curv << endl;
}

////////////////////////////////////////////////////////////////////////////////
void
AutoSeg::removePatchPlots ()
{
  while (max_patch_plot_id_ >= 0)
    viewer_->removeShape(boost::lexical_cast<string>(max_patch_plot_id_--)
                         + "_patch");
}
