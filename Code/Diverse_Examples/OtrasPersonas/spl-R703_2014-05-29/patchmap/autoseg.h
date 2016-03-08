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

#ifndef AUTOSEG_H_
#define AUTOSEG_H_

// SPL headers
#include "patch.h"
#include "patch_fit.h"
#include "patch_plot.h"
#include "sample_filter.h"
#include "sample_saliency.h"
#include "task.h"

// PCL headers
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace boost;
using namespace Eigen;

// Fixed values
#define RESIDUAL_THRES 0.01 //TBD

class AutoSeg
{
  /** \brief Automatically segments a point cloud dataset into a set of patches
    * for a single frame.  It is required to set the input point cloud 
    * (setInputCloud) and a task (setTask).  The steps for segmentation are:
    *   -- filter the cloud
    *   -- find seed points
    *   -- find their neighborhood
    *   -- fit patches and evaluate them
    * The output is a set of fitted patch for the specific cloud.
    *
    * \author Dimitrios Kanoulas
    */
  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointCloudIn;
    typedef PointCloudIn::Ptr PointCloudInPtr;
    typedef PointCloudIn::ConstPtr PointCloudInConstPtr;
    typedef PointCloud<Normal> PointCloudNormal;
    typedef PointCloudNormal::Ptr PointCloudNormalPtr;

    /** \brief Contructor. */
    AutoSeg () :
      filtered_cloud_ (new PointCloudIn),
      normal_cloud_ (new PointCloudNormal),
      seed_cloud_ (new PointCloudIn),
      nn_cloud_ (new PointCloudIn),
      maxs (10),
      ns (0),
      use_gravity_ (false),
      no_stats_ (false),
      max_patch_plot_id_(-1),
      t_residual_ (0.01),
      t_max_curv_ (17.46),
      t_min_curv_ (-12.06),
      do_nms_ (false), do_validation_ (false), do_vis_ (false),
      show_filtered_ (false), show_fixation_ (false),
      show_nn_ (false),
      show_patches_ (false),
      show_patch_normals_ (false),
      show_salient_ (false),
      show_sal_curvatures_ (false),
      show_sal_filtered_ (false),
      show_sal_normals_ (false)
    {
      // reset statistics
      total_cos_angles_.resize(0);
      total_kx_.resize(0);
      total_ky_.resize(0);
    }
    
    /** \brief Contructor given input cloud and task.
      *
      * \param[in] input_point_cloud_arg the input point cloud
      * \param[in] task_arg the input task
      */
    
    AutoSeg (const PointCloudInConstPtr &input_cloud_arg,
             boost::shared_ptr<Task> task_arg) :
      input_cloud_ (input_cloud_arg),
      filtered_cloud_ (new PointCloudIn),
      normal_cloud_ (new PointCloud<Normal>),
      seed_cloud_ (new PointCloudIn),
      nn_cloud_ (new PointCloudIn),
      input_task_ (task_arg),
      maxs (10),
      ns (0),
      use_gravity_ (false),
      no_stats_ (false),
      t_residual_ (0.01),
      t_max_curv_ (17.46),
      t_min_curv_ (-12.06),
      do_nms_ (false),
      do_validation_ (false),
      do_vis_ (false),
      show_filtered_ (false),
      show_fixation_ (false),
      show_nn_ (false),
      show_patches_ (false),
      show_patch_normals_ (false),
      show_salient_ (false),
      show_sal_curvatures_ (false),
      show_sal_normals_ (false)
    {
      // size of point cloud
      nd = input_cloud_->points.size ();
      
      // set the radius
      radius_ = input_task_->getMaxPatchSize ();
      
      // reset statistics
      total_cos_angles_.resize(0);
      total_kx_.resize(0);
      total_ky_.resize(0);
    }

    /** \brief Virtual destructor. */
    virtual
    ~AutoSeg ()
    {
      cout << "Destroying AutoSeg object" << endl;
    }

    /** \brief Set the input cloud. */
    inline void
    setInputCloud (const PointCloudInConstPtr &input_cloud_arg)
    {
      this->input_cloud_ = input_cloud_arg;
      this->nd = input_cloud_->points.size ();
    }

    /** \brief Set the input task. */
    inline void
    setTask (boost::shared_ptr<Task> task_arg)
    {
      this->input_task_ = task_arg;
      radius_ = input_task_->getMaxPatchSize ();
    }

    /** \brief Set the viewer. */
    inline void
    setViewer (boost::shared_ptr<PCLVisualizer> viewer_arg)
    {
      viewer_ = viewer_arg;
      if (viewer_) {
        //(viewer_)->removeAllShapes ();
        do_vis_ = true;
      } else do_vis_ = false;
    }

    /** \brief Set the gravity vector. */
    inline void
    setG (VectorXf g_arg)
    {
      this->g_ = g_arg;
      use_gravity_ = true;
    }

    /** \brief Main autoseg algorithm. */
    void
    autoSegment();
    
    /** \brief Main loop for manual autoseg algorithm.
      *
      * \param[in] indices the seed where the patches will be fitted
      */
    void
    manSegment (const vector<int> &indices);

    /** \brief Whether to terminate the main segment loop if one of the
      * following conditions hold:
      * -- generated maximum seeds, i.e. ns>=maxs 
      *
      * \param[out] whether to terminate
      */
    bool
    terminate();

    /** \brief Generates seed points. */
    void
    generateSeed();

    /** \brief Validates a seed point. */
    bool
    isSeedValid();

    /** \brief Updates the cloud viewer with the filtered one. */
    void
    showFilteredCloud();

    /** \brief Show seed points. */
    void
    showSeed();
    
    /** \brief Show nearest neighbors. */
    void
    showNN();
    
    /** \brief Whether to visualize filtered cloud. */
    inline void setShowFiltered (bool show) { show_filtered_ = show;}

    /** \brief Whether to visualize fixation point. */
    inline void setShowFixation (bool show) { show_fixation_ = show;}

    /** \brief Whether to visualize salient points. */
    inline void setShowSalient (bool show) { show_salient_ = show;}

    inline void setShowSalFiltered (bool show) { show_sal_filtered_ = show;}

    /** \brief Whether to visualize salient point curvatures. */
    inline void setShowSalCurvatures (bool show) { show_sal_curvatures_ = show;}

    /** \brief Whether to visualize salient point normals. */
    inline void setShowSalNormals (bool show) { show_sal_normals_ = show; }

    /** \brief Whether to visualize neighborhoods. */
    inline void setShowNN (bool show) { show_nn_ = show; }
 
    /** \brief Whether to visualize patches. */
    inline void setShowPatches (bool show) { show_patches_= show; }

    /** \brief Whether to visualize patch normals. */
    inline void setShowPatchNormals (bool show) { show_patch_normals_ = show; }

    /** \brief Whether to do nonmaximal suppression. */
    inline void setNMS (bool nms) { do_nms_ = nms; }

    /** \brief Whether to validate the patchesa after fitting. */
    inline void setDoValidation (bool do_validation_arg) { do_validation_ = do_validation_arg; }
    
    /** \brief Whether to do statistics (print and save them). */
    inline void setNoStats (bool no_stats_arg) { no_stats_ = no_stats_arg;  }

    /** \brief Get number of invalid points. */
    inline int
    numValidPoints(PointCloudInConstPtr cloud)
    {
      int counter = 0;
      for (int i=0; i<cloud->points.size (); i++)
      {
        if (!pcl_isfinite (cloud->points[i].x) ||
            !pcl_isfinite (cloud->points[i].y) ||
            !pcl_isfinite (cloud->points[i].z))
          continue;
        counter++;
      }
      return (counter);
    }

    /** \brief Keep patch statistics. */
    void
    saveStat (Patch &p);

    /** \brief Print out statistics about patches. */
    void
    printStat ();

    /** \brief Remove existing patch plots. */
    void
    removePatchPlots ();

    /** \brief cos(angles) between patch's normals and gravity vector. */
    vector<double> total_cos_angles_;

    /** \brief patches curvature. */
    vector<double> total_kx_;
    vector<double> total_ky_;

  protected:
    /** \brief The input point cloud. */
    PointCloudInConstPtr input_cloud_;

    /** \brief The filtered point cloud. */
    PointCloudInConstPtr filtered_cloud_;

    /** \brief For storing normal clouds. */
    PointCloudNormalPtr normal_cloud_;

    /** \brief The seeds point cloud.  Used for visualization. */
    PointCloudInPtr seed_cloud_;

    /** \brief Nearest neighbor point cloud. */
    PointCloudInPtr nn_cloud_;

    /** \brief The input task. */
    boost::shared_ptr<Task> input_task_;

    /** \brief Pointer to the viewer. */
    boost::shared_ptr<PCLVisualizer> viewer_;

    /** \brief For sample saliency. */
    SampleSaliency *sal_;

    /** \brief Input gravity vector. */
    Vector3f g_;

    /** \brief If positive, the maximum number of seed points. */
    int maxs;

    /** \brief Point cloud size. */
    int nd;

    /** \brief Number of seeds that are generated so far. */
    int ns;

    /** \brief Seed of point index. */
    int seed;

    /** \brief Seeds of point indices. */
    vector< int > seeds;

    /** \brief Nearest neighbor radius search distance in m. */
    double radius_;

    /** \brief Indices to nearest neighbor of seed point. */
    vector <int> nn_indices;

    /** \brief Sqruare distancews from seed point to nearest neighbors. */
    vector< float >  nn_sqr_distances;

    /** \brief Whether to use the gravity vector. */
    bool use_gravity_;

    /** \brief Whether to do statistics (save and print them). */
    bool no_stats_;

    /** \brief Largest ID of previously plotted patch. */
    int max_patch_plot_id_;

    /** \brief The threshold for the geometric residual (only if do_validation). */
    double t_residual_;

    /** \brief The thresohld for the max curvature (only if do_validation). */
    double t_max_curv_;

    /** \brief The thresohld for the max curvature (only if do_validation). */
    double t_min_curv_;
    
    /** \brief Whether to run Non-Maximum Suppression (NMS) . */
    bool do_nms_;

    /** \brief Whether to validate patches after fitting them. */
    bool do_validation_;
    
    /** \brief Whether to visualize data. */
    bool do_vis_;

    /** \brief Whether to show only the filtered cloud. */
    bool show_filtered_;

    /** \brief Whether to show the fixation point. */
    bool show_fixation_;

    /** \brief Whether to show the neighborhoods. */
    bool show_nn_;

    /** \brief Whether to show the fitted patches. */
    bool show_patches_;

    /** \brief Whether to show the patch normal vectors. */
    bool show_patch_normals_;

    /** \brief Whether to show the salient points. */
    bool show_salient_;

    /** \brief Whether to show the curvatures. */
    bool show_sal_curvatures_;

    /** \brief Whether to show only the salient points. */
    bool show_sal_filtered_;

    /** \brief Whether to show the salient points' normal vectros. */
    bool show_sal_normals_;
};

#endif // #ifndef AUTOSEG_H_
