/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-, Dimitrios Kanoulas
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

#ifndef MAP_KINFU_APP_H_
#define MAP_KINFU_APP_H_

// STD headers
#include <unistd.h>

// PCL headers
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d.h>

// RXKINFU headers
#include <rxkinfu/kinfu_app.h>

// SPL-CPP headers
#include "patch.h"
#include "patch_coverage.h"
#include "patch_fit.h"
#include "patch_plot.h"
#include "patch_select.h"
#include "sample_saliency.h"
#include "map_patch.h"
#include "map_cell.h"

using namespace std;
using namespace pcl;
using namespace pcl::search;
using namespace Eigen;

namespace rxkinfu
{
  class MapKinfuApp: public KinfuApp
  {
    /** \brief Implements the Patch Mapping and Tracking algorithm using the
      * Moving Volume Kinect Fusion environment representation [1] for the 
      * 6DOF camera pose. The overall approach is described in [2-5]
      *
      *
      * [1] "Moving Volume KinectFusion", Roth, Vona, BMVC 2012.
      * [2] "Curved Surface Contact Patches with Quantified Uncertainty",
      *     Vona, Kanoulas, IROS 2011.
      * [3] "Sparse Surface Modeling with Curved Patches", Kanoulas, Vona,
      *     ICRA 2013.
      * [4] "Bio-Inspired Rough Terrain Contact Patch Perception", Kanoulas,
      *     Vona, ICRA 2014.
      * [5] "TBD", Kanoulas, Vona, 2014.
      *
      * \author Dimitrios Kanoulas
      */
    public:
      typedef PointXYZ PointIn;
      typedef PointCloud<PointIn> PointInCloud;
      typedef PointCloud<PointIn>::Ptr PointInCloudPtr;
      typedef PointCloud<PointIn>::ConstPtr PointInCloudConstPtr;

      /** \brief Constructor. */
      MapKinfuApp (int argc, char **argv);

      /** \brief Destructor. */
      virtual ~MapKinfuApp ();

      /** \brief Setup for timing. */
      void setupTime ();
      
      /** \brief Setup for timing. */
      void setupCounters ();
      
      /** \brief Setup for drawing. */
      void setupDraw (int argc, char **argv);

      /** \brief Setup for sample saliency. */
      void setupSaliency (int argc, char **argv);

      /** \brief Setup for Patch Mapping and Tracking. */
      void setupMap (int argc, char **argv);

      /** \brief Setup for patch fitting validation. */
      void setupValidation (int argc, char **argv);

      /** \brief Setup for Patch Selection. */
      void setupPatchSelection (int argc, char **argv);

      /** \brief Print usage help. */
      static void usageHelp (const char *pfx = "", bool with_inputs = true);

      /** \brief Set print_status */
      void setPrintStatus (bool print_status_arg);

      /** \brief Set the grid cells info type to be displayed. */
      void setDrawGridCellsInfoType (int type);

      /** \brief Set the max patch map time. */
      void setMaxPatchMapTime (float time);

      /** \brief Set the max number of patches in the map. */
      void setMaxPatches (int max_patches);

      /** \brief Get the max number of patches in the map. */
      int getMaxPatches ();
      
      /** \brief Set the max number of patches per cell in the map. */
      void setMaxPatchesPerCell (int max_patches_per_cell);

      /** \brief Get the max number of patches per cell in the map. */
      int getMaxPatchesPerCell ();

      /** \brief Set the max number of patches in the map. */
      void setMapFittedPatches (int map_fitted_patches);

      /** \brief Get the max number of patches in the map. */
      int getMapFittedPatches ();

      /** \brief Set the cell decimate factor. */
      void setCellDecFactor (int cell_dec_factor);

      /** \brief Set the map cells tries. */
      void setMapCellsTries (int map_cells_tries);

      /** \brief Set the radius. */
      void setRadius (double radius);

      /** \brief Creates the sampling grid cells on the ground , which is the 
        * TSDF volume xz-axis in the max y-distance. (the cells are row-wised
        * starting from the lowerleft, which is also the TSDF volume 
        * xz-projected frame; x are the rows, z are the cols).
        */
      void initMapCells ();

      /** \brief Updates the sampling grid cells by updating first the position
        * with repsect to the camera projection on the ground xz-axis and re-
        * ordering them in the vector wrt this distance.  The purpose of the
        * update is to make closer cells filled first with patches.
        */
      void updateMapCells ();

      /** \brief Resets the map cells. */
      void resetMapCells ();
      
      /** \brief Get row-wised cell's id given a 3D point. */
      int getMapCellId (Vector3f c);

      /** \brief Configures the number of patches in the map. */
      void initMapPatches ();

      /** \brief Clean the list of patches from invalid ones. */
      void cleanMapPatches ();

      /** \brief Resets the patches in the map. */
      void resetMapPatches ();

      /** \brief Updates each patch's pose wrt the TSDF volume movement. */
      void updatePatchPose ();

      /** \brief Matches cell grids with the points that are included. */
      void matchPatchesCells (PointInCloudPtr &cloud_ptr);

      /** \brief Extracts seed points for fitting patches.  It first updates the
        * position of the old seeds when the TSDF volume moves and removes those
        * that are outside the volume, and then adds more seed points in the 
        * following way:
        * -- splits the space into grids
        * -- makes sure that in each grid there are at most a fixed number of 
        *    random points
        */
      virtual void findSeeds (PointInCloudPtr &cloud_ptr);
      
      /** \brief Find the salient points.
        *
        * \param[in] in_cloud_ptr the input cloud
        * \param[out] out_cloud_ptr the output cloud
        */
      void findSalientCloud (PointInCloudPtr &in_cloud_ptr,
                             PointInCloudPtr &out_cloud_ptr);

      /** \brief Execute bubble raycast and patch mapping and tracking. */
      void execBubbleRaycast (const Affine3f &bubble_pose);

      /** \brief Execute external algs, eg control related. */
      void execExt ();

      /** \brief Execute visualization. */
      void execViz (bool has_data, bool has_image, const Affine3f &bubble_pose);

      /** \brief Visualize the salient points. */
      void visSaliency ();

      /** \brief Visualize the fitted patches.
        *
        * \param[in] level display only every level'th patch
        */
      void visPatches (int level=1);

      /** \brief Visualize patches' normals. */
      void visPatchNormals ();

      /** \brief Visualize the projected camera on the lower TSDF plane. */
      void visProjCam ();
      
      /** \brief Visualize the grid cells on the lower TSDF plane. */
      void visGridCells ();

      /** \brief Visualize grid cell information.
        *
        * \param[in] type the info to be printed in the lower TSDF plane:
        * -- 1: the cell ids
        * -- 2: the weights
        * -- 3: the number of patches
        */
      void visGridCellsInfo (int type=1);

      /** \brief Visualize selected patch(es). */
      void visSelPatch ();

      /** \brief Printing timing infoi.
        *
        * \return number of newlines printed
        */
      int printStatus ();

      /** \brief TBD */
      void updateSeeds ();

      /** \brief TBD */
      void searchNeighbors (PointInCloudPtr &cloud_ptr, int seed);

      /** \brief TBD */
      int numberOfValidPoints (PointInCloudPtr cloud_ptr);


      /** PATCH SELECTION */
      /** \brief Select patch (world frame). 
        *
        * The patch should have the following properties:
        * -- be in distance at most sel_ptr_ from point sel_pt_
        * -- have curvature in [sel_min_curv_,sel_max_curv_]
        * -- have slope in [sel_min_slope_, sel_max_slope_] between normal and
        *    -gravity vectors
        *
        * \return the patch id in the map_patches vector
        */
      int selectPatch ();

      /** \brief Reset the selected patches vector. */
      void resetSelPatches ();

    private:
      /** \brief Sensor origin. */
      float vp_x_, vp_y_, vp_z_;

      /** \brief Number of nomals in the visualizer. */
      int vis_num_normals_;

    protected:
      /** \brief Whether to draw the grid cells on the plane perpedicular to
        * the gravity vector.
        */
      bool draw_grid_cells_;

      /** \brief Whether to draw the salient points. */
      bool draw_sal_points_;

      /** \brief Whether to draw the fitted patches. */
      bool draw_fitted_patches_;

      /** \brief The level of fitted patches visualization (see visPatches). */
      int draw_fitted_patches_level_;
      
      /** \brief Whether to draw the patch normal (local z-coord). */
      bool draw_patch_normals_;

      /** \brief Whether to draw the selected patches. */
      bool draw_selected_patches_;

      /** \brief Timing for drawing */
      float vis_patches_ms_;

      /** \brief Whether to draw the projected camera point on the lower plane
        * of the TSDF volume box.
        */
      bool draw_proj_cam_;

      /** \brief Whether to draw the grid cells info on the lower TSDF plane. */
      bool draw_grid_cells_info_;

      /** \brief The grid cells info type to be displayed. */
      int draw_grid_cells_info_type;

      /** \brief Whether to use saliency. */
      bool use_sal_;

      /** \brief Whether to consider DoN, DoNG, or DtFP in saliency. */
      bool sal_DoN_, sal_DoNG_, sal_DtFP_;

      /** \brief The fixation point properties for saliency in the order: 1. 
        * Distance to the ground, 2. Distance of two steps, 3. Distance from 
        * the body (all in m).
        */
      double sal_DtFP_fdg_, sal_DtFP_ftsd_, sal_DtFP_fdfb_;
      
      /** \brief Whether fixation point new props have been set. */
      bool sal_DtFP_props_;

      /** \brief If positive, the DoN threshold (max cos angle). */
      double sal_max_DoN_;

      /** \brief If positive, the DoNG threshold. (max cos angle) */
      double sal_max_DoNG_;

      /** \brief If positive, the max distance to the fixation point for being
        * salient.
        */
      double sal_max_DtFP_;

      /** \brief Waiting cycles until the patchMap starts. */
      int start_cycle_;

      /** \brief The counter for waiting cycles. */
      int start_cycle_counter_;

      /** \brief Timer. */
      StopWatch timer_;
      
      /** \brief Timing for patch mapping. */
      float find_seeds_ms_;

      /** \brief Timing for patch mapping. */
      float map_ms_;

      /** \brief Maximum time (in ms) for the patch mapping (default 30ms). */
      float max_patch_map_time_;

      /** \brief Whether the algorithm runs for first time. */
      bool firstRun_;

      /** \brief Whether the visualizer runs for first time. */
      bool firstVisRun_;

      /** \brief If positive, the max number of patches in the map. */
      int max_patches_;
      
      /** \brief The max number of patches in the map. */
      int max_patches_per_cell_;

      /** \brief The current number of patches in the map. */
      int map_fitted_patches_;
      
      /** \brief The current number of patches fitted in the current frame. */
      int frame_fitted_patches_;
      
      /** \brief The current number of patches fitted from the beginning. */
      int total_fitted_patches_;

      /** \brief Totally removed patches from the beginning. */
      int total_removed_patches_;

      /** \brief Fitted patches. To remove a patch from the map both seed and
        * patch needs to be set to false.
        */
      vector<MapPatch> map_patches_;

      /** \brief The decimation factor for grid cell creation.  E.g. if it is 2
        * then the whole pixel grid will be splitted to 2x2 grid. */
      float cell_dec_factor_;

      /** \brief Grid cells width (along x-axis). */
      int map_cells_w_;

      /** \brief Grid cells height. */
      int map_cells_h_;

      /** \brief Grid cells. */
      vector<MapCell> map_cells_;
      
      /** \brief Grid cells. */
      vector<MapCell> map_cells_copy_;

      /** \brief Number of tries to find a seed point in a cell.  The reasons
        * for not finding a seed point is that either there not a good one or
        * not a valid (NAN) one.
        */
      int map_cells_tries_;


      /** \brief Point cloud viewer. */
      boost::shared_ptr<PCLVisualizer> cloud_viewer_ptr_;

      /** \brief Seed point for the patch. */
      PointXYZ seed_pt;

      /** \brief For organized neighborhood search. */
      OrganizedNeighbor<PointIn> search_;

      /** \brief Sphere radius for organized neighborhood seearch. */
      double radius_;

      /** \brief Seed point's nearest neighbors. */
      vector <int> nn_indices;

      /** \brief Seed point's square distances from the nearest neighbors. */
      vector <float>  nn_sqr_distances;

      /** \brief Seed point's nearest neighbors cloud. */
      PointInCloudPtr nn_cloud_;

      /** \brief For patch fit. */
      PatchFit *patchFit;
     
      /** \brief Fitted patch. */
      //TBD: init with fixed values
      vector<Patch> patches_;
      vector<bool> is_patch_valid_;

      /** \brief For patch plot. */
      PatchPlot *patch_plot;

      /** \brief Whether to validate the patch wrt residual and coverage. */
      bool do_validation_;

      /** \brief Residual threshold if do_validation_ is active. */
      double residual_thres_;

      /** \brief Minimum coverage percent for kept patches. */
      double cov_thres_;

      /** \brief For sample saliency. */
      SampleSaliency *sal_;

      /** \brief Salient point cloud. */
      PointInCloudPtr filtered_cloud_;

      /** \brief The 2D projection of the camera on the plane. */
      Vector2f proj_cam_;

      /** \brief A vector of map patch ids for patches to be removed from the 
        * graphics.
        *
        * \note map patches should have a unique id through the whole run
        */
      vector<int> map_patches_to_remove_;


      /** PATCH SELECTION */
      /** \brief Selected patch: whether to select patches. */
      bool sel_p_;

      /** \brief Selected patch: the center point of the selected patch (camera frame). */
      Vector3d sel_pt_;
      
      /** \brief Selected patch: the radius around the selected patch's center point. */
      double sel_pt_r_;

      /** \brief Selected patch: min&max curvature. */
      double sel_min_curv_, sel_max_curv_;
      
      /** \brief Selected patch: min&max slope. */
      double sel_min_slope_, sel_max_slope_;
      
      /** \brief Selected patch: all selected patch ids potentially good. */
      vector<int> sel_patches_;
      
      /** \brief Selected patch: first selected patch index in the map_patches vector. */
      int sel_patch_index_;

      /** \brief A vector of selected patch ids for patches to be removed from the 
        * graphics.
        *
        * \note map patches should have a unique id through the whole run
        */
      vector<int> sel_patches_to_remove_;
  };
}

#endif //#ifndef MAP_KINFU_APP_H_
