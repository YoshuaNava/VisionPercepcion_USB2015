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

//STD headers
#include <iostream>
#include <algorithm>

// PCL headers
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/search/organized.h>

// RXKINFU headers
#include <rxkinfu/kinfu_app.h>

// SPL headers
#include "map_kinfu_app.h"
#include "patch.h"
#include "patch_fit.h"
#include "patch_plot.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;
using namespace rxkinfu;

#define KINFU_APP rxkinfu::MapKinfuApp
#define DEF_PATCH_LIB "patch_lib/"
#define DEF_PATCH_LIB_TXT "patch_lib.txt"

namespace rxkinfu
{
class MapKinfuAppMatch: public KINFU_APP
{
  /** \brief Running KinFu App for patch loading, picking, and matching.
   *
   * A patch is loaded and then the user pauses, picks a point (shift+click)
   * a patch is fitted and is getting compared with the loaded patch for
   * matching.
   *
   * \author Dimitrios Kanoulas
   */
  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointInCloud;
    typedef PointCloud<PointIn>::Ptr PointInCloudPtr;
    typedef PointCloud<PointIn>::ConstPtr PointInCloudConstPtr;

    /** \brief Constructor. */
    MapKinfuAppMatch (int argc, char **argv);

    /** \brief Destructor. */
    virtual ~MapKinfuAppMatch ();

    /** \brief Setup the input files. */
    void
    setupInputFiles (int argc, char **argv);

    /** \brief Print usage help. */
    static void usageHelp (const char *pfx = "", bool with_inputs = true);

    /** \brief Extracts seed points for fitting patches.  The seeds are points
      * around the loaded patches.
      */
    void findSeeds (PointInCloudPtr &cloud_ptr);

    /** \brief Execute bubble raycast and patch mapping and tracking. */
    void execBubbleRaycast (const Affine3f &bubble_pose);

    /** \brief Execute visualization. */
    void execViz (bool has_data, bool has_image, const Affine3f &bubble_pose);

    /** \brief Load the patch library.
      *
      * \param[in] filename the file to load
      */
    void
    loadPatchLib (const std::string &filename);


  protected:
    /** \brief Lib patches and sequences filenames. */
    vector <std::string> lib_patches_str_, lib_seqs_str_;

    /** \brief Loaded patches from the library in camera and volume frame. */
    vector<Patch> lib_patches_cf_, lib_patches_vf_;
    
    /** \brief Whether patch was loaded. */
    bool lib_patch_loaded_;
    
    /** \brief Lib patch seeds. */
    vector <int> lib_patches_seeds_;
    
    /** \brief Viewpoint. */
    float vp_x_, vp_y_, vp_z_;

    /** \brief For organized neighborhood search. */
    OrganizedNeighbor<PointXYZ> *seed_search_;

    /** \brief Seed's decimation factor wrt the patch radius size. */
    double seed_r_dec_;

    /** \brief Seed point's nearest neighbors. */
    vector <int> nn_ind_;

    /** \brief Seed point's square distances from the nearest neighbors. */
    vector <float>  nn_sqr_dist_;

    /** \brief Picked point's coords. */
    float pt_x_, pt_y_, pt_z_;

    /** \brief Whether point has been picked. */
    bool pt_picked_;

    /** \brief Whether to visualize the picked points. */
    bool vis_pt_picked_;

    /** \brief Whether to fit a patch to the picked point. */
    bool do_fit_;

    /** \brief Neighborhood size. */
    double radius_;

    /** \brief Seed point's nearest neighbors cloud. */
    PointInCloudPtr nn_cloud_;

    /** \brief Patch to fit. */
    Patch patch;

    /** \brief For patch fit. */
    PatchFit *patchFit;

    /** \brief Point cloud viewer. */
    boost::shared_ptr<PCLVisualizer> cloud_viewer_ptr_;

    /** \brief For patch plot. */
    PatchPlot *patch_plot_;

    /** \brief Whether to draw fitted patch. */
    bool draw_patch_;

    /** \brief Whether to start looking for patch match. */
    bool do_patch_match_;
    
    /** \brief Patch match id. */
    int patch_match_id_;
    
    /** \brief Patch match found. */
    bool patch_match_found_;
    
    /** \brief Patch matches position in the map_patch that need to be deleted
      * from the patch map in the end of the frame.
      */
    vector<int> patch_match_for_del_;
};
}
