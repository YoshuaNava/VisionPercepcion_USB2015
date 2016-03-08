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

#include "map_kinfu_app_match.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;
using namespace rxkinfu;

////////////////////////////////////////////////////////////////////////////////
MapKinfuAppMatch::MapKinfuAppMatch (int argc, char **argv) :
  KINFU_APP (argc, argv),
  seed_search_ (new OrganizedNeighbor<PointXYZ> (true)),
  nn_cloud_ (new PointCloud<PointXYZ>),
  patch_plot_ (new PatchPlot)
{
  lib_patch_loaded_ = false;
  vp_x_ = vp_y_ = vp_z_ = 0.0;
  print_status = false;
  seed_r_dec_ = 3.0;
  pt_x_ = pt_y_ = pt_z_ = 0.0;
  pt_picked_ = false;
  vis_pt_picked_ = false;
  do_fit_ = false;
  radius_ = 0.05;
  draw_patch_ = false;
  do_patch_match_ = false;
  patch_match_id_ = -1;
  patch_match_found_ = false;

  setupInputFiles (argc, argv);
}

////////////////////////////////////////////////////////////////////////////////
MapKinfuAppMatch::~MapKinfuAppMatch ()
{
  delete (seed_search_);
  delete (patch_plot_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::setupInputFiles (int argc, char **argv)
{
  std::string patch_lib_file = std::string (DEF_PATCH_LIB) + DEF_PATCH_LIB_TXT;
  parse_argument(argc, argv, "-patchlib", patch_lib_file);
  print_highlight("Loading patchlib file \"%s\"\n", patch_lib_file.c_str());
  loadPatchLib (patch_lib_file);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::usageHelp (const char *pfx, bool with_inputs)
{
  // call superclass implementation
  KINFU_APP::usageHelp (pfx, with_inputs);

  cout << "\nMap KinFu App Match Options\n";
  cout << pfx << "  -patchlib file.txt: (default " << DEF_PATCH_LIB << ")\n";
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::findSeeds (PointInCloudPtr &cloud_ptr)
{
  //TBD: call superclass findSeeds

  // return if patch match have been found
  if (!do_patch_match_ || patch_match_found_) { return; }

  // set the sensor viewpoint
  vp_x_ = cloud_ptr->sensor_origin_.coeff (0);
  vp_y_ = cloud_ptr->sensor_origin_.coeff (1);
  vp_z_ = cloud_ptr->sensor_origin_.coeff (2);

  lib_patches_seeds_.clear ();
  for (int i=0; i<lib_patches_vf_.size(); i++)
  {
    lib_patches_vf_[i] = lib_patches_cf_[i];
    lib_patches_vf_[i].updatePose (kinfu->getCameraPose().cast<double>());

    // flip patches 
    lib_patches_vf_[i].flipPatchFrameTowardsViewpoint (vp_x_, vp_y_, vp_z_);

    vector <int> seed_ind;
    vector <float> seed_sqr_dist;
    Vector3d seed = lib_patches_vf_[i].getC();

    seed_search_->setInputCloud (cloud_ptr);
    seed_search_->radiusSearch (PointXYZ(seed(0), seed(1), seed(2)),
                                radius_/seed_r_dec_, seed_ind, seed_sqr_dist);

    //TBD: better sorting
    lib_patches_seeds_.insert (lib_patches_seeds_.end(), seed_ind.begin(),
                               seed_ind.end());
  }

  lib_patch_loaded_ = true;

  // add new seeds to the map patch list
  patch_match_for_del_.clear ();
  for (int i=0; i<lib_patches_seeds_.size(); i++)
  {
    MapPatch map_patch;
    map_patch.setSeed (lib_patches_seeds_[i]);
    map_patch.setIsSeedValid (true);
    Vector3f seed;
    seed(0) = cloud_ptr->points[lib_patches_seeds_[i]].x;
    seed(1) = cloud_ptr->points[lib_patches_seeds_[i]].y;
    seed(2) = cloud_ptr->points[lib_patches_seeds_[i]].z;
    map_patch.setCell (getMapCellId (seed));
    map_patches_.push_back (map_patch);

    // mark for deletion after the end of the frame
    patch_match_for_del_.push_back (map_patches_.size()-1);
  }

}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::execBubbleRaycast (const Affine3f &bubble_pose)
{
  // call superclass implementation
  KINFU_APP::execBubbleRaycast (bubble_pose);

  // proceed only if need to do a patch matching
  if (!do_patch_match_) { return; }

  // match patches
  int map_patch_match_id = -1;
  for (int i=0; i<map_patches_.size(); i++)
  {
    if (!map_patches_[i].getIsValid ())
      continue;

    Patch patch = map_patches_[i].getP ();
    patch.flipPatchFrameTowardsViewpoint (vp_x_, vp_y_, vp_z_);

    for (int j=0; j<lib_patches_vf_.size(); j++)
    {
      bool is = lib_patches_vf_[j].isSimilar (patch, 0.015, 5.0, 20.0, 0.01);

      if (is && !patch_match_found_)
      {
        //cout << "Similar patch: " << lib_patches_str_[j] << endl;
        patch_match_found_ = true;
        patch_match_id_ = j;
        map_patch_match_id = i;
        break;
      }
    }
  }

  // mark patches for deletion
  for (int i=0; i< patch_match_for_del_.size(); i++)
  {
    int j = patch_match_for_del_[i];
    
    // proceed only if the patch is valid
    if (!map_patches_[j].getIsValid())
      continue;

    // make patch invalid and decrease the counter in the corresponding cell
    map_patches_[j].makeInvalid();
    map_cells_[map_patches_[j].getCell()].decCurPatchesNum ();

    // keep only the map patch that was matched
    if (patch_match_found_ && patch_match_for_del_[i]==map_patch_match_id)
    {
      //cout << patch_match_for_del_[i] << " added back" << endl;
      map_patches_[patch_match_for_del_[i]].makeValid();
      map_cells_[map_patches_[patch_match_for_del_[i]].getCell()].incCurPatchesNum ();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::execViz (bool has_data, bool has_image, const Affine3f &bubble_pose)
{
  // call superclass implementation
  KINFU_APP::execViz (has_data, has_image, bubble_pose);

  // patch map visualizer
  cloud_viewer_ptr_ = scene_cloud_view->getVisualizer ();

  // remove and add the loaded patch
  if (lib_patch_loaded_)
  {
    for (int i=0; i<lib_patches_vf_.size(); i++)
    {
      patch_plot_->setP (lib_patches_vf_[i]);
      patch_plot_->showPatch (cloud_viewer_ptr_, 0.0f, 1.0f, 0.0f, "lib_" + lib_patches_str_[i]);
    }
  }

  // remove old patch
  patch_plot_->removePatch (cloud_viewer_ptr_, "patch");
  patch_plot_->removeFrame (cloud_viewer_ptr_, "frame");
  
  // visualize the patch only if it is paused
  //if (!paused) { return; }

  // visualize picked point
  /*
  if (vis_pt_picked_)
  {
    cloud_viewer_ptr_->removeShape ("point");
    cloud_viewer_ptr_->addSphere (PointXYZ(pt_x_, pt_y_, pt_z_),0.005,1,0,0,"point");
    vis_pt_picked_ = false;
  }
  */

  // visualize the fitted patch
  /*
  if (draw_patch_)
  {
    patch_plot_->setP (patch);
    Affine3f aff = (patch.getPose()).cast<float>();
    patch_plot_->showFrame (cloud_viewer_ptr_, "frame", aff);
    patch_plot_->showPatch (cloud_viewer_ptr_, 0.0f, 0.0f, 1.0f, "patch");
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuAppMatch::loadPatchLib (const std::string &filename)
{
  std::fstream fs (filename.c_str());

  if (!fs.is_open()) throw std::runtime_error ("could not open " + filename);

  std::string patch_name, seq_name;
  while (fs >> patch_name >> seq_name)
  {
    // save filename in vectors
    lib_patches_str_.push_back (patch_name);
    lib_seqs_str_.push_back (seq_name);

    // load patches
    Patch patch;
    patch.loadPatch (std::string (DEF_PATCH_LIB) + patch_name);
    patch.setAutoSS ();
    patch.infer_params ();
    patch.gs();
    lib_patches_cf_.push_back (patch);
    lib_patches_vf_.push_back (patch);
    
    //cout << "patch: " << patch.getR().transpose() << " -- " << patch.getC ().transpose() << endl;
  }
}
