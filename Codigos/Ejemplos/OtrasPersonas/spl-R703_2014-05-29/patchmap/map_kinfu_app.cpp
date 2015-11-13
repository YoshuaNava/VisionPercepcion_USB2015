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

// PATCHMAP headers
#include "map_kinfu_app.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace Eigen;
using namespace rxkinfu;

////////////////////////////////////////////////////////////////////////////////
MapKinfuApp::MapKinfuApp (int argc, char **argv) :
  KinfuApp (argc, argv),
  nn_cloud_ (new PointCloud<PointXYZ>),
  filtered_cloud_ (new PointCloud<PointXYZ>)
{
  firstRun_ = true;
  firstVisRun_ = true;
  
  patch_plot = new PatchPlot (); 
  sal_ = new SampleSaliency ();

  // time setup
  setupTime ();

  // counters setup
  setupCounters ();

  // draw setup
  setupDraw (argc, argv);

  // sample saliency setup
  setupSaliency (argc, argv);

  // map setup
  setupMap (argc, argv);

  // patch fitting validation setup
  setupValidation (argc, argv);

  // path planning setup
  setupPatchSelection (argc, argv);
}

////////////////////////////////////////////////////////////////////////////////
MapKinfuApp::~MapKinfuApp ()
{
  delete (patch_plot);
  delete (sal_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupTime ()
{
  // defaults
  vis_patches_ms_ = find_seeds_ms_ = map_ms_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupCounters ()
{
  // defaults
  start_cycle_counter_ = 0;
  setMapFittedPatches (0);
  total_fitted_patches_ = frame_fitted_patches_ = total_removed_patches_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupDraw (int argc, char **argv)
{
  // defaults
  draw_grid_cells_ = true;
  draw_sal_points_ = false;
  draw_fitted_patches_ = false;
  draw_fitted_patches_level_ = 1;
  draw_patch_normals_ = false;
  draw_selected_patches_ = false;
  draw_proj_cam_ = draw_grid_cells_info_ = false;
  setDrawGridCellsInfoType (1);
  map_patches_to_remove_.clear();

  vis_num_normals_ = 0;
  
  // cmd line args
  draw_grid_cells_ = find_switch (argc, argv, "-mapdrawcells");
  draw_sal_points_ = find_switch (argc, argv, "-mapdrawsal");
  draw_fitted_patches_ = find_switch (argc, argv, "-mapdrawpatches");
  draw_patch_normals_ = find_switch (argc, argv, "-mapdrawpn");
  draw_selected_patches_ = find_switch (argc, argv, "-mapdrawsp");
  draw_proj_cam_ = find_switch (argc, argv, "-mapdrawprojcam");
  draw_grid_cells_info_= find_switch (argc, argv, "-mapdrawgci");
  parse_argument(argc, argv, "-mapdrawgcit", draw_grid_cells_info_type);
  parse_argument(argc, argv, "-mapdrawpl", draw_fitted_patches_level_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupSaliency (int argc, char **argv)
{
  // defaults
  use_sal_ = sal_DoN_ = sal_DoNG_ = sal_DtFP_ = sal_DtFP_props_ = false;
  sal_max_DoN_ = sal_max_DoNG_ = sal_max_DtFP_ = -1;

  // cmd line args
  use_sal_ = find_switch (argc, argv, "-usesal");
  sal_DoN_ = find_switch (argc, argv, "-saldon");
  sal_DoNG_ = find_switch (argc, argv, "-saldong");
  sal_DtFP_ = find_switch (argc, argv, "-saldtfp");
  if (parse_3x_arguments(argc, argv, "-salfpp", sal_DtFP_fdg_, sal_DtFP_ftsd_,
                         sal_DtFP_fdfb_) > -1)
  { sal_DtFP_props_ = true;}
  parse_argument(argc, argv, "-salmdon", sal_max_DoN_);
  parse_argument(argc, argv, "-salmdong", sal_max_DoNG_);
  parse_argument(argc, argv, "-salmdtfp", sal_max_DtFP_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupMap (int argc, char **argv)
{
  //defaults
  setPrintStatus (false);
  start_cycle_ = 0;
  setMaxPatchMapTime (0.03f);
  setMaxPatches (-1);
  setMaxPatchesPerCell (1);
  setCellDecFactor (2);
  setMapCellsTries (5);
  setRadius (0.1);
  
  // cmd line args
  print_status = find_switch (argc, argv, "-mapps");
  parse_argument(argc, argv, "-mapsc", start_cycle_);
  parse_argument(argc, argv, "-mapmpmt", max_patch_map_time_);
  parse_argument(argc, argv, "-mapmp", max_patches_);
  parse_argument(argc, argv, "-mapmppc", max_patches_per_cell_);
  parse_argument(argc, argv, "-mapcdf", cell_dec_factor_);
  parse_argument(argc, argv, "-mapmct", map_cells_tries_);
  parse_argument(argc, argv, "-mapr", radius_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupValidation (int argc, char **argv)
{
  do_validation_ = true;
  residual_thres_ = 0.01;
  cov_thres_ = 0.75;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setupPatchSelection (int argc, char **argv)
{
  // defaults
  sel_p_ = false;
  sel_pt_[0] = sel_pt_[1] = sel_pt_ [2] = 0.0;
  sel_pt_r_ = 0.0;
  sel_min_curv_ = sel_max_curv_ = 0.0;
  sel_min_slope_ = sel_max_slope_ = 0.0;
  sel_patch_index_ = -1;

  // cmd line args
  double x,y,z;
  if (parse_3x_arguments(argc, argv, "-selpt", x, y, z) > -1)
  {
    sel_pt_[0] = x;
    sel_pt_[1] = y;
    sel_pt_ [2] = z;
  }
  sel_p_ = find_switch (argc, argv, "-selp");
  parse_argument(argc, argv, "-selptr", sel_pt_r_);
  parse_argument(argc, argv, "-selmincurv", sel_min_curv_);
  parse_argument(argc, argv, "-selmaxcurv", sel_max_curv_);
  parse_argument(argc, argv, "-selminslope", sel_min_slope_);
  parse_argument(argc, argv, "-selmaxslope", sel_max_slope_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::usageHelp (const char *pfx, bool with_inputs)
{
  // call superclass implementation
  KinfuApp::usageHelp (pfx, with_inputs);
  
  cout << "\nPatch Mapping and Tracking Visualization Options\n";
  cout << pfx << "  -mapdrawcells: draw the grid cells\n"
       << pfx << "  -mapdrawsal: draw the salient points\n"
       << pfx << "  -mapdrawpatches: draw the fitted patches\n"
       << pfx << "  -mapdrawpn: draw the fitted patches normals\n"
       << pfx << "  -mapdrawpl: the level for the fitted patches visualization\n"
       << pfx << "  -mapdrawprojcam: draw the projected camera on the lower TSDF plane\n"
       << pfx << "  -mapdrawgci: draw grid cells info on the lower TSDF plane\n"
       << pfx << "  -mapdrawgcit: draw grid cells info type {1,2,3}\n";
 

  cout << "\nSample Saliency Options\n";
  cout << pfx << "  -usesal: whether to use saliency\n"
       << pfx << "  -saldon: Difference of Normals saliency.\n"
       << pfx << "  -saldong: Difference of Normals-Gravity saliency.\n"
       << pfx << "  -saldtfp: Distance to Fixation Point saliency.\n"
       << pfx << "  -salfpp <f1,f2,f3>: Fix Point properties.\n"
       << pfx << "  -salmdon: max DoN cos angle theshold. \n"
       << pfx << "  -salmdong: max DoNG cos angle threshold. \n"
       << pfx << "  -salmdtfp: max dist to the Fix Point. \n";

  
  cout << "\nPatch Mapping and Tracking Options\n";
  cout << pfx << "  -mapps: print status\n"
       << pfx << "  -mapsc: the starting cycle\n"
       << pfx << "  -mapmpmt: max time (in ms) for the patch mapping\n"
       << pfx << "  -mapmp: max number of patches in the map\n"
       << pfx << "  -mapmppc: max number of patches per cell in the map\n"
       << pfx << "  -mapcdf: cell decimating factor\n"
       << pfx << "  -mapmct: the number of tries to find a valid seed in a cell\n";
 

  cout << "\nPatch Selection Options\n";
  cout << pfx << "  -selp: whether to select patches.\n"
       << pfx << "  -selpt <x,y,z>: set patch selection point.\n"
       << pfx << "  -selptr: set patch selection radius.\n"
       << pfx << "  -selmincurv: set selected patch min curvature.\n"
       << pfx << "  -selmaxcurv: set selected patch max curvature.\n"
       << pfx << "  -selminslope: set selected patch min slope (deg).\n"
       << pfx << "  -selmaxslope: set selected patch max slope (deg).\n";
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setPrintStatus (bool print_status_arg)
{
  this->print_status = print_status_arg;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setDrawGridCellsInfoType (int type)
{
  this->draw_grid_cells_info_type = type;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setMaxPatchMapTime (float time)
{
  this->max_patch_map_time_ = time;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setMaxPatches (int max_patches)
{
  this->max_patches_ = max_patches;
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::getMaxPatches ()
{
  return (this->max_patches_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setMaxPatchesPerCell (int max_patches_per_cell)
{
  this->max_patches_per_cell_ = max_patches_per_cell;
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::getMaxPatchesPerCell ()
{
  return (this->max_patches_per_cell_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setMapFittedPatches (int map_fitted_patches)
{
  this->map_fitted_patches_ = map_fitted_patches;
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::getMapFittedPatches ()
{
  return (this->map_fitted_patches_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setCellDecFactor (int cell_dec_factor)
{
  this->cell_dec_factor_ = cell_dec_factor;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setMapCellsTries (int map_cells_tries)
{
  this->map_cells_tries_ = map_cells_tries;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::setRadius (double radius)
{
  this->radius_ = radius;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::initMapCells ()
{
  // cloud's dimensions in meters; in the plane parallel to the ground
  Vector3f tsdf_size = kinfu->volume().getSize();
  float w = tsdf_size [0]; //along x-axis
  float h = tsdf_size [2]; //along z-axis
 
  // grid's dimensions
  map_cells_w_ = (int)ceil(w/cell_dec_factor_);
  map_cells_h_ = (int)ceil(h/cell_dec_factor_);
  
  map_cells_.resize (map_cells_w_*map_cells_h_);

  // for each map cell
  for (int c=0; c<map_cells_w_; c++)
  {
    for (int r=0; r<map_cells_h_; r++)
    {
      // row-wise grid cells
      int id = (r*map_cells_w_) + c;

      // set the MapCell object
      map_cells_[id].setId (id);
      map_cells_[id].setW (w);
      map_cells_[id].setH (h);
      map_cells_[id].setMinCol (c*cell_dec_factor_);
      map_cells_[id].setMaxCol (min ((float)(c+1)*cell_dec_factor_, w));
      map_cells_[id].setMinRow (r*cell_dec_factor_);
      map_cells_[id].setMaxRow (min ((float)(r+1)*cell_dec_factor_, h));
      map_cells_[id].setMaxPatchesNum (max_patches_per_cell_);
      map_cells_[id].setCurPatchesNum (0);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::updateMapCells ()
{
  // Step 1: project the camera reference point to the xz-plane on the ground
  // Note that the projection is on the lower xz plane of the TSDF
  Affine3f cam_pose = kinfu->getCameraPose();
  Vector3f cam_pose_vec = cam_pose.translation();
  proj_cam_ << cam_pose_vec(0), cam_pose_vec(2);

  // Step 2: set the weight to the distance between the projected camera point
  // the center of the cell.
  
  // TBD: If the cells are behind the line (TBD angle) that is perdedicular the
  // projected forward vector consider them as lower importance ones.  Create 
  // two cell vectors, one for more and one for less important ones.  Sort them
  // independantly and then append them to the map_cells_.
  
  Vector2f cell_center;
  for (int i=0; i<map_cells_.size(); i++)
  {
    cell_center << map_cells_[i].getCenterCol(), map_cells_[i].getCenterRow();
    map_cells_[i].setWeight ((cell_center-proj_cam_).squaredNorm());
  }

  // Step 3: sort the map cells wrt their weight
  map_cells_copy_ = map_cells_; //TBD: waste of space in the heap
  std::sort(map_cells_copy_.begin(), map_cells_copy_.end());
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::resetMapCells ()
{
  for (int i=0; i<map_cells_.size(); i++)
  {
    map_cells_[i].setCurPatchesNum (0);
    map_cells_[i].resetPointIds ();
  }
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::getMapCellId (Vector3f c)
{
  float x = c[0]; //width
  float z = c[2]; //height
  
  return (floor(z/cell_dec_factor_)*map_cells_w_ + floor(x/cell_dec_factor_));
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::initMapPatches ()
{
  if (getMaxPatches()>0)
  {
    // if there is an upper map patches limit bound, reserve that much space
    map_patches_.reserve (getMaxPatches());
  }
  else
  {
    // TBD: reserves memory for all possible patches; because of TSDF volume 
    // transformations map patches may be more than the reserved space
    map_patches_.reserve (max_patches_per_cell_ * map_cells_w_ * map_cells_h_);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::cleanMapPatches ()
{
  // for each patch
  for (int i=0; i<map_patches_.size();)
  {
    if (map_patches_[i].getIsInvalid()) //remove the invalid patch
    {
      // update the corresponding cell
      //int cell_id = map_patches_[i].getCell();
      //map_cells_[cell_id].decCurPatchesNum ();

      // pop back the invalid patch
      map_patches_[i] = map_patches_.back();
      map_patches_.pop_back ();

      // decrease the global counter
      map_fitted_patches_--;
    }
    else if (map_patches_[i].isReadyToFit())
    {
      // pop back the invalid patch but not decrease since they were not
      // increased in the map cell
      map_patches_[i] = map_patches_.back();
      map_patches_.pop_back ();
    }
    else
    {
      i++;
    }
  }
 
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::resetMapPatches ()
{
  for (int i=0; i<map_patches_.size(); i++)
  {
    // add it to the list for graphics removal
    if (draw_fitted_patches_)
      map_patches_to_remove_.push_back (map_patches_[i].getId());
    
    total_removed_patches_++;
  }

  map_patches_.clear ();

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::updatePatchPose ()
{
  //TBD: change it with Affine transforms

  // Get the relative TSDF volume pose wrt its previous one.  The pose will be 
  // different than the identity matrix only if the volume moves.  Note that to
  // get the virtual bubble camera pose we can run: getBubblePose() and the
  // real camera pose: kinfu->getCameraPose().
  int np = kinfu->getNumberOfPoses();
  Affine3f cam_pose = kinfu->getRelativeVolumePose (np-1);
  Matrix4d pos (cam_pose.matrix().cast<double>());

  // update the patches and the cell grids only if the TSDF volume moved
  if (pos.cwiseEqual(MatrixXd::Identity(4,4)).count() == 16)
    return;

  // for each patch in the map
  Matrix4d patch_pose;
  
  for (int i=0; i<map_patches_.size(); i++)
  {
    // proceed only if the patch is valid
    if (!map_patches_[i].getIsValid())
      continue;

    // get patch's pose in previous frame
    patch_pose = MatrixXd::Identity(4, 4);
    patch_pose.block(0,0,3,3) = (map_patches_[i].getP()).getRot ();
    patch_pose.block(0,3,3,1) = (map_patches_[i].getP()).getC ();
    patch_pose = pos.inverse()*patch_pose;

    // set new patch's pose in the current frame
    map_patches_[i].setPatchR (patch_pose.block(0,0,3,3));
    map_patches_[i].setPatchC (patch_pose.block(0,3,3,0));

    // if patch (pick point +- radius) is out of the TSDF volume make it invalid
    // NOTE: it is needed to be removed from the graphics too
    Eigen::Vector3f tsdf_size = kinfu->volume().getSize();
    Vector3d c = (map_patches_[i].getP ()).getC();
    double radius = ((map_patches_[i].getP()).getD()).maxCoeff();
    int cell_id = map_patches_[i].getCell();

    if ((c[0]-radius) < 0.0 ||
        (c[1]-radius) < 0.0 ||
        (c[2]-radius) < 0.0 ||
        (c[0]+radius) > tsdf_size[0] ||
        (c[1]+radius) > tsdf_size[1] ||
        (c[2]+radius) > tsdf_size[2])
    {
      // consider it for graphics removal
      if (draw_fitted_patches_)
        map_patches_to_remove_.push_back (map_patches_[i].getId());

      // remove the patch
      map_patches_[i].makeInvalid();

      //TBD: here or in cleanMapPatches?
      int cell_id = map_patches_[i].getCell();
      map_cells_[cell_id].decCurPatchesNum ();
      
      total_removed_patches_++;

      // move to next patch in the map
      continue;
    }
    
    // find the new patch's cell id by back-projecting the patch's pick point
    int new_cell_id = getMapCellId (c.cast<float>());
    
    if (new_cell_id != cell_id)
    {
      // update patch info
      map_patches_[i].setCell (new_cell_id);

      // update patch's cell id
      //TBD: problem with counter in the cell (need to verify correctness)
      map_cells_[cell_id].decCurPatchesNum ();
      map_cells_[new_cell_id].incCurPatchesNum ();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::matchPatchesCells (PointCloud<PointXYZ>::Ptr &cloud_ptr)
{
  // for each cell grid
  for (int i=0; i<map_cells_.size (); i++)
  {
    map_cells_[i].resetPointIds ();
  }

  Vector3f point;
  for (int i=0; i<cloud_ptr->points.size(); i++)
  {
    // Working around the bug that points appear to be outside the TSDF volume
    // after the raycasting
    Vector3f tsdf_size = kinfu->volume().getSize();
    if (cloud_ptr->points[i].x < 0 ||
        cloud_ptr->points[i].y < 0 ||
        cloud_ptr->points[i].z < 0 ||
        cloud_ptr->points[i].x > tsdf_size[0] ||
        cloud_ptr->points[i].y > tsdf_size[1] ||
        cloud_ptr->points[i].z > tsdf_size[2])
      continue;

    if (!isFinite (cloud_ptr->points[i]))  
      continue;

    point = cloud_ptr->points[i].getVector3fMap();
    int cell_id = getMapCellId (point);
    
    // Working around the same bug as above since irelevant points may exist in
    // the TSDF itself.
    if (cell_id<0 || cell_id>map_cells_.size())
      continue;

    map_cells_[cell_id].addPointId (i);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::findSeeds (PointCloud<PointXYZ>::Ptr &cloud_ptr)
{
  int seed, number_of_tries;

  // for each cell
  for (int j=0; j<map_cells_copy_.size (); j++)
  {
    int i = map_cells_copy_[j].getId();

    // proceed only if we haven't reached the max number of cell's patches
    int cellCurPNum = map_cells_[i].getCurPatchesNum ();
    int cellMaxPNum = map_cells_[i].getMaxPatchesNum ();

    if (cellCurPNum >= cellMaxPNum)
      continue;

    // until reaching the cellMaxPNum number of patches in each cell
    for (int j=cellCurPNum; j<cellMaxPNum; j++)
    {
      number_of_tries = 0; //number of tries for finding new seed point in the cell
      do // until finding a valid point
      {
        // get a random seed point
        seed = map_cells_[i].getRandomSeed ();

        // did you reach max number of tries?
        number_of_tries++;
        if (map_cells_tries_ < number_of_tries)
          break;
      } while (!pcl_isfinite(cloud_ptr->points[seed].x) ||
               !pcl_isfinite(cloud_ptr->points[seed].y) ||
               !pcl_isfinite(cloud_ptr->points[seed].z) ||
               (seed<0));

      // valid seed point
      if ((map_cells_tries_>=number_of_tries) && (seed>=0))
      {
        // add the new seed/patch
        MapPatch map_patch;
        map_patch.setSeed (seed);
        map_patch.setIsSeedValid (true);
        map_patch.setCell (i);
        map_patches_.push_back (map_patch);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::findSalientCloud (PointInCloudPtr &in_cloud_ptr,
                               PointInCloudPtr &out_cloud_ptr)
{
  sal_->setInputCloud (in_cloud_ptr);
  sal_->setCamPose ((kinfu->getCameraPose()).translation());
  sal_->setNNSize (2.0f*radius_, bubble_fx[0]);
 
  sal_->setUseDoN (sal_DoN_);
  sal_->setUseDoNG (sal_DoNG_);
  sal_->setUseDtFP (sal_DtFP_);
  if (sal_DtFP_props_)
    sal_->setFPProps (sal_DtFP_fdg_, sal_DtFP_ftsd_, sal_DtFP_fdfb_);
  if (sal_max_DoN_>=0.0) {sal_->setNNCAThres (sal_max_DoN_);}
  if (sal_max_DoNG_>=0.0) {sal_->setNGCAThres (sal_max_DoNG_);}
  if (sal_max_DtFP_>=0.0) {sal_->setMaxDistFromFixation (sal_max_DtFP_);}
  if (sal_max_DoNG_ || sal_max_DtFP_) {sal_->setG (down);}

  sal_->extractSalientPoints ();

  // extract the indices
  ExtractIndices<PointXYZ> eifilter;
  eifilter.setInputCloud (in_cloud_ptr);
  eifilter.setIndices ((sal_->getIndxSalient()));
  eifilter.setKeepOrganized (true);
  eifilter.filter (*out_cloud_ptr);
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::selectPatch ()
{
  // camera pose wrt to TSDF volume
  Affine3f camera_to_volume = kinfu->getCameraPose();
  Vector3d pt = camera_to_volume.cast<double>() * sel_pt_;

  // patch select
  PatchSelect ps;
  ps.setInputMapPatches (&map_patches_);
  ps.setPt (pt, sel_pt_r_);
  ps.setCurv (sel_min_curv_, sel_max_curv_);
  ps.setVp (vp_x_, vp_y_, vp_z_);
  ps.setSlope (-down, deg2rad(sel_min_slope_), deg2rad(sel_max_slope_));
  int p_ind = ps.filter();
  sel_patches_ = ps.getOutputMapPatches();
  
  return (p_ind);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::resetSelPatches ()
{
  for (int i=0; i< sel_patches_.size(); i++)
    sel_patches_to_remove_.push_back (sel_patches_[i]);
  sel_patches_.clear ();
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::execBubbleRaycast (const Eigen::Affine3f &bubble_pose)
{
  // call superclass implementation
  KinfuApp::execBubbleRaycast (bubble_pose);

  // patch map timing
  map_ms_ = timer_.getTime ();

  // Step 0-a: checked whether the raycast is paused
  if (paused) {return;}

  // Step 0-b: if necessary wait for some cycles before you begin (helps with
  // filling the TSDF volume with points)
  if (start_cycle_ > start_cycle_counter_)
  {
    start_cycle_counter_++;
    return;
  }
  
  // Step 0-c: if it's the first run, i) create the grid map cells and ii) init
  // the map patches
  if (firstRun_)
  {
    initMapCells ();
    initMapPatches ();
    firstRun_ = false;
  }

  // Step 0-d: if kinfu resets, reset the map cells and the map patches
  if (kinfu->getFirstFrameAfterReset())
  {
    // reset map cells
    resetMapCells ();

    // reset the map patches
    resetMapPatches ();

    // reset the path planning vars
    resetSelPatches ();

    // reset the counters
    setupCounters ();

    // NOTE: we loose the first frame (deleting patches from visualizing)
    return;
  }

  // Step 1-a: update the map cells, i.e. order them wrt distance to camera
  updateMapCells ();

  // Step 1-b: update the patch pose, mark for deletion the invalid ones, and
  // update the corresponding cells
  updatePatchPose ();
  
  // Step 1-c: clean the list of patches from invalid ones (fitting failed or
  // not happened). NOTE: this is done in each frame and it could be potentially 
  // good to do it only when I have some patches deleted
  cleanMapPatches ();
  
  // Step 2-a: find the salient points and use the filtered cloud for seed points
  if (use_sal_)
    findSalientCloud (bubble_cloud_ptr[0], filtered_cloud_);
  else
    filtered_cloud_ = bubble_cloud_ptr[0];

  // set the sensor viewpoint
  vp_x_ = filtered_cloud_->sensor_origin_.coeff (0);
  vp_y_ = filtered_cloud_->sensor_origin_.coeff (1);
  vp_z_ = filtered_cloud_->sensor_origin_.coeff (2);
  
  // Step 2-b: go through points and match them with the right cell id
  matchPatchesCells (filtered_cloud_);

  // Step 3: select new seed points
  find_seeds_ms_ = timer_.getTime ();
  findSeeds (filtered_cloud_);
  find_seeds_ms_ = timer_.getTime () - find_seeds_ms_;
  
  // Step 4: fetch and validate neighborhood
  search_.setInputCloud (bubble_cloud_ptr[0]);

  // patches fitted during this frame
  frame_fitted_patches_ = 0;

  // Step 5: fit patches
  for (int i=0; i<map_patches_.size (); i++)
  {
    // stop if you have reached time limits
    if ((timer_.getTime()-map_ms_) > max_patch_map_time_)
      break;

    // check total number of patches
    if ((max_patches_>0) && (map_fitted_patches_>=max_patches_))
      break;

    // if patch is valid
    if (!map_patches_[i].getIsSeedValid () ||
        map_patches_[i].getIsPatchValid ())
      continue;

    // find neighborhood
    searchNeighbors (bubble_cloud_ptr[0], map_patches_[i].getSeed ());

    if (nn_indices.size()<10) //TBD: fix fitting patch code
      continue;

    nn_cloud_->points.clear();
    for (int j=0; j<nn_indices.size(); j++)
      nn_cloud_->points.push_back (bubble_cloud_ptr[0]->points[nn_indices[j]]);

    // fit a patch to the neighborhood
    patchFit = new PatchFit (nn_cloud_);
    patchFit->setSSmax (50); // TBD: hard-coded number
    patchFit->fit();

    Patch patch;
    patch = *patchFit->p_;

    delete (patchFit);
    
    // infer some params
    patch.setAutoSS ();
    patch.infer_params();
    patch.gs();

    // Step 5: validate the fitting (residual and coverage)
    if (do_validation_)
    {
      // residual
      patch.computeResidual (nn_cloud_);
      if (patch.getResidual() > residual_thres_)
        continue;

      //TBD: coverage
      PatchCoverage pc (patch, nn_cloud_, true);
      pc.setNc (50);
      double cov_val = pc.findCoverage();

      if (cov_val < cov_thres_)
        continue;
    }

    //TBD; id is constantly increasing
    //patch.setID (i);
    patch.setID (total_fitted_patches_);
    
    // Add the fitted patch to the map
    map_patches_[i].setId (patch.getID());
    map_patches_[i].setP (patch);
    map_patches_[i].makeValid();

    // update cell info
    map_cells_[map_patches_[i].getCell()].incCurPatchesNum ();
    
    // set the counters
    total_fitted_patches_++; // total fitted patches from the beginning
    map_fitted_patches_++; // total fitted patches in the map
    frame_fitted_patches_++; // fitted patches in this frame
  }

  // patch selection
  if (sel_p_)
    if (sel_patch_index_ < 0) { sel_patch_index_ = selectPatch (); }

  map_ms_ = timer_.getTime () - map_ms_;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::execExt ()
{
  // call superclass implementation
  KinfuApp::execExt ();
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::execViz(bool has_data, bool has_image,
                     const Eigen::Affine3f &bubble_pose)
{
  // call superclass implementation
  KinfuApp::execViz (has_data, has_image, bubble_pose);

  // wait until it is the right cycle to start,runs once; note that firstRun_
  // changes to FALSE when the execBubbleRaycast fully runs once
  if (firstRun_) {return;}
 
  // patch map visualizer
  cloud_viewer_ptr_ = scene_cloud_view->getVisualizer ();

  // visualize the salient points
  if (draw_sal_points_) {visSaliency ();}
 
  // draw the fitted patches
  vis_patches_ms_ = timer_.getTime ();
  if (draw_fitted_patches_) {visPatches (draw_fitted_patches_level_);}
  vis_patches_ms_ = timer_.getTime () - vis_patches_ms_;

  // draw patches' normals
  if (draw_patch_normals_) {visPatchNormals ();}

  // draw the projected camera
  if (draw_proj_cam_) {visProjCam ();}
  
  // draw the grid cells on the lower TSDF plane (they don't change over time)
  if (draw_grid_cells_ && firstVisRun_) {visGridCells ();}

  // draw the grid cells info on the lower TSDF plane
  if (draw_grid_cells_info_) {visGridCellsInfo (draw_grid_cells_info_type);}

  // visualizer run at least once
  if (firstVisRun_) {firstVisRun_ = false;}

  // visualize next patch to step
  if (sel_p_ && draw_selected_patches_) {visSelPatch ();}

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visSaliency ()
{
  sal_->setViewer (cloud_viewer_ptr_);
  if (sal_DtFP_)
  {
    sal_->showFixation ();
    sal_->showDtFPCloud ();
  }
  sal_->showSalPoints (true);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visPatches (int level)
{
  // remove invalid patches
  for (int i=0; i<map_patches_to_remove_.size(); i++)
  {
    int id = map_patches_to_remove_[i];
    patch_plot->removePatch (cloud_viewer_ptr_,
        boost::lexical_cast<string>(id) + "_patch" );
    //total_removed_patches_ ++;
  }
  map_patches_to_remove_.clear ();
  
  // display every level'th patch
  for (int i=0; i<map_patches_.size(); i+=level)
  {
    if (!map_patches_[i].getIsValid ())
      continue;

    patch_plot->setP (map_patches_[i].getP ());
    patch_plot->showPatch (cloud_viewer_ptr_, 0.0f, 1.0f, 0.0f,
        boost::to_string((map_patches_[i].getP()).getID ()) + "_patch");
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visPatchNormals ()
{
  // remove all normals
  if (vis_num_normals_>0)
  {
    for (int i=0; i<vis_num_normals_; i++)
    {
       cloud_viewer_ptr_->removeShape (boost::lexical_cast<string>(i) + "normal");
    }
  }

  // add the new ones
  Patch p;
  vis_num_normals_ = 0;
  float scale = 0.01;
  
  for (int i=0; i<map_patches_.size(); i++)
  {
    if (!map_patches_[i].getIsValid())
      continue;

    p = map_patches_[i].getP();
    Vector3f nb = p.getC().cast<float>();
    Vector3f n = ((p.getRot()).col(2)).cast<float>();
    flipNormalTowardsViewpoint (PointXYZ(nb.x(),nb.y(),nb.z()),
                                vp_x_, vp_y_, vp_z_, n);
    Vector3f ne = nb + scale*n;
   

    cloud_viewer_ptr_->addLine(PointXYZ(nb.x(),nb.y(),nb.z()),
                               PointXYZ(ne.x(),ne.y(),ne.z()),
                               0, 0, 1,
                               boost::lexical_cast<string>(vis_num_normals_) +
                               "normal");
    vis_num_normals_++;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visProjCam ()
{
  // get TSDF size
  Vector3f tsdf_size = kinfu->volume().getSize();
  
  // projected camera position
  PointXYZ cam_pt;
  cam_pt.x = proj_cam_[0];
  cam_pt.y = tsdf_size[1];
  cam_pt.z = proj_cam_[1];
 
  // remove old projected camera
  cloud_viewer_ptr_->removeShape ("proj_cam");

  // display the new projected camera
  cloud_viewer_ptr_->addSphere(cam_pt, 0.2, "proj_cam");
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visGridCells ()
{
  PointXYZ start_pt, end_pt;
  Vector3f tsdf_size = kinfu->volume().getSize();
  start_pt.y = end_pt.y = tsdf_size[1];
  
  // draw lines across the x-axis
  start_pt.z = 0.0f; end_pt.z = cell_dec_factor_*map_cells_h_;
  for (int i=0; i<map_cells_w_-1; i++)
  {
    start_pt.x = end_pt.x = (i+1)*cell_dec_factor_;
    
    cloud_viewer_ptr_->addLine (start_pt, end_pt, 1, 0, 0,
                                boost::lexical_cast<string>(i) + "grid_cell_w");
  }
  
  // draw lines across the z-axis
  start_pt.x = 0.0f; end_pt.x = cell_dec_factor_*map_cells_w_;
  for (int i=0; i<map_cells_h_-1; i++)
  {
    start_pt.z = end_pt.z = (i+1)*cell_dec_factor_;

    cloud_viewer_ptr_->addLine (start_pt, end_pt, 1, 0, 0,
                                boost::lexical_cast<string>(i) + "grid_cell_h");
  }
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visGridCellsInfo (int type)
{
  // get the TSDF size
  Vector3f tsdf_size = kinfu->volume().getSize();
 
  // create the position of the grid cell text
  PointXYZ grid_cell_txt_pose;
  grid_cell_txt_pose.y = tsdf_size[1];
  
  // display the text per grid cell
  std::string type_str;
  for (int i=0; i<map_cells_.size(); i++)
  {
    grid_cell_txt_pose.x = map_cells_[i].getCenterCol();
    grid_cell_txt_pose.z = map_cells_[i].getCenterRow();
  
    // choose the type of text
    if (type == 1)
      type_str = boost::to_string(map_cells_[i].getId());
    else if (type == 2)
      type_str = boost::to_string(map_cells_[i].getWeight());
    else if (type ==3)
    {
      if (map_cells_[i].getCurPatchesNum()==0)
        continue;

      type_str = boost::to_string(map_cells_[i].getCurPatchesNum());
    }

    // remove the old one
    cloud_viewer_ptr_->removeText3D (boost::lexical_cast<string>(i) +
                                     "grid_cell_txt");
    
    // visualize the new one
    cloud_viewer_ptr_->addText3D (type_str, grid_cell_txt_pose, 0.05, 1.0, 0.0,
                                  0.0, boost::lexical_cast<string>(i) +
                                  "grid_cell_txt");
  }
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::visSelPatch ()
{
  // proceed only if there are selected patches to visualize
  if (sel_patches_.empty())
  {
    // remove the patch
    patch_plot->removePatch (cloud_viewer_ptr_, "next_patch");

    sel_patch_index_ = -1;
    return;
  }

  // remove the patch if it exists
  //if (draw_fitted_patches_)
  //  patch_plot->removePatch (cloud_viewer_ptr_, map_patches_[sel_patch_index_].getId());
  
  // add new one
  Patch p = map_patches_[sel_patches_[0]].getP();
  //int patch_id = p.getID();
  patch_plot->setP (p);
  patch_plot->showPatch (cloud_viewer_ptr_, 0.0f, 0.0f, 1.0f, "next_patch");
  cout << "done" << endl;
}

////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::printStatus ()
{
  // call superclass implementation
  int num_new_lines = KinfuApp::printStatus ();

  print_info("\nmap: %d patches (%d/%d total/removed), "
             "%2.0fms seed, %2.fms mapping, %2.fms vis\r",
             frame_fitted_patches_, map_fitted_patches_, total_removed_patches_,
             find_seeds_ms_, map_ms_, vis_patches_ms_);

  return (num_new_lines+1);
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::updateSeeds ()
{
  /*
    Vector3d c_patch;
    c_patch = patch_.getC();

    seed_pt.x = c_patch(0);
    seed_pt.y = c_patch(1);
    seed_pt.z = c_patch(2);
  */
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
MapKinfuApp::searchNeighbors (PointInCloudPtr &cloud_ptr, int seed)
{   
  search_.radiusSearch (cloud_ptr->points[seed], radius_, nn_indices,
                        nn_sqr_distances);
}


////////////////////////////////////////////////////////////////////////////////
int
MapKinfuApp::numberOfValidPoints (PointCloud<PointXYZ>::Ptr cloud_ptr)
{
  int count = 0;
  for (int i=0; i<cloud_ptr->size(); i++)
  {
    if (pcl_isfinite(cloud_ptr->points[i].x) ||
        pcl_isfinite(cloud_ptr->points[i].y) ||
        pcl_isfinite(cloud_ptr->points[i].z))
    {
      count++;
    }
  }

  return (count);
}
