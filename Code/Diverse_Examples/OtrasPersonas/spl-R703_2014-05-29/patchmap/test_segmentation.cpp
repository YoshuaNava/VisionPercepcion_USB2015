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

#include <iostream>
#include <pcl/console/parse.h>
#include <imucam/pick_player.h>
#include "task.h"
#include "autoseg.h"

using namespace std;
using namespace pcl;
using namespace imucam;
using namespace Eigen;

typedef PointCloud<PointXYZ> PointCloudIn;
typedef PointCloudIn::ConstPtr PointCloudInConstPtr;

/** \brief Whether to use manual segmentation, o/w auto. */
bool do_autoseg;

/** \brief Whether to visualize the segmentation results. */
bool do_vis;

/** \brief Viewer configuration. */
bool show_picks;
bool show_camera_axes;
bool show_um6_in_camera, show_um6_orientation, show_um6_gravity;
float cloud_alpha = 0;

/** \brief Current point cloud. */
PointCloudInConstPtr cloud;
    
/** \brief AutoSeg object. */
//AutoSeg *autoseg;
boost::shared_ptr<AutoSeg> autoseg (new AutoSeg);

/** \brief Task object. */
boost::shared_ptr<Task> task (new Task);

/** \brief Extends the PickPlayer class for processing only particular picks in
  * per frame.  To run: test_segmentation FRAME_DIR -picks picks.txt -loop -man
  * 
  */
class MyPlayer : public PickPlayer
{

public:

  MyPlayer()
  {
    //uncomment this to disable the status line printouts
    print_status = false;
  }

  string getAppName() { return std::string("test_segmentation"); }

  protected:

  /** \brief set visualization options
   */
  bool makeViewer(int argc, char **argv) {

    if (!PickPlayer::makeViewer(argc, argv)) return false;

    viewer->setShowCameraAxes(show_camera_axes);
    viewer->setUM6Opts(show_um6_in_camera, show_um6_orientation,
                       show_um6_gravity);
    viewer->setCloudAlpha(cloud_alpha);
     
    return true;
  }

  /** \brief disable pick highlighting unless show_picks is set
   */
  void afterViewUpdate(Viewer *viewer)
  {
    if (show_picks) PickPlayer::afterViewUpdate(viewer);
  }

    /** \brief Assign the Autoseg as a new task for the Viewer.
      *
      * \param[in] indices the picks (ie seed points) index numbers
      * \param[in] frame the frame that is processed
      */
    void
    offerPicks(const vector<int> &indices, Frame::ConstPtr frame)
    {
      // create a new task for using the underlying cloud viewer
      if (do_vis && viewer)
        viewer->addTask(boost::bind<void>(&MyPlayer::runAutoseg, this, frame,
                                          indices, _1),
                        false, //iff_needed = false, i.e. always schedule
                        true); //after_first_frame
      else runAutoseg(frame, indices, 0); //no viewer
    }

    /** \brief For a specific pick in a frame extracts the gravity vector, the
      * picks (i.e. seed point) for the particular frame, it assigns a task and
      * runs the manual segmentation.
      *
      * \param[in] frame the frame that is processed
      * \param[in] indices the picks (ie seed points) index numbers
      * \param[in] viewer the underlying Viewer
      */
    void
    runAutoseg(Frame::ConstPtr frame, const vector<int> &indices,
               Viewer *viewer)
    {
      //cerr << "runAutoseg(): thread " << boost::this_thread::get_id() << endl;

      //cerr << "runAutoseg() frame->time="
      //     << boost::posix_time::to_iso_string(frame->time) << endl;

      //comment this to disable the default behaviour of printing out the picks
      dumpPicks(indices, frame);

      // extract point cloud
      cloud = frame->fillCloud<PointXYZ>();
   
      // create a task
      task->setMaxPatchDistance (2.0); //2m
      task->setMaxPatchSize (0.10f); //30cm radius

      // create a new autoseg for the specific cloud and task
      //autoseg = new AutoSeg(cloud, task);
      autoseg->setInputCloud (cloud);
      autoseg->setTask (task);
      
      // extract and set the gravity vector
      Vector3f g;
      g = frame->getCameraToWorld()->transpose() * Vector3f::UnitZ();
      autoseg->setG (g);
     
      // need to do this here because the cloud viewer is created dynamically
      // when the first frame with cloud or UM6 data is displayed
      if (do_vis) autoseg->setViewer(viewer->getCloudViewer());

      // call the segmenation algorithm
      if (do_autoseg) autoseg->autoSegment ();
      else autoseg->manSegment (indices);
     
      // print out the statistics
      //autoseg->printStat ();
    }
};

/** \brief The main function. */
int
main (int argc, char **argv)
{

  do_autoseg = console::find_switch(argc, argv, "-auto");
  do_vis = console::find_switch(argc, argv, "-vis");
  
  show_picks = console::find_switch(argc, argv, "-show_picks");

  show_camera_axes = console::find_switch(argc, argv, "-show_camera_axes");
  show_um6_in_camera = console::find_switch(argc, argv, "-show_um6_in_camera");
  show_um6_orientation = console::find_switch(argc, argv,
                                              "-show_um6_orientation");
  show_um6_gravity = console::find_switch(argc, argv, "-show_um6_gravity");

  console::parse_argument(argc, argv, "-cloud_alpha", cloud_alpha);

  autoseg->setShowFiltered(console::find_switch(argc, argv, "-show_filtered"));
  autoseg->setShowFixation(console::find_switch(argc, argv, "-show_fixation"));
  autoseg->setShowSalient(console::find_switch(argc, argv, "-show_salient"));
  autoseg->setShowSalFiltered(console::find_switch(argc, argv, "-show_sal_filtered"));
  autoseg->setShowSalCurvatures(console::find_switch(argc, argv,
                                                     "-show_sal_curvatures"));
  autoseg->setShowSalNormals(console::find_switch(argc, argv,
                                                  "-show_sal_normals"));
  autoseg->setShowNN(console::find_switch(argc, argv, "-show_nn"));
  autoseg->setShowPatches(console::find_switch(argc, argv, "-show_patches"));
  autoseg->setShowPatchNormals(console::find_switch(argc, argv,
                                                    "-show_patch_normals"));

  autoseg->setNMS(console::find_switch(argc, argv, "-nms"));
  autoseg->setDoValidation(console::find_switch(argc, argv, "-val"));
  autoseg->setNoStats(console::find_switch(argc, argv, "-no_stats"));

  MyPlayer player;
  int pm;
  pm = player.main(argc, argv);
  
  return (pm);
}
