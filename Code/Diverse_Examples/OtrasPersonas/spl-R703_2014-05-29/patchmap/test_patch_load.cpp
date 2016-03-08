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

// PCL headers
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/search/organized.h>

// RXKINFU headers
#include <rxkinfu/kinfu_app.h>

// SPL headers
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

#define KINFU_BASE rxkinfu::KinfuApp

class PickKinfuApp: public KINFU_BASE
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
    PickKinfuApp (int argc, char **argv) :
      KinfuApp (argc, argv),
      nn_cloud_ (new PointCloud<PointXYZ>),
      patch_plot_ (new PatchPlot)
    {
      vp_x_ = vp_y_ = vp_z_ = 0.0;
      print_status = false;
      pt_x_ = pt_y_ = pt_z_ = 0.0;
      pt_picked_ = false;
      vis_pt_picked_ = false;
      do_fit_ = false;
      radius_ = 0.05;
      draw_patch_ = false;
    };

    /** \brief Destructor. */
    virtual ~PickKinfuApp () {}

    /** \brief Print usage help. */
    static void usageHelp (const char *pfx = "", bool with_inputs = true)
    {};

    /** \brief UI event callbacks. */
    void
    keyCB (const pcl::visualization::KeyboardEvent &e, void *cookie)
    {
      // call superclass implementation
      KinfuApp::keyCB (e, cookie);

      int key = e.getKeyCode();

      if (!(e.keyDown())) return;
      
      switch (key)
      {
        case 27: case (int)'q':
          quit(true);
          break;
        case (int)'c':
          do_fit_ = true;
          draw_patch_ = false;
          break;
      }
    }
    
    /** \brief Point picking callback function. */
    void
    pointCB (const visualization::PointPickingEvent &e, void *cookie)
    {
      // point picking happens only when it is paused
      if (!paused){ return; }

      // pick the point
      e.getPoint (pt_x_, pt_y_, pt_z_);
      pt_picked_ = vis_pt_picked_ = true;
      cout << "\npicked point (" << pt_x_ << ", " << pt_y_ << ", " << pt_z_ << ")" << endl;
    }
    

    /** \brief Execute bubble raycast and patch mapping and tracking. */
    void execBubbleRaycast (const Affine3f &bubble_pose)
    {
      // call superclass implementation
      KinfuApp::execBubbleRaycast (bubble_pose);

      // set the sensor viewpoint
      vp_x_ = bubble_cloud_ptr[0]->sensor_origin_.coeff (0);
      vp_y_ = bubble_cloud_ptr[0]->sensor_origin_.coeff (1);
      vp_z_ = bubble_cloud_ptr[0]->sensor_origin_.coeff (2);
     
      // load a patch
      lib_patch_.loadPatch ("patch_lib/patch_1.txt");
      lib_patch_.updatePose (kinfu->getCameraPose().cast<double>());
      lib_patch_.flipPatchFrameTowardsViewpoint (vp_x_, vp_y_, vp_z_);
      lib_patch_.setAutoSS ();
      lib_patch_.infer_params ();
      lib_patch_.gs();
      //lib_patch_.printInfo(kinfu->getCameraPose().cast<double>());

      // fit a patch to a picked point
      if (do_fit_)
      {
        // fetch and validate neighborhood
        search_.setInputCloud (bubble_cloud_ptr[0]);

        search_.radiusSearch (PointXYZ(pt_x_, pt_y_, pt_z_),
                              radius_, nn_ind_, nn_sqr_dist_);
        
        // check neighborhood size
        if (nn_ind_.size()<10)
        {
          do_fit_ = draw_patch_ = false;
          return;
        }

        nn_cloud_->points.clear();
        for (int j=0; j<nn_ind_.size(); j++)
          nn_cloud_->points.push_back (bubble_cloud_ptr[0]->points[nn_ind_[j]]);

        // fit a patch to the neighborhood
        patchFit = new PatchFit (nn_cloud_);
        patchFit->setSSmax (50);
        patchFit->fit();
        
        patch = *patchFit->p_;

        // flip the patch frame
        patch.flipPatchFrameTowardsViewpoint (vp_x_, vp_y_, vp_z_);
        
        delete (patchFit);

        // infer some params
        patch.setAutoSS ();
        patch.infer_params();
        patch.gs();
       
        // print out patch's info
        patch.printInfo (kinfu->getCameraPose().inverse().cast<double>());

        // check for similarities
        cout << "similar: " << lib_patch_.isSimilar (patch, 0.015, 5.0, 5.0, 0.01) << endl;


        draw_patch_ = true;
        do_fit_ = pt_picked_ = false;
      }

    }

    /** \brief Execute external algs, eg control related. */
    void execExt ()
    {
      // call superclass implementation
      KinfuApp::execExt (); 
    }

    /** \brief Execute visualization. */
    void execViz (bool has_data, bool has_image, const Affine3f &bubble_pose)
    {
      // call superclass implementation
      KinfuApp::execViz (has_data, has_image, bubble_pose);

      // patch map visualizer
      cloud_viewer_ptr_ = scene_cloud_view->getVisualizer ();

      // remove and add the loaded patch
      patch_plot_->setP (lib_patch_);
      patch_plot_->showPatch (cloud_viewer_ptr_, 0.0f, 1.0f, 0.0f, "lib_patch");

      // remove old patch
      patch_plot_->removePatch (cloud_viewer_ptr_, "patch");
      patch_plot_->removeFrame (cloud_viewer_ptr_, "frame");

      // visualize the patch only if it is paused
      if (!paused) { return; }

      // visualize picked point
      if (vis_pt_picked_)
      {
        cloud_viewer_ptr_->removeShape ("point");
        cloud_viewer_ptr_->addSphere (PointXYZ(pt_x_, pt_y_, pt_z_),0.005,1,0,0,"point");
        vis_pt_picked_ = false;
      }

      // visualize the fitted patch
      if (draw_patch_)
      {
        patch_plot_->setP (patch);
        Affine3f aff = (patch.getPose()).cast<float>();
        patch_plot_->showFrame (cloud_viewer_ptr_, "frame", aff);
        patch_plot_->showPatch (cloud_viewer_ptr_, 0.0f, 0.0f, 1.0f, "patch");
      }
    }

  protected:
    /** \brief Loaded patch from the library. */
    Patch lib_patch_;

    /** \brief Viewpoint. */
    float vp_x_, vp_y_, vp_z_;
      
    /** \brief For organized neighborhood search. */
    OrganizedNeighbor<PointXYZ> search_;
  
    /** \brief Seed point's nearest neighbors. */
    vector <int> nn_ind_;

    /** \brief Seed point's square distances from the nearest neighbors. */
    vector <float>  nn_sqr_dist_;
    
    /** \brief Picked point's coords. */
    float pt_x_, pt_y_, pt_z_;
    
    /** \brief Whether point has been picked. */
    bool pt_picked_;
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
};

int
main (int argc, char **argv)
{   
  if (find_switch(argc, argv, "-h")) { PickKinfuApp::usageHelp(); return 0; }
  else print_highlight("run walkctl -h for command line help, "
      "hit h for online help\n");

  try { PickKinfuApp (argc, argv).mainLoop (); }
  catch (const PCLException &e) {
    cerr << "PCLException: " << e.what() << endl;
  } catch (const std::bad_alloc &e) {
    cerr << "Bad alloc: " << e.what() << endl;
  } catch (const std::exception &e) {
    cerr << "Exception: " << e.what() << endl;
  }

  return 0;
}
