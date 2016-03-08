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
#include "task.h"
#include "autoseg.h"

// PCL libraries
#include <pcl/io/openni_grabber.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace boost;

/** \brief MACRO for framerate printing. */
#define FPS_CALC(_WHAT_) \
do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
  { \
    std::cout << "Average framerate("<< _WHAT_ << "): " \
    << double(count)/double(now - last) << " Hz" <<  std::endl; \
    count = 0; \
    last = now; \
  } \
}while(false)

/** \brief Print help for the interactive key actions. */
void
interactiveUsage ()
{
  print_highlight("INTERACTIVE KEYBOARD COMMANDS FOR AUTOSEG:\n");
  cout << "  q, ctrl-c : stop and quit\n";
  cout << "  h         : show this help\n";
  cout << "  space     : toggle pause if untriggered, else trigger\n";
  cout << "  a         : run autoseg on current paused frame\n";
}

class OpenNIViewer
{
  /** \brief An OpenNIGrabber interface for calling autoseg on point cloud.
    *
    * Displays OpenNI frames taken live with a Grabber.  Run:  ./test_autoseg
    * For running the autoseg on the cloud, hit space and press 'a'.
    *
    * \author Dimitrios Kanoulas
    */
  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointCloudIn;
    typedef PointCloudIn::Ptr PointCloudInPtr;
    typedef PointCloudIn::ConstPtr PointCloudInConstPtr;

    /** \brief Constructor. */
    OpenNIViewer () :
      cloud_viewer_ (new PCLVisualizer ("PCL OpenNI Viewer")),
      task (new Task),
      pause_cloud_ (false), call_autoseg (false)
    {
      // Init cloud viewer
      cloud_viewer_->setCameraPosition(0,0,-2,0,-1,0,0);
      cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboardCallback,
                                              *this, (void*)&cloud_viewer_);
    }

    /** \brief Callback function for point cloud */
    void
    cloudCallback (const PointCloudInConstPtr& callback_cloud)
    {
      // lock
      if (cloud_mutex_.try_lock())
      {
        cloudConstPtr = callback_cloud;
        cloud_mutex_.unlock();
        received_new_data_ = true;
      }
    }

    /** \brief Callback function for keyboard events. */
    void
    keyboardCallback(const visualization::KeyboardEvent &event,
                     void* viewer_void)
    {
      if (event.getKeySym () == "h" && event.keyDown ())
        interactiveUsage ();
      else if (event.getKeySym () == "space" && event.keyDown ())
        pause_cloud_ = !pause_cloud_; // pause/unpause
      else if (event.getKeySym () == "a" && pause_cloud_ && event.keyDown ())
        call_autoseg = true; //whether to call autoseg
    }

    /** \brief Main loop. */
    void
    run ()
    {
      // Create a task
      task->setMaxPatchDistance (2.0);
      task->setMaxPatchSize (0.15f);
      
      // OpenNI grabber init
      OpenNIGrabber* grabber = new OpenNIGrabber();
      
      boost::function<void (const PointCloudInConstPtr&)> f =
        boost::bind (&OpenNIViewer::cloudCallback, this, _1);

      grabber->registerCallback (f);

      // start reading Kinect live stream
      grabber->start ();

      // main loop
      while (!cloud_viewer_->wasStopped())
      {
        cloud_viewer_->spinOnce();

        if(!pause_cloud_) // if the cloud is paused don't display new cloud
        {
          if (received_new_data_ && cloud_mutex_.try_lock ())
          {
            received_new_data_ = false;
            cloud_mutex_.unlock ();

            // Draw the cloud
            cloud_viewer_->removeAllPointClouds();
            cloud_viewer_->removeAllShapes();
            cloud_viewer_->addPointCloud<PointIn> (cloudConstPtr, "input_cloud");
          }
        }
        else
        {
          // Call autoseg
          if (call_autoseg)
          {
            autoseg = new AutoSeg (cloudConstPtr, task);
            autoseg->setViewer (cloud_viewer_);
            autoseg->setShowPatches (true);
            autoseg->setNoStats (true);
            autoseg->setDoValidation (true);
            autoseg->autoSegment ();
            delete autoseg;
            
            call_autoseg = false;
          }
        }
      }

      // stop the grabber
      cloud_mutex_.lock ();
      grabber->stop ();
      cloud_mutex_.unlock ();

      return;
    }

  protected:
    /** \brief OpenNI cloud viewer. */
    boost::shared_ptr<PCLVisualizer> cloud_viewer_;
    
    /** \brief Point cloud pointer. */
    PointCloudInConstPtr cloudConstPtr;
    
    /** \brief Task object. */
    boost::shared_ptr<Task> task;

    /** \brief AutoSeg object. */
    AutoSeg *autoseg;

    /** \brief Mutex. */
    boost::mutex cloud_mutex_;

    /** \brief Whether to pause the cloud. */
    bool pause_cloud_;

    /** \brief Whether new data are available. */
    bool received_new_data_;

    /** \brief Whether to call autoseg. */
    bool call_autoseg;
};

/** \brief Simple example for autoseg using OpenNI Grabber. */
int
main (int argc, char** argv)
{
  // Input arguments
  if (console::find_switch(argc, argv, "-h"))
  {
    interactiveUsage ();
    return 1;
  }

  // Create new grabber
  OpenNIViewer viewer;
  viewer.run ();

  return 1;
}
