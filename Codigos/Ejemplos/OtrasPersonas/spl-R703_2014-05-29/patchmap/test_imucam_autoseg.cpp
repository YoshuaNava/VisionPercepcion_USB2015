/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-, Marsette A. Vona, III
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

#include "imucam.h"
#include "task.h"
#include "autoseg.h"

#include <iostream>
#include <stdio.h>
#include <ctype.h>

#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;
using namespace imucam;

const string appname("example");

//the static keyword here limits linkage scope to this file

typedef PointCloud<PointXYZ> PointCloudIn;
typedef typename PointCloudIn::Ptr PointCloudInPtr;
typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

/* Cloud for calling autoseg. */
PointCloudInConstPtr cloud_;

/** \brief Task object */
boost::shared_ptr<Task> task_ (new Task);

/** \brief AutoSeg object. */
AutoSeg *autoseg;

/** \brief The viewer. */
Viewer::Ptr viewer;

/** \brief Gravity vector. */
Eigen::Vector3f g;

/* UI event callbacks. */
static void keyCB(const visualization::KeyboardEvent &e);
static void mouseCB(const visualization::MouseEvent &e);
static void pointCB(const visualization::PointPickingEvent &e);
static void areaCB(const visualization::AreaPickingEvent &e);

/* Show command line help. */
static void usage();

/* Latest frame waiting to be processed. */
static Frame::ConstPtr latestFrame; //initialized to null

/* Guards latestFrame. */
static boost::mutex frame_mtx;

/* Frame callback updates latestFrame. */
static void frameCB(const Frame::ConstPtr &frame) {
  if (frame_mtx.try_lock()) {
    if (!latestFrame) latestFrame = frame;
    frame_mtx.unlock();
  }
}

/* Latest frame waiting to be processed. */
static PointCloud<PointXYZRGBA>::ConstPtr latestCloud; //initialized to null

/* Guards latestCloud. */
static boost::mutex cloud_mtx;

/* Cloud callback updates latestCloud. */
static void cloudCB(const PointCloud<PointXYZRGBA>::ConstPtr &cloud) {
  if (cloud_mtx.try_lock()) {
    if (!latestCloud) latestCloud = cloud;
    cloud_mtx.unlock();
  }
}

/* \brief Basic example.
 *
 * Loads a sequence of OpenNI+UM6 Frames (or even just a single one) from disk
 * using a Reader and displays them with a Viewer which is directly registered
 * as a FrameConsumer on the Reader.
 *
 * See play_frames.cpp for an example of how to use the higher-level Player
 * class which has interactive keyboard and mouse actions and richer console
 * output.
 *
 * TBD:
 * - generalize to optionally use a Receiver for live grabbing
 */
int main(int argc, char **argv) {

  cout << "imucam example, use -h for options\n";

  if (console::find_switch(argc, argv, "-h")) { usage(); return(0); }

  //Create an OpenNI+UM6 Frame Reader configured from command line args.  There
  //are also other constructors to configure it directly.
  //
  //Note: Reader extends pcl::Grabber, so you should be able to use it in the
  //usual ways.
  Reader::Ptr reader(new Reader(argc, argv));

  //Viewer::Ptr viewer;
  boost::shared_ptr<visualization::PCLVisualizer> pclVisualizer;
  if (!console::find_switch(argc, argv, "-noviewer")) {

    //Create an OpenNI+UM6 Frame Viewer configured from command line args.
    //There are also other constructors to configure it directly.  The callbacks
    //are optional.
    viewer.reset(new Viewer
                 (argc, argv,
                  boost::bind(keyCB, _1), boost::bind(mouseCB, _1), //rgb
                  boost::bind(keyCB, _1), boost::bind(mouseCB, _1), //depth
                  boost::bind(keyCB, _1), boost::bind(mouseCB, _1), //cloud
                  boost::bind(pointCB, _1), boost::bind(areaCB, _1)));

    //For a callback that's a member function (of the same class as this) then
    //use boost::bind(fooCB, this, _1) instead

    //Have the reader send frames directly to the viewer.  This is just more
    //convenient than registering a Frame callback on the reader and forwarding
    //frames manually by calling viewer.offer().  The result is the same.
    reader->setConsumer(viewer);

  } else if (!console::find_switch(argc, argv, "-nocloudviz")) {

    //Use a regular PCLVisualizer instead of an imucam::Viewer.

    //This is mainly here as an example to show how to use
    //Reader::registerCallback() to get pointclouds just like from any other PCL
    //Grabber.

    //See the imucam::Viewer class header doc for the advantages of using an
    //imucam::Viewer instead of a PCLVisualizer alone.

    //If you want you can just delete this entire else block, delete the
    //pclVisualizer variable, delete the if (pclVisualizer) { ... } block in the
    //foreground loop below, and remove the code pertaining to the "-noviewer"
    //switch (one line above and one line in usage()).  Then you will always use
    //a Viewer.
    
    pclVisualizer.reset(new visualization::PCLVisualizer("cloud viewer"));

    pclVisualizer->registerKeyboardCallback(keyCB);
    pclVisualizer->registerMouseCallback(mouseCB);
    pclVisualizer->registerPointPickingCallback(pointCB);
    pclVisualizer->registerAreaPickingCallback(areaCB);

    //Set up the viewpoint so that x-right y-down pointclouds look rightside up.
    pclVisualizer->setCameraPosition(0, 0, -3, //camera location
                                     0, 0, 1, //view point = spin center
                                     0, -1, 0); //up

    //Have the reader generate XYZRGBA point clouds for visualization.
    boost::function<FrameProducer::sig_cb_openni_point_cloud_rgba> cloud_cb =
      boost::bind(cloudCB, _1);
    reader->registerCallback(cloud_cb);
  }

  //Register to get callbacks when new OpenNI+UM6 Frames are available.  A Frame
  //includes the OpenNI rgb and depth images (if any) corresponding to a single
  //frame capture and the UM6 IMU messages that were received during that frame
  //capture.  You can also register to receive callbacks on just the images or
  //on point clouds derived from the images (see example above when using a
  //PCLVisualizer instead of a Viewer).  The advantage off receiving Frames is
  //that they package the image and IMU data together.

  boost::function<FrameProducer::sig_cb_imucam_frame> frame_cb =
    boost::bind(frameCB, _1);
  reader->registerCallback(frame_cb);

  //The viewer and reader internal threads are not running until start()ed.
  if (viewer) viewer->start();
  reader->start();

  //foreground loop
  while (true) {

    FPS::sleepMS(10);

    //These examples shos how to handoff data from callbacks.
    
    //We could instead (or in addition) poll with reader->getCurrent*() here.
    //
    //For example
    //
    //Frame::ConstPtr frame = reader->getCurrentFrame();
    //PointCloud<PointXYZRGBA>::ConstPtr cloud;
    //if (frame) cloud = frame->fillCloud();

    if (pclVisualizer) {

      pclVisualizer->spinOnce();

      //Update visualizer with latestCloud, if needed.

      PointCloud<PointXYZRGBA>::ConstPtr cloud; //initialized to null
      {
        boost::mutex::scoped_lock lock(cloud_mtx);
        cloud = latestCloud;
        latestCloud.reset(); //now forget about this cloud to allow new ones in
      }

      if (cloud) { //a new cloud has been received

        //Unfortunately we can't do this directly in cloudCB because all
        //PCLVisualizer APIs must be called from a single thread.  We called
        //some above from this thread, but cloudCB runs in a different thread.
        if (!pclVisualizer->updatePointCloud(cloud))
          pclVisualizer->addPointCloud(cloud);
      }
    }

    //Note: if we are not using a PCLVisualizer then we have an imucam::Viewer
    //instead, and it is updated directly, we don't need to intervene.

    //Now see if a new OpenNI+UM6 Frame has been received.

    Frame::ConstPtr frame; //initialized to null
    {
      boost::mutex::scoped_lock lock(frame_mtx);
      frame = latestFrame;
      latestFrame.reset(); //now forget about this Frame to allow new ones in
    }

    if (frame) {

      //Got new a rgb+depth+imu data frame.

      //We could do whatever we want with it, including update any viewers.
      
      //However in this example code the viewers have already been updated
      //directly in callbacks.

      //If we wanted an XYZRGBA point cloud for this frame, for example:
      //PointCloud<PointXYZRGBA>::ConstPtr cloud = frame->fillCloud();
      //PointCloud<PointXYZ>::ConstPtr cloud = frame->fillCloud<PointXYZ>();
      cloud_ = frame->fillCloud<PointXYZ>();

      //Here we just print out the new Frame's timestamp and the gravity vector.

      g = frame->getCameraToWorld()->transpose() * Eigen::Vector3f::UnitZ();

      //"\r" is carriage return with no newline, so keep overwriting same line.
      //
      //We use printf because it's pretty annoying to print floats with fixed
      //field (including substituting a space for the minus sign when the value
      //is nonnegative) with using c++ iostreams.  And we always want the line
      //to be exactly the same length because if we don't overwrite everything
      //we had printed previously there will be junk characters past the end of
      //our new printout.
      printf("%s gravity in camera: %6.3f %6.3f %6.3f\r",
             boost::posix_time::to_iso_string(frame->time).c_str(),
             g(0), g(1), g(2));
      cout << flush; //otherwise the console will take its time to update
    }

    //Hang around until the last frame has been viewed, or indefinitely if we're
    //only viewing a single frame.
    if ((reader->atEnd() &&
         !reader->getPaused() && (reader->getNumFrames() > 1)) &&
        (!viewer || (viewer->getCurrentFrameTimestamp() ==
                     reader->getCurrentFrameTimestamp()))) break;
  }

  cout << endl;

  if (viewer) viewer->stop();
  reader->stop();

  return (0);
}

void keyCB(const visualization::KeyboardEvent &e) {

  if (!e.keyDown()) return;

  int key = e.getKeyCode();
  if (e.getKeySym() == "space") key = ' ';
  if (e.getKeySym() == "a") key = 'a';
  if (e.isShiftPressed()) key = toupper(key);

  cout << endl;
  if (isalnum(key))
  {
    if (static_cast<char>(key) == 'a')
    {
      // Create a task
      task_->setMaxPatchDistance (2.0); //2m
      task_->setMaxPatchSize (0.15f); //30cm radius

      // Create a autoseg object. */
      autoseg = new AutoSeg(cloud_, task_);
      autoseg->setG (g);
      autoseg->setViewer (viewer->getCloudViewer());
      autoseg->autoSegment ();
      delete autoseg;
      cout << "Print out the cloud" << *cloud_ << endl;
    }
    cout << "'" << static_cast<char>(key) << "' pressed\n";
  }
  else if (isspace(key))
  {
    cout << "space pressed\n";
  }
  else
  {
    cout << "nonprinting key " << key << " pressed\n";
  }
}

void mouseCB(const visualization::MouseEvent &e) {
  if (e.getType() != visualization::MouseEvent::MouseButtonPress) return;
  if (e.getButton() != visualization::MouseEvent::LeftButton) return;
  cout << "\nleft button pressed\n";
}

void pointCB(const visualization::PointPickingEvent &e) {
  float x, y, z;
  e.getPoint(x, y, z);
  cout << "\npicked point (" << x << ", " << y << ", " << z << ")";
}

void areaCB(const visualization::AreaPickingEvent &e) {
  vector<int> indices;
  e.getPointsIndices(indices);
  cout << "\narea pick " << indices.size() << " points\n";
}

void usage() {

  const char *sn = string(appname.size()+1, ' ').c_str();

  cout << appname; 

  Reader::usageShort(" ", sn);
  Viewer::usageShort(sn, sn);

  cout << sn << "[-noviewer] [-h]\n";

  Reader::usageHelp();
  Viewer::usageHelp();

  cout << "-noviewer: PCL viewers instead of imucam::Viewer "
       << "(view options may be ignored)\n";

  cout << "-h: show this help\n";
}
