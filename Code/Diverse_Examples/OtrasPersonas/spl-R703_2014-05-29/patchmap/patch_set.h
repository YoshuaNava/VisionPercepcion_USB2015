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

#ifndef PATCH_SET_H_
#define PATCH_SET_H_

// SPL headers
#include "patch.h"
#include "patch_plot.h"
#include "patch_sample.h"
#include "patch_fit.h"

// PCL headers
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace Eigen;

class PatchSet
{
  /** \brief Creates patches of different types for the purpose of running tests
    *        on them, e.g. plot, sample, fit etc.
    *
    * \author Dimitrios Kanoulas
    */
  public:
    /** \brief Constructor. */
    PatchSet() :
      patch_viewer_ (new PCLVisualizer ("3D Viewer")),
      sample_cloud_ (new PointCloud<PointXYZ>),
      dss_ (0.4),
      next_patch_ (false),
      do_patch_plot_ (false),
      do_patch_sample_ (false),
      do_sample_plot_ (false),
      do_patch_fit_ (false)
    {
      patch_viewer_->registerKeyboardCallback (&PatchSet::keyboardCallback, *this, (void*)&patch_viewer_);
      create_patches();
    }

    /** \brief Destructor. */
    ~PatchSet ()
    {};

    /** \brief Get patches. */
    vector<Patch>
    getPatches ();

    /** \brief Creates primary test patches. */
    void
    create_patches ();
        
    /** \brief Runs tests on patches of different types.
      *
      * \return boolean indicating tests success.
      */
    int
    run_tests ();
    
    /** \brief Shows the viewer. */
    void
    showViewer ();
    
    /** \brief Callback function for keyboard events. */
    inline void
    keyboardCallback (const visualization::KeyboardEvent &event,
                      void* viewer_void)
    {
      // test next patch in the vector list
      if (event.getKeySym () == "n" && event.keyDown ())
        next_patch_ = true;
    }

    /** \brief Visualizes a patch to the viewer. */
    void
    showPatch (Patch p);

    /** \brief Samples a patch. */
    void
    patchSample (Patch p);
    
    /** \brief Visualizes samples to the viewer. */
    void
    showSample ();
    
    /** \brief Fit a patch to a point cloud. */
    void
    doPatchFit ();
 
    /** \brief Set whether to test the patch plot. */
    inline void
    setDoPatchPlot (bool flag)
    {
      do_patch_plot_ = flag;
    }

    /** \brief Set whether to test the patch sampling. */
    inline void
    setDoPatchSample (bool flag)
    {
      do_patch_sample_ = flag;
    }
    
    /** \brief Set whether to test the sample plot. */
    inline void
    setDoSamplePlot (bool flag)
    {
      do_sample_plot_ = flag;
    }
    
    /** \brief Set whether to test the patch fitting. */
    inline void
    setDoPatchFit (bool flag)
    {
      do_patch_fit_ = flag;
    }
    
    //TBD: delete
    int
    test_patches (int (*tfun)(Patch p));

  protected:
    /** \brief Patch viewer, interacting with keyboard. */
    boost::shared_ptr<PCLVisualizer> patch_viewer_;
    
    /** \brief A vector of patches. */
    std::vector<Patch> patches_;

    /** \brief Cloud of sample points. */
    PointCloud<PointXYZ>::Ptr sample_cloud_;

    /** \brief TBD*/
    PatchFit *patchFit;

    /** \brief Patch sample size. */
    double dss_;

    /** \brief Whether to procceed with tests to next patch. */
    bool next_patch_;

    /** \brief Fitted patch. */
    Patch fitted_p_;

    /** Whether to test patch plot. */
    bool do_patch_plot_;

    /** Whether to test patch sampling. */
    bool do_patch_sample_;

    /** Whether to test sample plot. */
    bool do_sample_plot_;
    
    /** Whether to test patch fitting. */
    bool do_patch_fit_;
};

#endif // #ifndef PATCH_SET_H_
