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

#ifndef PATCH_PLOT_H_
#define PATCH_PLOT_H_

// SPL headers
#include "patch.h"

// PCL headers
#include <pcl/visualization/pcl_visualizer.h>

// VTK headers
#include <vtkLineSource.h>

using namespace std;
using namespace pcl::visualization;

class PatchPlot
{
  /** \brief Plots patch p (see patch.h for patch details).
    *
    * It adds a plot of patch p in a PCLVisualizer viewer that comes as input
    * to the showPatch method.  The patch is plotted as a set of VTK polydata
    * lines.  If the same patch exists already in the viewer then only its pose
    * is updated.
    *
    * \author Dimitrios Kanoulas
    */
  
  public:
    /** \brief Constructor.
      *
      * \param[in] p the patch to be plotted.
      */
    PatchPlot (Patch p)
    {
      p_ = p;
    }
    
    PatchPlot () {};

    /** \brief Set patch. */
    void
    setP (Patch p);

    /** \brief Adding a polydata line from (x1,y1,z1) to (x2,y2,z2) having 
      * (r,g,b) color.
      *
      * \param[out] polydata the polydata stucture that be be updated with the
      * line.
      * \param[out] colors the polydata color to be updated.
      *
      * \param[in] x1 the x-component of the first point.
      * \param[in] y1 the y-component of the first point.
      * \param[in] z1 the z-component of the first point.
      * \param[in] x2 the x-component of the second point.
      * \param[in] y2 the y-component of the second point.
      * \param[in] z2 the z-component of the second point.
      * \param[in] r the red color component.
      * \param[in] g the green color component.
      * \param[in] b the blue color component.
      */
    void
    addLine (vtkSmartPointer<vtkAppendPolyData> polydata,
             vtkSmartPointer<vtkUnsignedCharArray> colors,
             float x1, float y1, float z1,
             float x2, float y2, float z2,
             float r, float g, float b);

    /** \brief Shows the patch p in the viewer.
      *
      * \param[in] viewer the pointer to the viewer.
      * \param[in] r,g,b the red/green/blue color component.
      */
    void
    showPatch (boost::shared_ptr<PCLVisualizer> &viewer,
               float r=0, float g=1, float b=0,
               const std::string id = "_patch");

    /** \brief Shows the patch frame in the viewer.
      *
      * \param[in] viewer the pointer to the viewer.
      * \param[in] id the id of the triad
      * \param[in] t the pose of the triad
      * \param[in] scale the length of the triad segments
      * \param[in] normal whether to show only the z-axis, i.e. the normal
      */
    void
    showFrame (boost::shared_ptr<PCLVisualizer> &v, const std::string id,
               const Eigen::Affine3f &t, double scale = 0.05, bool normal=false);

    /** \brief Removes the patch p from the viewer.
      *
      * \param[in] viewer the pointer to the viewer.
      * \param[in] patch id.
      */
    void
    removePatch (boost::shared_ptr<PCLVisualizer> &viewer,
                 const std::string id);
    
    /** \brief Removes the patch frame from the viewer.
      *
      * \param[in] viewer the pointer to the viewer.
      * \param[in] id frame id.
      */
    void
    removeFrame (boost::shared_ptr<PCLVisualizer> &viewer,
                 const std::string id);

  protected:
    /** \brief The input patch. */
    Patch p_;
};

#endif // #ifndef PATCH_PLOT_H_
