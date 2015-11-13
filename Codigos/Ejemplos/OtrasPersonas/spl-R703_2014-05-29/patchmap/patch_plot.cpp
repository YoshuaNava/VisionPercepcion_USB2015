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

// SPL headers
#include "patch_plot.h"

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace Eigen;

#define AXES_WIDTH 4.0f

////////////////////////////////////////////////////////////////////////////////
void
PatchPlot::setP (Patch p)
{
  this->p_ = p;
}

////////////////////////////////////////////////////////////////////////////////
void 
PatchPlot::addLine (vtkSmartPointer<vtkAppendPolyData> polydata,
                    vtkSmartPointer<vtkUnsignedCharArray> colors,
                    float x1, float y1, float z1,
                    float x2, float y2, float z2,
                    float r, float g, float b)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
  line->SetPoint1 (x1, y1, z1); 
  line->SetPoint2 (x2, y2, z2); 
  line->Update ();
  polydata->AddInput (line->GetOutput ()); 
  unsigned char rgb[] = {r * 255.0, g * 255.0, b * 255.0};
  colors->InsertNextTupleValue (rgb);
}

////////////////////////////////////////////////////////////////////////////////
void
PatchPlot::showPatch(boost::shared_ptr<PCLVisualizer> &viewer,
                     float r, float g, float b,
                     const std::string id)
{
  static const float LINE_WIDTH = 2.0f;
  float x1, y1, z1, x2, y2, z2;

  // generate patch's pose
  Affine3f pose = Translation3f (p_.getC().cast<float>()) *
                  AngleAxisf (p_.rexp(p_.getR()).cast<float>());

  // generate patch's id name
  //const std::string id = boost::to_string(p_.getID ()) + "_patch";

  // first try a pose update to an existing shape
  if (!viewer->updateShapePose(id,pose))
  {
    // new polydata object if it is a new patch
    vtkSmartPointer<vtkAppendPolyData> polydata =
       vtkSmartPointer<vtkAppendPolyData>::New ();

    vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");

    // add patch lines that connect the sampled patch points
    for (int i=0; i<p_.getGV ().size(); i++)
    {
      for (int j=0; j<p_.getGV ().at(i)->points.size()-1; j++)
      {
        x1 = p_.getGV ().at(i)->points[j].x;
        y1 = p_.getGV ().at(i)->points[j].y;
        z1 = p_.getGV ().at(i)->points[j].z;
        x2 = p_.getGV ().at(i)->points[j+1].x;
        y2 = p_.getGV ().at(i)->points[j+1].y;
        z2 = p_.getGV ().at(i)->points[j+1].z;
        
        addLine (polydata, colors, x1, y1, z1, x2, y2, z2, r, g, b);
      }
    }
    
    polydata->Update ();
    vtkSmartPointer<vtkPolyData> line_data = polydata->GetOutput ();
    line_data->GetCellData ()->SetScalars (colors);

    // add polydata in the viewer
    viewer->addModelFromPolyData (line_data, id);
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, LINE_WIDTH, id);
    viewer->updateShapePose (id, pose);
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PatchPlot::showFrame (boost::shared_ptr<PCLVisualizer> &v, const std::string id,
                      const Eigen::Affine3f &t, double scale, bool normal)
{
  if (v->updateShapePose (id,t)) { return; }

  vtkSmartPointer<vtkAppendPolyData> polydata =
    vtkSmartPointer<vtkAppendPolyData>::New();

  vtkSmartPointer<vtkUnsignedCharArray> colors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents (3);
  colors->SetName ("Colors");

#define ADD_LINE(x1, y1, z1, x2, y2, z2, r, g, b) {                     \
  vtkSmartPointer<vtkLineSource> l = vtkSmartPointer<vtkLineSource>::New(); \
  l->SetPoint1((x1), (y1), (z1));                                     \
  l->SetPoint2((x2), (y2), (z2));                                     \
  l->Update();                                                        \
  polydata->AddInput(l->GetOutput());                                 \
  unsigned char rgb[] = {static_cast<unsigned char>((r)*255.0),       \
                         static_cast<unsigned char>((g)*255.0),       \
                         static_cast<unsigned char>((b)*255.0)};      \
  colors->InsertNextTupleValue(rgb); }

  if (!normal)
  {
    ADD_LINE(0, 0, 0, scale, 0, 0, 1, 0, 0); //red
    ADD_LINE(0, 0, 0, 0, scale, 0, 0, 1, 0); //green
    ADD_LINE(0, 0, 0, 0, 0, scale, 0, 0, 1); //blue
  }
  else
  {
    ADD_LINE(0, 0, 0, 0, 0, scale, 1, 0, 1); //magenta
  }
#undef ADD_LINE

  polydata->Update();
  vtkSmartPointer<vtkPolyData> line_data = polydata->GetOutput();
  line_data->GetCellData()->SetScalars(colors);

  v->addModelFromPolyData (line_data, id);
  v->setShapeRenderingProperties (visualization::PCL_VISUALIZER_LINE_WIDTH,
                                      (normal) ? 2*AXES_WIDTH : AXES_WIDTH, id);
  v->updateShapePose (id, t);
}

////////////////////////////////////////////////////////////////////////////////
void
PatchPlot::removePatch (boost::shared_ptr<PCLVisualizer> &viewer,
                        const std::string id)
{
  viewer->removeShape (id);
  //viewer->removeShape (boost::lexical_cast<string>(id) + "_patch");
}

////////////////////////////////////////////////////////////////////////////////
void
PatchPlot::removeFrame (boost::shared_ptr<PCLVisualizer> &viewer,
                        const std::string id)
{
  viewer->removeShape (id);
}
