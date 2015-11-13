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

#ifndef PATCH_SELECT_H_
#define PATCH_SELECT_H_

#include <pcl/common/angles.h>
#include <pcl/common/eigen.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include "map_patch.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace Eigen;

class PatchSelect
{
    /** \brief Select a set of patches that pass particular constraints wrt:
      * i) position, ii) curvature, iii) slope (in the particular importance 
      * order).
      *
      * In particular patches that are: i) in radius pt_r_ from a point pt_,
      * ii) with curvature (min_curv_, max_curv_), iii) with 
      * (min_slope_, max_slope_) between the normal vector and a slope_vec_
      * vector, are selected.
      *
      * \note All the params coordinates are in the same frame as the patch.
      *
      * \author Dimitrios Kanoulas
      */
    public:
      /** \brief Constructor. */
      PatchSelect ();

      /** \brief Destructor. */
      ~PatchSelect ();

      /** \brief Set the pointer to the input map patches vector. */
      void setInputMapPatches (vector<MapPatch>* input_map_patches);
      
      /** \brief Set the point and the max radius thresholds. */
      void setPt (Vector3d pt, double pt_r);
      
      /** \brief Set the curvature thresholds. */
      void setCurv (double min_curv, double max_curv);
     
      /** \brief Set sensor's origin. */
      void setVp (float vp_x, float vp_y, float vp_z);

      /** \brief Set the vector and slope thresholds (rad). */
      void setSlope (Vector3f slope_vec, double min_slope, double max_slope);
    
      /** \brief Get the output map patches indices. */
      vector<int> getOutputMapPatches ();

      /** \brief Filter the patches using the defined thresholds. */
      int filter ();

    private:
      /** \brief Returns between two points. */
      double findPtDist (Vector3d c);
      
      /** \brief Returns the max curvature. */
      double findMaxCurv (VectorXd curv);
      
      /** \brief Returns the min curvature. */
      double findMinCurv (VectorXd curv);

      /** \brief Returns the angle between two vectors. */
      double findSlope (Vector3f vec);

      /** \brief Fidn the angle (in rad) between 2 vectors. */
      double findAngle (Vector3f a, Vector3f b);

    private:
      /** \brief Whether to use */
      bool use_pt_, use_curv_, use_slope_;

    protected:
      /** \brief The input map patches vector. */
      vector<MapPatch>* input_map_patches_;

      /** \brief The output map patches vector. */
      vector<int> output_map_patches_;

      /** \brief The xyz-coords of the wanted patch's center. */
      Vector3d pt_;

      /** \brief The radius of the sphere around where the patch center should
        * lie.
        */
      double pt_r_;

      /** \brief Max curvature threshold. */
      double max_curv_;
      
      /** \brief Min curvature threshold. */
      double min_curv_;
     
      /** \brief Sensor's origin. */
      float vp_x_, vp_y_, vp_z_;

      /** \brief Slope vector. */
      Vector3f slope_vec_;
      
      /** \brief Max threshold angle for slope vector. (rad) */
      double max_slope_;
      
      /** \brief Min threshold angle for slope vector. (rad) */
      double min_slope_;
      
};

#endif //PATCH_SELECT_H_
