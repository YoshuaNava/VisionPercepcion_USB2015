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

#ifndef PATCH_SAMPLE_H_
#define PATCH_SAMPLE_H_

// SPL headers
#include "patch.h"
#include "xform3.h"

// PCL headers
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

class PatchSample
{
  /** /brief Computes range samples of a patch.
    *
    * Given a patch model p and the the coordinates of the measurements ray
    * direction vectors (mx, my, mz) starting from a known point (cx, cy, cz)
    * in the space, computes the range samples on the patch with coordinates
    * cx+mx.*r, cy+my.*r, cz+mz.*r, where r is the range value.
    * 
    *
    * /author Dimitrios Kanoulas
    */
  public:
    typedef PointXYZ PointIn;

    /** \brief Constructor.
      *
      * \param[in] p the input patch.
      */
    PatchSample (Patch p)
    {
      p_ = p;
      world_frame_ = true;
    }

    /** \brief Computes the sample point cloud as the intersection between
      * the measurements rays and the patch.
      *
      * \param[out] cloud the sampled point cloud.
      *
      * \param[in] mx the x-coordinates of the measurement ray direction vectors.
      * \param[in] my the y-coordinates of the measurement ray direction vectors.
      * \param[in] mz the z-coordinates of the measurement ray direction vectors.
      * \param[in] cx the x-coordinates of the starting point.
      * \param[in] cy the x-coordinates of the starting point.
      * \param[in] cz the x-coordinates of the starting point.
      */
    void      
    findSamples(PointCloud<PointIn>::Ptr &cloud,
                MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
                double cx, double cy, double cz);

    /** \brief Vectorize column-wise a matrix.
      *
      * \param[in] m matrix
      *
      * \return vectorized matrix vector
      */
    RowVectorXd
    vectorizeColWise (MatrixXd &m);

    /** \brief Generates range sample measurement vectors.
      *
      * \param[in] hfov horizontal field of view in radians
      * \param[in] vfov vertical field of view in radians
      * \param[in] nh row size of measurement ray matrix
      * \param[in] nv col size of measurement ray matrix
      * \param[in] p pointing direction
      * \param[in] u vector giving the "up" direction
      *
      * \param[out] mx the x-coordinate of the measurement ray direction vec
      * \param[out] my the y-coordinate of the measurement ray direction vec
      * \param[out] my the z-coordinate of the measurement ray direction vec
      */
    void
    sample_frustum(double hfov, double vfov, int nh, int nv,
                   Vector3d p, Vector3d u,
                   MatrixXd &mx, MatrixXd &my, MatrixXd &mz);

    /** \brief Set whether measurement rays are in world frame. */
    inline void
    setWorldFrame (bool world_frame)
    {
      this->world_frame_ = world_frame;
    }
    
  protected:
    /** \brief The input patch (see patch.h). */
    Patch p_;
    
    /** \brief Whether the coordinate frame of the supplied measurement rays is
      * in world frame.
      */
    bool world_frame_;
    
    /** \brief Range values.
      *
      * \note The matrix has the same size as measurement rays mx, my, mz.
      */
    MatrixXd r_;
};

#endif // #ifndef PATCH_SAMPLE_H_
