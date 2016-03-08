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

#ifndef PATCH_COVERAGE_H_
#define PATCH_COVERAGE_H_

// SPL headers
#include "patch.h"

// PCL headers
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

class PatchCoverage
{
  /** \brief Computes percent of good cells for boundary of patch p.
    *
    * The coverage evaluation is described in the paper:
    * -- "Sparse Surface Modeling with Curved Patches" by Dimitrios Kanoulas and
    *    Marsette Vona, ICRA 2013.
    * 
    * A grid of square cells is imposed on the local frame XY plane.  Cells are
    * categorized as good if (a) they contain a sufficient number of in-bounds
    * points relative to the portion of their area inside the patch boundary and
    * (b) they do not contain too many out-of-bounds points relative to their
    * area outside the patch boundary.
    *
    * \author Dimitrios Kanoulas
    */

  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointInCloud;
    typedef PointCloud<PointIn>::Ptr PointInCloudPtr;

    /** \brief Constructor.
      *
      * \param[in] p the patch to be plotted.
      * \param[in] nn_cloud the patches neighborhood.
      * \param[in] iw whether the input is in world frame.
     */
    PatchCoverage (Patch p, PointInCloudPtr nn_cloud, bool iw);

    /** \brief Set the nc variable. */
    void
    setNc (int nc);

    /** \brief Transform to local frame. */
    void
    world2local (PointInCloudPtr nn_cloud);

    /** \brief */
    double
    findCoverage ();

  private:
    /** \brief Similar to Matlab's colon: j:i:k
      *
      * \return the size of the o vector.
      */
    int
    colon (int j, int i, int k, RowVectorXi &o);

    /** \brief Similar to Matlab's meshgrid.
      *
      * Replicates the grid vectors xgv and ygv to produce the coordinates of a 
      * rectangular grid (X, Y). The grid vector xgv is replicated ygv.size() 
      * times to form the columns of X.  The grid vector ygv is replicated 
      * xgv.size() times to form the rows of Y.
      *
      **/
    void
    meshgrid (RowVectorXi xgv, VectorXi ygv, MatrixXi &X, MatrixXi &Y);

    /** \brief Similar to Matlab's rot90. */
    MatrixXi
    rot90 (MatrixXi A);

    /** \brief Similar to Matlab's fliplr */
    MatrixXi
    fliplr (MatrixXi A);

    /** \brief Similar to Matlab's flipud */
    MatrixXd
    flipud (MatrixXd A);

  protected:
    /** \brief The input patch (in world frame). */
    Patch p_;

    /** \brief The input set of points from which the patch was fitted (world 
      * frame).
      */
    PointInCloudPtr nn_cloud_;

    /** \brief Whether the input is in world frame. */
    bool iw_;

    /** \brief The output percentage, normalized to [0,1], is calculated as:
      *
      * pct = (ng-eo)/(nc-eo)
      *
      * which is the ratio of good cells excluding exterior empty cells to the
      * total number cells excluding exterior empty cells.
      */

    /** \brief The cell side length if positive. */
    double wc_;

    /* \brief Sets the desired number of cells if positive.
     *
     * \note The actual number of cells may be greater because local frame
     * origin must fall on cell boundaries.  Exactly one of wc, nc must be
     * positive.
     */
    int nc_;

    /** \brief Whether the desired number of cells has been set. */
    bool inc_;

    /** \brief Explicit conversion factors from cell in/out area to ic/oc
      * thresholds.
      */
    double ti_, to_;

    /** \brief Whether the explicit conversion factors from cell in/out area to
      * ic/oc thresholds have been set.
      */
    bool iti_, ito_;

    /** \brief Conversion factors from ne to ti and to.  Exactly one of ti, zi
      * must be positive, same for to, zo.
      */
    double zi_, zo_;

    /** \brief If positive overrides number of expected points per in-bounds 
      * cell.  Otherwise ne = nd*wc*wc/ba.
      */
    double ne_;

    /** \brief Whether the number of expected points per in-bounds cell has 
      * been set.
      */
    bool ine_;

    /** \brief TBD */
    double pct_;

  private:
    /** \brief Total number of points in neighborhood. */
    int nd_;

    /** \brief Num cells left/right/above/below local frame origin. */
    int lc_, rc_, tc_, bc_;
};

#endif // #ifndef PATCH_COVERAGE_H_

