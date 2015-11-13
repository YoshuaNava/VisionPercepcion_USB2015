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

#ifndef PATCH_FIT_H_
#define PATCH_FIT_H_

// SPL headers
#include "xform3.h"
#include "patch.h"

// PCL headers
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>

// BOOST headers
#include <boost/math/special_functions/erf.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;

class PatchFit
{
  /** \brief Fits a bounded curved patch to a point cloud.
    *
    * Given a set of sample points, a general surface type, and boundary type
    * (only if surface type is plane) and a boundary containment probability,
    * it returns the fitted patch in 9 steps.
    *
    * /author Dimitrios Kanoulas
    */
  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn> PointCloudIn;
    typedef PointCloudIn::Ptr PointCloudInPtr;

    /** \brief Constructor.
      *
      * \param[in] cloud the point cloud to be fit.
      */
    PatchFit (const PointCloudInPtr &cloud)
    {
      // init patch
      Vector2d d;
      Matrix<double,1,1> k;
      Vector3d r = MatrixXd::Zero(3,1);
      Vector3d c = MatrixXd::Zero(3,1);

      p_ = new Patch("plane (ellipse)", Patch::s_p, Patch::b_e, d, k, r, c);
      
      // set fitting options
      setCloud (cloud);
      st_a = true;
      setBt (Patch::b_e);
      setBcp (0.95);
      setSSmax (0);
      setKtol (1);
      setCcon (true);
      
      // set LM params
      setMaxi (100);
      setMinrd (1e-3);
      
      // init params
      xh = Vector3d::UnitX();
      yh = Vector3d::UnitY();
      zh = Vector3d::UnitZ();
    }

    /** \brief Destructor. */
    virtual ~PatchFit ()
    {
      //delete [] p_;
    }


    /** \brief Set the point cloud. */
    void
    setCloud (const PointCloudInPtr cloud);

    /** \brief Set the fitted boundary type. */
    void
    setBt (Patch::b_type bt);

    /** \brief Set the boundary containment probability in (0,1], otherwise 0.95. */
    void
    setBcp (double bcp);

    /** \brief Set the ssmax option. */
    void
    setSSmax (int ssmax);

    /** \brief Set curvature comparison tolerance, must be nonnegative,
      * otherwise 0.1.
      */
    void
    setKtol (double ktol);

    /** \brief Set whether to constrain the center point of fitted paraboloid
      * patches to the line through the centroid of the data in the direction of
      * the normal of a plane fit to the data.  This behavior is implied for
      * non-paraboloid patches.
      */
    void
    setCcon (bool ccon);

    /** \brief Set the maximum number of iterations for fitting termination. */
    void
    setMaxi (int maxi);

    /** \brief Set the residual threshold for fitting termination. */
    void
    setMinrd (double minrd);

    /** \brief Perform patch fitting on the input point cloud.
      *
      * \return 1 if the fitting was succesful, 0 o/w.
      */
    int
    fit();

    /** \brief Pointer to the patch. */
    Patch *p_;

  private:
    /** \brief 8 parameter algebraic form for paraboloid.
      *
      * \param[in] q is ndx3 data.
      * \param[in] a is the parameters:
      *            -- a(0:1) are the x, y curvatures
      *            -- a(2:4) is the orientation vector
      *            -- a(5:7) is the center point
      * 
      * \return NDx1 error as a function of a.
      *
      * \note Results were verified.
      */
    MatrixXd
    parab (Matrix<double, Dynamic, 3> q, VectorXd a);

    /** \brief Calculates the ndx3 Jacobian of parab(q,a) with respect to
      * a, where q are the ndx3 data and a are the parameters.
      *
      * \note Results are verified.
      */
    MatrixXd
    parab_dfda (VectorXd a);

    /** \brief General form for (partial f)/(partial a).
      *
      * \param[in] l is the (ndx3) data in local frame
      * \param[in] rr is the 3x3 local to world rotation matrix
      * \param[in] dqldr is from dqldr()
      * \param[in] dfdql is from dfdql()
      *
      * \return j is the (ndx9) Jacobian of the objective func wrt the
      * parameters in order k1 k2 k3 r1 r2 r3 c1 c2 c3.
      */
    Matrix<double,Dynamic,9>
    general_dfda (MatrixXd ql, Matrix3d rr, MatrixXd dfdql, vector<MatrixXd> dqldr);

    /** \brief General form for (partial f)/(partial ql).
      *
      * \param[in] ql is the ndx3 data in local frame.
      * \param[in] k3 is the 3x1 vector of curvature parameters.
      *
      * \return the (nd x 3) Jacobian of the objective func wrt to data in local
      * frame.
      *
      * \note Results verified.
      */
    Matrix<double,Dynamic,3>
    dfdql (MatrixXd &ql, Vector3d &k3);

    /** \brief General form for (partial ql)/(partial r).
      *
      * \param[in] qc is the ndx3 data in world frame minus the local frame
      * origin c.
      * \param[in] drr is the (3x1) vector including the (3x3) derivative of rr
      * with respect to the 3x1 rotation vector r.
      *
      * \return the (nd x 3 x 3) Jacobian of the local frame data wrt rotation vector r.
      *
      * \note Results verified.
      */
    vector<MatrixXd>
    dqldr (MatrixXd &qc, vector<Matrix3d> &drr);
    
    /** \brief Solves linear least squares system.
      *
      * The algorithm uses svd() and sets singular values less than an
      * appropriate tolerance to zero.
      *
      * \param[in] J is an MxN matrix.
      * \param[in] b is an Mx1 vector.
      *
      * \return Nx1 column vector a containing the solution such that J*a = b.
      */
    VectorXd
    lls (MatrixXd J, VectorXd b);

    /** \brief Levenberg-Marquardt minimization.
      *
      * \param[in] TBD
      *
      * \return TBD
      *
      */
    VectorXd
    lm(VectorXd a);
   
    /** \brief chi^2 optimization by weighted Levenberg-Marquardt.
      *
      * \param[in] d TBD
      * \param[in] a_init TBD
      *
      * \return TBD
      */
    VectorXd
    wlm(MatrixXd d, VectorXd a_init);

  protected:
    /** \brief Input point cloud for fitting. */
    PointCloudInPtr cloud_;

    /** \brief General type of fitted surface.  Must be either 'a' for general
      * paraboloid or 'p' for plane fitting.
      */
    //Patch::s_type st_;
    bool st_a;

    /** \brief The fitted boundary type if 'st' is  'p' (plane). */
    Patch::b_type bt_;
    
    /** \brief Boundary containment probability in (0,1], otherwise 0.95. */
    double bcp_;

    /** \brief If positive then this limits the maximum number of data samples
      * used for surface fitting.
      */
    int ssmax_;

    /** \brief Curvature comparison tolerance, must be nonnegative. */
    double ktol_;

    /** \brief Constrain the center point of fitted paraboloid patches to the
      * line through the centroid of the data in the direction of the normal 
      * of a plane fit to the data.
      */
    bool ccon_;

    /** \brief If nonnegative, sets a maximum limit on the number of iterations.
      * May be zero to just compute outputs for unmodified parameter vector.
      */
    int maxi_;

    /** \brief If positive, iterations terminate whenever the decrease in
      * residual is less than minrd times the previous residual (for lm algo).
      */
    double minrd_;

    /** \brief Vectors UnitX=[1;0;0], UnitY=[0;1;0], and UnitZ=[0;0;1]. */
    Vector3d xh, yh, zh;

    /** \brief Patch centroid. */
    VectorXd plane_c;

    /** \brief Patch normal vector. */
    VectorXd plane_n;
    
    /** \brief TBD */
    MatrixXd cloud_vec;
    
    /** \brief TBD */
    MatrixXd fit_cloud_vec;
    
    /** \brief Standard principal curvatures. */
    double kx, ky;

    /** \brief TBD */
    double lambda;
};

#endif // #ifndef PATCH_FIT_H_
