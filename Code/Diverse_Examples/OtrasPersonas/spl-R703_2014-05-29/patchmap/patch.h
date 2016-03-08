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

#ifndef PATCH_H_
#define PATCH_H_

// SPL headers
#include "xform3.h"

// PCL headers
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// BOOST headers
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

// EIGEN headers
#include <unsupported/Eigen/Polynomials>

using namespace std;
using namespace pcl;
using namespace Eigen;

class Patch
{
  /** \brief Data structure and functions for curved surface patches.
    *
    * Patches have primary and derived fields (see below for details).  The
    * patches are described in details in [1], [2], and [3].
    *
    * [1] Vona, Kanoulas, "Curved Surface Contact Patches with Quantified
    *     Uncertainty", IROS 2012.
    * [2] Kanoulas, Vona, "Sparse Surface Modeling with Curved Patches", ICRA
    *     2013.
    * [3] Kanoulas, Vona, "Bio-Inspired Rough Terrain Contact Patch Perception",
    *     ICRA 2014.
    *
    * \author Dimitrios Kanoulas
    */

  public:
    typedef PointXYZ PointIn;
    typedef PointCloud<PointIn>::Ptr PointCloudPtr;
    typedef PointCloud<PointIn>::ConstPtr PointCloudConstPtr;

    /** \brief Patch surface type. */
    enum s_type
    {
      s_e, // elliptic paraboloid
      s_h, // hyperbolic paraboloid
      s_y, // cylindric paraboloid
      s_o, // circular paraboloid
      s_p  // plane
    };

    /** \brief Patch boundary type. */
    enum b_type
    {
      b_c, // circle
      b_e, // ellipse
      b_r  // axis aligned rectangle 
    };

    /** \brief Contructor with default primary fields. */
    Patch () :
      ss_ (1.0),
      gd_ (1)
    {
      //cout << "Warning: default patch created" << endl;
      
      // DEFAULT PRIMARY FIELDS
      setID (0);
      setName ("plane (ellipse)"); // name
      setS (s_p); // surface type
      setB (b_e); // boundary type
      setD (Vector2d::Ones()); // boundary parameters
      setK (Vector2d::Zero()); // curvature parameters
      setR (Vector3d::Zero()); // rotation vector
      setC (Vector3d::Zero()); // centre vector

      // set helping variables
      infer_params();
    };
    
    /** \brief Constructor with primary patch fields including rotation and 
      * translation (r,c) from local to world frame.
      * 
      * \param[in] name_arg the name
      * \param[in] s_arg the surface type
      * \param[in] b_arg boundary type
      * \param[in] d_arg boundary parameters
      * \param[in] k_arg curvature parameters
      * \param[in] r_arg rexp(r_arg) is basis for local frame of patch
      * \param[in] c_arg origin of local frame of patch
      */
    Patch (string name_arg, s_type s_arg, b_type b_arg, VectorXd d_arg,
           VectorXd k_arg, Vector3d r_arg, Vector3d c_arg) :
      ss_ (1.0),
      gd_ (1) 
    {
      // primary fields
      setID (0);
      setName (name_arg);
      setS (s_arg);
      setB (b_arg);
      setD (d_arg);
      setK (k_arg);
      setR (r_arg);
      setC (c_arg);

      // set helping variables
      infer_params();
    }

    /** \brief Destructor. */
    virtual ~Patch () {};

    /** \brief Set patch's id.
      *
      * \param[in] id unique patch id.  The id plays a role in visualization
      * when trying to view different patches in the same viewer.
      */
    void
    setID (int id);

    /** \brief Get patch's id. */
    int
    getID ();

    /** \brief Set patch's name.
      *
      * \param[in] name patch's name.  The name is not involved in any
      * algorithm, but works for visualization and debugging.
      */
    void
    setName (string name);

    /** \brief Get patch's name. */
    string
    getName ();

    /** \bief Set patch's surface type.
      *
      * \param[in] s patch's surface type (see enum type s_type above).
      */
    void
    setS (s_type s);

    /** \bief Get patch's surface type. */
    s_type
    getS ();

    /** \bief Set patch's boundary type.
      *
      * \param[in] b patch's boundary type (see enum type b_type above).
      */
    void
    setB (b_type b);

    /** \bief Get patch's boundary type. */
    b_type
    getB ();

    /** \bief Set patch's boundary parameters.
      *
      * \param[in] d is a vector of nd boundary parameters, where nd
      * depends on the boundary type:
      *        'c' : nd=1
      *   'e', 'r' : nd=2
      * 
      * The circle, ellipse, and the AA rect parameters are radii.  Ellipse
      * and AA rect radii are given in the order d = [dx dy].  All boundary 
      * parameters must be positive, otherwise they take value 1.  If d has 
      * more than nd entries, the latter ones are ignored.
      */
    void
    setD (VectorXd d);

    /** \bief Get patch's boundary parameters (vector of size nd). */
    VectorXd
    getD ();

    /** \bief Set patch's curvature parameters.
      *
      * \param[in] k is a vector of nk curvature parameters, where nk depends on
      * the surface type:
      *   'e', 'h' : nk=2
      *   'y', 'o' : nk=1
      *        'p' : nk=1 (k=0, even if different value has been set)
      */
    void
    setK (VectorXd k);

    /** \brief Get patch's curvature parameters. */
    VectorXd
    getK();

    /** \bief Set the rotation vector for the basis for local frame of patch.
      * 
      * \param[in] r the rotation vector.
      */
    void
    setR (Vector3d r);

    /** \bief Get the rotation vector for the basis for local frame of patch. */
    Vector3d
    getR ();

    /** \brief Get the rotation vector for the basis for local frame of patch
      * after applying an affine transformation.
      *
      * \param[in] pose the affine tranformation to be applied
      */
    Vector3d
    getR (Affine3d pose);

    /** \brief Get the rotation matrix. */
    Matrix3d getRot ();

    /** \bief Set the origin of local frame of patch.
      *
      * \param[in] c the origin of local frame of patch.
      */
    void
    setC (Vector3d c);

    /** \bief Get the origin of local frame of patch. */
    Vector3d
    getC ();
    
    /** \bief Get the origin of local frame of patch after applying an affine
      * transformation.
      *
      * \param[in] pose the affine tranformation to be applied
      */
    Vector3d
    getC (Affine3d pose);

    /** \brief Get the rotation matrix. */
    Matrix3d
    getRM ();
    
    /** \brief Get the rigid-body homogenous transformation matrix. */
    Matrix4d
    getPM ();
    
    /** \brief Get the rigid-body homogenous affine transformation. */
    Affine3d
    getPose ();

    /** \brief Get the normal vector. */
    Vector3d
    getNormal ();
    
    /** \brief Get the normal vector. */
    Vector3d
    getYFrameAxis ();

    /** \brief Get the standard principal curvature (always 2). */
    Vector2d
    getSK ();

    /** \bief Get the number of curvature  parameters for the boundary type. */
    int
    getNK ();

    /** \bief Get the number of boundary parameters for the patch surface. */
    int
    getND ();
    
    /** \bief Set the spacing.
      *
      * \param[in] ss the spacing value (should be positive).
      */
    void
    setSS (double ss);
    
    /** \brief Set automatically ss given the boundary radiis. */
    void
    setAutoSS ();

    /** \brief Get the spacing. */
    double
    getSS ();
    
    /** \bief Set the grid decimation factor.
      *
      * It is used for sampling.  Grid samples are points on a grid with spacing
      * gd*ss that are inside or on the boundary curve.  If non-positive we take
      * its absolute value.
      *
      * \param[in] gd the grid decimation factor.
      */
    void
    setGD (int gd);

    /** \brief Get the grid decimation factor. */
    int
    getGD ();

    /** \brief Get the patch bounding box. */
    Matrix2d
    getBB ();
    
    /** \brief Get the patch bounding box. */
    double
    getBA ();

    /** \brief Returns the vector of gv cloud pointers. */
    vector<PointCloudConstPtr>
    getGV ();

    /** \brief Get patch residual. */
    double
    getResidual ();
    
    /** \brief Set the cos angle between two normal vectors. */
    void
    setCnn (double cnn);
    
    /** \brief Get the cos angle between the two normal vectors. */
    double
    getCnn ();

    /** \brief Set the cos angle between normal vector and gravity. */
    void
    setCng (double cng);
    
    /** \brief Get the cos angle between normal vector and gravity. */
    double
    getCng ();

    /** \brief Updates patch's pose. */
    void
    updatePose (Affine3d pose);

    /** \brief Computes the 3x3 rotation matrix corresponding to 3x1 rotation
      * vec.  Uses Rodrigues' rotation formula.  Numerically stable as norm(r)
      * approaches zero.
      *
      * \param[in] r the orientation vector.
      *
      * \return 3x3 rotation matrix.
      */
    Matrix3d
    rexp (Vector3d r);

    /** \brief Computes the Jacobian of rexp(r).
      *
      * \param[in] r the rotation vector.
      *
      * \return a 3x1 vector storing 3x3 matrices, where each ith entry is the
      * Jacobian of (exp(r))(:) with respect to r(i).
      *
      * \note Results verified.
      */
    vector<Matrix3d>
    drexp (Vector3d r);

    /** \brief Computes the 3x1 orientation vector corresponding to 3x3
      * rotation matrix.  Return is always canonical, i.e. the invariant
      * rreparam(rlog(m)) == rlog(m) should always hold.  Numerically stable
      * as m approaches identity.
      *
      * \param[in] m the 3x3 rotation matrix.
      *
      * \return 3x1 rotation vector.
      */
    Vector3d
    rlog (Matrix3d m);

    /** \brief Maps orientation vector r to equivalent with length at most pi.
      *
      * \param[in] r the orientation vector
      *
      * \return orientation vector where norm(r)<=pi
      */
    Vector3d
    rreparam (Vector3d r);

    /** \brief Canonical 2DoF orientation vector holding axis p fixed.
      *
      * \param[in] r orientation vector
      * \param[in] p must be 0 for x, 1 for y, or 2 for z axis
      * \param[in] d must be 2 or 3
      *
      * \return the dx1 return orientation vector r2 is found such that 
      * r2 = eye(d,3)*rr, rr(p) = 0 and R2(:,p) = R(:,p), where R2 = rexp(rr)
      * and R = rexp(r)
      */
    VectorXd
    rcanon2(Vector3d r, int p, int d);
    
    /** \brief Check and infer surface and boundary type params. */
    void
    infer_params();

    /** \brief Computes the pose (rm,pm). */
    void
    gp();

    /** \brief Grid samples [gx, gy, gz] and boundary samples. */
    void
    gs();

    /** \brief Returns the geometric residual of the patch.
      *
      * \param[in] cloud the point cloud (in world frame) that the patch was
      * fitted from.
      *
      * \return the RMS geometric residual.
      */
    double
    computeResidual (PointCloudPtr &cloud);
    
    /** \brief An implicit form of the surface in local frame.
      *
      * \param[in] (x,y,z) is a 3D point to be checked.
      *
      * \return sl(x,y,z)==0 iff (x,y,z) is a point on the surface in local 
      * frame.  Otherwise the return is the algebraic error of the test point:
      * positive for points below the surface and negative for points above the
      * surface.
      */ 
    double
    sl (double x, double y, double z);

    /** \brief The gradient of sl(x,y,z).
      *
      * \param[in] xyz Nx3 data.
      *
      * \return the gradient for each point.
      */
    Matrix<double,Dynamic,3>
    sg (Matrix<double,Dynamic,3> xyz);

    /** \brief Is an explicit form of the surface in local frame, i.e.
      * z=zl(x,y) where (x,y,z) is a point on the surface in local frame.
      *
      * \param[in] x the local x parameter.
      * \param[in] y the local y parameter.
      *
      * \return the local z parameter.
      */
    RowVectorXd
    zl (RowVectorXd x, RowVectorXd y);

    /** \brief Computes the range(s) of intersection(s) of 3D rays from start
      * point(s) (cx,cy,cz) in the direction(s) (mx,my,mz) with the patch 
      * surface.  Clipping to the patch boundary is not applied.  Failed 
      * intersections yeild either infinite or complex range.
      *
      * \param[in] mx the x-coordinate of the measurement ray direction vec.
      * \param[in] my the y-coordinate of the measurement ray direction vec.
      * \param[in] my the z-coordinate of the measurement ray direction vec.
      * \param[in] cx the x-coordinate of the starting point.
      * \param[in] cy the y-coordinate of the starting point.
      * \param[in] cz the z-coordinate of the starting point.
      *
      * \return the range per sample point.  Upon return, the coordinates of the
      * sample points are always cx+mx.*r, cy+my.*r, cz+mz.*r.
      */
    MatrixXd
    ri (MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
        double cx, double cy, double cz);

    /** \brief The intersections of an axis-alinged line with the bounding curve
      * in the local frame XY plane.
      *
      * \param[in] i when i=0 intersection is for the  vertical line at x=u, and
      * when i=1 for horizontal line at y=u.
      *
      * \param[out] [a b], where min=a and max=b intersection, or [nan nan] if none.
      *
      * \return 1 if values returned succesfully, 0 otherwise.
      */
    int
    bi (int i, Matrix<double,1,Dynamic> u, Matrix<double,Dynamic,2> &o);

    /** \brief An implicit form of the boundary curve in the local frame XY
      * plane.
      *
      * \param[in] x the x-coordinate of the data.
      * \param[in] y the y-coordinate of the data.
      *
      * \param[out] bl(x,y)==0 iff (x,y) is a point on the boundary curve.
      * Otherwise the return is the algebraic error of the test point: positive
      * for points outside the curve, negative for points inside the curve.
      *
      * \return 1 if values returned succesfully, 0 otherwise.
      */
    int
    bl (MatrixXd x, MatrixXd y, MatrixXd &o);

    /** \brief Computes the intersection area of the boundary and a square with
      * lower left corner (x*w,y*w) and side length w (which may be either scalar
      * or vector).
      *
      * \param[in] x TBD
      * \param[in] y TBD
      * \param[in] w TBD
      *
      * \note in the current implementation x and y must be integers in matrices
      * of and the same size and the computed intersection may be approximate.
      */
    int
    bc (MatrixXi x, MatrixXi y, double w, MatrixXd &o);

    /** \brief Flip (in place) the patch frame towards a given viewpoint
      *
      * \param[in] vp_x the X coordinate of the viewpoint
      * \param[in] vp_y the X coordinate of the viewpoint
      * \param[in] vp_z the X coordinate of the viewpoint
      */
    void
    flipPatchFrameTowardsViewpoint (float vp_x, float vp_y, float vp_z);
    
    /** \brief Compares two patches for similarity.
      *
      * \param[in] p patch to compare with
      * \param[in] d_thres d threshold
      * \param[in] k_thres k threshold
      * \param[in] t_thres angle (in degrees) threshold
      * \param[in] r_thres position (in m) threshold
      *
      * \return whether the patches are similar
      */
    bool
    isSimilar (Patch &p, double d_thres=0.0, double k_thres=0.0,
               double t_thres=0.0, double r_thres=0.0);

    /** \brief Print patch's params.
      *
      * \param[in] pose the pose transofrmation for changing (r,c)
      */
    void
    printInfo (Affine3d pose = Affine3d::Identity());

    /** \brief Load a patch from a file. */
    bool
    loadPatch (const string &fn);

  private:
    /** \brief Solves Lagrange polynomial.
      *
      * \param[in] qx,qy,qz point's coordinates
      *
      * \return minimum corresponding squared distance
      */
    double
    polysolve (VectorXd &coeffs, double qx, double qy, double qz);
    
    /** \brief Generate range intersection function for planes.
      *
      * \note For args see ri function.
      */
    MatrixXd
    riplane(MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
            double cx, double cy, double cz);

    /** \brief Generate quadratic form range intersection function.
      *
      * \note For args see ri function.
      */
    MatrixXd
    riquadratic(MatrixXd &mx, MatrixXd &my, MatrixXd &mz,
                double cx, double cy, double cz);
    
    /** \brief Ellipse boundary intersection function.  Returns the 
      * intersections of an axis-alinged line with the bounding curve in the 
      * local frame XY plane: i=0 for vertical line at x=u, i=1 for 
      * horizontal line at y=u. Output [a b] is the min=a and max=b
      * intersection, or [nan nan] if none.
      *
      * \param[in] i vertical or horizontal line intersection
      * \param[in] u point of vertical/horizontal line
      *
      * \param[out] min, max ordinates of the intersection of vert (i=0) or
      * horiz (i=1) line at abscissa u.
      */
    Matrix<double,Dynamic,2>
    biellipse(int i, RowVectorXd &u);

    /** \brief Axis-aligned quadrilateral boundary intersection function.
      *  Returns the intersections of an axis-alinged line with the bounding
      *  curve in the local frame XY plane: i=0 for vertical line at x=u, 
      *  i=1 for horizontal line at y=u. Output [a b] is the min=a and max=b
      *  intersection, or [nan nan] if none.
      *
      * \param[in] i vertical or horizontal line intersection
      * \param[in] u point of vertical/horizontal line
      *
      * \param[out] min, max ordinates of the intersection of vert (i=0) or
      * horiz (i=1) line at abscissa u.
      */
    Matrix<double,Dynamic,2>
    biaaquad(int i, RowVectorXd &u);
    
    /** \brief Generate implicit boundary function for ellipse.
      *
      * \notes see bl method for params.
      */
    MatrixXd
    blellipse(MatrixXd &x, MatrixXd &y);

    /** \brief Generate implicit boundary function for axis aligned quad.
      *
      * \notes see bl method for params.
      */
    MatrixXd
    blaaquad(MatrixXd &x, MatrixXd &y);

    /** \brief Generate implicit boundary function for quadrilateral.
      *
      * \param[in] r 4 quad verts in CCW order (4x2)
      *
      * \param[out] see bl method for params.
      */
    MatrixXd
    blquad(MatrixXd &x, MatrixXd &y, Matrix<double,4,2> &r);
   
    /** \brief Generate squre/boundary intersection area function for ellipse
      * uses secant approximation, requires x, y to be integer.
      */
    MatrixXd
    bcellipse (MatrixXi x, MatrixXi y, double w);

    /** \brief Generate squre/boundary intersection area function for axis
      * aligned quad exact.
      */
    MatrixXd
    bcaaquad (MatrixXi x, MatrixXi y, double w);

    /** \brief Boundary box function for axis aligned quad.
      *
      * \param[in] d the boundary parameters.
      * 
      * \return [xmin, xmax; ymin, ymax] gives the axis aligned bounding box of
      * the boundary curve in the local frame XY plane.
      *
      */
    Matrix<double,2,2>
    bbaaquad(Vector2d d);

    /** \brief Generate ellipse boundary area.
      *
      * \notes see ba method for params.
      */
    double
    baellipse (Vector2d d);
      
    /** \brief Generate axis-aligned quadrilateral boundary area.
      *
      * \notes see ba method for params.
      */
    double
    baaaquad(Vector2d d);

    /** Generate samples from s to e, always at integer multiples of ss
      * s is always included (even if not an integer multiple of ss) if cs=1
      * e is always included (even if not an integer multiple of ss) if ce=1
      * s is never included (even if an integer multiple of ss) if cs=0
      * e is never included (even if an integer multiple of ss) if ce=0
      */
    Matrix<double,1,Dynamic> 
    samples(double s, double e, double ss, double cs, double ce);

    /** \brief Removes rowwise entries that equal to NaN.
      *
      * \param[in] m_in the input matrix
      *
      * \param[out] m_out the output matrix
      * \param[out] index the indices to m_in that correspond to rowwise non-NaN
      */
    void
    removeRowwiseNaN (Matrix<double,Dynamic,2> &m_in,
                      Matrix<double,Dynamic,2> &m_out,
                      std::vector<int> &index);

    /** \brief Find angle between two vectors (in rad). */
    double
    findAngle (Vector3d a, Vector3d b);

  protected:
    /** Each patch have primary and derived fields:
      * - PRIMARY FIELDS:
      *   -- id: unique id number
      *   -- name: name
      *   -- s: surface type
      *   -- b: boundary type
      *   -- d: boundary parameters
      *   -- k: curvature parameters
      *   -- r: rexp(p.r) is basis for local frame of patch
      *   -- c: origin of local frame of patch
      *
      * - DERIVED FIELDS:
      *   -- pose: patch's pose
      *   -- sk: the curvatures of the patch in local frame
      *   -- nk: the number of curvature for the patch surface
      *   -- nd: the number of boundary parameters for the patch boundary types
      *   -- rm: a 3x3 rotation matrix from local to extrinsic frame
      *   -- pm: a rigid-body homogenous transformation matrix with top three
      *          rows [p.rm, p.c] and bottom row [0 0 0 1]
      *   -- ss: pacing for axis-aligned grid sampling
      *   -- bb: [xmin, xmax; ymin, ymax] gives the axis aligned bounding box of
      *          the boundary curve in the local frame XY plane
      *   -- ba: the area inside the bounding curve in the local frame XY plane
      *   -- gd: grid decimation factor relative to ss.  Must be a positive integer.  1 gives no decimation.
      *   -- residual: patch's residual
      *   -- cos_normal_normal: cos(angle) between patch's normal in two neighborhoods
      *   -- cos_normal_g: cos(angle) between patch's normal and gravity vector (if available)
      *   -- gv: sample points on the patch
      */

    /** \brief Patch's id number. */
    int id_;

    /** \brief Patch's name. */
    string name_;

    /** \brief Patch's surface type. */
    s_type s_;

    /** \brief Patch's boundary type. */
    b_type b_;

    /** \brief Patch's boundary parameters. */
    VectorXd d_;

    /** \brief Curvature constants. */
    VectorXd k_;

    /** \brief rexp(r) is basis for local frame of patch */
    Vector3d r_;

    /** \brief origin of local frame of patch */
    Vector3d c_;
    
    
    /** \brief Rotation matrix from local to extrinsic frame. */
    Matrix3d rm_;

    /** \brief A rigid-body homogenous transformation matrix with top three
      * rows [p.rm, p.c] and bottom row [0 0 0 1].
      */
    Matrix4d pm_;

    /** \brief Patvch's pose. */
    Affine3d pose_;

    Vector3d normal_;
    
    /** \brief Standard curvatures. */
    Vector2d sk_;

    /** \brief The number of curvature parameters for the patch surface type. */
    int nk_;

    /** \brief The number of boundary parameters for the patch boundary type. */
    int nd_;

    /** \brief Spacing for axis-aligned grid sampling.  Should always be
      * positive number, o/w it is set to 1.0.
      */
    double ss_;
    
    /** \brief Grid decimation factor.  Grid samples are points on a grid with
      * spacing p.gd*p.ss that are inside or on the boundary curve.  It should
      * be positive number, o/w we take its absolute value.
      */
    int gd_;

    /** \brief The field bb = [xmin, xmax; ymin, ymax] gives the axis aligned
      * bounding box of the boundary curve in the local frame XY plane.
      */
    Matrix2d bb_;

    /** \brief Boundary area. */
    double ba_;
    
    /** \brief The set of NGx3 const pointer to point clouds.  Each point cloud represents
      * 
      * The field gv is a 1x3 cell array of NGx1 cell arrays gx, gy,
      * gz. Each entry in the latter is a column vector of sample point 
      * coordinates in order along one of the NG gridlines. Note that each 
      * gridline may have a different number of vertices.
      */
    vector<PointCloudConstPtr> gv_;

    /** \brief Patch residual. */
    double residual_;
    
    /** \brief The cos(angle) between patch's normal in two neighborhoods. */
    double cnn_;

    /** \brief The cos(angle) between patch's normal and gravity vector. */
    double cng_;
};

#endif  //#ifndef PATCH_H_
