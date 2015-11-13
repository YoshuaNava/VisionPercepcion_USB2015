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

#ifndef FILTER_NMS_H_
#define FILTER_NMS_H_

// PCL libraries
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

class NMS
{
  /** \brief NMS filter computes the Non-Maximum Suppression on a 2D image.  
    * Instead of using the efficient computation introduced in [2] we prefered
    * the one introduced in [1], which is less complex in implementation but
    * achieves similar performance.  The algorithm uses different solutions for:
    * -- 3x3-neighborhood: Scan-Line
    * -- 9x9-neighborhood: Scan-Line with Spiral Indexing
    * -- otherwise: Quarter-Block Partitioning
    *
    * [1] Pham, "Non-Maximum Suppression using Fewer than two Comparisons per 
    *     Pixel", ACIVS 2010.
    * [2] Neubeck, Gool, "Efficient Non-Maximum Suppression", Pattern 
    *     Recognition 2006.
    *
    * \note Note that the implementation should take care of the NaN entries.
    *
    * \author Dimitrios Kanoulas
    */
  public:
    /** \brief Empty contructor. */
    NMS () :
      im_ (),
      n_ (0),
      id_om_ (new vector<int>),
      lmn_ (0)
    {};

    /** \brief Empty destructor. */
    virtual
    ~NMS()
    {};
    
    /** \brief Set input matrix. */
    inline void
    setIM (MatrixXd &im)
    {
      //set input image
      this->im_ = im;
      
      //set input image size
      h_ = im_.rows();
      w_ = im_.cols();
      
      //resize output mask image
      om_.resize(h_, w_);
    }
    
    /** \brief Set size of the window. */
    inline void
    setN (int n)
    {
      this->n_ = n;
    }

    /** \brief Get the indices to local maxima. */
    inline boost::shared_ptr<vector<int> >
    getIdOm ()
    {
      return (this->id_om_);
    }

    /** \brief Apply nms to a matrix M.
      *
      * \param[out] number of local maxima
      */
    int
    applyNMS();

    /** \brief Scan-line algorithm for 3x3 neighborhood NMS.
      *
      * \param[in] im_ the input matrix
      * 
      * \param[out] om_ the output mask matrix
      * \param[out] id_om_ the indices to local maxima
      *
      * \return number of local maxima
      */
    int
    nms3x3 (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_);

    /** \brief Finding the spiral index. */
    void
    spiralindex (VectorXi &r, VectorXi &c);
    
    /** \brief Scan-line using the spiral indexing algorithm for (2n+1)x(2n+1)
      * neighborhood NMS.
      *
      * \param[in] im_ the input matrix
      * 
      * \param[out] om_ the output mask
      * \param[out] id_om_ the indices to local maxima
      *
      * \return number of local maxima
      */
    int
    nmsScanline (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_);

    /** \brief Quarter-block partitioning algorithm for (2n+1)x(2n+1)
      * neighborhood NMS.
      *
      * \param[in] im_ the input matrix
      * 
      * \param[out] om_ the output mask
      * \param[out] id_om_ the indices to local maxima
      *
      * \return number of local maxima
      */
    int
    nmsQuarterblock (MatrixXd &im_, MatrixXd &om_, vector<int> &id_om_);

  protected:
    /** \brief Input 2D image. */
    MatrixXd im_;
    
    /** \brief Input neighborhood window size: (2n_+1)x(2n_+1). */
    int n_;
    
    /** \brief Input image im_ sizes. */
    int h_, w_;
    
    /** \brief Output image mask of the same size as the input image matrix. */
    MatrixXd om_;
    
    /** \brief Output row-wised indices of local maxima. */
    boost::shared_ptr<vector<int> > id_om_;
    
    /** \brief Output number of local maxima. */
    int lmn_;
};

#endif //#ifndef FILTER_NMS_H
