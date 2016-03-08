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

#ifndef MAP_PATCH_H_
#define MAP_PATCH_H_

//SPL-CPP headers
#include "patch.h"

class MapPatch
{
  /** \brief The patches in the map. They can have the following states
    * (seed, patch):
    * I. F,F: invalid
    * II. T,F: valid seed
    * III. T,T: valid patch
    *
    * There are the following transitions:
    * 1. I->II (when new seed is added)
    * 2. II->I (when seed is deleted before fitting)
    * 3. II->III (valid fitting)
    * 4. III->I (patch deleted)
    *
    * \author Dimitrios Kanoulas
    */
  public:
    MapPatch ()
    {
      // make patch invalid
      isSeedValid_ = false;
      isPatchValid_ = false;
      weight_ = 0.0;
    };

    ~MapPatch () {};

    /** \brief New operator< wrt the weight. */
    bool operator< (const MapPatch& mp) const;

    /** \brief Set seed point. */
    void setSeed (int seed);
    
    /** \brief Get seed point. */
    int getSeed ();
    
    /** \brief Set whether the seed is valid. */
    void setIsSeedValid (bool isSeedValid);
    
    /** \brief Get whether the seed is valid.*/
    bool getIsSeedValid ();

    /** \brief Set patch's id. */
    void setId (int id);
    
    /** \brief Get patch's id. */
    int getId ();
    
    /** \brief Set the patch. */
    void setP (Patch p);

    /** \brief Get the patch. */
    Patch getP ();

    /** \brief Set whether the patch is valid. */
    void setIsPatchValid (bool isPatchValid);
    
    /** \brief Get whether the patch is valid.*/
    bool getIsPatchValid ();
    
    /** \brief Set patch's r.*/
    void setPatchR (Eigen::Matrix3d m);
    
    /** \brief Set patch's c.*/
    void setPatchC (Eigen::Vector3d c);

    /** \brief Set the cell number. */
    void setCell (int cell);

    /** \brief Get the cell number. */
    int getCell ();

    /** \brief Set the weight. */
    void setWeight (double weight_);

    /** \brief Get the weight. */
    double getWeight ();

    /** \brief Get whether the map patch is valid.  This is true when all three
      * conditions are met: i) seed is valid, ii) patch isw valid, iii) vis is
      * valid.
      */
    bool getIsValid ();
    
    /** \brief Whether the map patch is invalid. */
    bool getIsInvalid ();
    
    /** \brief Makes the map patch invalid. */
    void makeInvalid ();

    /** \brief The seed is valid, but the patch is not fitted yet. */
    void makeReadyToFit ();

    /** \brief Whether the patch is ready to fit. */
    bool isReadyToFit ();

    /** \brief Makes the map patch valid. */
    void makeValid ();
  
  private:
    /** \brief The corresponding seed point id in the point cloud. */
    int seed_;
    
    /** \brief Whether the seed is valid. */
    bool isSeedValid_;

    /** \brief Map patch's id (usually the same as the patch's id). */
    int id_;

    /** \brief The patch. */
    Patch p_;

    /** \brief Whether the map patch is valid. */
    bool isPatchValid_;

    /** \brief Current cell number of the map patch. */
    int cell_;

    /** \brief The weight represents the importance of the map patch wrt other
      * patches and is the value for sorting a list of map patches.
      */
    double weight_;
};

#endif // MAP_PATCH_H_
