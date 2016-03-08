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

#ifndef MAP_CELL_H_
#define MAP_CELL_H_

#include "map_patch.h"

class MapCell
{
  /** \brief The grid cells are row-wise ordered.
    *
    * \author Dimitrios Kanoulas
    */
  public:
    MapCell () {}; 

    ~MapCell () {};

    /** \brief New operator< wrt the weight. */
    bool operator< (const MapCell& mapcell) const;

    /** \brief Set patch's id. */
    void setId (int id);

    /** \brief Get patch's id. */
    int getId ();
    
    /** \brief Set cloud's width.  */
    void setW (float w);
    
    /** \brief Set cloud's height.  */
    void setH (float h);

    /** \brief Set the min col dimension. */
    void setMinCol (float min_col);
    
    /** \brief Get the min col dimension. */
    float getMinCol ();
    
    /** \brief Set the max col dimension. */
    void setMaxCol (float max_col);
    
    /** \brief Get the max col dimension. */
    float getMaxCol ();
    
    /** \brief Set the min row dimension. */
    void setMinRow (float min_row);
    
    /** \brief Get the min row dimension. */
    float getMinRow ();
    
    /** \brief Set the max row dimension. */
    void setMaxRow (float max_row);
    
    /** \brief Get the max row dimension. */
    float getMaxRow ();
    
    /** \brief Set the max number of the patches that could be in the cell. */
    void setMaxPatchesNum (int max_patches_num);
    
    /** \brief Get the max number of the patches that could be in the cell. */
    int getMaxPatchesNum ();

    /** \brief Set the number of the patches in the cell. */
    void setCurPatchesNum (int cur_patches_num);
    
    /** \brief Get the number of the patches in the cell. */
    int getCurPatchesNum ();
    
    /** \brief Increase by 1 the number of the patches in the cell. */
    void incCurPatchesNum ();

    /** \brief decrease by 1 the number of the patches in the cell. */
    void decCurPatchesNum ();
    
    /** \brief Return the center col number. */
    float getCenterCol ();

    /** \brief Return the center row number. */
    float getCenterRow ();

    /** \brief Return a random seed index in the cloud. */
    int getRandomSeed ();

    /** \brief Reset the points IDs vector. */
    void resetPointIds ();

    /** \brief Add a point ID in the cell. */
    void addPointId (int id);
  
    /** \brief Get number of points. */
    int getNumberOfPoints ();

    /** \brief Set the weight. */
    void setWeight (float weight);
    
    /** \brief Get the weight. */
    float getWeight ();

  private:
    /** \brief Grid cell's id number. */
    int id_;

    /** \brief The width of the cloud that grid is created for. */
    float w_;

    /** \brief The height of the cloud that grid is created for. */
    float h_;
    
    /** \brief Min col dimension in the pixel grid. */
    float min_col_;
    
    /** \brief Max col dimension in the pixel grid (not inlcuded). */
    float max_col_;
    
    /** \brief Min row dimension in the pixel grid. */
    float min_row_;
    
    /** \brief Max row dimension in the pixel grid (not included). */
    float max_row_;
   
    /** \brief The max number of the patches in the cell. */
    int max_patches_num_;

    /** \brief The number of the patches in the cell. */
    int cur_patches_num_;

    /** \brief Point IDs in this cell. */
    vector<int> point_ids_;

    /** \brief The weight of the cell wrt the task.  The bigger the weight the
      * more important the cell is for the task.
      */
    float weight_;
};

#endif //MAP_CELL_H_
