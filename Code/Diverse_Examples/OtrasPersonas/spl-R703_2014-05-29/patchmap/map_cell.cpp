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

#include "map_cell.h"
#include <stdlib.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
bool
MapCell::operator< (const MapCell& mapcell) const
{
  return (weight_ < mapcell.weight_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setId (int id)
{
  this->id_ = id;
}

////////////////////////////////////////////////////////////////////////////////
int
MapCell::getId ()
{
  return (this->id_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setW (float w)
{
  this->w_ = w;
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setH (float h)
{
  this->h_ = h;
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setMinCol (float min_col)
{
  this->min_col_ = min_col;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getMinCol ()
{
  return (this->min_col_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setMaxCol (float max_col)
{
  this->max_col_ = max_col;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getMaxCol ()
{
  return (this->max_col_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setMinRow (float min_row)
{
  this->min_row_ = min_row;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getMinRow ()
{
  return (this->min_row_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setMaxRow (float max_row)
{
  this->max_row_ = max_row;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getMaxRow ()
{
  return (this->max_row_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setMaxPatchesNum (int max_patches_num) 
{
  this->max_patches_num_ = max_patches_num;
}

////////////////////////////////////////////////////////////////////////////////
int
MapCell::getMaxPatchesNum ()
{
  return (this->max_patches_num_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setCurPatchesNum (int cur_patches_num) 
{
  this->cur_patches_num_ = cur_patches_num;
}

////////////////////////////////////////////////////////////////////////////////
int
MapCell::getCurPatchesNum ()
{
  return (this->cur_patches_num_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::incCurPatchesNum () 
{
  this->cur_patches_num_ = this->cur_patches_num_ + 1;
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::decCurPatchesNum () 
{
  this->cur_patches_num_ = this->cur_patches_num_ - 1;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getCenterCol ()
{
  return ((getMaxCol()+getMinCol())/2.0);
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getCenterRow ()
{
  return ((getMaxRow()+getMinRow())/2.0);
}

////////////////////////////////////////////////////////////////////////////////
int
MapCell::getRandomSeed ()
{
  int id = rand()%point_ids_.size();
  
  if (point_ids_.size())
    return (point_ids_[id]);
  else
    return (-1);
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::resetPointIds ()
{
  this->point_ids_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::addPointId (int id)
{
  this->point_ids_.push_back (id);
}

////////////////////////////////////////////////////////////////////////////////
int
MapCell::getNumberOfPoints ()
{
  return (this->point_ids_.size ());
}

////////////////////////////////////////////////////////////////////////////////
void
MapCell::setWeight (float weight)
{
  this->weight_ = weight;
}

////////////////////////////////////////////////////////////////////////////////
float
MapCell::getWeight ()
{
  return (this->weight_);
}
