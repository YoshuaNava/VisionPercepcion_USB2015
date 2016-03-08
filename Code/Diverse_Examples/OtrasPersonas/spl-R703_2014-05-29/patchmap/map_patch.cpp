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

#include "map_patch.h"

////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::operator< (const MapPatch& mp) const
{
  return (weight_ < mp.weight_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setSeed (int seed)
{
  this->seed_ = seed;
}

////////////////////////////////////////////////////////////////////////////////
int
MapPatch::getSeed ()
{
  return (this->seed_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setIsSeedValid (bool isSeedValid)
{
  this->isSeedValid_ = isSeedValid;
}

////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::getIsSeedValid ()
{
  return (this->isSeedValid_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setId (int id)
{
  this->id_ = id;
}

////////////////////////////////////////////////////////////////////////////////
int
MapPatch::getId ()
{
  return (this->id_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setP (Patch p)
{
  this->p_ = p;
}

////////////////////////////////////////////////////////////////////////////////
Patch
MapPatch::getP ()
{
  return (this->p_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setIsPatchValid (bool isPatchValid)
{
  this->isPatchValid_ = isPatchValid;
}

////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::getIsPatchValid ()
{
  return (this->isPatchValid_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setPatchR (Eigen::Matrix3d m)
{
  this->p_.setR (this->p_.rlog(m));
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setPatchC (Eigen::Vector3d c)
{
  this->p_.setC (c);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setCell (int cell)
{
  this->cell_ = cell;
}

////////////////////////////////////////////////////////////////////////////////
int
MapPatch::getCell ()
{
  return (this->cell_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::setWeight (double weight)
{
  this->weight_ = weight;
}

////////////////////////////////////////////////////////////////////////////////
double
MapPatch::getWeight ()
{
  return (this->weight_);
}
  
////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::getIsValid ()
{
  return (isSeedValid_ && isPatchValid_);
}

////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::getIsInvalid ()
{
  return (!isSeedValid_ && !isPatchValid_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::makeInvalid ()
{
  // F,F
  setIsSeedValid (false);
  setIsPatchValid (false);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::makeReadyToFit ()
{
  // T,F
  setIsSeedValid (true);
  setIsPatchValid (false);
}

////////////////////////////////////////////////////////////////////////////////
bool
MapPatch::isReadyToFit ()
{
  return (isSeedValid_ && !isPatchValid_);
}

////////////////////////////////////////////////////////////////////////////////
void
MapPatch::makeValid ()
{
  // T,T
  setIsSeedValid (true);
  setIsPatchValid (true);
}
