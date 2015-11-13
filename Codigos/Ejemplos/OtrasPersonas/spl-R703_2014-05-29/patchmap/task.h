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

#ifndef TASK_H_
#define TASK_H_

// STD headers
#include <math.h>

class Task
{
  /** \brief The task with respect to patch characteristics.
   *
   * \author Dimitrios Kanoulas
   */

  public:
    /** \brief Default constructor. */
    Task ()
    : min_patch_distance (0.0)
    , max_patch_distance (0.0)
    , min_patch_curvature (0.0)
    , max_patch_curvature (0.0)
    , min_patch_size (0.0)
    , max_patch_size (0.0)
    , min_normal_gravity_angle (0.0)
    , max_normal_gravity_angle (0.0)
    { };

    /** \brief Constructor.
      *
      * \param[in] min_pd_arg min patch distance
      * \param[in] max_pd_arg max patch distance
      * \param[in] min_pc_arg min patch curvature
      * \param[in] max_pc_arg max patch curvature
      * \param[in] min_ps_arg min patch size
      * \param[in] max_ps_arg max patch size
      *
      */
       Task (double min_pd_arg, double max_pd_arg,
             double min_pc_arg, double max_pc_arg,
             double min_ps_arg, double max_ps_arg,
             double min_nga_arg, double mac_nga_arg)
       : min_patch_distance (min_pd_arg)
       , max_patch_distance (max_pd_arg)
       , min_patch_curvature (min_pc_arg)
       , max_patch_curvature (max_pc_arg)
       , min_patch_size (min_ps_arg)
       , max_patch_size (max_ps_arg)
       , min_normal_gravity_angle (min_nga_arg)
       , max_normal_gravity_angle (mac_nga_arg)
       { };
    
    /** \brief Destructor. */
    virtual
    ~Task () {};

    /** \brief Set min patch distance.  If negtive then the absolute value is
      * stored.
      */
    inline void
    setMinPatchDistance (double min_pd)
    {
      if (min_pd>0.0)
        this->min_patch_distance = min_pd;
      else
        this->min_patch_distance = fabs(min_pd);
    }
    
    /** \brief Set max patch distance.  If negtive then the absolute value is
      * stored.
      */
    inline void
    setMaxPatchDistance (double max_pd)
    {
      if (max_pd>0.0)
        this->max_patch_distance = max_pd;
      else
        this->max_patch_distance = fabs(max_pd);
    }
    
    /** \brief Get max patch distance.
      *
      * \param[out] max_patch_distance
      */
    inline double
    getMaxPatchDistance ()
    {
      return (this->max_patch_distance);
    }

    /** \brief Set min patch curvature. */
    inline void
    setMinPatchCurvature (double min_pc)
    {
      this->min_patch_curvature = min_pc;
    }
    
    /** \brief Set max patch curvature. */
    inline void
    setMaxPatchCurvature (double max_pc)
    {
      this->max_patch_curvature = max_pc;
    }
    
    /** \brief Set min patch size.  If negtive then the absolute value is
      * stored.
      */
    inline void
    setMinPatchSize (double min_ps)
    {
      if (min_ps > 0.0)
        this->min_patch_size = min_ps;
      else
        this->min_patch_size = fabs(min_ps);
    }
    
    /** \brief Set max patch size.  If negtive then the absolute value is
      * stored.
      */
    inline void
    setMaxPatchSize (double max_ps)
    {
      if (max_ps > 0.0)
        this->max_patch_size = max_ps;
      else
        this->max_patch_size = fabs(max_ps);
    }

    /** \brief Get max patch size. */
    inline double
    getMaxPatchSize ()
    {
      return (this->max_patch_size);
    }

  protected:
    /** \brief min patch distance from the camera center. */
    double min_patch_distance;

    /** \brief max patch distance from the camera center. */
    double max_patch_distance;

    /** \brief min patch curvature. */
    double min_patch_curvature;

    /** \brief max patch curvature. */
    double max_patch_curvature;

    /** \brief min patch size (in m). */
    double min_patch_size;

    /** \brief max patch size (in m). */
    double max_patch_size;

    /** \brief min angle (radians) with respect to the gravity vector. */
    double min_normal_gravity_angle;

    /** \brief max angle (radians) with respect to the gravity vector. */
    double max_normal_gravity_angle;
};

#endif // TASK_H_
