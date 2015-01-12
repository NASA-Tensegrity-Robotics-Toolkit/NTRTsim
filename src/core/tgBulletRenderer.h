/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file tgBulletRenderer.h
 * @brief Contains the definition of concrete class tgBulletRenderer
 * @author Brian Tietz
 * $Id$
 */

#ifndef TG_BULLET_RENDERER_H
#define TG_BULLET_RENDERER_H

// This application
#include "tgModelVisitor.h"

// Forward declarations
class tgSpringCableActuator;
class tgModel;
class tgRod;
class tgWorld;

/**
 * A concrete tgRenderer for Bullet Physics.
 */
class tgBulletRenderer : public tgModelVisitor
{
public:

  /**
   * The only constructor.
   * @param[in,out] world a reference to the tgWorld being rendered
   */
  tgBulletRenderer(const tgWorld& world);

  /**
   * Render a tgSpringCableActuator.
   * @param[in] linearString a const reference to a tgSpringCableActuator to render
   */
  virtual void render(const tgSpringCableActuator& mSCA) const;
 
  /**
   * Render a tgRod.
   * @param[in] rod a const reference to a tgRod to render
   */
  virtual void render(const tgRod& rod) const;
        
  /**
   * Render a tgModel.
   * @param[in] model a const reference to a tgModel to render.
   */
  virtual void render(const tgModel& model) const;

private:

  /**
   * A reference to the tgWorld being rendered.
   */
  const tgWorld& m_world;
};

#endif
