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


#ifndef TG_WORLD_H
#define TG_WORLD_H

/**
 * @file tgWorld.h
 * @brief Contains the definition of class tgWorld
 * $Id$
 */

// Forward declarations
class tgWorldImpl;
class tgGround;

/**
 * Represents the world in which the Tensegrities operate, including
 * terrain, gravity and atmospheric conditions.
 */
class tgWorld
{
public:

  /**
   * World configuration information used by tgWorld constructors.
   * This is Plain Old Data.
   */
  struct Config
  {
	Config(double g = 9.81, double ws = 1000);
    /**
     * Gravitational acceleration.
     * The units are application depenent.
     * Whether negative values are accepted is application dependent.
     */
    double gravity;
    /**
     * Size of the world for broadphase collision detection. Indicates
     * the length of one side of the detection cube. Must be positive.
     */
    double worldSize;
  };

  /** Construct with the default configuration. */
  tgWorld();
    
  /**
   * Construct with a supplied configuration.
   * @param[in] config a tgWorld::Config
   */
  tgWorld(const Config& config);
  
  /**
   * Construct with a supplied configuration and a supplied ground
   * @param[in] config a tgWorld::Config
   * @param[in] ground a tgGround, stored in pointer m_pGround
   * @todo can we make the ground const?
   */
  tgWorld(const Config& config, tgGround* ground);

  /** Delete the implementation. */
  ~tgWorld();

  /** Replace the implementation. */
  void reset();

  /**
   * Replace the implementation with a new config.
   * @param[in] config configuration POD
   */
  void reset(const Config& config);

  /**
   * Replace the implementation with a new ground.
   * @param[in] ground the new ground
   */
  void reset(tgGround* ground);
    
  /**
   * Advance the simulation.
   * @param[in] dt the number of seconds since the previous call;
   * std::invalid_argument is thrown if dt is not positive 
   */
  void step(double dt) const;

  /**
   * Return a pointer to the implementation.
   * @return a pointer to the implementation; may be NULL.
   * Consider making this private, and making its clients friends.
   */
  tgWorldImpl& implementation() const
  {
    return *m_pImpl;
  }
 
private:

  /** Integrity predicate */
  bool invariant() const;

 private:

  /**
   * The configuration data passed at construction or upon reset.
   */
  Config m_config;
  
  /** Implementation of the ground, such as a box, hills or ramp */
  tgGround* m_pGround;

  /** The implementation of the tgWorld. */
  tgWorldImpl * m_pImpl;
};

#endif //TG_BULLET_WORLD_H
