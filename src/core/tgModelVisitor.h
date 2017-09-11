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

#ifndef TG_MODEL_VISITOR_H
#define TG_MODEL_VISITOR_H


/**
 * @file tgModelVisitor.h
 * @brief Contains the definition of interface class tgModelVisitor.
 * @author Brian Mirletz, Drew Sabelhaus
 * $Id$
 */

// Forward declarations
class tgSpringCableActuator;
class tgModel;
class tgBaseRigid;
class tgBulletGround;
class tgCompressionSpringActuator;

/**
 * Interface for ModelVisitor.
 */
class tgModelVisitor {
    
public:

  /** Virtual base classes must have a virtual destructor. */
  virtual ~tgModelVisitor() { }
    
  /**
   * Render a tgBaseRigid.
   */
  virtual void render(const tgBaseRigid& rigid) const {};
  
  /**
   * Render a tgBulletGround.
   */
  virtual void render(const tgBulletGround& ground) const {};

  /**
   * Render a tgSpringCableActuator.
   * @param[in] linearString a const reference to a tgSpringCableActuator to render
   */
  virtual void render(const tgSpringCableActuator& linearString) const {};

 /**
   * Render a tgCompressionSpringActuator.
   * @param[in] compressionSpringActuator a const reference to a tgCompressionSpringActuator to render
   */
  virtual void render(const tgCompressionSpringActuator& compressionSpringActuator) const {};
 
  /**
   * Render a tgModel.
   * @param[in] model a const reference to a tgModel to render.
   */
  virtual void render(const tgModel& m) const {};
  
  int getPass() const { return m_pass; };
  void setPass(int pass) { m_pass = pass; };
  
  int getDebugMode() const { return m_debugMode; };
  void setDebugMode(int debugMode) { m_debugMode = debugMode; };
  
  private:
  int m_pass;
  int m_debugMode;
  
};

#endif
