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
 * @file ForcePlateModel.cpp
 * @brief Contains the implementation of class Force Plate Model.
 * @author Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This module
#include "ForcePlateModel.h"
// This library
#include "core/tgUnidirectionalCompressionSpringActuator.h"
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgUnidirectionalCompressionSpringActuatorInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
//#include "LinearMath/btVector3.h" //included in this header.
// The C++ Standard Library
#include <stdexcept>
#include <iostream>

/**
 * Constructor for the Config struct. Assigns variables.
 * The constructorAux function performs the checking.
 * The syntax "Config::Config" means the constructor for Config
 * that's present in the Config struct. It's similar to
 * the syntax ForcePlateModel::ForcePlateModel, except that
 * Config is part of ForcePlateModel so that extra namespace
 * must be specified too.
 */
ForcePlateModel::Config::Config(double length,
				double width,
				double height,
				double thickness,
				double platethickness,
				double wallGap,
				double bottomGap,
				double lateralStiffness,
				double verticalStiffness,
				double lateralDamping,
				double verticalDamping,
				double lateralRestLength,
				double verticalRestLength,
				double springAnchorOffset) :
  L(length),
  w(width),
  h(height),
  t(thickness),
  pt(platethickness),
  wgap(wallGap),
  bgap(bottomGap),
  latK(lateralStiffness),
  vertK(verticalStiffness),
  latD(lateralDamping),
  vertD(verticalDamping),
  latRL(lateralRestLength),
  vertRL(verticalRestLength),
  sOff(springAnchorOffset)
{
  // do nothing.
}

/**
 * This function calculates all the locations of the node positions for the 
 * force plate. This requires that m_config be populated first.
 */
void ForcePlateModel::calculatePlateNodePositions() {
  // check that m_config is populated
  assert(invariant());
  // Calculate each of the locations of the corners of the force plate
  // box. These are all assuming the plate is centered at
  // (0, h - pt/2, 0).
  // All these positions are (X, Y, Z) where Y is the vertical coordinate.

  // Example: a1 = ( -w/2 + t + wgap,   h - pt,   -L/2 + t + wgap).
  a1 = btVector3( -(m_config.w/2) + m_config.t + m_config.wgap,
		  m_config.h - m_config.pt,
		  -(m_config.L/2) + m_config.t + m_config.wgap);
  // a2 is translated up to the top of the box, e.g. Y == h.
  a2 = a1 + btVector3( 0, m_config.pt, 0);
  
  b1 = btVector3( (m_config.w/2) - m_config.t - m_config.wgap,
		  m_config.h - m_config.pt,
		  -(m_config.L/2) + m_config.t + m_config.wgap);
  b2 = b1 + btVector3( 0, m_config.pt, 0);

  c1 = btVector3( (m_config.w/2) - m_config.t - m_config.wgap,
		  m_config.h - m_config.pt,
		  (m_config.L/2) - m_config.t - m_config.wgap);
  c2 = c1 + btVector3( 0, m_config.pt, 0);

  d1 = btVector3( -(m_config.w/2) + m_config.t + m_config.wgap,
		  m_config.h - m_config.pt,
		  (m_config.L/2) - m_config.t - m_config.wgap);
  d2 = d1 + btVector3( 0, m_config.pt, 0);

  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated plate node positions:" << std::endl;
    std::cout << "a1, a2: " << a1 << "   " << a2 << std::endl;
    std::cout << "b1, b2: " << b1 << "   " << b2 << std::endl;
    std::cout << "c1, c2: " << c1 << "   " << c2 << std::endl;
    std::cout << "d1, d2: " << d1 << "   " << d2 << std::endl << std::endl;
  }

  // Now calculate the positions of the force anchor points on the force plate.
  // See this file's .h for more information about where each of these
  // points are located.
  
  s_ab = btVector3( a1.x() + m_config.sOff,
		    m_config.h - (m_config.pt/2),
		    a1.z());
  
  s_ba = btVector3( b1.x() - m_config.sOff,
		    m_config.h - (m_config.pt/2),
		    b1.z());

  s_bc = btVector3( b1.x(),
		    m_config.h - (m_config.pt/2),
		    b1.z() + m_config.sOff);

  s_cb = btVector3( c1.x(),
		    m_config.h - (m_config.pt/2),
		    c1.z() - m_config.sOff);

  s_cd = btVector3( c1.x() - m_config.sOff,
		    m_config.h - (m_config.pt/2),
		    c1.z());

  s_dc = btVector3( d1.x() + m_config.sOff,
		    m_config.h - (m_config.pt/2),
		    b1.z());

  s_da = btVector3( d1.x(),
		    m_config.h - (m_config.pt/2),
		    d1.z() - m_config.sOff);

  s_ad = btVector3( a1.x(),
		    m_config.h - (m_config.pt/2),
		    a1.z() + m_config.sOff);
  
  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated lateral spring anchor positions on the force plate itself:" << std::endl;
    std::cout << "s_ab, s_ba: " << s_ab << "   " << s_ba << std::endl;
    std::cout << "s_bc, s_cb: " << s_bc << "   " << s_cb << std::endl;
    std::cout << "s_cd, s_dc: " << s_cd << "   " << s_dc << std::endl;
    std::cout << "s_da, s_ad: " << s_da << "   " << s_ad << std::endl << std::endl;
  }

  // Finally, calculate the spring anchor locations for the vertical springs,
  // located on the bottom side of the plate.
  // As always, see the .h file for more information.

  s_bot_a = btVector3( s_ab.x(), a1.y(), s_ad.z() );
  s_bot_b = btVector3( s_ba.x(), b1.y(), s_bc.z() );
  s_bot_c = btVector3( s_cd.x(), c1.y(), s_cb.z() );
  s_bot_d = btVector3( s_dc.x(), d1.y(), s_da.z() );

  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated vertical spring anchor positions on the bottom of the force plate:" << std::endl;
    std::cout << "s_bot_a: " << s_bot_a << std::endl;
    std::cout << "s_bot_b: " << s_bot_b << std::endl;
    std::cout << "s_bot_c: " << s_bot_c << std::endl;
    std::cout << "s_bot_d: " << s_bot_d << std::endl << std::endl;
  }
}

/** 
 * helper function for the constructor(s).
 * performs some checks on the parameters that are passed in.
 * @TO-DO: implement this.
 */
void ForcePlateModel::constructorAux()
{
  // DEBUGGING:
  if( m_debugging ) {
    std::cout << "Constructor for ForcePlateModel." << std::endl;
  }

  // Do a check on all the parameters.
  if( m_config.L <= 0.0 ) {
    throw std::invalid_argument("Length (L) must be greater than zero.");
  }
  if( m_config.w <= 0.0 ) {
    throw std::invalid_argument("Width (w) must be greater than zero.");
  }
  if( m_config.h <= 0.0 ) {
    throw std::invalid_argument("Height (h) must be greater than zero.");
  }
  if( m_config.t <= 0.0 ) {
    throw std::invalid_argument("Wall thickness (t) must be greater than zero.");
  }
  if( m_config.pt <= 0.0 ) {
    throw std::invalid_argument("Plate thickness (pt) must be greater than zero.");
  }
  if( m_config.wgap <= 0.0 ) {
    throw std::invalid_argument("Wall gap (wgap) must be greater than zero.");
  }
  if( m_config.wgap >= (0.5 * m_config.w) - m_config.t ) {
    // The force plate cannot have zero width.
    throw std::invalid_argument("Error, force plate would be zero width. Adjust t, w, and/or wgap.");
  }
  if( m_config.bgap <= 0.0 ) {
    throw std::invalid_argument("Bottom gap (bgap) must be greater than zero.");
  }
  if( m_config.bgap >= m_config.h - m_config.pt ) {
    // There must be a bottom surface to the force plate housing.
    throw std::invalid_argument("Error, plate thickness and bottom gap would cut through the bottom of the housing. Adjust pt, h, and/or bgap.");
  }
  if( m_config.latK <= 0.0 ){
    throw std::invalid_argument("Lateral spring constant (latK) must be positive.");
  }
  if( m_config.vertK <= 0.0 ){
    throw std::invalid_argument("Vertical spring constant (vertK) must be positive.");
  }
  if( m_config.latD < 0.0 ) {
    throw std::invalid_argument("Lateral damping constant (latD) must be nonnegative.");
  }
  if( m_config.vertD < 0.0 ) {
    throw std::invalid_argument("Vertical damping constant (vertD) must be nonnegative.");
  }
  if( m_config.latRL <= 0.0 ) {
    throw std::invalid_argument("Lateral spring rest length (latRL) must be positive.");
  }
  if( m_config.vertRL <= 0.0 ) {
    throw std::invalid_argument("Vertical spring rest length (vertRL) must be positive.");
  }
  if( m_config.sOff <= 0.0 ){
    throw std::invalid_argument("Spring anchor offset (sOff) must be positive.");
  }
  // Note that since we're using unidirectional compression springs that are
  // attached at the free end, the spring will provide force whether or not
  // its rest length is greater or less than wgap or bgap (respectively), so there
  // is no need to check things like latRL < wgap, for example.

  // DEBUGGING: output the config variables passed in.
  if( m_debugging) {
    std::cout << std::endl << "Config variables passed in to ForcePlateModel: "
	      << std::endl;
    std::cout << "L: " <<  m_config.L << std::endl;
    std::cout << "w: " <<  m_config.w << std::endl;
    std::cout << "h: " <<  m_config.h << std::endl;
    std::cout << "t: " <<  m_config.t << std::endl;
    std::cout << "pt: " <<  m_config.pt << std::endl;
    std::cout << "wgap: " <<  m_config.wgap << std::endl;
    std::cout << "bgap: " <<  m_config.bgap << std::endl;
    std::cout << "latK: " <<  m_config.latK << std::endl;
    std::cout << "vertK: " <<  m_config.vertK << std::endl;
    std::cout << "latD: " <<  m_config.latD << std::endl;
    std::cout << "vertD: " <<  m_config.vertD << std::endl;
    std::cout << "latRL " <<  m_config.latRL << std::endl;
    std::cout << "vertRL: " <<  m_config.vertRL << std::endl;
    std::cout << "sOff: " << m_config.sOff << std::endl << std::endl;
  }

  // When the force plate is constructed, the node positions should
  // be calculated.
  // Creation of the actual NTRT objects happens in setup, though.
  calculatePlateNodePositions();
}

/**
 * Constructor: assigns some variables and that's it.
 * All the good stuff happens when the 'Info' classes
 * are passed to the tgBuilders and whatnot.
 */
ForcePlateModel::ForcePlateModel(const ForcePlateModel::Config& config,
				 btVector3& location) :
  tgModel(),
  m_config(config),
  m_location(location),
  m_debugging(false)
{
  // Call the constructor helper that will do all the checks on
  // these variables.
  constructorAux();
}

/**
 * Constructor with debugging flag passed in.
 */
ForcePlateModel::ForcePlateModel(const ForcePlateModel::Config& config,
				 btVector3& location, bool debugging) :
  tgModel(),
  m_config(config),
  m_location(location),
  m_debugging(debugging)
{
  // Call the constructor helper that will do all the checks on
  // these variables.
  constructorAux();
}

// Delete is handled by teardown.
ForcePlateModel::~ForcePlateModel()
{ 
}

// a helper function to add a bunch of nodes
void ForcePlateModel::addNodes(tgStructure& s)
{
  /*
  s.addNode(0, 0, 0);              // 0, origin
  s.addNode(0, 2 * c.boxLength, 0);      // 1, top of box 1
  s.addNode(0, 4 * c.boxLength, 0);  // 2, bottom of box 2
  s.addNode(0, 5 * c.boxLength, 0);  // 3, top of box 3
  */
}

// helper function to tag two sets of nodes as boxes
void ForcePlateModel::addBoxes(tgStructure& s)
{
  //s.addPair( 0,  1, "box");
  //s.addPair( 2,  3, "box");
}

// helper function to add our single compression spring actuator
void ForcePlateModel::addSprings(tgStructure& s)
{
  // spring is vertical between top of box 1 and bottom of box 2.
  //s.addPair(1, 2,  "compressionSpring");
  //s.addPair(1, 2,  "basicActuator");
}

// Finally, create the model!
void ForcePlateModel::setup(tgWorld& world)
{

  /*
    // config struct for the rods
    const tgBox::Config boxConfig(c.boxWidth, c.boxHeight, c.density, 
				  c.friction, c.rollFriction, c.restitution);

    // config struct for the compression spring
    //tgCompressionSpringActuator::Config compressionSpringConfig(c.isFreeEndAttached,
    //				c.stiffness, c.damping, c.springRestLength,
    //				c.moveCablePointAToEdge, c.moveCablePointBToEdge);
    tgUnidirectionalCompressionSpringActuator::Config compressionSpringConfig(
				c.isFreeEndAttached, c.stiffness, c.damping,
				c.springRestLength, c.moveCablePointAToEdge,
				c.moveCablePointBToEdge, c.direction);

    //tgBasicActuator::Config basActConfig(c.stiffness, c.damping, c.pretension,
    //					 c.hist, c.maxTens, c.targetVelocity);

    #if (1)
    std::cout << "TwoBoxesModel::setup. Direction is: ";
    std::cout << "(" << c.direction->x() << ",";
    std::cout << c.direction->y() << ",";
    std::cout << c.direction->z() << ")" << std::endl;
    #endif    
    
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addBoxes(s);
    addActuators(s);


    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("box", new tgBoxInfo(boxConfig));
    //spec.addBuilder("compressionSpring", new tgCompressionSpringActuatorInfo(compressionSpringConfig));
    spec.addBuilder("compressionSpring", new tgUnidirectionalCompressionSpringActuatorInfo(compressionSpringConfig));
    //spec.addBuilder("basicActuator", new tgBasicActuatorInfo(basActConfig));

    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgCompressionSpringActuator> (getDescendants());
    //allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);

  */
}

void ForcePlateModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void ForcePlateModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

/*
const std::vector<tgCompressionSpringActuator*>& TwoBoxesModel::getAllActuators() const
//const std::vector<tgBasicActuator*>& TwoBoxesModel::getAllActuators() const
{
    return allActuators;
}
*/
    
void ForcePlateModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

/**
 * Checks to make sure the member variables are all declared and valid.
 */
bool ForcePlateModel::invariant() const
{
  // a simple check: if one of the member variables in Config is bad,
  // then the m_config struct is most likely also bad.
  return (m_config.h > 0.0);
}
