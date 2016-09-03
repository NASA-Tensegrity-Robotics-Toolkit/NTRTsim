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
#include "core/tgBasicActuator.h"
#include "core/tgBox.h"
#include "core/tgBoxMoreAnchors.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgUnidirectionalCompressionSpringActuatorInfo.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgBoxMoreAnchorsInfo.h"
#include "tgcreator/tgRodInfo.h"
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
				double massPlate,
				double massHousing,
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
  mP(massPlate),
  mH(massHousing),
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
  // Note that tgNode is a subclass of btVector3, so has those constructors
  // and functions.

  // Example: a1 = ( -w/2 + t + wgap,   h - pt,   -L/2 + t + wgap).
  a1 = tgNode( -(m_config.w/2) + m_config.t + m_config.wgap,
	       m_config.h - m_config.pt,
	       -(m_config.L/2) + m_config.t + m_config.wgap, "a1");
  // a2 is translated up to the top of the box, e.g. Y == h.
  a2 = a1 + tgNode( 0, m_config.pt, 0);
  // add the tag to the tgNode, since the "+" operator doesn't include tags.
  a2.addTags("a2");
  
  b1 = tgNode( (m_config.w/2) - m_config.t - m_config.wgap,
	       m_config.h - m_config.pt,
	       -(m_config.L/2) + m_config.t + m_config.wgap, "b1");
  b2 = b1 + tgNode( 0, m_config.pt, 0);
  b2.addTags("b2");

  c1 = tgNode( (m_config.w/2) - m_config.t - m_config.wgap,
	       m_config.h - m_config.pt,
	       (m_config.L/2) - m_config.t - m_config.wgap, "c1");
  c2 = c1 + tgNode( 0, m_config.pt, 0);
  c2.addTags("c2");

  d1 = tgNode( -(m_config.w/2) + m_config.t + m_config.wgap,
	       m_config.h - m_config.pt,
	       (m_config.L/2) - m_config.t - m_config.wgap, "d1");
  d2 = d1 + tgNode( 0, m_config.pt, 0);
  d2.addTags("d2");

  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated plate node positions:" << std::endl;
    std::cout << "a1, a2: " << a1 << "   " << a2 << std::endl;
    std::cout << "b1, b2: " << b1 << "   " << b2 << std::endl;
    std::cout << "c1, c2: " << c1 << "   " << c2 << std::endl;
    std::cout << "d1, d2: " << d1 << "   " << d2 << std::endl << std::endl;
  }

  // Now calculate the positions of the spring anchor points on the force plate.
  // See this file's .h for more information about where each of these
  // points are located.
  
  s_ab = tgNode( a1.x() + m_config.sOff,
		 m_config.h - (m_config.pt/2),
		 a1.z(),
		 "s_ab");
  
  s_ba = tgNode( b1.x() - m_config.sOff,
		 m_config.h - (m_config.pt/2),
		 b1.z(),
		 "s_ba");

  s_bc = tgNode( b1.x(),
		 m_config.h - (m_config.pt/2),
		 b1.z() + m_config.sOff,
		 "s_bc");

  s_cb = tgNode( c1.x(),
		 m_config.h - (m_config.pt/2),
		 c1.z() - m_config.sOff,
		 "s_cb");

  s_cd = tgNode( c1.x() - m_config.sOff,
		 m_config.h - (m_config.pt/2),
		 c1.z(),
		 "s_cd");

  s_dc = tgNode( d1.x() + m_config.sOff,
		 m_config.h - (m_config.pt/2),
		 d1.z(),
		 "s_dc");

  s_da = tgNode( d1.x(),
		 m_config.h - (m_config.pt/2),
		 d1.z() - m_config.sOff,
		 "s_da");

  s_ad = tgNode( a1.x(),
		 m_config.h - (m_config.pt/2),
		 a1.z() + m_config.sOff,
		 "s_ad");
  
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

  s_bot_a = tgNode( s_ab.x(), a1.y(), s_ad.z(), "s_bot_a" );
  s_bot_b = tgNode( s_ba.x(), b1.y(), s_bc.z(), "s_bot_b" );
  s_bot_c = tgNode( s_cd.x(), c1.y(), s_cb.z(), "s_bot_c" );
  s_bot_d = tgNode( s_dc.x(), d1.y(), s_da.z(), "s_bot_d" );

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
 * This function calculates all the locations of the node positions for the 
 * housing structure. This requires that m_config be populated first.
 */
void ForcePlateModel::calculateHousingNodePositions() {
  // check that m_config is populated
  assert(invariant());
  
  // Each of hb_(whatever) are the points for the sides of the housing closest
  // to each of the four corners of the plate.
  hb_a = tgNode( -(m_config.w/2) + m_config.t/2,
		 m_config.h/2,
		 -(m_config.L/2) + m_config.t/2 );

  hb_b = tgNode( m_config.w/2 - (m_config.t/2),
		 m_config.h/2,
		 -(m_config.L/2) + m_config.t/2 );
  
  hb_c = tgNode( m_config.w/2 - (m_config.t/2),
		 m_config.h/2,
		 m_config.L/2 - (m_config.t/2) );

  hb_d = tgNode( -(m_config.w/2) + m_config.t/2,
		 m_config.h/2,
		 m_config.L/2 - (m_config.t/2) );

  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated node coordinates for the housing structure sides: " << std::endl;
    std::cout << "hb_a: " << hb_a << std::endl;
    std::cout << "hb_b: " << hb_b << std::endl;
    std::cout << "hb_c: " << hb_c << std::endl;
    std::cout << "hb_d: " << hb_d << std::endl << std::endl;
  }

  // Then, calculate the spring anchor positions on the housing.
  // NOTE THAT THIS REQUIRES THAT ALL THE s_() NODES ARE POPULATED!
  // @TO-DO: implement a check on this.

  s_ab_housing = s_ab + tgNode( 0, 0, -m_config.wgap);
  s_ab_housing.addTags("s_ab_housing");
  s_ba_housing = s_ba + tgNode( 0, 0, -m_config.wgap);
  s_ba_housing.addTags("s_ba_housing");

  s_bc_housing = s_bc + tgNode( m_config.wgap, 0, 0);
  s_bc_housing.addTags("s_bc_housing");
  s_cb_housing = s_cb + tgNode( m_config.wgap, 0, 0);
  s_cb_housing.addTags("s_cb_housing");

  s_cd_housing = s_cd + tgNode( 0, 0, m_config.wgap);
  s_cd_housing.addTags("s_cd_housing");
  s_dc_housing = s_dc + tgNode( 0, 0, m_config.wgap);
  s_dc_housing.addTags("s_dc_housing");

  s_da_housing = s_da + tgNode( -m_config.wgap, 0, 0);
  s_da_housing.addTags("s_da_housing");
  s_ad_housing = s_ad + tgNode( -m_config.wgap, 0, 0);
  s_ad_housing.addTags("s_ad_housing");

  // DEBUGGING:
  if( m_debugging ) {
    std::cout << std::endl << "Calculated lateral spring anchor positions on the housing:" << std::endl;
    std::cout << "s_ab_housing, s_ba_housing: " << s_ab_housing
	      << "   " << s_ba_housing << std::endl;
    std::cout << "s_bc_housing, s_cb_housing: " << s_bc_housing
	      << "   " << s_cb_housing << std::endl;
    std::cout << "s_cd_housing, s_dc_housing: " << s_cd_housing
	      << "   " << s_dc_housing << std::endl;
    std::cout << "s_da_housing, s_ad_housing: " << s_da_housing
	      << "   " << s_ad_housing << std::endl << std::endl;
  }
}

/**
 * Calculates the positions of the two nodes that are the from and to nodes
 * for the bottom plate of the housing.
 */
void ForcePlateModel::calculateBottomHousingNodePositions() {
  // check that m_config is populated
  assert(invariant());
  // The node on the ab face is at x=0 and the z-position of the side housing node.
  // The height is found by subtracting the plate thickness and bottom gap from the
  // total height. The center of the rectangle is then half that remaining dist.
  hb_bot_ab = tgNode( 0, (m_config.h - m_config.pt - m_config.bgap)/2, hb_a.z(),
		      "hb_bot_ab");
  hb_bot_cd = tgNode( 0, (m_config.h - m_config.pt - m_config.bgap)/2, hb_d.z(),
		      "hb_bot_cd");

  // The spring bottom positions are exactly bgap below the positions on the plate.
  s_bot_a_housing = s_bot_a - tgNode( 0, m_config.bgap, 0);
  s_bot_a_housing.addTags("s_bot_a_housing");
  s_bot_b_housing = s_bot_b - tgNode( 0, m_config.bgap, 0);
  s_bot_b_housing.addTags("s_bot_b_housing");
  s_bot_c_housing = s_bot_c - tgNode( 0, m_config.bgap, 0);
  s_bot_c_housing.addTags("s_bot_c_housing");
  s_bot_d_housing = s_bot_d - tgNode( 0, m_config.bgap, 0);
  s_bot_d_housing.addTags("s_bot_d_housing");

  //DEBUGGING
  if( m_debugging ) {
    std::cout << std::endl << "Calculated nodes of the bottom plate of the housing: "
	      << std::endl << "hb_bot_ab: " << hb_bot_ab
	      << std::endl << "hb_bot_cd: " << hb_bot_cd;
    
    std::cout << std::endl << "Calculated vertical spring anchor positions on the bottom of the housing:" << std::endl;
    std::cout << "s_bot_a_housing: " << s_bot_a_housing << std::endl;
    std::cout << "s_bot_b_housing: " << s_bot_b_housing << std::endl;
    std::cout << "s_bot_c_housing: " << s_bot_c_housing << std::endl;
    std::cout << "s_bot_d_housing: " << s_bot_d_housing << std::endl << std::endl;
  }
}

/** 
 * helper function for the constructor(s).
 * performs some checks on the parameters that are passed in.
 */
void ForcePlateModel::constructorAux()
{
  // DEBUGGING:
  if( m_debugging ) {
    std::cout << "Constructor for ForcePlateModel..." << std::endl;
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
  if( m_config.mP < 0.0 ) {
    throw std::invalid_argument("Mass of the plate (mP) must be greater than zero.");
  }
  if( m_config.mH < 0.0 ) {
    throw std::invalid_argument("Mass of the housing structure (mH) must be greater than zero.");
  }
  //TEMPORARY: this class can only take in housing masses that are zero.
  // This is because I do not have time to calculate the total volume of all the
  // housing boxes (including the support beams).
  if( m_config.mH != 0.0 ) {
    throw std::invalid_argument("Currently, this class only supports housing mass of zero (fixed in space.) Mass of the housing is not zero, exiting.");
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
    std::cout << "mP: " << m_config.mP << std::endl;
    std::cout << "mH: " << m_config.mH << std::endl;
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
  // Housing node position calculations rely on the plate positions,
  // so this function MUST be called after the above one.
  calculateHousingNodePositions();
  // Similarly, the bottom part of the housing depends on the sides of
  // the housing being created. MUST be called after the above one.
  calculateBottomHousingNodePositions();
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

// helper function to tag two sets of nodes as boxes
void ForcePlateModel::addLateralPlateBoxesPairs(tgStructure& s)
{
  /**
   * Newer form of this function. Only creates one box
   * for the force plate.
   * First calculates the two endpoints, then
   * adds them as a pair.
   * The two endpoints are at the center of two rectangles on the face of the plate.
   */
  // Note that even though rigid bodies are created properly when
  // pairs are specified directly, connectors will fail unless
  // their pairs are specified as numerical indices into the list
  // of nodes (which requires that nodes are added first before
  tgNode plateBoxFrom = tgNode( 0, m_config.h - (m_config.pt/2), a1.z() );
  tgNode plateBoxTo = tgNode( 0, m_config.h - (m_config.pt/2), d1.z() );
  // Add the nodes:
  s.addNode(plateBoxFrom); // 0
  s.addNode(plateBoxTo);  // 1
  // Add the pair that connects the nodes:
  s.addPair(0, 1, "plateBox");
}

// helper function to tag two sets of nodes as boxes
void ForcePlateModel::addHousingBoxesPairs(tgStructure& s)
{
  // Note that even though rigid bodies are created properly when
  // pairs are specified directly, connectors will fail unless
  // their pairs are specified as numerical indices into the list
  // of nodes (which requires that nodes are added first before
  // adding pairs.) For consistency, use this method too for the rigids.
  // Add the nodes for the sides:
  s.addNode( hb_a ); // 2
  s.addNode( hb_b ); // 3
  s.addNode( hb_c ); // 4
  s.addNode( hb_d ); // 5
  // Add the pairs for the sides:
  s.addPair( 2, 3, "housingWallX" ); // hb_a, hb_b
  s.addPair( 3, 4, "housingWallZ" ); // hb_b, hb_c
  s.addPair( 4, 5, "housingWallX" ); // hb_c, hb_d
  s.addPair( 5, 2, "housingWallZ" ); // hb_d, hb_a
  // Add the nodes for the bottom:
  s.addNode( hb_bot_ab ); // 6
  s.addNode( hb_bot_cd ); // 7
  // Add the pair for the bottom:
  s.addPair( 6, 7, "housingBottom" ); // hb_bot_ab, hb_bot_cd
  // Finally, add the "supporting beams" that are needed to auto-compound
  // the sides of the housing to the bottom of the housing.
  // Even though only one beam is needed, technically, let's add four for symmetry.
  // These are at an angle.
  // Without these, the bottom of the housing would fall out, since because it
  // would not share any nodes with the sides, it would not be auto-compounded
  // with them.
  s.addPair( 2, 6, "supportBeam" ); // hb_a, hb_bot_ab
  s.addPair( 3, 6, "supportBeam" ); // hb_b, hb_bot_ab
  s.addPair( 4, 7, "supportBeam" ); // hb_c, hb_bot_cd
  s.addPair( 5, 7, "supportBeam" ); // hb_d, hb_bot_cd
}

// Adds the pairs for the lateral springs (attaching the housing to the plate.)
void ForcePlateModel::addLateralSpringsPairs(tgStructure& s)
{
  // On each side, springs can start from the housing and attach the free end
  // to the plate.
  // Add the nodes on the plate itself:
  s.addNode( s_ab ); // 8
  s.addNode( s_ba ); // 9
  s.addNode( s_bc ); // 10
  s.addNode( s_cb ); // 11
  s.addNode( s_cd ); // 12
  s.addNode( s_dc ); // 13
  s.addNode( s_da ); // 14
  s.addNode( s_ad ); // 15
  // Add the corresponding nodes on the housing:
  s.addNode( s_ab_housing ); // 16
  s.addNode( s_ba_housing ); // 17
  s.addNode( s_bc_housing ); // 18
  s.addNode( s_cb_housing ); // 19
  s.addNode( s_cd_housing ); // 20
  s.addNode( s_dc_housing ); // 21
  s.addNode( s_da_housing ); // 22
  s.addNode( s_ad_housing ); // 23
  
  // Add the pairs corresponding to each spring.
  // Note that there have to be different tags/labels for the springs
  // on each side of the plate, since they have to have different 'direction'
  // vectors for their tgUnidirectionalCompressionSpring.
  // These are named for the face of the force plate they are on.
  // The springs along x:
  s.addPair( 10, 18, "lateralSpringBC" ); // s_bc, s_bc_housing
  s.addPair( 11, 19, "lateralSpringBC" ); // s_cb, s_cb_housing
  s.addPair( 14, 22, "lateralSpringAD" ); // s_da, s_da_housing
  s.addPair( 15, 23, "lateralSpringAD" ); // s_ad, s_ad_housing
  // The springs along Z:
  s.addPair( 8, 16, "lateralSpringAB" ); // s_ab, s_ab_housing
  s.addPair( 9, 17, "lateralSpringAB" ); // s_ba, s_ba_housing
  s.addPair( 12, 20, "lateralSpringCD" ); // s_dc, s_cd_housing
  s.addPair( 13, 21, "lateralSpringCD" ); // s_dc, s_dc_housing
}

// Adds the pairs for the vertical springs (attaches the plate to the bottom
// of the housing.)
// Note that this must be called in order, otherwise the added pairs
// will not correspond to the order of the nodes.
void ForcePlateModel::addVerticalSpringsPairs(tgStructure& s) {
  // First, add the nodes for the vertical springs.
  // The nodes on the plate are:
  s.addNode( s_bot_a ); // 24
  s.addNode( s_bot_b ); // 25
  s.addNode( s_bot_c ); // 26
  s.addNode( s_bot_d ); // 27
  // The nodes on the housing are:
  s.addNode( s_bot_a_housing ); // 28
  s.addNode( s_bot_b_housing ); // 29
  s.addNode( s_bot_c_housing ); // 30
  s.addNode( s_bot_d_housing ); // 31
  // Finally, add the pairs for each spring.
  // Version where springs originate on the housing:
  s.addPair( 28, 24, "verticalSpring" ); // s_bot_a_housing, s_bot_a
  s.addPair( 29, 25, "verticalSpring" ); // s_bot_b_housing, s_bot_b
  s.addPair( 30, 26, "verticalSpring" ); // s_bot_c_housing, s_bot_c
  s.addPair( 31, 27, "verticalSpring" ); // s_bot_d_housing, s_bot_d
}

// Finally, create the model!
void ForcePlateModel::setup(tgWorld& world)
{
  // @TO-DO: Make sure that the nodes are assigned by now.
  
  // The structure that will be built into:
  //tgStructure s = tgStructure();
  tgStructure s;

  // Add the pairs for the force plate first.
  // Add the the boxes that will be used as the connecting points
  // for the lateral springs.
  addLateralPlateBoxesPairs(s);
  addHousingBoxesPairs(s);
  // Add the pairs for the springs
  addLateralSpringsPairs(s);
  addVerticalSpringsPairs(s);

  // Move the structure to the location passed in to the constructor.
  s.move(m_location);

  //DEBUGGING
  // Move the plate up a bit more.
  //s.move( btVector3(0, 2, 0) );

  
  // Create the config structs for the various different boxes and springs.

  // Config for the force plate box itself.
  // NOTE that tgBox uses half-extents, meaning that the width and height
  // passed in are the width of HALF the box and the height of HALF the box.
  // So, all heights and widths from Drew's calculations must be halved.
  // But, display plate width and height in the terms of my derivation.
  double plateWidth = m_config.w - (2 * m_config.t) - (2 * m_config.wgap);
  double plateHeight = m_config.pt;
  // Calculate the density of the plate based on its volume.
  // NOTE that this does in fact work properly, since the width and height here
  // are what we define, NOT what is passed in to Bullet's btBoxShape.
  // As per addLateralPlateBoxesPairs, the length of the box can be calculated as:
  double plateLength = abs( a1.z() - d1.z() );
  double plateVolume = plateLength * plateWidth * plateHeight;
  double plateDensity = m_config.mP / plateVolume;
  //DEBUGGING
  if( m_debugging ){
    std::cout << "Plate length: " << plateLength << ", Plate width: "
	      << plateWidth << ", Plate height: " << plateHeight
	      << ", Plate density: " << plateDensity << std::endl << std::endl;
  }
  // Here, apply the 1/2 factor to the width and height.
  // NOTE that tgBoxMoreAnchors also uses tgBox::Config.
  tgBox::Config plateBoxConfig( plateWidth/2, plateHeight/2, plateDensity);


  // Config for the housing walls: @TO-DO: CHANGE DENSITY.
  // Halve the width and height here also.
  tgBox::Config housingWallConfigZ( m_config.t/2, m_config.h/2, 0.0);
  tgBox::Config housingWallConfigX( m_config.h/2, m_config.t/2, 0.0);

  // Config for the bottom part of the housing: @TO-DO: CHANGE DENSITY.
  // Halve the width and height here also.
  // First, get the correct height of the bottom plate, based on m_config.
  double housingBottomHeight = ( m_config.h - m_config.pt - m_config.bgap)/2;
  tgBox::Config housingBottomConfig( m_config.w/2, housingBottomHeight, 0.0);

  // Config for the support beams that connect the housing walls to bottom,
  // so that it's all auto-compounded by tgcreator.
  // These beams should be extremely small and thin, contributing almost nothing
  // to the mass of the housing.
  // How much smaller should the beams be than m_config thickness?
  double supportBeamDivisor = 10;
  double supportBeamDensity = 0.0;
  tgBox::Config supportBeamConfig( m_config.t/supportBeamDivisor,
				   m_config.t/supportBeamDivisor,
				   supportBeamDensity);

  // For the springs:
  // Config here is isFreeEndAttached, stiffness, damping, restlength,
  // movecablepointA, movecablepointB, direction.
  // Note that since the direction changes for each face of the plate,
  // there will need to be four configs here.
  // They are named by face: AB, BC, CD, DA.
  // The springs along X:
  tgUnidirectionalCompressionSpringActuator::Config
    lateralSpringConfigBC(true, m_config.latK, m_config.latD, m_config.latRL,
			  false, false, new btVector3(1, 0, 0));
  tgUnidirectionalCompressionSpringActuator::Config
    lateralSpringConfigAD(true, m_config.latK, m_config.latD, m_config.latRL,
			  false, false, new btVector3(-1, 0, 0));
  // The springs along Z:
  tgUnidirectionalCompressionSpringActuator::Config
    lateralSpringConfigAB(true, m_config.latK, m_config.latD, m_config.latRL,
			  false, false, new btVector3(0, 0, -1));
  tgUnidirectionalCompressionSpringActuator::Config
    lateralSpringConfigCD(true, m_config.latK, m_config.latD, m_config.latRL,
			  false, false, new btVector3(0, 0, 1));
  
  // The vertical springs can all share the same config:
  tgUnidirectionalCompressionSpringActuator::Config
    verticalSpringConfig(true, m_config.vertK, m_config.vertD, m_config.vertRL,
			 false, false, new btVector3(0, 1, 0));
  
  // Create the build spec that uses tags to turn the structure into a real model
  tgBuildSpec spec;

  // Add the configs for the boxes
  spec.addBuilder("plateBox", new tgBoxMoreAnchorsInfo(plateBoxConfig));
  //spec.addBuilder("plateBox", new tgBoxInfo(testConfig));
  spec.addBuilder("housingWallZ", new tgBoxMoreAnchorsInfo(housingWallConfigZ));
  spec.addBuilder("housingWallX", new tgBoxMoreAnchorsInfo(housingWallConfigX));
  spec.addBuilder("housingBottom", new tgBoxMoreAnchorsInfo(housingBottomConfig));
  // Note that the support beams will not have any extra nodes on them, so use
  // a regular tgBox here for them.
  spec.addBuilder("supportBeam", new tgBoxInfo(supportBeamConfig));

  // Add the configs for the springs:
  spec.addBuilder("lateralSpringBC",
  		  new tgUnidirectionalCompressionSpringActuatorInfo(lateralSpringConfigBC));
  spec.addBuilder("lateralSpringAD",
  		  new tgUnidirectionalCompressionSpringActuatorInfo(lateralSpringConfigAD));
  spec.addBuilder("lateralSpringAB",
		  new tgUnidirectionalCompressionSpringActuatorInfo(lateralSpringConfigAB));
  spec.addBuilder("lateralSpringCD",
		  new tgUnidirectionalCompressionSpringActuatorInfo(lateralSpringConfigCD));
  spec.addBuilder("verticalSpring",
  		  new tgUnidirectionalCompressionSpringActuatorInfo(verticalSpringConfig));

  //DEBUGGING
  if( m_debugging){
    std::cout << s << std::endl;
  }

  // Create your structureInfo
  tgStructureInfo structureInfo(s, spec);

  // Use the structureInfo to build ourselves
  structureInfo.buildInto(*this, world);

  // call the onSetup methods of all observed things e.g. controllers
  notifySetup();

  // Actually setup the children
  tgModel::setup(world);
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
