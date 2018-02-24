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
 * @file CombinedSpineControllerRotVertPositionTraj.cpp
 * @brief Implementation of CombinedSpineControllerRotVertPositionTraj.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "CombinedSpineControllerRotVertPositionTraj.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgRod.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "core/abstractMarker.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <math.h>
#include <fstream> // for the CSV file
#include <stdexcept> // for some error handling
#include <stdlib.h> // for the atof function

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
CombinedSpineControllerRotVertPositionTraj::CombinedSpineControllerRotVertPositionTraj(double startTime,
						       std::string csvPath,
						       std::string rodHingeTag,
						       btDynamicsWorld* world) :
  m_startTime(startTime),
  m_csvPath(csvPath),
  m_rodHingeTag(rodHingeTag),
  m_world(world),
  m_timePassed(0.0),
  m_accumulatedError(0.0),
  m_prevError(0.0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // path to the CSV file must not be null
  else if( (m_csvPath.empty() )) {
    throw std::invalid_argument("CSV path is not present. Must provide a file with angle data.");
  }
  // @TODO: what checks to make on tags?
}

/**
 * For this controller, the onSetup method picks out the two rods with rodHingeTag
 * and "A" or "B" attached to the tag. The hingeconstraint is created here,
 * which is not great, but makes it so we don't need to make an extra model class.
 */
void CombinedSpineControllerRotVertPositionTraj::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up a CombinedSpineControllerRotVertPositionTraj with tag: "
	    << m_rodHingeTag << std::endl;
  // We have an A element and a B element with this tag.
  // In general, Drew is using "A" to be the one that's at the origin in the
  // local frame of the split vertebra.
  std::string rodHingeTagA = m_rodHingeTag + "A";
  std::string rodHingeTagB = m_rodHingeTag + "B";
  // Get what are (hopefully) exactly one rod each from the whole model:
  std::vector<tgRod*> allRodsA = subject.find<tgRod>(rodHingeTagA);
  std::vector<tgRod*> allRodsB = subject.find<tgRod>(rodHingeTagB);

  // Confirm that both these arryas have exactly one element.
  // Make sure this list is not empty:
  if( allRodsA.empty() ) {
    throw std::invalid_argument("No rods found with rodHingeTagA.");
  }
  if( allRodsB.empty() ) {
    throw std::invalid_argument("No rods found with rodHingeTagB.");
  }
  // Now, we know that element 0 exists for each.
  // Confirm that it is not a null pointer.
  if( allRodsA[0] == NULL) {
    throw std::runtime_error("Pointer to rod with rodHingeTagA is NULL.");
  }
  if( allRodsB[0] == NULL) {
    throw std::runtime_error("Pointer to rod with rodHingeTagB is NULL.");
  }

  // We'll need these pointers for applying torques later.
  hingedRodA = allRodsA[0];
  hingedRodB = allRodsB[0];

  // Next, create the rotating ("hinge") joint.
  btRigidBody* rodA_rb = allRodsA[0]->getPRigidBody();
  btRigidBody* rodB_rb = allRodsB[0]->getPRigidBody();
  // We also need to calculate their displacement so we can set the constraint
  // correctly.
  btVector3 rodA_com = allRodsA[0]->centerOfMass();
  btVector3 rodB_com = allRodsB[0]->centerOfMass();
  // If we do A minus B, then the displacement should be the second argument
  // to the hinge constraint.
  btVector3 net_com = rodA_com - rodB_com;
  // Create the hinge constraint
  // Constructor is: 2 x btRigidBody, 4 x btVector3, 1 x bool.  
  btHingeConstraint* rotHinge =
    new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(0, 0, 0),
			  net_com, btVector3(1, 0, 0),
			  btVector3(1, 0, 0), false);
  // Add to the world.
  m_world->addConstraint( rotHinge );
  // This segfaults on reset (space bar.) That's probably better for now,
  // actually, until we get this formalized...

  //
  //
  //
  //
  
  // Next, read the data from the CSV file into an array, for use down below.
  // DEBUGGING:
  //std::cout << "Opening the CSV of trajectory set point angles..." << std::endl;
  // Next, open it as an input file stream
  // This needs to be a c-style string for some reason.
  std::ifstream csv_file(m_csvPath.c_str());
  // If the file wasn't opened, throw an exception.
  if( !csv_file.is_open() ) {
    throw std::invalid_argument("Couldn't open the CSV file, check the path.");
  }
  // We need to declare some temporary variables to use to store the data
  //   as it's parsed:
  std::string temp_row;
  // We need to keep a boolean variable to check if the arrays have been
  // initialized yet.
  bool areInitialized = false;
  // Now, loop over the lines in the file as they're read in.
  // Note that the getline function will return 0 when there are no more lines left.
  while( std::getline(csv_file, temp_row) ) {
    // Here, temp_row has one row of the data.
    // Convert the string into a stringstream, for more parsing of the line
    // into its individual components:
    std::stringstream temp_row_stream( temp_row );
    // Loop over it and output all elements (separated by a comma).
    // Note that the variable temp_row_stream will have value 0 when it's empty.
    // Also, we need a tracking variable to index into the vector of vectors.
    int column = 0;
    while( temp_row_stream ) {
      // Read in the next element from the stream into a specific string:
      std::string temp_element;
      std::getline( temp_row_stream, temp_element, ',');
      // For some reason, the getline function returns an empty string
      // at the end of a row. If temp_element is empty, break out of the loop here.
      if( temp_element.empty() ) {
	break;
      }
      // TESTING: just output to the command line.
      //std::cout << "Element from the input csv file: " << temp_element << std::endl;
      // FROM ANKITA'S CODE, WAS CALLED CABLE_TRAJECTORY
      // Finally, append this element to the proper list in setpointTrajectory.
      // The first index into setpointTrajectory is column.
      // If this is the first row (e.g., the arrays have not been initialized),
      // we need to create a new array then append it to the array of arrays:
      if( !areInitialized ) {
	// the array for this column:
	std::vector<double> tempArray;
	// add the first element of the column to the new array:
	// (the atof function turns a string into a double)
	tempArray.push_back( atof(temp_element.c_str()) );
	// add this array to the array of arrays:
	setpointTrajectory.push_back( tempArray );
      }
      else {
	// The arrays already exist, so we can just index into the right place:
	setpointTrajectory[column].push_back( atof(temp_element.c_str()) );
	// increment the counter.
	// Note that above, we don't need a counter, since no indexing is needed.
	column += 1;
      }
    }
    // Set the flag to true now that a line is read.
    // We're technically re-setting it every time, but who cares.
    // If it's true after the first row, it's true after every row.
    areInitialized = true;
  }
  // DEBUGGING:
  std::cout << "Finished reading in the CSV file." << std::endl;
  std::cout << "There were " << setpointTrajectory.size() << " columns and "
	    << setpointTrajectory[0].size() << " rows." << std::endl;
  /*
  // DEBUGGING:
  // Output the resulting vector of vectors, just to test that it looks right.
  std::cout << "CSV values are: " << std::endl;
  // The outer loop is along the LONG dimension (e.g., gives how many rows there are,
  // and the inner loop along the columns / moving through a single column.)
  // So, arbitrarily choose the first column vector, to get the number of rows:
  for( int i=0; i < setpointTrajectory[0].size(); i++ ){
    for( int j=0; j < setpointTrajectory.size(); j++) {
      // Note the order of indexing here, just because we want to
      // print out one row.
      std::cout << setpointTrajectory[j][i] << ",";
    }
    // At the end of a row, put a newline.
    std::cout << std::endl;
  }
  */
}

/**
 * The onStep method does one of the following things:
 * If between time zero and startTime: apply no torque.
 * If after startTime: apply a control to track the desired position (set angle).
 */
void CombinedSpineControllerRotVertPositionTraj::onStep(TensegrityModel& subject, double dt)
{
  //DEBUGGING:
  // Let's see if we get the positions of the abstract markers from the subject here.
  std::vector<abstractMarker> markers = subject.getMarkers();
  std::cout << "Inside rot vert controllers, markers are at: " << std::endl;
  for(int i=0; i < markers.size(); i++){
    std::cout << markers[i].getWorldPosition() << std::endl;
  }
  std::cout << std::endl;
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, check which action to perform:
  if( m_timePassed > m_startTime ) {
    // Calculate and apply a torque that will be controlled around a position
    // (angle between the two vertebrae.)
    
    // We want to obtain the rotation around the axis that aligns rod A and rod B.
    // Luckily enough, we already know that the frame will be aligned along the
    // X-axis, since that's how we've set up the btHingeConstraint.
    // So, we can take advantage of quaternions here:
    // a quaternion is (axis + angle), so just taking the angle of the quaternion
    // that represents this A-to-B rotation will give us what we want.

    // Here's the procedure that is used in this code.
    // We'll use rotation matrices first.
    // 1) Get the rotation matrix of each of the two frames (rod A, rod B), with
    //    respect to the world
    // 2) Get the net rotation from one frame to another
    // 2) Get the angle along the axis that connects these two frames (which, if R is going from A to B, just the rotation implied in R around R's unit vector.)
    // (note, we *know* that the unit vector u of R_net will really only have one component, along the x-direction, since we've constrainted along the other two.
    // 3) Calculate the difference between this angle, and the m_setPoint
    // 4) Calculate and apply a torque proportional to the difference (the control.)
    
    // Much credit to the wikipedia article on rotations:
    // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    // As well as some comments on coordinate systems from stackexchange:
    // https://math.stackexchange.com/questions/1870661/find-angle-between-two-coordinate-systems  
    
    // OK, so one way to get the net rotation matrix between the two is to
    // first rotate A back to the origin, then apply the rotation to get from
    // the origin to B. Since we have R_a_origin, and R_b_origin, given by the basis
    // for the transformation for both rigid body objects, this could be:
    btTransform rodaTransform = hingedRodA->getPRigidBody()->getWorldTransform();
    btTransform rodbTransform = hingedRodB->getPRigidBody()->getWorldTransform();
    // We can get the rotation matrices from each transform
    // (note that we want to ignore the translation!)
    btMatrix3x3 rodaWorldRotation = rodaTransform.getBasis();
    btMatrix3x3 rodbWorldRotation = rodbTransform.getBasis();
    // ...we might have been able to just use quaternions here, but this made more
    // sense to Drew at the time.
    // To get the net, invert the rod A transform, then apply the B transform.
    // Using Bullet's methods, we can say:
    btMatrix3x3 netRotation = rodaWorldRotation.inverse() * rodbWorldRotation;
    // The order doesn't matter here, just off by a +/-, since the only difference
    // when "looking" from one rod to another is the direction of the unit vector.

    // Next, let's get the quaternion for this net rotation, and extract its angle.
    btQuaternion netRotQuat;
    netRotation.getRotation(netRotQuat);
    btScalar netRotScalar = netRotQuat.getAngle();
    // for checking: let's get the axis here too. Should be only in one direction.
    //btVector3 netRotAxis = netRotQuat.getAxis();
    //DEBUGGING
    //std::cout << "netRotScalar: " << netRotScalar << std::endl;
    //std::cout << "netRotAxis: " << netRotAxis << std::endl;
    // GREAT THIS SEEMS TO WORK. netRotAxis is roughly (1, 0, 0),
    // and netRotScalar is some reasonable value.

    // Index into the set point trajectory for this timestep.
    // Let's do it this way. Start at time zero, and keep incrementing through the
    // first column of the array (the timestep) until the timestep is larger
    // than the accumulated time. Then, take the setpoint at the most recent element.
    // But, we'll need to check and reset the index if we've run off the end of
    // the array.
    double timestepIndex = 0;
    // We actually need to index into m_timePassed - m_startTime, otherwise
    // the control skips to m_timePassed in the sequence (we want to start from 0.)
    double m_controlTime = m_timePassed - m_startTime;
    // The max index into the array is size - 1, since "size" includes the 0 element.
    // While accumulated time greater than the time at timestep index
    // AND the index is smaller than the total array size,
    while( (m_controlTime > setpointTrajectory[0][timestepIndex]) &&
	   timestepIndex < (setpointTrajectory[0].size() - 1) )
    {
      // Look into the next entry into the array...
      timestepIndex = timestepIndex + 1;
    }
    //DEBUGGING
    //std::cout << "setpoint index (time): " << timestepIndex << std::endl;
    // get the tracked point at timestepIndex
    double m_setAngle = setpointTrajectory[1][timestepIndex];
    
    // Great. Let's perform the control.
    // First, control constants. The angle seems to be in the range of
    // like 0.00 to 0.03 radians, for our purposes. And the torque to apply
    // is on the order of 0.2. So maybe a K_P of 5 or 10?
    double K_P = 1000; // 50 worked, but lots of overshoot. Was 1000.
    // a really small constant for integral control worked best.
    double K_I = 0.5; // was 0.5

    // Calculate the new torque we want to apply, - K * (x - x_ref)... with I term.
    // we've arbitrarily choosen torques to be negative?
    // The error between the current and desired, x - x_ref, is
    double error = netRotScalar - m_setAngle;
    // Let's do an integral term also.
    m_accumulatedError = m_accumulatedError + error;
    // Combined control input is:
    double controlInput = - (K_P * error) - (K_I * m_accumulatedError);
    
    // As a torque vector, along the x-axis:
    btVector3 controlledTorque( controlInput, 0, 0);
    //DEBUGGING: what's the error that we're controlling around?
    // Need to develop a control such that this trends to zero.
    //std::cout << "Rotating vertebra tracking control error: " << error << std::endl;
    // it would be interesting to plot this.
    // torque, aligned with the world, referenced against rod A.
    // (choice is arbitrary, since we can just do +/- switching A to B.)
    btVector3 worldControlledTorque = rodaWorldRotation * controlledTorque;
    // ...apply the torque, opposite to each body.
    hingedRodA->getPRigidBody()->applyTorqueImpulse( worldControlledTorque );
    hingedRodB->getPRigidBody()->applyTorqueImpulse( -worldControlledTorque );
    // this worked. With just P control, seems to have steady-state error.
    // With I control, seems to track down to machine precision.
    // TO-DO: SEEMS TO ONLY USE FLOATS, NOT DOUBLES? We've only got 7 digits
    // of precision and not 14...
  }
}
	
 
