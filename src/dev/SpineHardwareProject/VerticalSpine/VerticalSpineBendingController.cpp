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
 * @file VerticalSpineRestLengthController.cpp
 * @brief Implementation of a rest length controller for VerticalSpineModel.
 * @author Drew Sabelhaus, Brian Tietz, Michael Fanton, ChanWoo Yang
 * $Id$
 */

// This module
#include "VerticalSpineBendingController.h"
// This application
#include "VerticalSpineModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgString.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

VerticalSpineBendingController::VerticalSpineBendingController():
  verticalRLA(4.0),
  verticalRLB(4.0),
  dL(0.01),        // Length Change
  state(-1.0),
  updateTime(0.0)
{
  // verticalRL = 1; // cm
    //verticalRL = 7.38; //cm
    //saddleRL1 = 13.037 ; // cm
    //saddleRL2 = 13.613 ; // cm
    //saddleRL3 = 14.189 ; // cm
    //saddleRL4 = 14.766 ; // cm
}

void VerticalSpineBendingController::onStep(VerticalSpineModel& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
      {
	updateTime += dt;
	if (updateTime >= 1.0/100)  //Speed of actuators
	  {
	    updateTime = 0.0;

	    // Bend & Unbend
	    if(verticalRLA <= 2.0 && state == -1.0)  //min length of cable
	      {
		state = 1.0;
	      }
	    else if (verticalRLA >= 4.0 && state == 1.0)  //stop at upright position
	      {
		state = -1.0;
	      }

	    if (state == -1.0)
	      {
		verticalRLA -= dL;
		verticalRLB += dL;
	      }
	    else if (state == 1.0)
	      {
		verticalRLA += dL;
		verticalRLB -= dL;
	      }
	  }
	
        // First, get all muscles (cables)
        const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
        
        // get all vertical muscles
        const std::vector<tgBasicActuator*> v_musclesA = subject.getMuscles("vertical a");
        const std::vector<tgBasicActuator*> v_musclesB = subject.getMuscles("vertical b");
        const std::vector<tgBasicActuator*> v_musclesC = subject.getMuscles("vertical c");
        const std::vector<tgBasicActuator*> v_musclesD = subject.getMuscles("vertical d");
        

        // set string length for vertical muscles
        for (size_t i = 0; i < v_musclesA.size(); ++ i)
        {
	    //A   **Contracting Cable
            tgBasicActuator * const pMuscleA = v_musclesA[i];
            assert(pMuscleA != NULL);
            pMuscleA->setControlInput(verticalRLA,dt);
            
            //B   **Elongating Cable
            tgBasicActuator * const pMuscleB = v_musclesB[i];
            assert(pMuscleB != NULL);
            pMuscleB->setControlInput(verticalRLB,dt);
            
            // //C
            // tgBasicActuator * const pMuscleC = v_musclesC[i];
            // assert(pMuscleC != NULL);
            // pMuscleC->setControlInput(verticalRL);
            
            // //D
            // tgBasicActuator * const pMuscleD = v_musclesD[i];
            // assert(pMuscleD != NULL);
            // pMuscleD->setControlInput(verticalRL);
        }
      
    }
}
