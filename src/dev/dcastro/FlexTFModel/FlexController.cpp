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
 * @file FlexController.cpp
 * @brief Contains the controller of the members of the class ThirdTFModel.
 *	Model is inspired by Tom Flemmons tensegrity model of the knee and is a working progress.
 * $Id$
 */

// This module
#include "FlexController.h"
// This application
#include "TFModel.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgBasicActuator.h"
#include "sensors/tgDataObserver.h"

// The Bullet Physics library
//#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <stdexcept>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <cassert>

FlexController::FlexController():
   verticalBFA1(12),
   verticalBFA2(12),
   verticalGSA3(10),
   verticalGSA4(10),
   verticalBFB1(10),
   verticalBFB2(10),
   verticalGSB3(8),
   verticalGSB4(8),
   dL(1),
   state(-1.0),
   m_updateTime(0.0) {}

void FlexController::onSetup(TFModel& subject){
}

void FlexController::onStep(TFModel& subject, double dt)
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

	    // logging
	    notifyStep(m_updateTime);

	    // Bend & Unbend
	    if(verticalBFA1 <= 10.0 && state == -1.0)  //min length of cable
	      {
		state = 1.0;
	      }
	    else if (verticalBFA1 >= 12.0 && state == 1.0)  //stop at upright position
	      {
		state = -1.0;
	      }

	    if (state == -1.0)
	      {
		verticalBFA1 -= dL;
		verticalBFA2 -= dL;
		verticalGSA3 -= dL;
		verticalGSA4 -= dL;
		
		verticalBFB1 += dL;
		verticalBFB2 += dL;
		verticalGSB3 += dL;
		verticalGSB4 += dL;
	      }
	    else if (state == 1.0)
	      {
		verticalBFA1 += dL;
		verticalBFA2 += dL;
		verticalGSA3 += dL;
		verticalGSA4 += dL;
		
		verticalBFB1 -= dL;
		verticalBFB2 -= dL;
		verticalGSB3 -= dL;
		verticalGSB4 -= dL;
	      }
	  }
	
        // First, get all muscles (cables)
        const std::vector<tgSpringCableActuator*> muscles = subject.getAllMuscles();
        
        // get all vertical muscles
        const std::vector<tgSpringCableActuator*> v_musclesA = subject.find<tgSpringCableActuator>("Semimebranosus");
        const std::vector<tgSpringCableActuator*> v_musclesB = subject.find<tgSpringCableActuator>("Bicep Femoris");
        const std::vector<tgSpringCableActuator*> v_musclesC = subject.find<tgSpringCableActuator>("GastLateral");
        const std::vector<tgSpringCableActuator*> v_musclesD = subject.find<tgSpringCableActuator>("GastMedial");
        

        // set string length for vertical muscles
	//A   **Contracting Cable
	   
	tgSpringCableActuator * const pMuscleA1 = v_musclesA[0];
	tgSpringCableActuator * const pMuscleA2 = v_musclesA[1];
	tgSpringCableActuator * const pMuscleA3 = v_musclesA[2];
	tgSpringCableActuator * const pMuscleA4 = v_musclesA[3];
	assert(pMuscleA1 != NULL);
	assert(pMuscleA2 != NULL);
	assert(pMuscleA3 != NULL);
	assert(pMuscleA4 != NULL);
	pMuscleA1->setControlInput(verticalBFA1,dt);
	pMuscleA2->setControlInput(verticalBFA2,dt);
	pMuscleA3->setControlInput(verticalGSA3,dt);
	pMuscleA4->setControlInput(verticalGSA4,dt);
            
	//B   **Elongating Cable
	tgSpringCableActuator * const pMuscleB1 = v_musclesB[0];
	tgSpringCableActuator * const pMuscleB2 = v_musclesB[1];
	tgSpringCableActuator * const pMuscleB3 = v_musclesB[2];
	tgSpringCableActuator * const pMuscleB4 = v_musclesB[3];
	assert(pMuscleB1 != NULL);
	assert(pMuscleB2 != NULL);
	assert(pMuscleB3 != NULL);
	assert(pMuscleB4 != NULL);
	pMuscleB1->setControlInput(verticalBFB1,dt);
	pMuscleB2->setControlInput(verticalBFB2,dt);
	pMuscleB3->setControlInput(verticalGSB3,dt);
	pMuscleB4->setControlInput(verticalGSB4,dt);
            
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

  
