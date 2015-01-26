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
#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

#include "helpers/FileHelpers.h"

VerticalSpineBendingController::VerticalSpineBendingController():
  verticalRLA1(4.0),
  verticalRLA2(4.0),
  verticalRLA3(4.0),
  verticalRLA4(4.0),
  verticalRLB1(4.0),
  verticalRLB2(4.0),
  verticalRLB3(4.0),
  verticalRLB4(4.0),
  dL(0.001),        // Length Change, 0.01
  state(-1.0),
  updateTime(0.0),
  m_dataObserver("/home/drew/NTRTsim_logs/vertspine")
{
  // verticalRL = 1; // cm
    //verticalRL = 7.38; //cm
    //saddleRL1 = 13.037 ; // cm
    //saddleRL2 = 13.613 ; // cm
    //saddleRL3 = 14.189 ; // cm
    //saddleRL4 = 14.766 ; // cm
//m_dataObserver = new tgDataObserver("logs/vertspine");
//m_dataObserver("logs/vertispine");
}
  //m_dataObserver("logs/vert_spine")

void VerticalSpineBendingController::onSetup(VerticalSpineModel& subject){
  m_dataObserver.onSetup(subject);
}


// //Gear Ratio 1: 1-2-3-4
// void VerticalSpineBendingController::onStep(VerticalSpineModel& subject, double dt)
// {
//     if (dt <= 0.0)
//     {
//         throw std::invalid_argument("dt is not positive");
//     }
//     else
//       {
// 	updateTime += dt;
// 	if (updateTime >= 1.0/100)  //Speed of actuators
// 	  {
// 	    updateTime = 0.0;

// 	    // Bend & Unbend
// 	    if(verticalRLA1 <= 2.0 && state == -1.0)  //min length of cable
// 	      {
// 		state = 1.0;
// 	      }
// 	    else if (verticalRLA1 >= 4.0 && state == 1.0)  //stop at upright position
// 	      {
// 		state = -1.0;
// 	      }

// 	    if (state == -1.0)
// 	      {
// 		verticalRLA1 -= dL;
// 		verticalRLB1 += dL;
// 	      }
// 	    else if (state == 1.0)
// 	      {
// 		verticalRLA1 += dL;
// 		verticalRLB1 -= dL;
// 	      }
// 	  }
	
//         // First, get all muscles (cables)
//         const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
        
//         // get all vertical muscles
//         const std::vector<tgBasicActuator*> v_musclesA = subject.getMuscles("vertical a");
//         const std::vector<tgBasicActuator*> v_musclesB = subject.getMuscles("vertical b");
//         const std::vector<tgBasicActuator*> v_musclesC = subject.getMuscles("vertical c");
//         const std::vector<tgBasicActuator*> v_musclesD = subject.getMuscles("vertical d");
        

//         // set string length for vertical muscles
//         for (size_t i = 0; i < v_musclesA.size(); ++ i)
//         {
// 	    //A   **Contracting Cable
//             tgBasicActuator * const pMuscleA = v_musclesA[i];
//             assert(pMuscleA != NULL);
//             pMuscleA->setControlInput(verticalRLA1,dt);
            
//             //B   **Elongating Cable
//             tgBasicActuator * const pMuscleB = v_musclesB[i];
//             assert(pMuscleB != NULL);
//             pMuscleB->setControlInput(verticalRLB1,dt);
            
//             // //C
//             // tgBasicActuator * const pMuscleC = v_musclesC[i];
//             // assert(pMuscleC != NULL);
//             // pMuscleC->setControlInput(verticalRL);
            
//             // //D
//             // tgBasicActuator * const pMuscleD = v_musclesD[i];
//             // assert(pMuscleD != NULL);
//             // pMuscleD->setControlInput(verticalRL);
//         }
      
//     }
// }


//Gear Ratio 2: 1-2-3.2-4.66
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

	    // logging
	    //std::cout << "got to right before onstep observer" << std::endl;
	    m_dataObserver.onStep(subject, updateTime);
	    //notifyStep(m_updateTime);

	    // Bend & Unbend
	    if(verticalRLA1 <= 2.0 && state == -1.0)  //min length of cable
	      {
		state = 1.0;
	      }
	    else if (verticalRLA1 >= 4.0 && state == 1.0)  //stop at upright position
	      {
		state = -1.0;
	      }

	    if (state == -1.0)
	      {
		verticalRLA1 -= dL;
		verticalRLA2 -= dL;
		verticalRLA3 -= dL*1.26;
		verticalRLA4 -= dL*1.4;
		
		verticalRLB1 += dL;
		verticalRLB2 += dL;
		verticalRLB3 += dL*1.26;
		verticalRLB4 += dL*1.4;
	      }
	    else if (state == 1.0)
	      {
		verticalRLA1 += dL;
		verticalRLA2 += dL;
		verticalRLA3 += dL*1.26;
		verticalRLA4 += dL*1.4;
		
		verticalRLB1 -= dL;
		verticalRLB2 -= dL;
		verticalRLB3 -= dL*1.26;
		verticalRLB4 -= dL*1.4;
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
	//A   **Contracting Cable
	   
	tgBasicActuator * const pMuscleA1 = v_musclesA[0];
	tgBasicActuator * const pMuscleA2 = v_musclesA[1];
	tgBasicActuator * const pMuscleA3 = v_musclesA[2];
	tgBasicActuator * const pMuscleA4 = v_musclesA[3];
	assert(pMuscleA1 != NULL);
	assert(pMuscleA2 != NULL);
	assert(pMuscleA3 != NULL);
	assert(pMuscleA4 != NULL);
	pMuscleA1->setControlInput(verticalRLA1,dt);
	pMuscleA2->setControlInput(verticalRLA2,dt);
	pMuscleA3->setControlInput(verticalRLA3,dt);
	pMuscleA4->setControlInput(verticalRLA4,dt);
            
	//B   **Elongating Cable
	tgBasicActuator * const pMuscleB1 = v_musclesB[0];
	tgBasicActuator * const pMuscleB2 = v_musclesB[1];
	tgBasicActuator * const pMuscleB3 = v_musclesB[2];
	tgBasicActuator * const pMuscleB4 = v_musclesB[3];
	assert(pMuscleB1 != NULL);
	assert(pMuscleB2 != NULL);
	assert(pMuscleB3 != NULL);
	assert(pMuscleB4 != NULL);
	pMuscleB1->setControlInput(verticalRLB1,dt);
	pMuscleB2->setControlInput(verticalRLB2,dt);
	pMuscleB3->setControlInput(verticalRLB3,dt);
	pMuscleB4->setControlInput(verticalRLB4,dt);
            
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

