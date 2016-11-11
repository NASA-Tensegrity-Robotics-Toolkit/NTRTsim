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
 * @file MGController.cpp
 * @brief Implementing a hand-tuned controller for MountainGoat's legs.
 * @author Dawn Hustig-Schultz
 * @date Aug 2016
 * @version 1.1.0
 * $Id$
 */

//This module
#include "MGController.h"
//This application
#include "dev/dhustigschultz/MountainGoat_Anchored/MountainGoatAnchored.h"

//This library
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"

//The C++ standard library
#include <string>
#include <stdexcept>
#include <cassert>
#include <vector>
#include <cmath>


# define M_PI 3.14159265358979323846    

MGController::MGController(double timestep) :
    m_totalTime(0.0),
    dt(timestep) {}

void MGController::onSetup(BaseQuadModelLearning& subject){
    this->m_totalTime=0.0;
}
    
void MGController::onStep(BaseQuadModelLearning& subject, double dt){

    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;    

    setLeftThighTargetLength(subject, dt);

    setRearLeftLegToAbdomenTargetLength(subject, dt);
    setLeftHipTargetLength(subject, dt);

    setSpineTargetLength(subject, dt);
    setOtherLegsTargetLength(subject, dt);
    setOtherHipsTargetLength(subject, dt);

    moveAllMotors(subject, dt);
}

void MGController::onTeardown(BaseQuadModelLearning& subject) {

} 

void MGController::moveAllMotors(BaseQuadModelLearning& subject, double dt){

    const std::vector<tgSpringCableActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = tgCast::cast<tgSpringCableActuator, tgBasicActuator>(muscles[i]);
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

}

void MGController::setLeftThighTargetLength(BaseQuadModelLearning& subject, double dt){
    
    //May need to adjust amplitudes and phases accordingly. 
    double newLengthMid = 0;
    double newLengthOuter = 0;
    double newLengthInner = 0;

    const double thighLength = 15.0;
    const double thighOuterLength = sqrt(185);

    const double amplitude1 = thighLength;
    const double amplitude2 = thighOuterLength/2;
    const double angular_freq = 8;
    const double phaseMid = 3*M_PI/2;
    const double phaseOuter = phaseMid;
    const double dcOffset = thighLength;
    const double dcOffset2 = thighOuterLength/2;

    const std::vector<tgBasicActuator*> thighMid = subject.find<tgBasicActuator>("left hindleg central thigh");
    const std::vector<tgBasicActuator*> thighOuter = subject.find<tgBasicActuator>("left hindleg outer thigh");
    const std::vector<tgBasicActuator*> thighInner = subject.find<tgBasicActuator>("left hindleg inner thigh");

    assert(thighMid[0] != NULL);
    assert(thighOuter[0] != NULL);
    assert(thighInner[0] != NULL);

    newLengthMid = amplitude1 * sin(angular_freq * m_totalTime + phaseMid) + dcOffset;
    newLengthOuter = amplitude2 * sin(angular_freq * m_totalTime + phaseOuter) + dcOffset2;
    newLengthInner = 0;
    
    thighMid[0]->setControlInput(newLengthMid);
    thighOuter[0]->setControlInput(newLengthOuter);
    thighInner[0]->setControlInput(newLengthOuter); 

}

void MGController::setRearLeftLegToAbdomenTargetLength(BaseQuadModelLearning& subject, double dt){

    double newLength1 = 0;
    double newLength2 = 0;
    const double frontLegToAbdomenLength = sqrt(486);
    const double rearLegToAbdomenLength = sqrt(286);

    const double amplitude1 = frontLegToAbdomenLength/3; 
    const double amplitude2 = rearLegToAbdomenLength/3; 
    const double angular_freq = 8;
    const double phase1 = M_PI/2;
    const double phase2 = 3*M_PI/2;
    
    const double dcOffset1 = frontLegToAbdomenLength;
    const double dcOffset2 = rearLegToAbdomenLength;

    const std::vector<tgBasicActuator*> frontLegToAbdomen = subject.find<tgBasicActuator>("left hindleg limb front abdomen"); 
    const std::vector<tgBasicActuator*> rearLegToAbdomen = subject.find<tgBasicActuator>("left hindleg limb rear abdomen");

    assert(frontLegToAbdomen[0] != NULL);
    assert(rearLegToAbdomen[0] != NULL);

    newLength1 = amplitude1 * sin(angular_freq * m_totalTime + phase1) + dcOffset1;
    newLength2 = amplitude2 * sin(angular_freq * m_totalTime + phase2) + dcOffset2;

    frontLegToAbdomen[0]->setControlInput(newLength1);
    rearLegToAbdomen[0]->setControlInput(newLength2);
}

void MGController::setLeftHipTargetLength(BaseQuadModelLearning& subject, double dt){

    double newLengthFront = 0;
    double newLengthRear = 0;
    double newLengthOther = 0;
    double newLengthOther2 = 0; //Short muscles

    const double leftFrontHipLength = sqrt(101);  
    const double leftRearHipLength = sqrt(101);
    const double leftOtherLength = sqrt(281); 
    const double leftOther2Length = sqrt(2);

    const double amplitude1 = leftFrontHipLength; 
    const double amplitude2 = leftRearHipLength; 
    const double angular_freq = 8;
    const double phaseRear = 3*M_PI/2;
    const double phaseFront = M_PI/2;
    const double dcOffset1 = leftFrontHipLength;
    const double dcOffset2 = leftRearHipLength;
 
    const std::vector<tgBasicActuator*> leftHipFront = subject.find<tgBasicActuator>("left hip front mid");
    const std::vector<tgBasicActuator*> leftHipRear = subject.find<tgBasicActuator>("left hip rear mid");
    const std::vector<tgBasicActuator*> leftHipOther = subject.find<tgBasicActuator>("other left hip");
    const std::vector<tgBasicActuator*> leftHipOther2 = subject.find<tgBasicActuator>("other2 left hip");

    assert(leftHipFront[0] != NULL);
    assert(leftHipRear[0] != NULL);
    assert(leftHipOther[0] != NULL);
    assert(leftHipOther2[0] != NULL);

    newLengthFront = dcOffset1 - amplitude1 * sin(angular_freq * m_totalTime + phaseFront);
    newLengthRear = dcOffset2 - amplitude2 * sin(angular_freq * m_totalTime + phaseRear);
    newLengthOther = 0.8*leftOtherLength; 
    newLengthOther2 = 0.8*leftOther2Length;

    leftHipFront[0]->setControlInput(newLengthFront);
    leftHipRear[0]->setControlInput(newLengthRear);
    
    for (std::size_t i=0; i < leftHipOther.size(); i++){
    	leftHipOther[i]->setControlInput(newLengthOther);
    }

    for (std::size_t i=0; i < leftHipOther2.size(); i++){
    	leftHipOther2[i]->setControlInput(newLengthOther2);
    }

}

void MGController::setSpineTargetLength(BaseQuadModelLearning& subject, double dt){

    //Upper and spiral muscle new lengths
    double newLengthUFR = 0;
    double newLengthUFL = 0;
    double newLengthSpiral = 0;
    double newLengthURR = 0;
    double newLengthURL = 0;
    double newLengthSpine2 = 0;

    //Lower muscle new lengths
    double newLengthLFR = 0;
    double newLengthLFL = 0;
    double newLengthLRR = 0;
    double newLengthLRL = 0;

    //Upper and spiral muscle lengths
    const double upperFRLength = sqrt(281);
    const double upperFLLength = upperFRLength;
    const double spiralLength = sqrt(321);
    const double upperRRLength = upperFRLength;
    const double upperRLLength = upperFRLength;
    

    //Lower muscle lengths
    const double lowerFRLength = sqrt(281);
    const double lowerFLLength = upperFRLength;
    const double lowerRRLength = upperFRLength;
    const double lowerRLLength = upperFRLength;
    const double spine2Length = 1;

    //Upper and spiral muscle amplitudes
    const double amplitude1 = upperFRLength; 
    const double amplitude2 = upperFLLength; 
    const double amplitude3 = spiralLength; 
    const double amplitude4 = upperRRLength;
    const double amplitude5 = upperRLLength;

    //Lower muscle amplitudes
    const double amplitude6 = lowerFRLength; 
    const double amplitude7 = lowerFLLength; 
    const double amplitude8 = lowerRRLength;
    const double amplitude9 = lowerRLLength;

    const double angular_freq = 8;

    //Upper and spiral muscle phases
    const double phaseUFR = 3*M_PI/2;
    const double phaseUFL = M_PI/2;
    const double phaseSpiral = 3*M_PI/2;
    const double phaseURR = M_PI/2;
    const double phaseURL = 3*M_PI/2;

    //Lower muscle phases
    const double phaseLFR = M_PI/2;
    const double phaseLFL = M_PI/2;
    const double phaseLRR = M_PI/2;
    const double phaseLRL = M_PI/2;

    //Upper and spiral muscle length offsets
    const double dcOffset1 = upperFRLength;
    const double dcOffset2 = upperFLLength;
    const double dcOffset3 = spiralLength; 
    const double dcOffset4 = upperRRLength;
    const double dcOffset5 = upperRLLength;

    //Lower muscle length offsets
    const double dcOffset6 = lowerFRLength;
    const double dcOffset7 = lowerFLLength; 
    const double dcOffset8 = lowerRRLength;
    const double dcOffset9 = lowerRLLength;

    //Upper and spiral muscles
    const std::vector<tgBasicActuator*> spineUpperFrontRight = subject.find<tgBasicActuator>("spine_all_main_front_upper_right");
    const std::vector<tgBasicActuator*> spineUpperFrontLeft = subject.find<tgBasicActuator>("spine_all_main_front_upper_left");
    const std::vector<tgBasicActuator*> spineSpiral = subject.find<tgBasicActuator>("spine_all_spiral");
    const std::vector<tgBasicActuator*> spineUpperRearRight = subject.find<tgBasicActuator>("spine_all_main_rear_upper_right");
    const std::vector<tgBasicActuator*> spineUpperRearLeft = subject.find<tgBasicActuator>("spine_all_main_rear_upper_left");

    //Lower muscles
    const std::vector<tgBasicActuator*> spineLowerFrontRight = subject.find<tgBasicActuator>("spine_all_main_front_lower_right");
    const std::vector<tgBasicActuator*> spineLowerFrontLeft = subject.find<tgBasicActuator>("spine_all_main_front_lower_left");
    const std::vector<tgBasicActuator*> spineLowerRearRight = subject.find<tgBasicActuator>("spine_all_main_rear_lower_right");
    const std::vector<tgBasicActuator*> spineLowerRearLeft = subject.find<tgBasicActuator>("spine_all_main_rear_lower_left");
    const std::vector<tgBasicActuator*> spine2 = subject.find<tgBasicActuator>("spine2");

    //Making sure none of the vectors are empty
    assert(spineUpperFrontRight[0] != NULL);
    assert(spineUpperFrontLeft[0] != NULL);
    assert(spineSpiral[0] != NULL);
    assert(spineUpperRearRight[0] != NULL);
    assert(spineUpperRearLeft[0] != NULL);

    assert(spineLowerFrontRight[0] != NULL);
    assert(spineLowerFrontLeft[0] != NULL);
    assert(spineLowerRearRight[0] != NULL);
    assert(spineLowerRearLeft[0] != NULL);
    assert(spine2[0] != NULL);

    //maybe just try setting some of these to the initial length, rather than using sinewaves, and see if it stays put as in the passive model.
    newLengthUFR = 0;//dcOffset1 - amplitude1 * sin(angular_freq * m_totalTime + phaseUFR);
    newLengthUFL = 0;//dcOffset2 - amplitude2 * sin(angular_freq * m_totalTime + phaseUFL);
    newLengthSpiral = 0;//dcOffset3 - amplitude3 * sin(angular_freq * m_totalTime + phaseSpiral);
    newLengthURR = 0;//dcOffset4 - amplitude4 * sin(angular_freq * m_totalTime + phaseURR);
    newLengthURL = 0;//dcOffset5 - amplitude5 * sin(angular_freq * m_totalTime + phaseURL);

    newLengthLFR = 0;//dcOffset6 - amplitude6 * sin(angular_freq * m_totalTime + phaseLFR);
    newLengthLFL = 0;//dcOffset7 - amplitude7 * sin(angular_freq * m_totalTime + phaseLFL);
    newLengthLRR = 0;//dcOffset8 - amplitude8 * sin(angular_freq * m_totalTime + phaseLRR);
    newLengthLRL = 0;//dcOffset9 - amplitude9 * sin(angular_freq * m_totalTime + phaseLRL); 
    newLengthSpine2 = 0;//spine2Length;

    //Moving all of the same with this tag.... if want to move last only, use size -1 to get the index of last
    for (std::size_t i=0; i < spineUpperFrontRight.size(); i++){
    	spineUpperFrontRight[i]->setControlInput(newLengthUFR);
    }

    for (std::size_t i=0; i < spineUpperFrontLeft.size(); i++){
    	spineUpperFrontLeft[i]->setControlInput(newLengthUFL);
    }

    for (std::size_t i=0; i < spineSpiral.size(); i++){
    	spineSpiral[i]->setControlInput(newLengthSpiral);
    }

    for (std::size_t i=0; i < spineUpperRearRight.size(); i++){
    	spineUpperRearRight[i]->setControlInput(newLengthURR);
    }

    for (std::size_t i=0; i < spineUpperRearLeft.size(); i++){
    	spineUpperRearLeft[i]->setControlInput(newLengthURL);
    }

    for (std::size_t i=0; i < spineLowerFrontRight.size(); i++){
    	spineLowerFrontRight[i]->setControlInput(newLengthLFR);
    }

    for (std::size_t i=0; i < spineLowerFrontLeft.size(); i++){
    	spineLowerFrontLeft[i]->setControlInput(newLengthLFL);
    }

    for (std::size_t i=0; i < spineLowerRearRight.size(); i++){
    	spineLowerRearRight[i]->setControlInput(newLengthLRR);
    }

    for (std::size_t i=0; i < spineLowerRearLeft.size(); i++){
    	spineLowerRearLeft[i]->setControlInput(newLengthLRL);
    }

    for (std::size_t i=0; i < spine2.size(); i++){
    	spine2[i]->setControlInput(newLengthSpine2);
    }
}

void MGController::setOtherHipsTargetLength(BaseQuadModelLearning& subject, double dt){

    double newLengthLong = 0;
    double newLengthShort = 0;

    const double shoulderHipLongLength = sqrt(281); 
    const double shoulderHipShortLength = sqrt(101); 

    const double amplitude1 = shoulderHipLongLength; 
    const double amplitude2 = shoulderHipShortLength; 

    const double angular_freq = 8;

    const double phaseLong = 3*M_PI/2;
    const double phaseShort = 3*M_PI/2;

    const double dcOffset1 = shoulderHipLongLength;
    const double dcOffset2 = shoulderHipShortLength;
 
    const std::vector<tgBasicActuator*> shoulderHipLong = subject.find<tgBasicActuator>("george");
    const std::vector<tgBasicActuator*> shoulderHipShort = subject.find<tgBasicActuator>("harold");

    assert(shoulderHipLong[0] != NULL);
    assert(shoulderHipShort[0] != NULL);
 
    newLengthLong = 0; //dcOffset1 - amplitude1 * sin(angular_freq * m_totalTime + phaseLong);
    newLengthShort = 0;  //dcOffset2 - amplitude2 * sin(angular_freq * m_totalTime + phaseShort);
    
    for (std::size_t i=0; i < shoulderHipLong.size(); i++){
    	shoulderHipLong[i]->setControlInput(newLengthLong);
    }

    for (std::size_t i=0; i < shoulderHipShort.size(); i++){
    	shoulderHipShort[i]->setControlInput(newLengthShort);
    }

}

void MGController::setOtherLegsTargetLength(BaseQuadModelLearning& subject, double dt){

    // Variables for all the new actuator lengths:
    double newLengthBicep = 0;
    double newLengthFrontAbdomen = 0;
    double newLengthHindAbdomen = 0;

    double newLengthFrontHorizAbdomen = 0;
    double newLengthHindHorizAbdomen = 0;

    double newLengthTricep = 0;
    double newLengthFrontTricep = 0;
    double newLengthMidBicep = 0;

    // All the resting actuator lengths:
    const double bicepLength = sqrt(185); 
    const double frontAbdomenLength = sqrt(125); 
    const double hindAbdomenLength = sqrt(126);

    const double frontHorizAbdomenLength = sqrt(66); 
    const double hindHorizAbdomenLength = sqrt(146);

    const double tricepLength = sqrt(165);
    const double frontTricepLength = sqrt(245);
    const double midBicepLength = 15;


    // Amplitudes:
    const double amplitude1 = bicepLength; 
    const double amplitude2 = frontAbdomenLength; 
    const double amplitude3 = hindAbdomenLength;
    const double amplitude4 = frontHorizAbdomenLength;
    const double amplitude5 = hindHorizAbdomenLength; 
    const double amplitude6 = tricepLength; 
    const double amplitude7 = frontTricepLength;
    const double amplitude8 = midBicepLength;


    // Angular frequency and phase offsets:
    const double angular_freq = 8;
    const double phase = 3*M_PI/2;

    // DC Offsets:
    const double dcOffset1 = bicepLength;
    const double dcOffset2 = frontAbdomenLength;
    const double dcOffset3 = hindAbdomenLength;
    const double dcOffset4 = frontHorizAbdomenLength;
    const double dcOffset5 = hindHorizAbdomenLength;
    const double dcOffset6 = tricepLength;
    const double dcOffset7 = frontTricepLength;
    const double dcOffset8 = midBicepLength;
 
    const std::vector<tgBasicActuator*> bicep = subject.find<tgBasicActuator>("bob");
    const std::vector<tgBasicActuator*> frontAb = subject.find<tgBasicActuator>("clive");
    const std::vector<tgBasicActuator*> hindAb = subject.find<tgBasicActuator>("neil");
    const std::vector<tgBasicActuator*> frontHorizAb = subject.find<tgBasicActuator>("reginald");
    const std::vector<tgBasicActuator*> hindHorizAb = subject.find<tgBasicActuator>("william");
    const std::vector<tgBasicActuator*> tricep = subject.find<tgBasicActuator>("clarence");
    const std::vector<tgBasicActuator*> frontTricep = subject.find<tgBasicActuator>("alyosis");
    const std::vector<tgBasicActuator*> midBicep = subject.find<tgBasicActuator>("xander");

    assert(bicep[0] != NULL);
    assert(frontAb[0] != NULL);
    assert(hindAb[0] != NULL);

    assert(frontHorizAb[0] != NULL);
    assert(hindHorizAb[0] != NULL);
    assert(tricep[0] != NULL);
    assert(frontTricep[0] != NULL);
    assert(midBicep[0] != NULL);
 
    newLengthBicep = 0;//dcOffset1 - amplitude1 * sin(angular_freq * m_totalTime + phase); // 0.8*bicepLength; 
    newLengthFrontAbdomen = 0;//dcOffset2 - amplitude2 * sin(angular_freq * m_totalTime + phase); //0.8*frontAbdomenLength; 
    newLengthHindAbdomen = 0;//dcOffset3 - amplitude3 * sin(angular_freq * m_totalTime + phase); //0.8*hindAbdomenLength; 

    newLengthFrontHorizAbdomen = 0;//dcOffset4 - amplitude4 * sin(angular_freq * m_totalTime + phase); //0.8*frontHorizAbdomenLength; 
    newLengthHindHorizAbdomen = 0;//dcOffset5 - amplitude5 * sin(angular_freq * m_totalTime + phase); //0.8*hindHorizAbdomenLength; 

    newLengthTricep = 0;//dcOffset6 - amplitude6 * sin(angular_freq * m_totalTime + phase); //0.8*tricepLength; 
    newLengthFrontTricep = 0;//dcOffset7 - amplitude7 * sin(angular_freq * m_totalTime + phase); //0.8*frontTricepLength;
    newLengthMidBicep = 0;//dcOffset8 - amplitude8 * sin(angular_freq * m_totalTime + phase); //0.8*midBicepLength;
    
    for (std::size_t i=0; i < bicep.size(); i++){
    	bicep[i]->setControlInput(newLengthBicep);
    }

    for (std::size_t i=0; i < frontAb.size(); i++){
    	frontAb[i]->setControlInput(newLengthFrontAbdomen);
    }

    for (std::size_t i=0; i < hindAb.size(); i++){
    	hindAb[i]->setControlInput(newLengthHindAbdomen);
    }

    for (std::size_t i=0; i < frontHorizAb.size(); i++){
    	frontHorizAb[i]->setControlInput(newLengthFrontHorizAbdomen);
    }

    for (std::size_t i=0; i < hindHorizAb.size(); i++){
    	hindHorizAb[i]->setControlInput(newLengthHindHorizAbdomen);
    }

    for (std::size_t i=0; i < tricep.size(); i++){
    	tricep[i]->setControlInput(newLengthTricep);
    }

    for (std::size_t i=0; i < frontTricep.size(); i++){
    	frontTricep[i]->setControlInput(newLengthFrontTricep);
    }

    for (std::size_t i=0; i < midBicep.size(); i++){
    	midBicep[i]->setControlInput(newLengthMidBicep);
    }

}
