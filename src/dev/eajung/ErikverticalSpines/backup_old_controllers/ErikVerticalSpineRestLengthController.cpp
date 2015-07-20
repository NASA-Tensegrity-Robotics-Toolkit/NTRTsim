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
 * @file ErikVerticalSpineRestLengthController.cpp
 * @brief Implementation of a rest length controller for ErikVerticalSpineModel.
 * @author Drew Sabelhaus, Brian Tietz, Michael Fanton
 * @version 1.0.0
 * $Id$
 */

// This module
#include "ErikVerticalSpineRestLengthController.h"
// This application
#include "ErikVerticalSpineModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgString.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

ErikVerticalSpineRestLengthController::ErikVerticalSpineRestLengthController()
{
    VerticalRL = 7.38 ; // cm
    saddleRL1 = 13.037 ; // cm
    saddleRL2 = 13.613 ; // cm
    saddleRL3 = 14.189 ; // cm
    saddleRL4 = 14.766 ; // cm
}

void ErikVerticalSpineRestLengthController::onSetup(ErikVerticalSpineModel& subject)
{
    // Do a one-time update of all cable rest lengths
    // First, get all muscles (cables)
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    
    // get all ErikVertical muscles
    const std::vector<tgBasicActuator*> v_musclesA = subject.getMuscles("Vertical a");
    const std::vector<tgBasicActuator*> v_musclesB = subject.getMuscles("Vertical b");
    const std::vector<tgBasicActuator*> v_musclesC = subject.getMuscles("Vertical c");
    const std::vector<tgBasicActuator*> v_musclesD = subject.getMuscles("Vertical d");
    
    // get saddle muscles and put them in individual vectors
    const std::vector<tgBasicActuator*> s_muscles0 = subject.getMuscles(tgString("saddle", 0));
    const std::vector<tgBasicActuator*> s_muscles1 = subject.getMuscles(tgString("saddle", 1));
    const std::vector<tgBasicActuator*> s_muscles2 = subject.getMuscles(tgString("saddle", 2));
    const std::vector<tgBasicActuator*> s_muscles3 = subject.getMuscles(tgString("saddle", 3));
    
    // set string length for ErikVertical muscles
    for (size_t i = 0; i < v_musclesA.size(); ++ i)
    {
        // A
        tgBasicActuator * const pMuscleA = v_musclesA[i];
        assert(pMuscleA != NULL);
        pMuscleA->setRestLengthSingleStep(VerticalRL);
        
        //B
        tgBasicActuator * const pMuscleB = v_musclesB[i];
        assert(pMuscleB != NULL);
        pMuscleB->setRestLengthSingleStep(VerticalRL);
        
        //C
        tgBasicActuator * const pMuscleC = v_musclesC[i];
        assert(pMuscleC != NULL);
        pMuscleC->setRestLengthSingleStep(VerticalRL);
        
        //D
        tgBasicActuator * const pMuscleD = v_musclesD[i];
        assert(pMuscleD != NULL);
        pMuscleD->setRestLengthSingleStep(VerticalRL);
    }
    
    // set string lengths for saddle muscles
    
    for (size_t i = 0; i < s_muscles0.size(); ++i)
    {
        
        // 1
        tgBasicActuator * const pMuscle1 = s_muscles0[i];
        assert(pMuscle1 != NULL);
        pMuscle1->setRestLengthSingleStep(saddleRL1);
        
        // 2
        tgBasicActuator * const pMuscle2 = s_muscles1[i];
        assert(pMuscle2 != NULL);
        pMuscle2->setRestLengthSingleStep(saddleRL2);
        
        // 3
        tgBasicActuator * const pMuscle3 = s_muscles2[i];
        assert(pMuscle3 != NULL);
        pMuscle3->setRestLengthSingleStep(saddleRL3);
        
        // 4
        tgBasicActuator * const pMuscle4 = s_muscles3[i];
        assert(pMuscle4 != NULL);
        pMuscle4->setRestLengthSingleStep(saddleRL4);
    }
    
    
   //    // first ErikVertical
//    for (size_t i = 0; i < 4; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//
//        double desiredRestLength = ErikVerticalRL;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
//    // 1st saddle
//    for (size_t i = 4; i < 8; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = saddleRL1;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//
//    // 2nd ErikVertical
//    for (size_t i = 8; i < 12; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = ErikVerticalRL;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
//    // 2nd saddle
//    for (size_t i = 12; i < 16; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = saddleRL2;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
//    // 3rd ErikVertical
//    for (size_t i = 16; i < 20; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = ErikVerticalRL;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
    // 3rd  saddle
//    for (size_t i = 20; i < 24; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = saddleRL3;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
//    // 4th ErikVertical
//    for (size_t i = 24; i < 28; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = ErikVerticalRL;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//    
    // 4th  saddle
//    for (size_t i = 28; i < 32; ++i)
//    {
//        tgBasicActuator * const pMuscle = muscles[i];
//        assert(pMuscle != NULL);
//        
//        double desiredRestLength = saddleRL4;
//        // Note that the single step version of setRestLength is used here,
//        // since we only want to call it once (not iteratively like the original.)
//        pMuscle->setRestLengthSingleStep(desiredRestLength);
//    }
//
//    
}

void ErikVerticalSpineRestLengthController::onStep(ErikVerticalSpineModel& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Do a one-time update of all cable rest lengths
        // First, get all muscles (cables)
        const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
        
        // get all ErikVertical muscles
        const std::vector<tgBasicActuator*> v_musclesA = subject.getMuscles("Vertical a");
        const std::vector<tgBasicActuator*> v_musclesB = subject.getMuscles("Vertical b");
        const std::vector<tgBasicActuator*> v_musclesC = subject.getMuscles("Vertical c");
        const std::vector<tgBasicActuator*> v_musclesD = subject.getMuscles("Vertical d");
        
        // get saddle muscles and put them in individual vectors
        const std::vector<tgBasicActuator*> s_muscles0 = subject.getMuscles(tgString("saddle", 0));
        const std::vector<tgBasicActuator*> s_muscles1 = subject.getMuscles(tgString("saddle", 1));
        const std::vector<tgBasicActuator*> s_muscles2 = subject.getMuscles(tgString("saddle", 2));
        const std::vector<tgBasicActuator*> s_muscles3 = subject.getMuscles(tgString("saddle", 3));
        
        // set string length for ErikVertical muscles
        for (size_t i = 0; i < v_musclesA.size(); ++ i)
        {
            // A
            tgBasicActuator * const pMuscleA = v_musclesA[i];
            assert(pMuscleA != NULL);
            pMuscleA->setRestLengthSingleStep(VerticalRL);
            
            //B
            tgBasicActuator * const pMuscleB = v_musclesB[i];
            assert(pMuscleB != NULL);
            pMuscleB->setRestLengthSingleStep(VerticalRL);
            
            //C
            tgBasicActuator * const pMuscleC = v_musclesC[i];
            assert(pMuscleC != NULL);
            pMuscleC->setRestLengthSingleStep(ErikVerticalRL);
            
            //D
            tgBasicActuator * const pMuscleD = v_musclesD[i];
            assert(pMuscleD != NULL);
            pMuscleD->setRestLengthSingleStep(VerticalRL);
        }
        
        // set string lengths for saddle muscles
        
        for (size_t i = 0; i < s_muscles0.size(); ++i)
        {
            
            // 1
            tgBasicActuator * const pMuscle1 = s_muscles0[i];
            assert(pMuscle1 != NULL);
            pMuscle1->setRestLengthSingleStep(saddleRL1);
            
            // 2
            tgBasicActuator * const pMuscle2 = s_muscles1[i];
            assert(pMuscle2 != NULL);
            pMuscle2->setRestLengthSingleStep(saddleRL2);
            
            // 3
            tgBasicActuator * const pMuscle3 = s_muscles2[i];
            assert(pMuscle3 != NULL);
            pMuscle3->setRestLengthSingleStep(saddleRL3);
            
            // 4
            tgBasicActuator * const pMuscle4 = s_muscles3[i];
            assert(pMuscle4 != NULL);
            pMuscle4->setRestLengthSingleStep(saddleRL4);
        }
        
//
//        // first ErikVertical
//        for (size_t i = 0; i < 4; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = ErikVerticalRL;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//        
//        // 1st saddle
//        for (size_t i = 4; i < 8; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = saddleRL1;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//
//        // 2nd ErikVertical
//        for (size_t i = 8; i < 12; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = ErikVerticalRL;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//        
//        // 2nd saddle
//        for (size_t i = 12; i < 16; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = saddleRL2;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//
//        // 3rd ErikVertical
//        for (size_t i = 16; i < 20; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = ErikVerticalRL;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//        
//        // 3rd  saddle
//        for (size_t i = 20; i < 24; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = saddleRL3;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//
//        // 4th ErikVertical
//        for (size_t i = 24; i < 28; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = ErikVerticalRL;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//        
//        // 4th  saddle
//        for (size_t i = 28; i < 32; ++i)
//        {
//            tgBasicActuator * const pMuscle = muscles[i];
//            assert(pMuscle != NULL);
//            
//            double desiredRestLength = saddleRL4;
//            // Note that the single step version of setRestLength is used here,
//            // since we only want to call it once (not iteratively like the original.)
//            pMuscle->setRestLengthSingleStep(desiredRestLength);
//        }
//        
        

    }
}
