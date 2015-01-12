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
 * @file T6PrefLengthController.cpp
 * @brief Preferred Length Controller for T6. Constant speed motors are used in muscles to reach preffered length
 * @author Atil Iscen
 * @version 1.0.0
 * $Id$
 */

// This module
#include "SuperBallLearningController.h"
// This application
#include "SuperBallModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
SuperBallPrefLengthController::SuperBallPrefLengthController(const double initialLength)
{
	this->m_initialLengths=initialLength;
	this->m_totalTime=0.0;

	evolution = new AnnealEvolution("superball");

}

//Fetch all the muscles and set their preferred length
void SuperBallPrefLengthController::onSetup(SuperBallModel& subject)
{
	m_totalTime=0;
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(this->m_initialLengths,0.0001);
	}

    btTransform boxTr;
    boxTr.setIdentity();
	double randomAngle=((rand() / (double)RAND_MAX) - 0.5) * 3.1415;
	randomAngle=3.1415/2.0;
	randomAngle=0.0;

	btDynamicsWorld *btworld=subject.getWorld();
	boxTr.setOrigin(btVector3(1000*cos(randomAngle),50,1000*sin(randomAngle)));
	btDefaultMotionState* motSt = new btDefaultMotionState(boxTr);
    btRigidBody::btRigidBodyConstructionInfo const rbInfo1(1,motSt, new btBoxShape(btVector3(1.0,1.0,1.0)));
	goalPoint = new btRigidBody(rbInfo1);
	btworld->addRigidBody(goalPoint);

	boxTr.setIdentity();
	boxTr.setOrigin(btVector3(0,1.1,0));
	btDefaultMotionState* motSt2 = new btDefaultMotionState(boxTr);
    btRigidBody::btRigidBodyConstructionInfo const rbInfo2(0.0,motSt2,new btBoxShape(btVector3(500.0,0.1,500.0)));
	btRigidBody *groundBox = new btRigidBody(rbInfo2);
	btworld->addRigidBody(groundBox);

	m_subject = &subject;

}

void SuperBallPrefLengthController::onStep(SuperBallModel& subject, double dt)
{
	if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

	//Store the initial position if the first step
    if(initialPosition.empty())
    {
    	btVector3 pos= subject.getCenter();
		initialPosition.push_back(pos[0]);
		initialPosition.push_back(pos[2]);
    }

    //Move motors for all the muscles
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}

	vector<double> state=subject.getSensorInfo();

////  Output sensor values for testing purposes.
//	std::cout<<m_totalTime;
//	for(int i=0;i<state.size();i++)
//	{
//		std::cout<<","<<state[i];
//	}
//	std::cout<<endl;


	vector< vector< double> > actions = receiveActionsFromEvolution();

	//transform them from 0-1 to the size of the structure
	actions = transformActions(actions);

	//apply these actions to the appropriate muscles according to the sensor values
	applyActions(subject,actions,state);

}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > SuperBallPrefLengthController::transformActions(vector< vector <double> > actions)
{
	double min=6;
	double max=11;
	double range=max-min;
	double scaledAct;
	for(int i=0;i<actions.size();i++)
	{
		for(int j=0;j<actions[i].size();j++)
		{
			scaledAct=actions[i][j]*(range)+min;
			actions[i][j]=scaledAct;
		}
	}
	return actions;
}


typedef std::pair<int,double> mypair;
bool comparator ( const mypair& l, const mypair& r)
   { return l.second < r.second; }


//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void SuperBallPrefLengthController::applyActions(SuperBallModel& subject, vector< vector <double> > actions, vector<double> nodeRayDistances)
{
	vector< btVector3 > nodePositions=subject.getSensorPositions();
	vector< btVector3 > nodeDirections=subject.getSensorOrientations();

	vector< mypair > nodeHeightWithIndex;
	for(int i=0;i<nodePositions.size();i++)
	{
		//use height of each node (closeness to the ground is not working yet.)
		nodeHeightWithIndex.push_back(make_pair(i,nodeRayDistances[i]));
	}
	std::sort(nodeHeightWithIndex.begin(),nodeHeightWithIndex.end(),comparator);

	vector<int> groundNodes;
	groundNodes.push_back(nodeHeightWithIndex[0].first);
	groundNodes.push_back(nodeHeightWithIndex[1].first);
	groundNodes.push_back(nodeHeightWithIndex[2].first);

	int a=groundNodes[0];
	int b=groundNodes[1];
	int c=groundNodes[2];

	int numberOfMusclesOnBase=0;
	if(subject.getMusclesPerNodes()[a][b]!=NULL)
		numberOfMusclesOnBase++;
	if(subject.getMusclesPerNodes()[a][c]!=NULL)
		numberOfMusclesOnBase++;
	if(subject.getMusclesPerNodes()[b][c]!=NULL)
		numberOfMusclesOnBase++;

	//if all 3 are connected
	if(numberOfMusclesOnBase==3)
	{
		//find the order or base points
		//assuming we go to z direction
		double maxDistance=-10000;
		double minDistance=10000;
		int furthestNode=-1;
		int closestNode=-1;
		for(int i=0;i<3;i++)
		{

			double dist=goalPoint->getCenterOfMassPosition().distance(nodePositions[groundNodes[i]]);
			if(dist>maxDistance)
			{
				furthestNode=i;
				maxDistance=dist;
			}
			if(dist<minDistance)
			{
				closestNode=i;
				minDistance=dist;
			}
		}
		int groundNodesInOrder[3]={-1,-1,-1};
		for(int i=0;i<3;i++)
		{
			if(i==furthestNode)
				groundNodesInOrder[2]=groundNodes[furthestNode];
			else if(subject.getMusclesPerNodes()[groundNodes[i]][subject.getOtherEndOfTheRod(groundNodes[furthestNode])] != NULL)
				groundNodesInOrder[1]=groundNodes[i];
			else
				groundNodesInOrder[0]=groundNodes[i];
		}
//		cout<<"ground nodes in order: "<<groundNodesInOrder[0]<<" "<<groundNodesInOrder[1]<<" "<<groundNodesInOrder[2]<<" "<<endl;
		//fill node mapping according to these new base points
		subject.fillNodeMappingFromBasePoints(groundNodesInOrder[0],groundNodesInOrder[1],groundNodesInOrder[2]);

		int actionNo=0;
		for(int i=0;i<13;i++)
		{
			for(int j=i+1;j<13;j++)
			{
				if(subject.muscleConnections[i][j]>=0)
				{
					int nodeStartOriginal=i;
					int nodeEndOriginal=j;
					//give the action that corresponds to these nodes originally to the ones with new mapping
					int nodeStartNew=subject.nodeMappingReverse[nodeStartOriginal];
					int nodeEndNew=subject.nodeMappingReverse[nodeEndOriginal];
					tgBasicActuator * correspondingMuscle = subject.musclesPerNodes[nodeStartNew][nodeEndNew];
					if(correspondingMuscle==NULL)
					{
						cout<<"NO MUSCLE EXISTS ACCORDING TO THE NEW MAPPING"<<endl;
//						exit(1);
					}
					if(actionNo>=actions.size())
					{
						cout<<"Warning: actions < number of active muscles, no input given"<<endl;
						continue;
					}
					correspondingMuscle->setControlInput(actions[actionNo][0]);
				}
			}
		}
	}
	else if(numberOfMusclesOnBase==2)
	{
		//3 points, but there are 2 connections.
		//We assume that it's the isoceles triangle.
//				cout<<"3 points on ground, but not connected"<<endl;
		double maxDistance=-50000;
		double minDistance=50000;
		int furthestNode=-1;
		int closestNode=-1;
		for(int i=0;i<3;i++)
		{

			double dist=goalPoint->getCenterOfMassPosition().distance(nodePositions[groundNodes[i]]);
			if(dist>maxDistance)
			{
				furthestNode=groundNodes[i];
				maxDistance=dist;
			}
			if(dist<minDistance)
			{
				closestNode=groundNodes[i];
				minDistance=dist;
			}
		}
		int middleNode=-1;
		for(int i=0;i<3;i++)
		{
			if(groundNodes[i]!=closestNode && groundNodes[i]!=furthestNode)
				middleNode=groundNodes[i];
		}
		//We want to do a flop over the closest node and connected edge.
		//Find closest and conected.
		int landingNode;
		int flippingNode;
		if(subject.musclesPerNodes[closestNode][middleNode]!=NULL)
		{
			flippingNode=middleNode;
		}
		else
		{
			flippingNode=furthestNode;
		}
		//find the node that is connected to these two
		for(int i=0;i<12;i++)
		{
			if(subject.musclesPerNodes[closestNode][i]!=NULL && subject.musclesPerNodes[flippingNode][i]!=NULL)
			{
				landingNode=i;
				break;
			}
		}
		//Select the base triangle based on that edge assuming we are coming from that base to this point.
		int triangle[3]={closestNode,flippingNode,landingNode};
		int triangleInOrder[3]={-1,-1,-1};
		triangleInOrder[2]=landingNode;
		if(subject.musclesPerNodes[closestNode][subject.getOtherEndOfTheRod(landingNode)] != NULL)
		{
			triangleInOrder[1]=closestNode;
			triangleInOrder[0]=flippingNode;
		}
		else
		{
			triangleInOrder[1]=flippingNode;
			triangleInOrder[0]=closestNode;
		}
		subject.fillNodeMappingFromBasePoints(triangleInOrder[0],triangleInOrder[1],triangleInOrder[2]);

		int actionNo=0;
		for(int i=0;i<13;i++)
		{
			for(int j=i+1;j<13;j++)
			{
				if(subject.muscleConnections[i][j]>=0)
				{
					int nodeStartOriginal=i;
					int nodeEndOriginal=j;
					//give the action that corresponds to these nodes originally to the ones with new mapping
					int nodeStartNew=subject.nodeMappingReverse[nodeStartOriginal];
					int nodeEndNew=subject.nodeMappingReverse[nodeEndOriginal];
					tgBasicActuator * correspondingMuscle = subject.musclesPerNodes[nodeStartNew][nodeEndNew];
					if(correspondingMuscle==NULL)
					{
						cout<<"NO MUSCLE EXISTS ACCORDING TO THE NEW MAPPING"<<endl;
						exit(1);
					}
					if(actionNo>=actions.size())
					{
						cout<<"Warning: actions < number of active muscles, no input given"<<endl;
						continue;
					}
					correspondingMuscle->setControlInput(actions[actionNo][0]);
				}
			}
		}
	}
}

void SuperBallPrefLengthController::onTeardown(SuperBallModel& subject)
{
	std::cout<<"TEARDOWN CALLED"<<std::endl;
	vector<double> scores;
	scores.push_back(calculateDistanceMoved());
	evolution->updateScores(scores);
	std::cout<<"Distance moved: "<<scores[0]<<std::endl;

	initialPosition.clear();
}

vector<vector<double> > SuperBallPrefLengthController::receiveActionsFromEvolution()
{
	vector<vector <double> > actions;

	vector<AnnealEvoMember *> members = evolution->nextSetOfControllers();
	for(int i=0;i<members.size();i++)
	{
		actions.push_back(members[i]->statelessParameters);
	}
	return actions;
}



double SuperBallPrefLengthController::calculateDistanceMoved()
{
	vector<double> scores;
	double x= m_subject->getCenter()[0] - goalPoint->getCenterOfMassPosition().getX();
	double z= m_subject->getCenter()[2] - goalPoint->getCenterOfMassPosition().getZ();
	double distanceNew=sqrt(x*x+z*z);
	double xx=initialPosition[0]-goalPoint->getCenterOfMassPosition().getX();
	double zz=initialPosition[1]-goalPoint->getCenterOfMassPosition().getZ();
	double distanceOld=sqrt(xx*xx+zz*zz);
	double distanceMoved=distanceOld-distanceNew;

	//If you want to calculate only the distance moved independent of the target:
//	distanceMoved=sqrt((x-xx)*(x-xx)+(z-zz)*(z-zz));

	return distanceMoved;
}
