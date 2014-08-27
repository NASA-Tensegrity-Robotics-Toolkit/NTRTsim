#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "NeuroAdapter.h"
#include "../Configuration/configuration.h"

using namespace std;

NeuroAdapter::NeuroAdapter() :
totalTime(0.0)
{
}
NeuroAdapter::~NeuroAdapter(){};

void NeuroAdapter::initialize(NeuroEvolution *evo,bool isLearning,configuration configdata)
{
	numberOfActions=configdata.getDoubleValue("numberOfActions");
	numberOfStates=configdata.getDoubleValue("numberOfStates");
	numberOfControllers=configdata.getDoubleValue("numberOfControllers");
	totalTime=0.0;

	//This Function initializes the parameterset from evo.
	this->neuroEvo = evo;
	if(isLearning)
	{
		currentControllers = this->neuroEvo->nextSetOfControllers();
	}
	else
	{
		currentControllers = this->neuroEvo->nextSetOfControllers();
		for(int i=0;i<currentControllers.size();i++)
		{
			stringstream ss;
			ss<<"logs/bestParameters-"<<this->neuroEvo->suffix<<"-"<<i<<".nnw";
			currentControllers[i]->loadFromFile(ss.str().c_str());
//			currentControllers[i]->getNn()->loadWeights(ss.str().c_str());
		}
	}
	errorOfFirstController=0.0;
}

vector<vector<double> > NeuroAdapter::step(double deltaTimeSeconds,vector<double> state)
{
	totalTime+=deltaTimeSeconds;
//	cout<<"NN adapter, state: "<<state[0]<<" "<<state[1]<<" "<<state[2]<<" "<<state[3]<<" "<<state[4]<<" "<<endl;
	vector< vector<double> > actions;
	if(numberOfStates>0)
	{
		double *inputs = new double[numberOfStates];
		if(numberOfStates!=3)
		{
			cout<<"Warning: numberOfStates is not 3"<<endl;
		}
		//scale inputs to 0-1 from -1 to 1 (unit vector provided from the controller).
		double length=sqrt(state[0]*state[0]+state[1]*state[1]+state[2]*state[2]);
		inputs[0]=state[0] / length / 2.0 + 0.5;
		inputs[1]=state[1] / length / 2.0 + 0.5;
		inputs[2]=state[2] / length / 2.0 + 0.5;
		for(int i=0;i<currentControllers.size();i++)
		{
			double *output=currentControllers[i]->getNn()->feedForwardPattern(inputs);
			vector<double> tmpAct;
			for(int j=0;j<numberOfActions;j++)
			{
				tmpAct.push_back(output[j]);
			}
			actions.push_back(tmpAct);
		}
		delete[]inputs;
	}
	else
	{
		for(int i=0;i<currentControllers.size();i++)
		{
			vector<double> tmpAct;
			for(int j=0;j<currentControllers[i]->statelessParameters.size();j++)
			{
				tmpAct.push_back(currentControllers[i]->statelessParameters[j]);
			}
			actions.push_back(tmpAct);
		}
	}


	return actions;
}

void NeuroAdapter::endEpisode(vector<double> scores)
{
	if(scores.size()==0)
	{
		vector< double > tmp(1);
		tmp[0]=-1;
		neuroEvo->updateScores(tmp);
		cout<<"Exploded"<<endl;
	}
	else
	{
		cout<<"Dist Moved: "<<scores[0]<<" energy: "<<scores[1]<<endl;
//		double combinedScore=scores[0]*1.0-scores[1]*1.0;
		neuroEvo->updateScores(scores);
	}
	return;
}
