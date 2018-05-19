/*
    2  * Copyright © 2012, United States Government, as represented by the
    3  * Administrator of the National Aeronautics and Space Administration.
    4  * All rights reserved.
    5  * 
    6  * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
    7  * under the Apache License, Version 2.0 (the "License");
    8  * you may not use this file except in compliance with the License.
    9  * You may obtain a copy of the License at
   10  * http://www.apache.org/licenses/LICENSE-2.0.
   11  * 
   12  * Unless required by applicable law or agreed to in writing,
   13  * software distributed under the License is distributed on an
   14  * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
   15  * either express or implied. See the License for the specific language
   16  * governing permissions and limitations under the License.
   17 */
   18 
   28 #include "AnnealEvolution.h"
   29 #include "learning/Configuration/configuration.h"
   30 #include "core/tgString.h"
   31 #include "helpers/FileHelpers.h"
   32 #include <iostream>
   33 #include <numeric>
   34 #include <string>
   35 #include <sstream>
   36 #include <stdexcept>
   37 
   38 using namespace std;
   39 
   40 #ifdef _WIN32
   41 
   42 //  Windows
   43 #define rdtsc  __rdtsc
   44 
   45 #else
   46 
   47 //  For everything else
   48 unsigned long long rdtsc(){
   49     unsigned int lo,hi;
   50     __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
   51     return ((unsigned long long)hi << 32) | lo;
   52 }
   53 
   54 #endif
   55 
   56 AnnealEvolution::AnnealEvolution(std::string suff, std::string config, std::string path) :
   57 suffix(suff),
   58 Temp(1.0)
   59 {
   60     currentTest=0;
   61     subTests = 0;
   62     generationNumber=0;
   63                 
   64                 if (path != "")
   65                 {
   66                                 resourcePath = FileHelpers::getResourcePath(path);
   67                 }
   68                 else
   69                 {
   70                                 resourcePath = "";
   71                 }
   72                 
   73                 std::string configPath = resourcePath + config;
   74                 
   75     configuration myconfigdataaa;
   76     myconfigdataaa.readFile(configPath);
   77     populationSize=myconfigdataaa.getintvalue("populationSize");
   78     numberOfElementsToMutate=myconfigdataaa.getintvalue("numberOfElementsToMutate");
   79     numberOfTestsBetweenGenerations=myconfigdataaa.getintvalue("numberOfTestsBetweenGenerations");
   80     numberOfSubtests=myconfigdataaa.getintvalue("numberOfSubtests");
   81     numberOfControllers=myconfigdataaa.getintvalue("numberOfControllers"); //shared with ManhattanToyController
   82     leniencyCoef=myconfigdataaa.getDoubleValue("leniencyCoef");
   83     coevolution=myconfigdataaa.getintvalue("coevolution");
   84     seeded = myconfigdataaa.getintvalue("startSeed");
   85     
   86     bool learning = myconfigdataaa.getintvalue("learning");
   87 
   88     srand(rdtsc());
   89     eng.seed(rdtsc());
   90 
   91     for(int j=0;j<numberOfControllers;j++)
   92     {
   93         populations.push_back(new AnnealEvoPopulation(populationSize,myconfigdataaa));
   94     }
   95     
   96     // Overwrite the random parameters based on data
   97     if(seeded) // Test that the file exists
   98     {
   99         for(int i = 0; i < numberOfControllers; i++)
  100         {
  101             AnnealEvoMember* seededPop = populations[i]->controllers.back();
  102             stringstream ss;
  103             ss<< resourcePath <<"logs/bestParameters-"<<this->suffix<<"-"<<i<<".nnw";
  104             seededPop->loadFromFile(ss.str().c_str());
  105         }
  106     }
  107     if(learning)
  108     {
  109         evolutionLog.open((resourcePath + "logs/evolution" + suffix + ".csv").c_str(),ios::out);
  110         if (!evolutionLog.is_open())
  111         {
  112                                                 throw std::runtime_error("Logs does not exist. Please create a logs folder in your build directory or update your cmake file");
  113                                 }
  114     }
  115 }
  116 
  117 AnnealEvolution::~AnnealEvolution()
  118 {
  119     // @todo - solve the invalid pointer that occurs here
  120     #if (0)
  121     for(std::size_t i = 0; i < populations.size(); i++)
  122     {
  123         delete populations[i];
  124     }
  125     populations.clear();
  126     #endif
  127 }
  128 
  129 void AnnealEvolution::mutateEveryController()
  130 {
  131     for(std::size_t i=0;i<populations.size();i++)
  132     {
  133         populations.at(i)->mutate(&eng,numberOfElementsToMutate, Temp);
  134     }
  135 }
  136 
  137 
  138 
  139 void AnnealEvolution::orderAllPopulations()
  140 {
  141     generationNumber++;
  142     double aveScore1 = 0.0;
  143     double aveScore2 = 0.0;
  144 #if (0)
  145     // Disable definition of unused variables to suppress compiler warning
  146     double maxScore1,maxScore2;
  147 #endif
  148     for(std::size_t i=0;i<scoresOfTheGeneration.size();i++)
  149     {
  150         aveScore1+=scoresOfTheGeneration[i][0];
  151         aveScore2+=scoresOfTheGeneration[i][1];
  152     }
  153     aveScore1 /= scoresOfTheGeneration.size();
  154     aveScore2 /= scoresOfTheGeneration.size();
  155 
  156 
  157     for(std::size_t i=0;i<populations.size();i++)
  158     {
  159         populations.at(i)->orderPopulation();
  160     }
  161     evolutionLog<<generationNumber*numberOfTestsBetweenGenerations<<","<<aveScore1<<","<<aveScore2<<",";
  162     evolutionLog<<populations.at(0)->getMember(0)->maxScore<<","<<populations.at(0)->getMember(0)->maxScore1<<","<<populations.at(0)->getMember(0)->maxScore2<<endl;
  163     
  164     
  165     // what if member at 0 isn't the best of all time for some reason? 
  166     // This seems biased towards average scores
  167     // We actually order the populations, so member 0 is the current best according to the assigned fitness
  168     ofstream logfileLeader;
  169     for(std::size_t i=0;i<populations.size();i++)
  170     {
  171         stringstream ss;
  172         ss << resourcePath << "logs/bestParameters-" << suffix << "-" << i << ".nnw";
  173 
  174         populations[i]->getMember(0)->saveToFile(ss.str().c_str());
  175     }
  176 }
  177 
  178 #if (0)
  179 double diffclock(clock_t clock1,clock_t clock2)
  180 {
  181     double diffticks=clock1-clock2;
  182     double diffms=(diffticks*10)/CLOCKS_PER_SEC;
  183     return diffms;
  184 }
  185 #endif
  186 
  187 vector <AnnealEvoMember *> AnnealEvolution::nextSetOfControllers()
  188 {
  189     int testsToDo=0;
  190     if(coevolution)
  191         testsToDo=numberOfTestsBetweenGenerations; //stop when we reach x amount of random tests
  192     else
  193         testsToDo=populationSize; //stop when we test each element once
  194 
  195     if(currentTest == testsToDo)
  196     {
  197         orderAllPopulations();
  198         mutateEveryController();
  199         Temp -= 0.0; // @todo - make this a parameter
  200 //        cout<<"mutated the populations"<<endl;
  201         this->scoresOfTheGeneration.clear();
  202 
  203         if(coevolution)
  204             currentTest=0;//Start from 0
  205         else
  206             currentTest=populationSize-numberOfElementsToMutate; //start from the mutated ones only (last x)
  207     }
  208 
  209     selectedControllers.clear();
  210     for(std::size_t i=0;i<populations.size();i++)
  211     {
  212         int selectedOne=0;
  213         if(coevolution)
  214             selectedOne=rand()%populationSize; //select random one from each pool
  215         else
  216             selectedOne=currentTest; //select the same from each pool
  217 
  218 //      cout<<"selected: "<<selectedOne<<endl;
  219         selectedControllers.push_back(populations.at(i)->getMember(selectedOne));
  220     }
  221     
  222     subTests++;
  223     
  224     if (subTests == numberOfSubtests)
  225     {
  226         currentTest++;
  227         subTests = 0;
  228     }
  229 //  cout<<"currentTest:"<<currentTest<<endl;
  230 
  231     return selectedControllers;
  232 }
  233 
  234 void AnnealEvolution::updateScores(vector <double> multiscore)
  235 {
  236     if(multiscore.size()==2)
  237         this->scoresOfTheGeneration.push_back(multiscore);
  238     else
  239         multiscore.push_back(-1.0);
  240     double score=1.0* multiscore[0] - 0.0 * multiscore[1];
  241     
  242     //Record it to the file
  243     ofstream payloadLog;
  244     payloadLog.open((resourcePath + "logs/scores.csv").c_str(),ios::app);
  245     payloadLog<<multiscore[0]<<","<<multiscore[1];
  246     
  247     for(std::size_t oneElem=0;oneElem<selectedControllers.size();oneElem++)
  248     {
  249         AnnealEvoMember * controllerPointer=selectedControllers.at(oneElem);
  250 
  251         controllerPointer->pastScores.push_back(score);
  252         double prevScore=controllerPointer->maxScore;
  253         if(prevScore>score)
  254         {
  255                 double newScore= leniencyCoef * prevScore + (1.0 - leniencyCoef) * score;
  256                 controllerPointer->maxScore=newScore;
  257         }
  258         else
  259         {
  260             controllerPointer->maxScore=score;
  261             controllerPointer->maxScore1=multiscore[0];
  262             controllerPointer->maxScore2=multiscore[1];
  263         }
  264         std::size_t n = controllerPointer->statelessParameters.size();
  265         for (std::size_t i = 0; i < n; i++)
  266         {
  267             payloadLog << "," << controllerPointer->statelessParameters[i];
  268         }
  269     }
  270 
       payloadLog<<endl;
       payloadLog.close();
       return;
   }
