Learning Library Walk-through
======================================

This is a brief tutorial on how to include the machine learning libraries found in src/learning into your code.
See section Using learning in Python for our current experimental multi-process Python scripts

Initial Steps
---------------
This example roughly follows the code in src/examples/learningSpines/BaseSpineCPGControl.h/cpp
There is a lot going on in this file, so I'll try to highlight the relevant points.
Note that craterEscape's escapeController provides a second example, but it doesn't utilize a few key features of the library.

**First**, place the following objects as member variables in your .h file (see BaseSpineCPGControl.h):

* A std::string that holds the filename of your configuration file
* A configuration object, which processes the configuration file
* An evolution object which holds all of the parameters for the population of controllers in memory
* An adapter to the evolution object which handles some of the interfacing with controllers
* A boolean that determines whether learning is active this run

**Second**, all of these should be initialized in your *constructor*.
If you wait for onSetup, key functions within the evolution and adapter objects will never be called.
You can probably just copy the constructor for BaseSpineCPGControl, +/- how complicated your config struct needs to be.

**Third** - here's what's important for onSetup:
1. Call adapter.initialize, this will get you a new controller queued up for this run (if learning is turned on)
2. If you're running with the same parameters for an entire trial (i.e. this part of your controller is open loop) go ahead and call adapter.step
dt can be zero, and the state vector can be empty, you're just getting a nested vector of random parameters so far.
3. These parameters are scaled 0 to 1, so you're going to have to scale them to what you want (each parameter is an 'action'). This is done in the scaleNodeActions function (which is simpler than scaleEdgeActions)
4. Apply these actions to your controller as appropriate. craterEscape is actually a good example here, since its simpler.
Note there are some controllers that need new parameters every step. You can find these in dev, we'll update the tutorials with that info once they're in examples (before v1.2).

**Fourth** - get your config.ini setup 
There's a preliminary listing of what all of these variables do here:
http://ntrt.perryb.ca/doxygen/config_full.html
The most important things are:
1. The learning variable
2. numberOfActions
3. numberOfControllers

You can choose to dump all of your parameters in one controller (numberOfActions = numberOfParameters), 
or divide it up by repeating parameters (i.e. sine waves have 4 parameters (actions) and you might use 8 or 24 of them in your controller).
There are advantages to each of these approaches depending on which learning algorithm you're using (see next section).

Note that this is best placed in the resources folder so that you can run your code from anywhere on the path.

At this point you should be ready to go! Email me: bmirletz (at) case (dot) edu if you have trouble

Algorithm Options
------------------------------------

(under construction)

1. Monte Carlo
2. Gaussian Sampling
3. Co-evolution
4. Genetic Algorithm

What's going on under the hood?
---------------------------------

For the default "one process" learning algorithm, here's what's going on:

When evolution objects are constructed, they create sets of random parameters in memory according to the specifications in config.ini.
Currently, one of the random sets of parameters is overwritten with the best prior set if the 'start seed' parameter in config is on.

When the adapter's initialize function is called, it asks the evolution object for the 'next set of controllers', which typically just supplies
it with a nested set of vectors (actions). The controller then proceeds with the rest of the 'trial' (running a simulation with these sets of parameters)
until the simulation ends (typically after a certain number of steps with graphics off). Then the simulation calls reset, onTeardown is called, and the controller provides
a score to the adapter which passes that score on to the evolution object. The next step of a reset is onSetup, so the process resumes.

However, after a certain number of trials (depends on the type of learning), counters within the 'next set of controllers' function trigger the end of a generation.
At this point, the controllers are sorted according to their scores, and then mutation, children, and elitism happen according to the algorithm.
Finally, a new controller is passed to the adapter out of the new population.

Using learning in Python
---------------------------

An alternative is to use Python to perform the sorting and mutating. In this case, parameters are often dumped straight to a common scores.csv file, and
then sorted out with Python scripts. These scripts can either generate another csv like file with just parameters (as in craterEscape), or new .nnw files
(this may still just be on a branch).

Our new (as of April 2015) multi-process learning script (scripts/NTRTJobMaster.py) uses JSON to interface with NTRT.
In order to use this you need to provide a JSON interface to your controller, and write a specification file.
Your controller must write the score back to the JSON file.
Your App must also support boost command line options.
The specification file fills the role of Config.ini, and has parameters as follows:

{
    **"filePrefix"**   : A string that forms the beginning of your filename
    
    **"fileSuffix"**   : A string that forms the suffix of your filename
    
    **"resourcePath"** : A string that points to the resources folder
    
    **"lowerPath"**    : A string that 
    
    **"executable"** : "./../build/dev/btietz/TC_goal/AppGoalTerrain",
    
    **"terrain"** : A nested array of booleans (0s and 1s) that is passed to command line options for terrain or similar.
    The number of arrays dictates the number of runs per job 
    
    **"learningParams"**: A container for learning parameters. (the JSON object is below, don't put things here) 
    
    {
    
        **"trialLength"** : An integer - the number of timesteps for each trial
    
        **"numTrials"** : An integer - the number of trials in each generation.
        If this is equal to the population size things will be run deterministically. Random controllers will fill in subsequent trials (cheap version of co-evolution)
    
        **"numGenerations"** : Total number of generations before exiting.
        As of May 2015 numTrials * numGenerations must be less than 30,000. We will post here when this is fixed.
    
        **"nodeVals"** : One of these for each type of parameters you want. This allows for heterogenous data types.
    
        {
    
            **"learning"** : Whether or not this process is learning
    
            **"startingControllers"** : How many controllers to read in. The script will start with filePrefix_0.fileSuffix and work up one at a time until this number
            Additional parameters to reach populationSize will be chosen randomly
    
            **"monteCarlo"** : Is learning using monteCarlo? If true, each set of parameters will be random
            As of May 2015 if you're running monteCarlo I would recommend numGenerations = 1, as files will be overwritten and data will be lost.
    
            **"numberOfStates"** : Integer. The number of inputs to the tuned controller. If 0 these are just data, if >= 1 a neural network will be used
    
            **"numberOfOutputs"** : Integer. The number of output parameters for the neural network or controller.
            
            **"numberHidden"** : Integer. Only matters if numberOfStates >= 1. The number of neurons in the ANN's hidden layer
    
            **"numberOfInstances"** : Integer. Must be > 0 if numberOfStates = 0. How many times are you going to interate through the outputs?
            Great for repeated parameters like weights. If numberOfStates > 0 this is ignored.
    
            **"populationSize"** : Integer. How many controllers are we testing?
    
            **"useAverage"** : Should the controllers be judged on their average value (true) or maximum value (False). Average recommended with co-evolution on,
    
            **"numberToMutate"** : Integer. How many controllers have their parameters changed by mutation? This + numberOfChildren must be less than population size
            I recommend around half. The top N will be mutated.
    
            **"numberOfChildren"** : Integer. How many times should the controllers 'mate' to cross-pollinate parameters? Mating pairs are choses
            by weighted probability, based on the scores.
    
            **"mutationChance"** : Double between 0 and 1 (inclusive) How often a single parameter within a controller is mutated. 1 is always 0 is never.
    
            **"mutationDev"** : Double between 0 and 1 (inclusive). What is the deviation of the normal distribution used to mutate parameters?
    
            **"paramMax"** : Double. What is the largest this parameter should be?
    
            **"paramMin"**: Double. What is the smallest this parameter shoudl be
    
            **"childMutationChance"** : Double. How often should a cross-pollinated child be mutated. This applies to the entire controller.
    
        } Add a comma here if you have more than one set of parameters.
    
    }

}

Note that replacing the explanations with numbers and added commas the end of each line, the above would lead to a valid JSON specification file, similar to:
https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/blob/master/scripts/TCSpec.json

See this issue for discussion and more instructions for running the script: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/133

Historical Notes and Future Work
----------------------------------

Many of the elements of the current learning library actually exist because we used to lose controller objects on reset. Therefore the evolution
object would be owned by main, and passed to a controller which would use an adapter to read the parameters. Our new architecture gives us a lot more 
flexibility, so we should take advantage of it.

Other opprotunities for future work:

* Update config.ini to JSON: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/42
* Merge features of annealEvolution and neuroEvolution: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/131
