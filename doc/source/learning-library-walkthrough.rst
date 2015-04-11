Learning Library Walk-through
======================================

This is a brief tutorial on how to include the machine learning libraries found in src/learning into your code.

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
See the constructor for how to 

At this point you should be ready to go! Email me: bmirletz (at) case (dot) edu if you 

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

A more sophisticated, multi-process algorithm may need to transition the entire learning process to python. We would write a 'python adapter' that interfaces with the 
master Python script, and the Python script would provide the files required for learning. This actually provides an opportunity for a clean break with the current 
structure, so we are free to specify whatever files we want, as long as the adapter receives them as a nested list of actions. The one exception is the neural network library,
where it may be easier to maintain the existing interface architecture.

Historical Notes and Future Work
----------------------------------

Many of the elements of the current learning library actually exist because we used to lose controller objects on reset. Therefore the evolution
object would be owned by main, and passed to a controller which would use an adapter to read the parameters. Our new architecture gives us a lot more 
flexibility, so we should take advantage of it.

Other opprotunities for future work:

* Update config.ini to JSON: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/42
* Merge features of annealEvolution and neuroEvolution: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/131