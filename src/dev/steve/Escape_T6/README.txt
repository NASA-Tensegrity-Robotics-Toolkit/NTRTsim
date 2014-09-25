Steven Lessard
Tensegrity Escape Algorithm
Summer 2014

ABSTRACT
--------
This module (Escape_T6) contains the files necessary for creating escape algorithms
for 6-bar tensegrities (SUPERBalls). The module includes Monte Carlo and
Machine Learning platforms for optimizing specific parameters of the controller.

HOW DOES THE CONTROLLER WORK?
-----------------------------
Currently, the controller functions by categorizing the 24 muscles (cables) of the 
tensegrity into 8 groups of 3 muscles. Each one of these "clusters" covers one of 
the triangular faces on the tensegrity. Each cluster actuates according to its 
unique sine function. The parameters for these sine functions (amplitude, 
phase offset, angular frequency, and DC offset) vary from cluster to cluster, but 
each one is regulated according to the mechanical constraints of the real world 
tensegrity. These real world values can be found in the model class.

The parameters of these sine waves can be generated via Monte Carlo simulation or
can be set manually, using the provided mechanisms in Escape_T6Controller.cpp.

CLASS ARCHITECTURE
------------------
The classes specific to this module are:
    - the model      (Escape_T6Model)
    - the controller (Escape_T6Controller)
    - the craters    (Crater and CraterDeep)

The application file (AppEscape_T6.cpp) contains a main with a working example of
the escape demo. In this main, a tensegrity (the model) is instatiated and provided
a controller. Both the tensegrity and the crater are added to the simulation as
models.
For the inheritance of any class, look at that class's respective header file (*.h)

RUNNING MONTE CARLO SIMULATIONS
-------------------------------
Running a monte carlo simulation for this module rapidly brute forces random, 
mechanically possible control patterns. Each trial run (an "episode") tests one
unique set of parameters for one tensegrity over the course of 60,000 steps (as
determined by the application). The results of each episode are stored in 
logs/scores.csv. In logs/scores.csv, each line represents a unique episode.
Each episode stores its scores (the distance and the energy spent) as well as
the 32 (4 sine wave parameters * 8 muscle clusters) unique monte carlo values
(ranging from 0 to 1 inclusive). All 34 values associated with an episode are
separated by commas.

To change the simulation to run according to "best parameters" instead of
truly random monte carlo, either the Config.ini file may be edited (speak to
Brian Merlitz) or the existing mechanism in Escape_T6Controller.cpp may be
toggled (see the function 'transformActions').

ISSUES TO BE RESOLVED
---------------------
- The energy spent calculation in the controller does not seem to correctly
  calculate how much energy each muscle spends (may be wrong units)
- Change pretension to be a Monte Carlo variable (add it to actions<>)

QUESTIONS
---------
Email all questions to ntrtdev@lists.nasa.gov

