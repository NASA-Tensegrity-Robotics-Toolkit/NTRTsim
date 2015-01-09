Steven Lessard
Tensegrity Escape Algorithm

ABSTRACT
--------
This module (Escape) contains the files necessary for creating escape algorithms
for 6-bar tensegrities (SUPERBalls). The module includes a multi-level Monte Carlo
platform for optimizing specific parameters of the controller.

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
can be set manually, using the provided mechanisms in EscapeController.cpp.

CLASS ARCHITECTURE
------------------
The classes specific to this module are:
    - the model      (EscapeModel)
    - the controller (EscapeController)
    - the craters    (CraterShallow and CraterDeep)

The application file (AppEscape.cpp) contains a main with a working example of
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
logs/scores.csv. In logs/scores.csv, each line represents an individual episode.
Each episode stores its scores (the distance and the energy spent) as well as
the 32 (4 sine wave parameters * 8 muscle clusters) unique monte carlo values
(ranging from 0 to 1 inclusive). All 34 values associated with an episode are
separated by commas.

To change the simulation to run according to "best parameters" instead of
truly random monte carlo, either the Config.ini file may be edited (speak to
Brian Merlitz) or the existing mechanism in EscapeController.cpp may be
toggled (see the function 'transformActions').

RE-RUNNING BEST PARAMETERS
--------------------------
Once the initial Monte Carlo simulation has been run and the best controller
parameters have been determined empirically, re-running the simulation using
those best parameters will generate even better controller values. To re-run
simulations on this data:
1. In Config.ini, set startSeed to 1 and MonteCarlo to 0 (opposite from MC)
2. Ensure that logs/bestParameters.prm is populated (see LearningSpines for
    a sample format)
3. Re-run the data as normal

In this example directory, there is also specific code that allows for
running Monte Carlo on manual parameters. In these trials, parameters
are set to be randomly within 0.5% of the manual parameters. The purpose
of this is to optimize a parameter set that has already been shown to work
(i.e. generate an escape path for the tensegrity structure).

PYTHON PARSING TEST DATA
------------------------
Four python scripts in logs are used to parse test data (scores.csv).
It is recommended that you copy the contents of scores.csv to a separate file
before parsing and manipulating the data.
The four python scripts used are:
    - bestScores.py
    - cutOutliers.py
    - printDistances.py
    - printParams.py

These four files allow for the conversion of a comma-separated episodes list to 
reformatted data useful for re-running tests. The exact function of each script
is detailed in its header comments, but the syntax for running any of them is:
`python [script].py scores.csv > results.csv`

These files are currently located in src/dev/steve/Escape_T6/logs

QUESTIONS
---------
Email all questions to ntrtdev@lists.nasa.gov

