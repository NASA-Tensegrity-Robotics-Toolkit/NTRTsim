Learning Library Walk-through
======================================

This is a brief tutorial on how to include the machine learing libraries found in src/learning into your code.

Initial Steps
---------------
This example roughly follows the code in src/examples/learningSpines/BaseSpineCPGControl.h/cpp
There is a lot going on in this file, so I'll try to highlight the relavent points.
Note that craterEscape's escapeController provides a second example, but it doesn't utilize a few key features of the library.

First, place the following objects as member variables in your .h file (see BaseSpineCPGControl.h):

* A std::string that holds the filename of your configuration file
* A configuration object, which processes the configuration file
* An evolution object which holds all of the parameters for the population of controllers in memory
* An adapter to the evolution object which handles some of the interfacing with controllers
* A boolean that determines whether learning is active this run

All of these should be initialized in your *constructor*.
If you wait for onSetup, key functions within the evolution and adapter objects will never be called.

What's going on under the hood?
---------------------------------



Using learning in Python
---------------------------

Historical Notes and Future Work
------------------------------
