NASA Tensegrity Robotics Toolkit
===============

About NTRT
---------

The NASA Tensegrity Robotics Toolkit (NTRT) is a collection of C++ and
MATLAB software modules for the modeling, simulation, and control of
Tensegrity Robots. The NTRT Simulator is a tensegrity-specific simulator
built to run ontop of the Bullet Physics Engine, version 2.82.

See INSTALL for instructions on how to install the library and build
from source. A getting started guide is available in src/README.dox,
which contains the main page of the DOxygen documentation.

The most accessible examples of tensegrity structures are in the
examples directory, we recommend starting with src/examples/3_prism
Additional README.dox files can be found in each folder with
additional information about that application or library. Additional 
examples can be found in example folders NestedTetrahedrons, SUPERball,
learningSpines or development folders: dev/tests and dev/btietz.

More information can be found at:

http://ti.arc.nasa.gov/tech/asr/intelligent-robotics/tensegrity/ntrt/

Documentation
--------

Doxygen documentation can be compiled in the source directory
(run "doxygen Doxyfile" when in src). Under Ubuntu 14.04, installing 
doxygen and graphviz (required) can be done by running "sudo apt-get 
install doxygen graphviz". HTML-based documentation can then be found 
under src/DoxyDocs/index.html.

Pre-built documentation for the repository can be found here:

http://ntrt.perryb.ca/doxygen

Tutorials
----------

Tutorials for NTRT (in progress) can be found at:

http://ntrtsim.readthedocs.org/en/latest/index.html

The source for that site is located under doc.

Build Status
---------

NTRTsim's automated build system (BuildBot) can be found at:

http://ntrt.perryb.ca/bb/waterfall

