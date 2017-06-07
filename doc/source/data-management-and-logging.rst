.. highlight:: cpp

Data Management and Logging
=========================================

Data management in NTRT can be done a variety of different ways, depending on how advanced your needs are.
Some NTRT components, such as the learning library, have custom-build data management systems.

However, a new data management and logging framework, the tgDataManager, has been included in NTRTsim as of January 2017.
This framework is intended to replace all past custom-written data logging code with a general solution for all NTRT applications.

This tutorial shows how to use one of the new data managers, a tgDataLogger2.
For new users who want to log data, please use tgDataLogger2. It will be easiest!

**PLEASE NOTE** that, at the moment, you cannot create data managers from within YAML files.
You can use data managers with YAML structures, but you must create a separate App file (instead of using yamlbuilder/BuildTensegrityModel.cpp).
See section below on using tgDataLogger2 with a YAML model.

Quick Start: Log data from rods and actuators using tgDataLogger2
-----------------------------------------

In your application directory, change the CMakeLists.txt file to include the *sensors* library.
For example, if your CMakeLists previous had the line: ::

  link_libraries(tgcreator controllers core)

You would change this to: ::

  link_libraries(tgcreator controllers core sensors)


The following code will direct your application to log data from tgRods and tgSpringCableActuators.
You would write this code in an App file, not in a tgModel file.

First, include the following header files: ::

	#include "sensors/tgDataLogger2.h"
	#include "sensors/tgRodSensorInfo.h"
	#include "sensors/tgSpringCableActuatorInfo.h"

Then, **after** you create your tgModel and add it to the simulation, perform the following steps:
	
#. Create a tgDataLogger2, passing in the name of the log file that you would like to create. Include a time interval for logging. If you don't include this, you will get a sample at each timestep, and your logfile will be extremely large. A suggested sampling time interval is 0.1 sec. ::
     
     std::string log_filename = "~/path_to_my_logs/example_logfile";
     double samplingTimeInterval = 0.1;
     tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);

#. Add the model to the data logger. If your pointer to your tgModel was myModel, ::

     myDataLogger->addSenseable(myModel);
   
#. Create the two sensor infos, one for tgRods and the other for tgSpringCableActuators. ::

     tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
     tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
	
#. Add the sensor infos to the data logger. ::

     myDataLogger->addSensorInfo(myRodSensorInfo);
     myDataLogger->addSensorInfo(mySCASensorInfo);
   
#. Add the data logger to the simulation. ::
	  
     simulation.addDataManager(myDataLogger);

All combined, the code in your application might look like: ::

  ...
  simulation.add(myModel);
  
  std::string log_filename = "/home/drew/NTRT_logs/demo_application";
  tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename);
  myDataLogger->addSenseable(myModel);

  tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
  tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
  myDataLogger->addSensorInfo(myRodSensorInfo);
  myDataLogger->addSensorInfo(mySCASensorInfo);

  simulation.addDataManager(myDataLogger);
  simulation.run();

When the simulation exits, you will have a log file written to /home/drew/NTRT_logs/demo_application_(some_timestamp).txt.

Be sure to change the log file path to something that exists on your computer. A folder under your home directory is a good choice.
Note, however, that if you write the log files somewhere outside the NTRT github directory, your logs will not be synced with the repository.
(It is a *good idea* to not sync your logs, since it will take up space on other people's computers.)

The structure of a tgDataLogger2 data file
-----------------------------------------

The tgDataLogger2 log file names are ended with a timestamp.
This timestamp is when you ran your application.
For example, 01082017_150631 is January 8th 2017 at 3:06pm (and 31 seconds.)

The log file from a tgDataLogger2 is a comma-separated-value file (CSV).
It can be read by most spreadsheet applications (e.g. MS Excel, LibreOffice Calc) as well as MATLAB (see for example MATLAB's csvread command: https://www.mathworks.com/help/matlab/ref/csvread.html).

The log file consist of the following:

1. A line of debugging information, stating what sensors have been created on the model, and the timestamp of the log file.
2. Headings for each of the sensor readings.

   These headings have the following structure:
   First, the sensor number, which is assigned arbitrarily by tgDataLogger2.
   Then, the type of sensor, then an open parenthesis "(" and the tags
   of the specific object that's being sensed, then a ")." and a label for the 
   specific field that will be output in that row.

   For example, if sensor 3 will be sensing a rod 
   with tags "t4 t5", its label for the X position would be "3_rod(t4 t5).X"

3. Rows of output of the sensor data

   Note that sensor data are taken at every timestep of the simulation, and these timesteps are saved as the first column of the log file.

An example first few lines of a log file with one rod sensor only, on a single model with two rods, with each rod having the tgTags "rod", is: ::

  tgDataLogger2 started logging at time 01082017_150631, with 2 sensors on 1 senseable objects.
  time,0_rod(rod).X,0_rod(rod).Y,0_rod(rod).Z,0_rod(rod).Euler1,0_rod(rod).Euler2,0_rod(rod).Euler3,0_rod(rod).mass,1_rod(rod).X,1_rod(rod).Y,1_rod(rod).Z,1_rod(rod).Euler1,1_rod(rod).Euler2,1_rod(rod).Euler3,1_rod(rod).mass,
  0.001,0,6,0,0,-0,0,38.9055,0,10,0,0,-0,0,38.9055,1.67374,2,200,
  0.002,0,5.9999,0,0,-0,0,38.9055,0,9.9999,0,0,-0,0,38.9055,1.67374,1.99998,199.987,
  0.003,0,5.99972,0,0,-0,0,38.9055,0,9.9997,0,0,-0,0,38.9055,1.67374,1.99995,199.969,
  ...

Note that, at the time of the writing of this tutorial, the "1 senseable objects" refers to the number of base tgSenseable objects attached to the data logger, NOT the total number of models and children.
E.g., this is the number of models/senseables that were explicitly attached using the addSenseable method in the App file.
The above example had 1 tgModel with 3 children (2 rods and 1 spring cable actuator), and sensors were only created for the rods.
  
Sensor data from a tgRod using tgRodSensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The tgRodSensor class outputs the following sensor data:

1. The X, Y, and Z positions of the center of mass of the rod

2. The rotation of the rod: its three Euler angles, via tgRod's getOrientation method. TO-DO: check and see which angles these are, exactly.

3. The mass of the rod. This does not change with timestep, and is provided for backwards compatibility with the original tgDataLogger.
      

Sensor data from a tgSpringCableActuator using tgSpringCableActuatorSensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The tgSpringCableActuatorSensor class outputs the following sensor data:

1. The rest length. This is like x0 in F = -k*(x - x0) for the spring in the spring-cable.

2. The current total length of the cable. This is like x in F = -k*(x - x0) for the spring-cable.

3. The tension in the cable. This is like F in F = -k*(x - x0).      

Using tgDataLogger2 with YAML models
-----------------------------------------

Copy yamlbuilder/BuildTensegrityModel.cpp to a new folder, and add the code from the section above.
For an example of how this is done, refer to the AppSpineKinematicsTest application, under src/dev/ultra-spine/SpineKinematicsTest/AppSpineKinematicsTest.cpp.
This file is a copy of BuildTensegrityModel that contains a controller and a tgDataLogger2.

Note that since tgBasicActuator is a tgSpringCableActuator, the tgSpringCableActuatorSensor and its Info class will work fine with the tgBasicActuators created by the YAML builder.

Using tgCompoundRigidSensor
-----------------------------------------

Also new in the tgDataLogger2 infrastructure is a sensor that logs information about compound rigid bodies.
Called tgCompoundRigidSensor, it detects tgModels that have been compounded together, using a specific tag that's appended to each model in a compound (see src/tgcreator/tgRigidAutoCompound for more information about this tag hash).
This sensor outputs the position and orientation of a compound rigid body.

The position of a compound is defined as the average of the centers of mass of each of its constituent models.
Note that this is NOT necessarily the center of mass of the compound itself: for example, if the compound structure contains models of different sizes, the average of the centers-of-mass will not take the different masses into account.
See issue #202 for more information. https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/202

As of 2017-02-03, the orientation of a compound rigid body is not implemented yet. Currently, an empty string is placed in each of the 'orientation' columns. See issue #203 for more information. https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/203

The output of a tgCompoundRigidSensor looks like: ::

  0_compound(compound_4cBWDx).X,0_compound(compound_4cBWDx).Y,0_compound(compound_4cBWDx).Z,0_compound(compound_4cBWDx).Euler1,0_compound(compound_4cBWDx).Euler2,0_compound(compound_4cBWDx).Euler3,0_compound(compound_4cBWDx).mass,
  -35.9804,15,2.13853e-16,,,,0.195487,

The 'mass' parameter is a sum of all the masses of the models in the compound rigid body.
  
Like the rods and cables, the word "compound" is pre-pended to each column.
Currently, the only tag that's written between the parentheses in the heading is the tag that identifies all the models in the rigid compound.
This is always the word "compound" with an underscore, then a 6-digit alphanumeric hash that's randomly created for each compound.
This hash will (should!) change with each run of the simulator, so your log files will have different headings each time you run it.
This is necessary for consistency between simulations of the same type of compound (e.g. a spine vertebra with a specific size) in possibly multiple positions in the same App, or in similar uses between different Apps.

Note also that these compounds are not ordered in any manner.
It will be up to you to figure out which compound corresponds with which of your physical objects in the simulation.
For example, the AppSpineKinematicsTest application logs vertebrae in some weird order, like 2-1-3-4-6-5.
We suggest you look at the compound's position at t=0 and compare that to what you program in your YAML file or model .cpp file.

A suggested fix, if someone wants to implement it, would be to have the sensor output the union of all tags of its constituent models.
See issue #204. https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/issues/204


Advanced Uses of tgDataManager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This framework allows for other tgDataManagers to be created, not just loggers. For folks doing message passing using JSON, for example, you could create a class like tgMessagePasser that inherits from tgDataManager, and all the sensors and sensor infos will still work.

To create new sensors, you will need to make both a new sensor and a new sensor info class.
The sensor info class is what allows a tgDataManager to create the appropriate sensors for tgSenseable objects.

At the moment, only tgModels are sensed (they are the only classes that inherit from tgSenseable.)
However, it would be very possible to sense a controller, or something else, by having that inherit from tgSenseable and then by adding it to the data manager using the addSenseable method.

Note that the data manager does NOT create nor destroy its senseable objects.
It only stores pointers to those objects, and on setup/teardown and in the destructor, only deletes those pointers not the objects themselves.
Remember, tgModel.teardown is handled by tgSimulation.

Other Notes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* You can use the ~ character ("tilde") to represent your HOME directory in the log file name that's passed in to tgDataLogger2.

