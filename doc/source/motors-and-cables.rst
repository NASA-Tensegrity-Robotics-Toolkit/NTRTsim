How to Choose and Configure NTRT's Motor and Cable Models
==============================================================
This tutorial assumes you're familiar with the basics of building structures in NTRT.
The Doxygen documentation and examples are currently the best place to get that information:
http://ntrt.perryb.ca/doxygen/

The basic model we're using is that the robot has a motor connected to each cable,
which is in turn connected to a spring. The motor adjusts the rest length of this
system according to rules defined by the motor model.

NTRT provides two options for motors, and two options for cables.
All of these are a subclass of tgSpringCable_ and 
use the same attributes in the config struct
These can be subclassed into a whole range of realistic simulations for your application.
The basic options in core are as follows:

1. Cables can either have contact dynamics (interact with the world) or only act on the bodies they actuate

2. Motors can be perfect actuators (easy to control) or possess inertia, friction, and a torque speed curve.

Both of these are chosen with simple switches (similar child classes) in the builder tools.

Cables
--------------
A cable without contact dynamics is a tgBasicActuator_ these are constructed by passing a tgBasicActuatorInfo_
to a spec. If you want contact dynamics, just use tgBasicContactCableInfo_ instead.
There are no differences in the config structs.
An example with a conditional compile can be found at line 195 of the Octahedral Complex demo:
https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/blob/master/src/examples/learningSpines/OctahedralComplex/FlemonsSpineModelLearningCL.cpp 

Motors
-----------------
tgBasicActuator_ is a perfect actuator. As long as you don't exceed the force or speed constraints set
by the config (from tgSpringCable_) then it will make your desired changes to the cable's rest length.
Great for getting started, but not the way motors really work.

For more complex behaviors, use tgKinematicActuator_. This adds four new parameters to the specification,
the radius of the motor, motor friction, motor inertia, and whether the motor is backdrivable (a boolean).
More details can be found on the Doxygen page.
To use it, just use tgKinematicActuatorInfo_.
Note that the config for a tgKinematicActuator_ is a subclass of tgSpringCable_,
so you can pass that config to either actuator class.
These can also have contact dynamics by calling tgKinematicContactCableInfo_.
An example with a conditional compile can be found at line 161 of the TetrahedralComplex example:
https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/blob/master/src/examples/learningSpines/TetrahedralComplex/FlemonsSpineModelLearning.cpp

If you need a different torque speed curve or friction behavior, you should be able to subclass
the functions getAppliedTorque or integrateRestLength in tgKinematicActuator_

Controlling Each Type of Motor
--------------------------------
tgBasicActuator_ controls rest length directly, these are the commands passed by setControlInput.
tgKinematicActuator_ controls the motor's applied torque, which is then integrated to produce a rest
length change. Thus control needs to be a little more complicated. Look at the controller_ classes
for PID controllers, and other things that are useful for more complex low level controls.

Tutorials on controllers will hopefully get written in August 2015.

If you have questions contact Brian at bmirletz (at) case (dot) edu

.. _tgSpringCable: http://ntrt.perryb.ca/doxygen/classtg_spring_cable.html
.. _tgBasicActuator: http://ntrt.perryb.ca/doxygen/classtg_basic_actuator.html
.. _tgBasicActuatorInfo: http://ntrt.perryb.ca/doxygen/classtg_basic_actuator_info.html
.. _tgBasicContactCableInfo: http://ntrt.perryb.ca/doxygen/classtg_basic_contact_cable_info.html
.. _tgKinematicActuator: http://ntrt.perryb.ca/doxygen/classtg_kinematic_actuator.html
.. _tgKinematicActuatorInfo: http://ntrt.perryb.ca/doxygen/classtg_kinematic_actuator_info.html
.. _tgKinematicContactCableInfo : http://ntrt.perryb.ca/doxygen/classtg_kinematic_contact_cable_info.html
.. _controller: https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/tree/master/src/controllers
