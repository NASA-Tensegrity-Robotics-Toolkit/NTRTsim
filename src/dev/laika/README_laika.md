A brief description of all the laika applications in this directory, and
the correct way to run them. -Drew 2018-02-10

laikaWalkingDRL: the application for Brian and Ed's attempts at running deep
		 reinforcement learning on a model of Laika. This folder doesn't
		 include any of the DRL code, just the model. See the laika_drl
		 branch for the machine learning work. The model in this folder
		 should just be the one with hinged legs (BUT, Drew thinks it's
		 improved in the other branch... better hinge constraints.)

RotatingVertebra: uses a hinged rotating vertebra, applies a torque, senses foot
		  forces with force plates. Run the App with the
		  TetrahedralSpineWithRotJoint.yaml file.

SpineModels: Contains the different geometries of spines, and some code to test
	     them out, from the M.Eng project 2016-2017. This is where the
	     C-shape, "rotated W", etc. are.

v0.1_bending: basic Laika model, with a controller that pulls on the side
	      cables and causes bending. This was the first work on the force
	      plates with sensing, I believe. No rotating vertebra.

v0.1_torsion: same basic Laika model, just with a controller that pulls on a
	      saddle cable instead, and rotates the spine. Also includes a
	      "reverse" controller, which extends a cable instead of contracting
	      (I think?)


Also - the BaseStructuresLaika directory has most of the YAML files that are
shared among the apps.

Happy simulating. -D

