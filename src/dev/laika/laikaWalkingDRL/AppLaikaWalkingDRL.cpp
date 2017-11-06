/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file AppLaikaWalkingDRL.cpp
 * @brief Contains the definition function main() for AppLaikaWalkingDRL.
 * which builds a horizontal spine structure defined in YAML (under the laika branch)
 * but which will be tied to ROS / deep reinforcement learning code in the laika_drl
 * branch.
 * @author Edward Zhu, Andrew Sabelhaus
 * $Id$
 */

// This application
// #include "yamlbuilder/TensegrityModel.h"
#include "LaikaWalkingController.h"
#include "LaikaWalkingModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// for the sensors...
#include "sensors/tgDataLogger2.h"
#include "sensors/tgFullStateMonitor.h"
#include "sensors/tgCompoundRigidSensorInfo.h"
#include "sensors/tgRodSensorInfo.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>

// Added for ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

// Class for control callbacks
class control_cb_class {
  public:
    std::string control_msg = "step";
    void cb(const std_msgs::String::ConstPtr& msg) {
      control_msg = msg->data;
      ROS_INFO("Control: %s", msg->data.c_str());
    }
};

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */
int main(int argc, char** argv)
{
    // Normally, for an App that uses the YAML parser, we would need to check
    // the number of arguments passed in. However, since we'll be hard-coding the
    // path to the YAML file in the app, no arguments are desired.
    if (argv[1] != NULL)
    {
       throw std::invalid_argument("This app does not take in a YAML file, it's hard-coded to for DRL purposes. Use another Laika app for testing different YAML files, or edit the string path to the YAML file in this app.");
    }

    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds

    // Two different simulation views. Use the graphical view for debugging...
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // ...or the basic view for running DRL.
    //tgSimView view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    // Note that we hard-code the path to the correct YAML file here!
    // If the LaikaWalkingModel tgModel is not passed the correct YAML file, it
    // will segfault. Drew doesn't have time to code the right checks just yet...

    // NOTE: change this path to the correct one on your computer.
    // Otherwise, this app won't run.
    std::cout << "WARNING: Be sure to change the hard-coded path to the YAML file in AppLaikaWalking, which will depend on the folder path on your computer." << std::endl;

    // The YAML file absolute paths for Drew's and Brian's computers:
    //std::string model_path("/home/brian/NTRTsim/src/dev/laika/laikaWalkingDRL/LaikaWith1DOFLegs.yaml");
    //std::string model_path("/home/drew/repositories/NTRTsim/src/dev/laika/BaseStructuresLaika/StickLegs.yaml");

    // a RELATIVE path to the YAML file...
    std::string model_path("../../../../src/dev/laika/laikaWalkingDRL/LaikaWith1DOFLegs.yaml");
    std::cout << "Setting up the LaikaWalkingModel..." << std::endl;
    LaikaWalkingModel* const myModel = new LaikaWalkingModel(model_path.c_str(),false);

    // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file, BUT it only actually works
    // with the specific HorizontalSpine YAML file.
    // @TODO: should this throw an error when attached to a model that
    // wasn't built with the HorizontalSpine YAML file?

    // Parameters for the Horizontal Spine Controller are specified in that .h file,
    // repeated here:
    double startTime = 10.0;
    double minLength = 0.8;
    double rate = 0.25;

    // Call the constructor for the controller
    LaikaWalkingController* const controller =
      new LaikaWalkingController(startTime, minLength, rate);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    // A test of the getLaikaWalkingStates method.
    std::cout << "Testing the getLaikaWalkingStates method: " << std::endl;
    std::vector<double> states = myModel->getLaikaWalkingModelStates();
    std::cout << "There are " << states.size() << " states, which are: "
	      << std::endl;
    for(int i=0; i < states.size(); i++) {
      std::cout << states[i] << std::endl;
    }

    // ROS stuff
    control_cb_class control_cb;

    // Initialize ROS node
    ros::init(argc,argv,"laika_model");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::UInt32>("state",1);
    ros::Subscriber sub = n.subscribe("control",1,&control_cb_class::cb,&control_cb);

    int counter = 0;
    std_msgs::UInt32 msg;
    ros::Rate loop_rate(10);

    while (ros::ok()) {
      ros::spinOnce();
      msg.data = counter;

      if (control_cb.control_msg == "reset") {
        simulation.reset();
        simulation.run(1);
        counter = 0;
        std::cout << "Simulation reset" << std:: endl;
      }
      else if (control_cb.control_msg == "step") {
        simulation.run(1);
        pub.publish(msg);
      }
      ROS_INFO("State: %d", counter);

      ++counter;

      loop_rate.sleep();
    }

    // Finally, run the simulation.
    // simulation.run();

    // teardown is handled by delete
    return 0;
}
