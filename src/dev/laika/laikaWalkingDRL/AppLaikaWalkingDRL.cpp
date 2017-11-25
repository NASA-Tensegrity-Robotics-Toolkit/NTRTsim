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
 * @author Edward Zhu, Brian Cera, Andrew Sabelhaus
 * $Id$
 */

// This application
// #include "yamlbuilder/TensegrityModel.h"
#include "LaikaWalkingController.h"
#include "LaikaWalkingModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgImportGround.h"
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
#include "Laika_ROS/LaikaState.h"
#include "Laika_ROS/LaikaStateArray.h"
#include "Laika_ROS/LaikaAction.h"
#include "Laika_ROS/LaikaCommand.h"

#define PI 3.14159

// Class for action callbacks
class action_cb_class {
public:
  action_cb_class(LaikaWalkingController* controller) : m_controller(controller) {};
  std::vector<double> cable_action_msg;
  std::vector<double> leg_action_msg;
  LaikaWalkingController* m_controller;
  double dt;
  double target_velocity;
  bool action_ready = false;

  void cb(const Laika_ROS::LaikaAction::ConstPtr& msg) {
    action_ready = true;
    cable_action_msg.clear();
    leg_action_msg.clear();
    cable_action_msg.assign(msg->actions.begin(), msg->actions.end()-4);
    leg_action_msg.assign(msg->actions.end()-4, msg->actions.end());
    m_controller->updateRestLengths(cable_action_msg, target_velocity, dt);
    // m_controller->updateRestLengthsContinuous(cable_action_msg, target_velocity, dt);
    m_controller->updateTorques(leg_action_msg);
  }
};

// Class for command callbacks
class cmd_cb_class {
  public:
    std::string cmd_msg = "";
    int msg_time = 0;
    // void cb(const std_msgs::String::ConstPtr& msg) {

    void cb(const Laika_ROS::LaikaCommand::ConstPtr& msg) {
      cmd_msg = msg->cmd;
      msg_time = msg->header.stamp.nsec;
      // ROS_INFO("action: %s", msg->cmd.c_str());
      std::cout << "Message received: " << cmd_msg << std::endl;
    }
};

// Function for resetting the simulation
bool reset(tgSimulation* simulation, int steps) {
  simulation->reset();
  simulation->run(steps);
  return true;
}

// Function for resetting the simulation after an initial command has been assigned
bool reset_with_action(tgSimulation* simulation, int steps, bool action_ready) {
  bool done = false;
  if (action_ready) {
    simulation->reset();
    simulation->run(steps);
    done = true;
  }
  return done;
}

tgImportGround* getImportedGround(std::string filename)
{
  // Set ground parameters
  const double yaw = 0.0;
  const double pitch = 0.0*PI/180; // About x axis, in z direction
  const double roll = 0.0*PI/180; // About z axis, in x direction
  btVector3 orientation(yaw, pitch, roll);
  const double friction = 1;
  const double restitution = 0.0;
  btVector3 origin(0.0, -10.0, 0.0);
  const double margin = 0.05;
  const double offset = 0.0;
  const double scalingFactor = 10;
  const int interp = 0;
  const bool twoLayer = false;
  const bool flipZY = false;

  // Configure ground characteristics
  const tgImportGround::Config groundConfig(orientation, friction, restitution,
      origin, margin, offset, scalingFactor, interp, twoLayer, flipZY);

  // Check filename
  if (filename.find(".txt") == std::string::npos) {
      std::cout << "Incorrect filetype, input file should be a .txt file" << std::endl;
      exit(EXIT_FAILURE);
  }

  //Create filestream
  std::fstream file_in;
  file_in.open(filename.c_str(), std::fstream::in);

  // Check if input file opened successfully
  if (!file_in.is_open()) {
      std::cout << "Failed to open input file" << std::endl;
      exit(EXIT_FAILURE);
  }
  else {
      std::cout << "Building world with file: " << filename << std::endl;
  }
  tgImportGround* ground = new tgImportGround(groundConfig, file_in);
  return ground;
}

tgBoxGround* getBoxGround()
{
  // Set ground parameters
  const double yaw = 0.0;
  const double pitch = 0.0*PI/180; // About x axis, in z direction
  const double roll = 0.0*PI/180; // About z axis, in x direction
  btVector3 orientation(yaw, pitch, roll);
  const double friction = 1.0;
  const double restitution = 0.0;
  btVector3 size(1000.0, 1.0, 1000.0);
  btVector3 origin(0.0, 0.0, 0.0);
  const tgBoxGround::Config groundConfig(orientation,friction,restitution,size,origin);
  tgBoxGround* ground = new tgBoxGround(groundConfig);
  return ground;
}
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

    // // create the ground and world. Specify ground rotation in radians
    // const double yaw = 0.0;
    // const double pitch = 0.0;
    // const double roll = 0.0;
    // const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // // the world will delete this
    // tgBoxGround* ground = new tgBoxGround(groundConfig);

    tgBoxGround* ground = getBoxGround();

    // std::string filename = "flat_slope_transition_5deg.txt";
    // std::string filename = "flat_terrain.txt";
    // tgImportGround* ground = getImportedGround(filename);

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    // const double timestep_physics = 0.0001; // seconds
    const double timestep_physics = 0.002;
    const double timestep_graphics = 1.f/60.f; // seconds

    // Two different simulation views. Use the graphical view for debugging...
    //tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // ...or the basic view for running DRL.
    tgSimView view(world, timestep_physics, timestep_graphics);

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

    // Call the constructor for the controller
    bool train = true;
    const double target_velocity = 12.0; // MAKE SURE THIS MATCHES WITH THE YAML FILE!!!
    LaikaWalkingController* const controller =
      new LaikaWalkingController(train, target_velocity);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    // ROS stuff
    cmd_cb_class cmd_cb;
    action_cb_class action_cb(controller);
    action_cb.dt = timestep_physics;
    action_cb.target_velocity = target_velocity;

    // Initialize ROS node
    ros::init(argc,argv,"laika_model");
    ros::NodeHandle n;
    ros::Publisher pub_state = n.advertise<Laika_ROS::LaikaStateArray>("state", 1);
    ros::Subscriber sub_act = n.subscribe("action",1,&action_cb_class::cb,&action_cb);
    ros::Subscriber sub_cmd = n.subscribe("cmd", 1, &cmd_cb_class::cb, &cmd_cb);

    int counter = 0;
    ros::Rate loop_rate(100);

    // Get number of bodies
    simulation.run(1);
    std::vector<double> states = myModel->getLaikaWalkingModelStates();
    int bodies = int(states.size()/12);
    //simulation.reset();

    int last_cmd_msg_time = 0;

    bool publish_state = false;
    bool reset_done = false;
    int steps_after_reset = 200;

    reset_done = reset(&simulation, steps_after_reset);
    reset_done = false;

    // Step simulation
    while (ros::ok()) {
      ros::spinOnce();

      // Handle command
      if (cmd_cb.cmd_msg == "reset") {
        // if (cmd_cb.msg_time != last_cmd_msg_time) {
        // reset_done = reset_with_action(&simulation, steps_after_reset, action_cb.action_ready);
        reset_done = reset(&simulation, steps_after_reset);
        if (reset_done) {
          cmd_cb.cmd_msg = "";
          publish_state = true;
          action_cb.action_ready = false;
          counter = 0;
          std::cout << "Simulation reset" << std::endl;
        }
        // simulation.reset();
        // simulation.run(30);
      }
      else if (cmd_cb.cmd_msg == "step" && action_cb.action_ready) {
        // if (cmd_cb.msg_time != last_cmd_msg_time) {
      	action_cb.action_ready = false;
      	cmd_cb.cmd_msg = "";
      	publish_state = true;
      	simulation.run(1);
      }
      else {
        // std::cout << "Command not recognized" << std::endl;
      }

      if (publish_state) {
        publish_state = false;
        std::cout << "Publishing state message" << std::endl;
        std::vector<double> states = myModel->getLaikaWalkingModelStates();
        std::vector<double> cableRL = myModel->getLaikaWalkingModelCableRL();

        Laika_ROS::LaikaStateArray state_array_msg;
        state_array_msg.header.seq = counter;
        state_array_msg.header.stamp = ros::Time::now();

        for(int i = 0; i < bodies; i++) {
          Laika_ROS::LaikaState state_msg;
          state_msg.body_id = i;
          for (int j = 0; j < 12; j++) {
            switch(j) {
              case 0: state_msg.position.x = states[12*i+j];
              case 1: state_msg.position.y = states[12*i+j];
              case 2: state_msg.position.z = states[12*i+j];
              case 3: state_msg.orientation.x = states[12*i+j];
              case 4: state_msg.orientation.y = states[12*i+j];
              case 5: state_msg.orientation.z = states[12*i+j];
              case 6: state_msg.lin_vel.x = states[12*i+j];
              case 7: state_msg.lin_vel.y = states[12*i+j];
              case 8: state_msg.lin_vel.z = states[12*i+j];
              case 9: state_msg.ang_vel.x = states[12*i+j];
              case 10: state_msg.ang_vel.y = states[12*i+j];
              case 11: state_msg.ang_vel.z = states[12*i+j];
            }
          }
          state_array_msg.states.push_back(state_msg);
        }

        state_array_msg.cable_rl.assign(cableRL.begin(),cableRL.end());
        pub_state.publish(state_array_msg);
      }
      // for (int i = 0; i < cableRL.size(); i++) {
      //   std::cout << cableRL[i] << ", ";
      // }
      // std::cout << std::endl;

      // ROS_INFO(state_array_msg);

      last_cmd_msg_time = cmd_cb.msg_time;

      ++counter;

      loop_rate.sleep();
    }

    // Finally, run the simulation.
    // simulation.run();

    // teardown is handled by delete
    return 0;
}
