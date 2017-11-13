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

// Class for action callbacks
class action_cb_class {
  public:
    action_cb_class(LaikaWalkingController* controller) : m_controller(controller) {};
    std::vector<double> cable_action_msg;
    std::vector<double> leg_action_msg;
    LaikaWalkingController* m_controller;
    double dt;
    double target_velocity;

    void cb(const Laika_ROS::LaikaAction::ConstPtr& msg) {
      cable_action_msg.clear();
      leg_action_msg.clear();
      cable_action_msg.assign(msg->actions.begin(), msg->actions.end()-4);
      leg_action_msg.assign(msg->actions.end()-4, msg->actions.end());
      // m_controller->updateRestLengths(cable_action_msg);
      m_controller->updateRestLengthsDiscrete(cable_action_msg, target_velocity, dt);
      m_controller->updateTorques(leg_action_msg);
    }
};

// Class for command callbacks
class cmd_cb_class {
  public:
    std::string cmd_msg = "step";
    int msg_time = 0;
    // void cb(const std_msgs::String::ConstPtr& msg) {

    void cb(const Laika_ROS::LaikaCommand::ConstPtr& msg) {
      cmd_msg = msg->cmd;
      msg_time = msg->header.stamp.nsec;
      ROS_INFO("action: %s", msg->cmd.c_str());
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

    std::string policy_file("/home/edward/gps-spine/python/gps/model_rl/models_builder/saved_model.pb");
    bool test_policy = false;

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
    // const double timestep_physics = 0.0001; // seconds
    const double timestep_physics = 0.002;
    const double timestep_graphics = 1.f/60.f; // seconds

    // Two different simulation views. Use the graphical view for debugging...
    // tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
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
    // LaikaWalkingController* const controller =
    //   new LaikaWalkingController(policy_file,test_policy);
    LaikaWalkingController* const controller =
      new LaikaWalkingController();
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    const double target_velocity = 12.0; // MAKE SURE THIS MATCHES WITH THE YAML FILE!!!

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
    simulation.reset();

    int last_cmd_msg_time = 0;

    // Step simulation
    while (ros::ok()) {
      ros::spinOnce();

      Laika_ROS::LaikaStateArray state_array_msg;
      state_array_msg.header.seq = counter;
      state_array_msg.header.stamp = ros::Time::now();

      // Handle command
      if (cmd_cb.cmd_msg == "reset") {
        if (cmd_cb.msg_time != last_cmd_msg_time) {
          simulation.reset();
          simulation.run(1);
          // counter = 0;
          std::cout << "Simulation reset" << std::endl;
        }
        else {
          std::cout << "Reset message stale" << std::endl;
        }
      }
      else if (cmd_cb.cmd_msg == "step") {
        if (cmd_cb.msg_time != last_cmd_msg_time) {
          simulation.run(1);
        }
        else {
          std::cout << "Step message stale" << std::endl;
        }
      }

      last_cmd_msg_time = cmd_cb.msg_time;

      std::vector<double> states = myModel->getLaikaWalkingModelStates();
      std::vector<double> cableRL = myModel->getLaikaWalkingModelCableRL();
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

      // for (int i = 0; i < cableRL.size(); i++) {
      //   std::cout << cableRL[i] << ", ";
      // }
      // std::cout << std::endl;

      // ROS_INFO(state_array_msg);

      ++counter;

      loop_rate.sleep();
    }

    // Finally, run the simulation.
    // simulation.run();

    // teardown is handled by delete
    return 0;
}
