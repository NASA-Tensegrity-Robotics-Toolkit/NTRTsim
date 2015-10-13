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
 * @file AppSUPERball.cpp
 * @brief Contains the definition function main() for the SUPERball applicaiton
 * application.
 * $Id$
 */

// This application
#include "T6Model.h"

// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/tgRod.h"

// Controller
#include "controllers/T6zeroMQController.h"

// Bullet Physics
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "gps_agent_pkg/SUPERballState.h"
#include "gps_agent_pkg/SUPERballStateArray.h"

// OSG
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/ref_ptr>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Array>
#include <osg/TexEnv>
#include <osgGA/NodeTrackerManipulator>

// The C++ Standard Library
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

class motor_pos_cb_class {
public:
	float motor_pos = 0.;
	void cb(const std_msgs::Float32::ConstPtr& msg){
		motor_pos = msg->data;
	}
};

class control_cb_class {
public:
	std::string action = "";
	void cb(const std_msgs::String::ConstPtr& msg){
		action = msg->data;
	}	
};

class timestep_cb_class {
public:
	unsigned timesteps = 0;
	void cb(const std_msgs::UInt16::ConstPtr& msg){
		timesteps = msg->data;
	}
};

void makeCheckImage64x64x3(GLubyte img[64][64][3], int numSquaresPerEdge, int lowVal, int highVal) {
    const int widthInPixels=64;
    const int heightInPixels=64;
    assert(lowVal >= 0 && lowVal <= 255);
    assert(highVal >= 0 && highVal <= 255);

    int wOn=0;
    int hOn=0;
    for (int i=0; i < widthInPixels; i++) {
        if ((i % (widthInPixels/numSquaresPerEdge)) == 0) {
            wOn = wOn ? 0 : 1;
        }
        for (int j=0; j < heightInPixels; j++) {
            if ((j % (heightInPixels/numSquaresPerEdge)) == 0) {
                hOn = hOn ? 0 : 1;
            }
            int c = (wOn^hOn);
            if (c==0) {
                c = lowVal;
            } else {
                c = highVal;
            }
            img[i][j][0] = (GLubyte) c;
            img[i][j][1] = (GLubyte) c;
            img[i][j][2] = (GLubyte) c;
        }
    }
}

osg::Image *createCheckImage() {
    osg::Image *img = new osg::Image;
    img->allocateImage(64, 64, 3, GL_RGB, GL_UNSIGNED_BYTE);

    GLubyte checkImage[64][64][3];

    // Draw the chess-board in memory
    makeCheckImage64x64x3(checkImage, 2, 12, 200);

    // copy to the image
    int n = 0;
    for (uint i=0; i < 64; i++) {
        for (uint j=0; j < 64; j++) {
            for (uint k=0; k < 3; k++) {
                img->data()[n++] = checkImage[i][j][k];
            }
        }
    }

    return img;
}

osg::Node* createGroundPlane() {
    // Create the geode
    osg::Geode* groundPlaneGeode_ = new osg::Geode;

    // Create the texture
    osg::ref_ptr<osg::Image> checkImage = createCheckImage();

    osg::ref_ptr<osg::Texture2D> checkTexture = new osg::Texture2D;
    // protect from being optimized away as static state:
    checkTexture->setDataVariance(osg::Object::DYNAMIC); 
    checkTexture->setImage(checkImage.get());

    // Tell the texture to repeat
    checkTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    checkTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

    // Create a new StateSet with default settings: 
    osg::ref_ptr<osg::StateSet> groundPlaneStateSet = new osg::StateSet();

    // Assign texture unit 0 of our new StateSet to the texture 
    // we just created and enable the texture.
    groundPlaneStateSet->setTextureAttributeAndModes(0, checkTexture.get(), osg::StateAttribute::ON);

    // Texture mode
    osg::TexEnv* texEnv = new osg::TexEnv;
    texEnv->setMode(osg::TexEnv::DECAL); // (osg::TexEnv::MODULATE);
    groundPlaneStateSet->setTextureAttribute(0, texEnv);

    // Associate this state set with our Geode
    groundPlaneGeode_->setStateSet(groundPlaneStateSet.get());

    // Create the ground plane
    osg::ref_ptr<osg::Geometry> groundPlaneGeometry = new osg::Geometry();
    double groundPlaneSquareSpacing_ = 1;

    const double infty=100;

    const double metresPerTile=2*groundPlaneSquareSpacing_;
    const double texCoordExtreme=2*infty/metresPerTile;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    groundPlaneGeometry->setVertexArray(vertices.get());
    
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;
    groundPlaneGeometry->setTexCoordArray(0, texCoords.get());

    vertices->push_back(osg::Vec3d(-infty, -infty, 0));
    texCoords->push_back(osg::Vec2(0, 0));
    vertices->push_back(osg::Vec3d(infty, -infty, 0));
    texCoords->push_back(osg::Vec2(texCoordExtreme, 0));
    vertices->push_back(osg::Vec3d(infty,  infty, 0));
    texCoords->push_back(osg::Vec2(texCoordExtreme, texCoordExtreme));
    vertices->push_back(osg::Vec3d(-infty,  infty, 0));
    texCoords->push_back(osg::Vec2(0, texCoordExtreme));

    osg::ref_ptr<osg::DrawElementsUInt> quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS);
    for (uint i=0; i < vertices->size(); i++) {
        quad->push_back(i);
    }

    groundPlaneGeometry->addPrimitiveSet(quad.get());

    groundPlaneGeode_->addDrawable(groundPlaneGeometry.get());
    return groundPlaneGeode_;
}


int main(int argc, char** argv)
{
    std::cout << "AppSUPERball" << std::endl;
    ros::init(argc,argv,"AppSUPERball");
    ros::NodeHandle n;

    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0;//M_PI/15.0;
    //const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.

    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimView view(world, timestep_physics, timestep_graphics); //no graphics

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    T6Model* const myModel = new T6Model();

    // Fifth, select the controller to use, and attach it to the model.
    // For example, you could run the following to use the T6TensionController:
    T6zeroMQController* const pTC = new T6zeroMQController();
    myModel->attach(pTC);

    // Finally, add out model to the simulation
    simulation.addModel(myModel);
 
    //set up ROS publishers and subscribers
    //create subscribers for motor positions
    std::vector<ros::Subscriber> motor_pos_sub;
    std::vector<motor_pos_cb_class*> motor_pos_cb;
    for (unsigned i=0; i<12; ++i) {
        unsigned board_id = i%2;
        unsigned bbb, sub_index;
        if (board_id==0) {
            bbb = i+2;
            board_id = 0x71;
            sub_index = 0x2;
        } else {
            bbb = i+1;
            board_id = 0x1;
            sub_index = 0x1;
        }
        std::stringstream ss;
        ss << "/bbb" <<bbb<<std::hex << "/0x" <<board_id << "_0x2040_0x" << sub_index << std::dec;
        std::cout << std::dec <<ss.str() << std::endl;
        motor_pos_cb_class* m = new motor_pos_cb_class();
        motor_pos_sub.push_back(n.subscribe(ss.str(), 1, &motor_pos_cb_class::cb, m));
        motor_pos_cb.push_back(m);
    }

    //create a subscriber for control (time step/reset)
    control_cb_class ctrl_cb;
    timestep_cb_class step_cb;
    ros::Subscriber control_sub = n.subscribe("/superball/control", 1, &control_cb_class::cb, &ctrl_cb);
    ros::Subscriber timestep_sub = n.subscribe("/superball/timestep", 1, &timestep_cb_class::cb, &step_cb);

    //create publishers for the current state
    ros::Publisher robot_state_pub = n.advertise<gps_agent_pkg::SUPERballStateArray>("/superball/state", 1);

    // Run until the user stops the simulation
    bool publish_state_update = false;
    std::vector <tgRod*> rods = myModel->find<tgRod>("rod"); 
    // This is added to support Ali's Model
    std::vector <tgRod*> rodmps = myModel->find<tgRod>("rodmp"); 
    float motor_targets[12];

    // Set up some plotting and rendering
    osg::ref_ptr<osg::Group> m_root = new osg::Group;
    osg::ref_ptr<osg::Group> m_robot = new osg::Group;
    osgViewer::Viewer m_viewer;
    std::vector<osg::MatrixTransform*> tfs;
    m_root->addChild(m_robot);
    m_root->addChild(createGroundPlane());
    // Changed to support Ali's Model
    for (unsigned i=0; i < 6; ++i) {
        osg::MatrixTransform* tf = new osg::MatrixTransform;
        tfs.push_back(tf);
        m_robot->addChild(tf);
        osg::Cylinder* bar = new osg::Cylinder();
        bar->setRadius(0.035);
        bar->setHeight(rods[i]->length()/10);
        osg::Geode* geode = new osg::Geode;
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(bar);
        geode->addDrawable(drawable);
        tf->addChild(((osg::Node*) geode));
    }
    m_viewer.setSceneData(m_root.get());
    m_viewer.setUpViewInWindow(0, 0, 640, 480);
    m_viewer.realize();

    osg::ref_ptr<osgGA::NodeTrackerManipulator> nodeTracker = new osgGA::NodeTrackerManipulator;
    nodeTracker->setHomePosition(osg::Vec3(0.0, -5.0, 0.0), osg::Vec3(), osg::Z_AXIS);
    nodeTracker->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
    nodeTracker->setRotationMode(osgGA::NodeTrackerManipulator::TRACKBALL);
    nodeTracker->setTrackNode(m_robot);
    m_viewer.setCameraManipulator(nodeTracker);
    m_viewer.frame();

    while (ros::ok()) {
        //get new ROS messages
        ros::spinOnce();

        //do we need to reset?
        if (ctrl_cb.action=="reset") {
            std::cout << "resetting simulation" << std::endl;
            ctrl_cb.action = "";
            simulation.reset();
            rods = myModel->find<tgRod>("rod"); 
        } 

        //do we need to run a number of simulation steps?
        else if (step_cb.timesteps>0) {
            std::cout << "advancing simulation " << step_cb.timesteps << "ms" << std::endl;
            publish_state_update = true;

            //update motor target positions
            for (unsigned i=0; i<12; ++i) {
                motor_targets[i] = 9.5 - 10.*(0.018*M_PI*(motor_pos_cb[i]->motor_pos/(2*M_PI))); //input is in radians (motor spindle OD = 18mm) output is in decimeter
                //std::cout << motor_pos_cb[i]->motor_pos << "\t"<< motor_targets[i] << "\t";
            }
            //std::cout<<std::endl;
            pTC->setTargetLengths(motor_targets);

            simulation.run(step_cb.timesteps);
            step_cb.timesteps = 0;
        }

        //do we need to send out a state update?
        if (publish_state_update) {
            publish_state_update = false;
            gps_agent_pkg::SUPERballStateArray state_msg;
            const std::vector<tgBasicActuator*> springCables = myModel->getAllActuators();
            //state_msg.header.frame_id = "/base"; 
            for (unsigned i=0; i<6; ++i) {
                gps_agent_pkg::SUPERballState state;
                const tgRod* const rod = rods[i];
                btScalar x = rod->centerOfMass().x()/10; //output in meters
                btScalar y = rod->centerOfMass().y()/10;
                btScalar z = rod->centerOfMass().z()/10;
                btVector3 com = btVector3(x, y, z);
                btMatrix3x3 rot = btMatrix3x3(((tgBaseRigid*) rod)->getPRigidBody()->getOrientation());
                x = 0.0;
                y = (rod->length())/20;
                z = 0.0;
                btVector3 orig1 = btVector3(x, y, z);
                btVector3 orig2 = btVector3(x, -y, z);
                btVector3 pos1 = (rot * orig1) + com;
                btVector3 pos2 = (rot * orig2) + com;
                state.pos1.x = pos1.getX();
                state.pos1.y = pos1.getY();
                state.pos1.z = pos1.getZ();
                state.pos2.x = pos2.getX();
                state.pos2.y = pos2.getY();
                state.pos2.z = pos2.getZ();
                const unsigned idx = 2 * i;
                const double motor_pos1 = springCables[idx]->getRestLength();
                const double motor_pos2 = springCables[idx+1]->getRestLength();
                state.motor_pos1.data = (9.5 - motor_pos1) / 0.09;
                state.motor_pos2.data = (9.5 - motor_pos2) / 0.09;
                state_msg.states.push_back(state);
            }
        	robot_state_pub.publish(state_msg);
    	}

        for (unsigned i=0; i<6; ++i) {
            const tgRod* const rod = rods[i];
            btScalar x = rod->centerOfMass().x()/10;
            btScalar y = rod->centerOfMass().y()/10;
            btScalar z = rod->centerOfMass().z()/10;
            osg::Vec3 com(x, y, z);
            btScalar qx = ((tgBaseRigid*) rod)->getPRigidBody()->getOrientation().getX();
            btScalar qy = ((tgBaseRigid*) rod)->getPRigidBody()->getOrientation().getY();
            btScalar qz = ((tgBaseRigid*) rod)->getPRigidBody()->getOrientation().getZ();
            btScalar qw = ((tgBaseRigid*) rod)->getPRigidBody()->getOrientation().getW();
            osg::Quat rot(qx, qy, qz, qw);
            osg::Matrix mat;
            mat.setTrans(com);
            mat.setRotate(rot);
            mat.preMultRotate(osg::Quat(M_PI*0.5, osg::Vec3(1.0, 0.0, 0.0))); // TODO: explain why this is here
            osg::Matrix rotmat(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            mat.postMult(rotmat);
            tfs[i]->setMatrix(mat);
        }
        m_viewer.frame();
    }

    simulation.reset();
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
