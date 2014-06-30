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

//#define BT_USE_DOUBLE_PRECISION

#ifndef CORDE_MODEL
#define CORDE_MODEL

/**
 * @file CordeModel.h
 * @brief Defines structure for the Corde softbody String Model
 * @author Brian Mirletz
 * $Id$
 */

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// The C++ Standard Library
#include <vector>

class CordeModel
{
public:
	struct Config
	{
		Config(const std::size_t res,
				const double r, const double d,
				const double ym, const double shm,
				const double stm, const double csc,
				const double gt, const double gr);
		
		const std::size_t resolution;
		const double radius;
		const double density;
		const double YoungMod;
		const double ShearMod;
		const double StretchMod;
		const double ConsSpringConst;
		/**
		 * For really short segments (< .001 length) consider decreasing
		 * these further or changing length to cubic (currently ^5)
		 */
		const double gammaT;
		const double gammaR;
	};
	
	/**
	 * A constructor which assumes uniformally distributed mass
	 * points and rotation
	 * pos1 and pos2 specify the start and end points of the rod.
	 * quat1 and quat2 need to be computed based on the torsion in the rod.
	 * Note that if there is neither bending nor torsion one can say quat1 = quat2
	 * = btQuaternion((pos2 - pos1).normalize, 0) (axis-angle constructor)
	 * @todo develop a constructor that can handle more complex shapes
	 * i.e. wrapped around a motor. This one maxes out at 1 - eps rotations
	 */
	CordeModel(btVector3 pos1, btVector3 pos2, btQuaternion quat1, btQuaternion quat2, CordeModel::Config& Config);
	
	~CordeModel();
	
	void step (btScalar dt);
	
private:
	void computeConstants();

	void stepPrerequisites();

	void computeInternalForces();
	
	void unconstrainedMotion(double dt);
	
	/**
	 * Holds all of the data for one of the mass elements of the string
	 */
	struct CordePositionElement
	{
		/**
		 * Sets pos to p1, mass to m, everything else to zero
		 * Assumes rod is at rest on start
		 */
		CordePositionElement(btVector3 p1, double m);
		
		btVector3 pos;
		btVector3 vel;
		btVector3 force;
		double mass;
	};
	
	/**
	 * Holds all of the data for the centerline quaternions of the string
	 */
	struct CordeQuaternionElement
	{	
		/**
		 * Sets q to q1.normalized(), everything else to zero
		 */
		CordeQuaternionElement(btQuaternion q1);
		
		void transposeTorques();
		/**
		 * Must be called after transpose torques and omega is updated
		 */
		void updateQDot();
		
		btQuaternion q;
		btQuaternion qdot;
		/**
		 * Just a 4x1 vector, but easier to store this way.
		 */
		btQuaternion tprime;
		btVector3 torques;
		btVector3 omega;
	};
	
	CordeModel::Config m_config;
	
	std::vector<CordePositionElement*> m_massPoints;
	std::vector<CordeQuaternionElement*> m_centerlines;
	/**
	 * Should have length equal to m_massPoints.size()-1
	 */
	std::vector<double> linkLengths;
	/**
	 * Should have length equal to m_Centerlines.size()-1
	 */
	std::vector<double> quaternionShapes;
	
	/**
	 * Computed based on the values in config. Should have length 4
	 * 0: linear stiffness used by mass models
	 * 1 - 3: bending and torsion stiffnesses
	 * @todo can this be const?
	 */
	std::vector<double> computedStiffness;
	
	/**
	 * Computed based on the values in config. Should have length 3
	 * Assuming products of inertia are negligible as in the paper
	 */
	btVector3 computedInertia;
	btVector3 inverseInertia;
	
	bool invariant();
	
	double simTime;
};
 
 
#endif // CORDE_MODEL
