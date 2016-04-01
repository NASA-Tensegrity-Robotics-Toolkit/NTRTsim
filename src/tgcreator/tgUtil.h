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

/// Rand seeding simular to the evolution and terrain classes. 

#ifndef TG_UTIL_H
#define TG_UTIL_H

/**
 * @file tgUtil.h
 * @brief Contains the definition of class tgUtil and overloaded
 * operator<<() free functions.
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * @author Ryan Adams
 * $Id$
 */

#include "btBulletDynamicsCommon.h" // stream operations for collision shapes
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "tgRigidInfo.h"

/**
 * Contains only static constants and static member functions deemed
 * generally useful.
 */
class tgUtil 
{
public:

    /**
     * Get an 'up' vector (y axis is up)
     * @return a btVector3 (0,1,0)
     */
    inline static btVector3 upVector() 
    {
        return btVector3(0.0, 1.0, 0.0);
    };
    
    /**
     * Return a degree symbol
     * @return the unicode 'degrees' character
     */
    inline static std::string degSymbol() 
    {
        return "\u00B0";
    }

    /**
     * Return a btVector3 halfway between start and end.
     * @param[in] start a btVector3
     * @param[in] end a btVector3
     * @return a btVector3 halfway between start and end
     */
    inline static btVector3 center(const btVector3& start,
                       const btVector3& end)
    {
        return (start + end) / 2.0;
    }

    /**
     * Return the transform required to place a 'long' element (e.g., a
     * cylinder), rotating from startOrientation, with center of mass
     * between start and end.
     * @param[in] start a btVector3
     * @param[in] end a btVector3
     * @return the transform required to place a 'long' element (e.g., a
     * cylinder), rotating from startOrientation, with center of mass
     * between start and end
     */
    inline static btTransform getTransform(const btVector3& startOrientation,
                           const btVector3& start,
                           const btVector3& end)
    {

        const btVector3 origin = center(start, end);
        btTransform t = btTransform();
        t.setIdentity();
        t.setOrigin(origin);
        // If they for some reason gave us the same vector, keep identity
        
		t.setRotation(getQuaternionBetween(startOrientation,
					   getVector(start, end)));

        return t;
    }
    
    /**
     * Get a transform using 'up()' as the starting orientation.
     * @param[in] start a btVector3
     * @param[in] end a btVector3
     */
    inline static btTransform getTransform(const btVector3& start,
                           const btVector3& end)
    {
        return getTransform(upVector(), start, end);
    }

	inline static btTransform getTransform(const btVector3& center)
	{
		btTransform t = btTransform();
        t.setIdentity();
        t.setOrigin(center);
		// No point in rotating, keep identity
        return t;
	}

    /**
     * Get a vector that points from a to b.
     * @param[in] from a btVector3
     * @param[in] to a btVector3
     * @return the vector difference to - fromn
     */
    static btVector3 getVector(const btVector3& from,
                   const btVector3& to)
    {
        return to - from;
    }

    /**
     * Return a vector that points to the closest point on the edge of a
     * cylinder when it is added to a vector pointing to the top of the
     * cylinder.
     *
     *   --->  = returned vector, if the target is in this direction
     *   ^  |
     *   |  |  <- cylinder edge
     *   | <- cylinder center
     *
     * @param[in] axis a btVector3, passed by value
     * @param[in] radius unused
     * @param[in] traget a btVector3, passed by value
     * @return a vector that points to the closest point on the edge of a
     * cylinder when it is added to a vector pointing to the top of the
     * cylinder
     * @note This has not yet been tested as of 05/14/2013
     */
    static btVector3 getRadiusVector(btVector3 axis,
                     double radius,
                     btVector3 target) 
    {
        // NOTE: axis and target are passed by value,
            // so we can alter them.
        axis.normalize();
        target.normalize();
        // Get a vector normal to both
        const btVector3 norm = axis.cross(target);
        // Rotate the axis by 90 degrees around the normal vector
        axis.rotate(norm, (M_PI / 2.0)); 
        return axis;
    }

    /**
     * Check that two vectors are almost equal within a given precision
     * (default is 5 decimal places).
     * @param[in] from a btVector3
     * @param[in] to a btVector3
     * @param[in] precision the number of decimal places; defaults to 5 if
     * not supplied or negative
     */
    static bool almostEqual(const btVector3& from,
                const btVector3& to, 
                int precision = 5)
    {
        return from.distance(to) < pow(10.0, (precision < 0 ? 5 : -precision));
    }

    /**
     * Return a quaternion that, if applied, would rotate vector a to align
     * with vector b.
     * @param[in] a a btVector3, passed by value
     * @param[in] b a btVector3, passed by value
     * @param[in] fallbackAxis a btVector3, passed by reference
     * @return a quaternion that, if applied, would rotate vector a to align
     * with vector b
     * @todo get some sensible value if a or b = (0, 0, 0). See getTransform
     */
    static btQuaternion getQuaternionBetween(btVector3 a, btVector3 b, const btVector3& fallbackAxis = btVector3(0,0,0))
    {
        a.normalize();
        b.normalize();

        // The return value
        btQuaternion result;
		
        // Account for equal vectors (can't calculate c in this case)
        if (almostEqual(a, b)) {
            result = btQuaternion::getIdentity();
        } else if (almostEqual(a, -b)) {
            // Account for opposing vectors (can't calculate c in
            // this case either)
            if (!fallbackAxis.isZero())
            {
                result = btQuaternion(fallbackAxis, M_PI).normalize();
            }
            else if (a.dot(btVector3(1.0, 0.0, 0.0)) == 0.0)
            {
                // Gets around bad btTransforms with an up vector
                result = btQuaternion (-1.0, 0.0, 0.0, 0.0);
            }
            else
            {
                const btVector3 arb =
                a + getArbitraryNonParallelVector(a);
                const btVector3 c = (a.cross(arb)).normalize();
                result = btQuaternion(c, M_PI).normalize();
            }
        } else {
            // Create a vector normal to both a and b
            const btVector3 c = (a.cross(b)).normalize();

            // Create a quaternion that represents a rotation about
            // c by the angle between a and b
            result = btQuaternion(c, acos(a.dot(b))).normalize();
        }
        return result;
    }

    /**
     * Returns the mean position of a set of btVector3 points.
     * (added to accommodate structures encoded in YAML)
     * @param[in] points a vector of btVector3, passed by reference
     * @return a btVector3 that represents the centroid of the points vector
     */
    static btVector3 getCentroid(const std::vector<btVector3>& points) {
        int numPoints = points.size();
        btVector3 centroid = btVector3(0, 0, 0);
        for (int i = 0; i < numPoints; i++) {
            centroid += points[i];
        }
        centroid /= numPoints;
        return centroid;
    }

    /**
     * Return a random btVector3 that is not parallel to v.
     * @param[in] v a btVector3, passed by value
     * @return a random btVector3 that is not parallel to v
     */
    inline static btVector3 getArbitraryNonParallelVector(btVector3 v)
    {
        btVector3 arb;
        v.normalize();
        do {
            arb = btVector3(rand()%10, rand()%10, rand()%10).normalize();
        } while (arb == v || arb == -v);
        return arb;
    }

    /** 
     * Rotate the provided vector (v) around a fixed point
     */
    inline static void addRotation(btVector3& v, 
                                   const btVector3& fixedPoint, 
                                   const btVector3& axis, 
                                   double angle)
    {
        // Get a vector from fixedPoint to this
        btVector3 relative = v - fixedPoint;

        // Rotate the relative vector
        btVector3 rotated = relative.rotate(axis, angle);

        // Set our new values
        v.setX(fixedPoint.x() + rotated.x());
        v.setY(fixedPoint.y() + rotated.y());
        v.setZ(fixedPoint.z() + rotated.z());
    }

    /** 
     * Rotate the provided vector (v) around a fixed point
     */
    inline static void addRotation(btVector3& v, 
                                   const btVector3& fixedPoint, 
                                   const btVector3& fromOrientation, 
                                   const btVector3& toOrientation)
    {
        btQuaternion rotation = getQuaternionBetween(fromOrientation, toOrientation);
        addRotation(v, fixedPoint, rotation);
    }
    
    /** 
     * Rotate the provided vector (v) around a fixed point
     */
    inline static void addRotation(btVector3& v, 
                                   const btVector3& fixedPoint, 
                                   const btQuaternion& rotation)
    {
        addRotation(v, fixedPoint, rotation.getAxis(), rotation.getAngle());
    }
	
	/**
	 * @todo write a function that returns the normal of a vector without overwriting the original vector
	 * line btVector3's .normalize() functions
	 */

    /**
     * Convert radians to degrees.
     * @param[in] radians an angle measured in radians
     * @return the angle as measured in degrees
     * @todo Normalize the return value so that it is in [0.0, 360.0).
     */
    inline static double rad2deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    /**
     * Convert degrees to radians.
     * @param[in] degrees an angle measured in degrees
     * @return the angle as measured in radians
     * @todo Normalize the return value so that it is in [0.0, 2 * pi).
     */
    inline static double deg2rad(double degrees)
    {
        return degrees * (M_PI / 180.0);
    }

    /**
     * Return a string representation of the given number of degees with the
     * degree symbol appended.
     * @param[in] degrees a measure in degrees
     * @return a string representation of the given number of degees with the
     * degree symbol appended
     */
    inline static std::string strDeg(double degrees) {
            std::ostringstream s;
            s << degrees << degSymbol();
            return s.str();
    }
    //static std::string strDeg(double degrees);

    inline static double round(double d, int precision = 5)
    {
        const double base = 10.0;
        const int m = static_cast<int>(pow(base, precision));
        return floor(d * m + 0.5)/m;
    }
    
    static void seedRandom();
    
    /// @todo is this necessary? If everyone uses the above function we can just change the 
    /// definition of rdtsc to seed random everywhere. 
    static void seedRandom(int seed);
};

/**
 * Overload operator<<() to handle a btQuaternion.
 * @param[in,out] os an ostream
 * @param[in] q a btQuaternion
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btQuaternion& q)
{
        os << "btQuaternion( " 
       << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
       << " )";
    return os;
}

/**
 * Overload operator<<() to handle a btVector3.
 * @param[in,out] os an ostream
 * @param[in] v a btVector3
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btVector3& v)
{
        os << "btVector3( " << v.x() << ", " << v.y() << ", " << v.z() << " )";
    return os;
}

/**
 * Overload operator<<() to handle a btTransform.
 * @param[in,out] os an ostream
 * @param[in] xf a btTransform
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btTransform& xf)
{
        os << "btTransform: origin = " << xf.getOrigin()
       << "; rotation = " << xf.getRotation();
    return os;
}

/**
 * Overload operator<<() to handle a btRigidBody.
 * @param[in,out] os an ostream
 * @param[in] rb a btRigidBody
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btRigidBody& rb)
{
    os << "btRigidBody: " << rb.getCollisionShape();
    return os;
}

/**
 * Overload operator<<() to handle a btCollisionShape.
 * @param[in,out] os an ostream
 * @param[in] cs a btCollisionShape
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btCollisionShape& cs)
{
    os << "btCollisionShape: " << cs.getShapeType();
    return os;
}

/**
 * Overload operator<<() to handle a btCompoundShape.
 * @param[in,out] os an ostream
 * @param[in] cs a btCompoundShape
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const btCompoundShape& cs)
{
        os << "btCompoundShape: " << cs.getShapeType() << std::endl
       << "  # Children: " << cs.getNumChildShapes() << std::endl;
    return os;
}



#endif  // TG_UTIL_H
