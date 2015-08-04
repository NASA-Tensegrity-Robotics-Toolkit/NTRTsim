/**
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

#ifndef CORE_TERRAIN_TG_HILLY_GROUND_H
#define CORE_TERRAIN_TG_HILLY_GROUND_H

/**
 * @file tgHillyGround.h
 * @brief Contains the definition of class tgHillyGround.
 * @author Steven Lessard
 * $Id$
 */

#include "tgBulletGround.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

// std::size_t
#include <cstddef>

// Forward declarations
class btRigidBody;
class btTriangleIndexVertexArray;

/**
 * A "hilly" ground, with randomized hills and valleys
 */
class tgHillyGround : public tgBulletGround
{
    public:

        struct Config
        {
            public:
                Config(btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0),
                       double friction = 0.5,
                       double restitution = 0.0,
                       btVector3 size = btVector3(500.0, 1.5, 500.0),
                       btVector3 origin = btVector3(0.0, 0.0, 0.0),
                       std::size_t nx = 50,
                       std::size_t ny = 50,
                       double margin = 0.05,
                       double triangleSize = 5.0,
                       double waveHeight = 5.0,
                       double offset = 0.5);

                /** Euler angles are specified as yaw pitch and roll */
                btVector3 m_eulerAngles;

                /** Friction value of the ground, must be between 0 to 1 */
                btScalar  m_friction;

                /** Restitution coefficient of the ground, must be between 0 to 1 */
                btScalar  m_restitution;

                /** Size of the ground, must be between non-negitive */
                btVector3 m_size;

                /** Origin position of the ground */
                btVector3 m_origin;

                /** Number of nodes in the x-direction */
                std::size_t m_nx;

                /** Number of nodes in the y-direction */
                std::size_t m_ny;

                /** See Bullet documentation on Collision Margin */
                double m_margin;

                /** Scale factor for the X and Z axes, varies according to m_nx and m_ny */
                double m_triangleSize;

                /** Scale factor for the Y axis */
                double m_waveHeight;

                /** Translation factor for the Y axis */
                double m_offset;
        };

        /**
         * Default construction that uses the default values of config
         * Sets up a collision object that is stored in the bulletGround
         * object
         */
        tgHillyGround();

        /**
         * Allows a user to specify their own config
         */
        tgHillyGround(const tgHillyGround::Config& config);

        /** Clean up the implementation. Deletes m_pMesh */
        virtual ~tgHillyGround();

        /**
         * Setup and return a return a rigid body based on the collision 
         * object
         */
        virtual btRigidBody* getGroundRigidBody() const;

        /**
         * Returns the collision shape that forms a hilly ground
         */
        btCollisionShape* hillyCollisionShape();

    private:  
        /** Store the configuration data for use later */
        Config m_config;

        /** Pre-condition: Quantity of triangles and vertices must each be greater than zero 
         *  Post-condition: Returns a mesh, as configured by the input parameters, 
         *                  to be used as a template for a btBvhTriangleMeshShape
         */
        btTriangleIndexVertexArray* createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]);

        /** Pre-condition: Given mesh is a valig btTriangleIndexVertexArray with all values initialized
         *  Post-condition: Returns a btBvhTriangleMeshShape in the shape of the hills as configured 
         */
        btCollisionShape *createShape(btTriangleIndexVertexArray * pMesh);

        /**
         * @param[out] A flattened array of vertices in the mesh
         */
        void setVertices(btVector3 vertices[]);

        /**
         * @param[out] A flattened array of indices in the mesh
         */
        void setIndices(int indices[]);
        
        // Store this so we can delete it later
        btTriangleIndexVertexArray* m_pMesh;
        btVector3 * m_vertices;
        int * m_pIndices;

};

#endif  // TG_HILLY_GROUND_H
