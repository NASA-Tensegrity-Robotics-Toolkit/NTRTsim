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

#ifndef CORE_TERRAIN_TG_IMPORT_GROUND_H
#define CORE_TERRAIN_TG_IMPORT_GROUND_H

/**
 * @file tgImportGround.h
 * @brief Contains the definition of class tgImportGround.
 * @author Edward Zhu
 * $Id$
 */

#include "tgBulletGround.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

// std::size_t
#include <cstddef>

#include <fstream>
#include <vector>

#include "BulletCollision/CollisionShapes/btTriangleMesh.h"

// Forward declarations
class btRigidBody;
//class btTriangleIndexVertexArray;
class btTriangelMesh;

/**
 * Ground imported from user generated .stl files
 */
class tgImportGround : public tgBulletGround
{
    public:

        struct Config
        {
            public:
                Config(btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0),
                       double friction = 0.5,
                       double restitution = 0.0,
                       //btVector3 size = btVector3(500.0, 1.5, 500.0),
                       btVector3 origin = btVector3(0.0, 0.0, 0.0),
                       //std::size_t nx = 50,
                       //std::size_t ny = 50,
                       double margin = 0.05,
                       //double triangleSize = 5.0,
                       //double waveHeight = 5.0,
                       double offset = 0.5,
                       double scalingFactor = 10,
                       int interp = 0,
                       bool twoLayer = false);

                /** Euler angles are specified as yaw pitch and roll */
                btVector3 m_eulerAngles;

                /** Friction value of the ground, must be between 0 to 1 */
                btScalar  m_friction;

                /** Restitution coefficient of the ground, must be between 0 to 1 */
                btScalar  m_restitution;

                /** Size of the ground, must be between non-negitive */
                //btVector3 m_size;

                /** Origin position of the ground */
                btVector3 m_origin;

                /** Number of nodes in the x-direction */
                //std::size_t m_nx;

                /** Number of nodes in the y-direction */
                //std::size_t m_ny;

                /** See Bullet documentation on Collision Margin */
                double m_margin;

                /** Scale factor for the X and Z axes, varies according to m_nx and m_ny */
                //double m_triangleSize;

                /** Scale factor for the Y axis */
                //double m_waveHeight;

                /** Translation factor for the Y axis */
                double m_offset;

                /** Scaling factor for the triangle mesh */
                double m_scalingFactor;

                /** Integer for how many times to perform interpolation */
                int m_interpolation;

                /** Boolean for turning on the double layered terrain to help with clipping */
                bool m_twoLayer;
        };

        /**
         * Default construction that uses the default values of config
         * Sets up a collision object that is stored in the bulletGround
         * object
         */
        tgImportGround(std::fstream& file);

        /**
         * Allows a user to specify their own config
         */
        tgImportGround(const tgImportGround::Config& config, std::fstream& file);

        /** Clean up the implementation. Deletes m_pMesh */
        virtual ~tgImportGround();

        /**
         * Setup and return a return a rigid body based on the collision 
         * object
         */
        virtual btRigidBody* getGroundRigidBody() const;

        /**
         * Returns the collision shape that forms the imported ground
         */
        //btCollisionShape* importCollisionShape();

        /**
         * Returns the collision shape that forms the imported ground from filestream
         */
        btCollisionShape* importCollisionShape_alt(std::fstream& file, double scalingFactor, int interp, bool twoLayer);

    private:  
        /** Store the configuration data for use later */
        Config m_config;

        /** Pre-condition: Quantity of triangles and vertices must each be greater than zero 
         *  Post-condition: Returns a mesh, as configured by the input parameters, 
         *                  to be used as a template for a btBvhTriangleMeshShape
         */
        //btTriangleIndexVertexArray* createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]);

        /** Pre-condition: Pointer to a filestream 
         *  Post-condition: Returns a mesh, as configured by the information read from the filestream
                            To be used as a template for a btBvhTriangleMeshShape
         */
        btTriangleMesh* createMesh_alt(std::fstream& file, double scalingFactor, int interp, bool twoLayer);

        /** Pre-condition: Given mesh is a valig btTriangleIndexVertexArray with all values initialized
         *  Post-condition: Returns a btBvhTriangleMeshShape in the shape of the hills as configured 
         */
        //btCollisionShape *createShape(btTriangleIndexVertexArray * pMesh);

        /** Pre-condition: Given mesh is a valid btTriangleMesh
         *  Post-condition: Returns a btBvhTriangleMeshShape of the user created landscape
         */
        btCollisionShape* createShape_alt(btTriangleMesh* pMesh);

        /** Pre-condition: A vector of the verticies triangles
         *  Post-condition: Returns a vector of points corresponding to the interpolated triangles
         */
        std::vector<btVector3> interpolateTriangles(std::vector<btVector3> verticies);
        
        /**
         * @param[out] A flattened array of vertices in the mesh
         */
        //void setVertices(btVector3 vertices[]);

        /**
         * @param[out] A flattened array of indices in the mesh
         */
        //void setIndices(int indices[]);
        
        // Store this so we can delete it later
        btTriangleMesh* m_pMesh;
        //btVector3 * m_vertices;
        //int * m_pIndices;

};

#endif  // TG_IMPORT_GROUND_H
