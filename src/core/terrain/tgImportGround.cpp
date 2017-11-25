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

/**
 * @file tgImportGround.cpp
 * @brief Contains the implementation of class tgImportGround
 * @author Edward Zhu
 * $Id$
 */

//This Module
#include "tgImportGround.h"

//Bullet Physics
#include "BulletCollision/CollisionShapes/btBoxShape.h"
//#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"

// The C++ Standard Library
#include <cassert>
#include <iostream>
#include <string>
#include <math.h>

tgImportGround::Config::Config(btVector3 eulerAngles,
        double friction,
        double restitution,
        //btVector3 size,
        btVector3 origin,
        double margin,
        double offset,
        double scalingFactor,
        int interp,
        bool twoLayer,
        bool flipZY) :
    m_eulerAngles(eulerAngles),
    m_friction(friction),
    m_restitution(restitution),
    //m_size(size),
    m_origin(origin),
    m_margin(margin),
    m_offset(offset),
    m_scalingFactor(scalingFactor),
    m_interpolation(interp),
    m_twoLayer(twoLayer),
    m_flipZY(flipZY)
{
    assert((m_friction >= 0.0) && (m_friction <= 1.0));
    assert((m_restitution >= 0.0) && (m_restitution <= 1.0));
    //assert((m_size[0] >= 0.0) && (m_size[1] >= 0.0) && (m_size[2] >= 0.0));
    assert(m_margin >= 0.0);
    assert(m_offset >= 0.0);
    assert(m_scalingFactor >= 0.0);
}

tgImportGround::tgImportGround(std::fstream& file) :
    m_config(Config())
{
    // @todo make constructor aux to avoid repeated code
    pGroundShape = importCollisionShape_alt(file, m_config.m_scalingFactor, m_config.m_interpolation, m_config.m_twoLayer, m_config.m_flipZY);
}

tgImportGround::tgImportGround(const tgImportGround::Config& config, std::fstream& file) :
    m_config(config)
{
    pGroundShape = importCollisionShape_alt(file, m_config.m_scalingFactor, m_config.m_interpolation, m_config.m_twoLayer, m_config.m_flipZY);
}

tgImportGround::~tgImportGround()
{
    delete m_pMesh;
    //delete[] m_pIndices;
    //delete[] m_vertices;
}

btRigidBody* tgImportGround::getGroundRigidBody() const
{
        std::cout << "Imported ground " << std::endl;
    const btScalar mass = 0.0;

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(m_config.m_origin);

    btQuaternion orientation;
    orientation.setEuler(m_config.m_eulerAngles[0], // Yaw
                         m_config.m_eulerAngles[1], // Pitch
                         m_config.m_eulerAngles[2]); // Roll
    groundTransform.setRotation(orientation);

    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
        new btDefaultMotionState(groundTransform);

    const btVector3 localInertia(0, 0, 0);

    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundBody = new btRigidBody(rbInfo);

    assert(pGroundBody);
    return pGroundBody;
}

/*
btCollisionShape* tgImportGround::importCollisionShape() {
    btCollisionShape * pShape = 0;
    // The number of vertices in the mesh
    // Hill Paramenters: Subject to Change
    const std::size_t vertexCount = m_config.m_nx * m_config.m_ny;

    if (vertexCount > 0) {
        // The number of triangles in the mesh
        const std::size_t triangleCount = 2 * (m_config.m_nx - 1) * (m_config.m_ny - 1);

        // A flattened array of all vertices in the mesh
        m_vertices = new btVector3[vertexCount];

        // Supplied by the derived class
        setVertices(m_vertices);
        // A flattened array of indices for each corner of each triangle
        m_pIndices = new int[triangleCount * 3];

        // Supplied by the derived class
        setIndices(m_pIndices);

        // Create the mesh object
        m_pMesh = createMesh(triangleCount, m_pIndices, vertexCount, m_vertices);

        // Create the shape object
        pShape = createShape(m_pMesh);

        // Set the margin
        pShape->setMargin(m_config.m_margin);
        // DO NOT deallocate vertices, indices or pMesh until simulation is over!
        // The shape owns them, but will not delete them
    }

    assert(pShape);
    return pShape;
}
*/

btCollisionShape* tgImportGround::importCollisionShape_alt(std::fstream& file, double scalingFactor, int interp, bool twoLayer, bool flipZY) {
    btCollisionShape * pShape = 0;

    // Create the mesh object
    m_pMesh = createMesh_alt(file, scalingFactor, interp, twoLayer, flipZY);

    // Create the shape object
    pShape = createShape_alt(m_pMesh);

    // Set the margin
    pShape -> setMargin(m_config.m_margin);

    assert(pShape);
    return pShape;
}

/*
btTriangleIndexVertexArray *tgImportGround::createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]) {
    const int vertexStride = sizeof(btVector3);
    const int indexStride = 3 * sizeof(int);

    btTriangleIndexVertexArray* const pMesh =
        new btTriangleIndexVertexArray(triangleCount,
                indices,
                indexStride,
                vertexCount,
                (btScalar*) &vertices[0].x(),
                vertexStride);
    return pMesh;
}
*/

btTriangleMesh* tgImportGround::createMesh_alt(std::fstream& file, double scalingFactor, int interp, bool twoLayer, bool flipZY) {
    // Lines are input in the following format: [x1,y1,z1] [x2,y2,z2] [x3,y3,z3]

    btVector3 v0, v1, v2;

    //double scalingFactor = 100;

    btTriangleMesh* const pMesh =
        new btTriangleMesh();

    double max_X = 0, max_Y = -10000, max_Z = 0, min_Y = 100*scalingFactor;
    double x, y, z;

    while (file.good()) {
    //for (int line = 0; line < 200; line++) {

        std::vector<btVector3> verticies;

        std::string line_in;
        std::getline(file, line_in);

        std::size_t found_left_brac_last = -1, found_right_brac_last = -1, found_comma_last = -1;

        // Iterate over 3 sets of vertecies
        for (int i = 0; i < 3; i++) {
            std::size_t found_left_brac = line_in.find_first_of("[", found_left_brac_last + 1);
            std::size_t found_right_brac = line_in.find_first_of("]", found_right_brac_last + 1);
            std::size_t found_comma_1 = line_in.find_first_of(",", found_comma_last + 1);
            std::size_t found_comma_2 = line_in.find_first_of(",", found_comma_1 + 1);

            // Extract x value
            std::string x_str = line_in.substr(found_left_brac + 1, found_comma_1 - 1 - found_left_brac);
            x = atof(x_str.c_str()) * scalingFactor;

            // Extract y value
            std::string y_str = line_in.substr(found_comma_1 + 1, found_comma_2 - 1 - found_comma_1);
            y = atof(y_str.c_str()) * scalingFactor;

            // Extrack z value
            std::string z_str = line_in.substr(found_comma_2 + 1, found_right_brac - 1 - found_comma_2);
            z = atof(z_str.c_str()) * scalingFactor;

            // Swap y and z values to match NTRT coordinate convention (y is height)
            if (flipZY) {
                double temp = y;
                y = z;
                z = -temp;
            }

            if (abs(x) > abs(max_X)) max_X = x;
            if (abs(y) > abs(max_Y)) max_Y = y;
            if (abs(z) > abs(max_Z)) max_Z = z;
            if (abs(y) < abs(min_Y) && abs(y) > 0) min_Y = y;

            // Update last found positions
            found_left_brac_last = found_left_brac;
            found_right_brac_last = found_right_brac;
            found_comma_last = found_comma_2;

            //std::cout << x << " " << y << " " << z << std::endl;

            switch (i) {
                case 0:
                    v0.setValue(x, y, z);
                    // verticies.push_back(v0);
                    break;
                case 1:
                    v1.setValue(x, y, z);
                    // verticies.push_back(v1);
                    break;
                case 2:
                    v2.setValue(x, y, z);
                    // verticies.push_back(v2);
                    break;
            }
        }

        double xBound = 100;
        double zBound = 100;

        if (v0.x() > xBound*scalingFactor || v0.y() <= 0 || v0.z() < -zBound*scalingFactor ||
            v1.x() > xBound*scalingFactor || v1.y() <= 0 || v1.z() < -zBound*scalingFactor ||
            v2.x() > xBound*scalingFactor || v2.y() <= 0 || v2.z() < -zBound*scalingFactor) {
            // continue;
        }

        // v0.setY(v0.y()-max_Y);
        // v1.setY(v1.y()-max_Y);
        // v2.setY(v2.y()-max_Y);

        // v0.setY(v0.y()-155000);
        // v1.setY(v1.y()-155000);
        // v2.setY(v2.y()-155000);

        verticies.push_back(v0);
        verticies.push_back(v1);
        verticies.push_back(v2);

        for (int k = 0; k < interp; k++) {
            verticies = interpolateTriangles(verticies);
        }

        for (int i = 0; i < verticies.size()/3; i++) {

            btVector3 v0 = verticies[i*3];
            btVector3 v1 = verticies[i*3+1];
            btVector3 v2 = verticies[i*3+2];

            pMesh -> addTriangle(v0, v1, v2);

            if (twoLayer) {
                v0.setY(v0.y()-1);
                v1.setY(v1.y()-1);
                v2.setY(v2.y()-1);

                pMesh -> addTriangle(v0, v1, v2);
            }
        }
    }

    // std::cout << "Max X coordinate: " << max_X << ", Max Y coordinate: " << max_Y << ", Max Z coordinate: " << max_Z << std::endl;
    // std::cout << "Min Y coordinate: " << min_Y << std::endl;

    /*
    // Test triangle
    btVector3 v0_test(1, 0, 0);
    btVector3 v1_test(0, 1, 0);
    btVector3 v2_test(0, 0, 1);

    pMesh -> addTriangle(v0_test, v1_test, v2_test);
    */

    return pMesh;
}

/*
btCollisionShape *tgImportGround::createShape(btTriangleIndexVertexArray *pMesh) {
    const bool useQuantizedAabbCompression = true;
    btCollisionShape *const pShape =
        new btBvhTriangleMeshShape(pMesh, useQuantizedAabbCompression);
    return pShape;
}
*/

btCollisionShape* tgImportGround::createShape_alt(btTriangleMesh *pMesh) {
    btCollisionShape *const pShape =
        new btBvhTriangleMeshShape(pMesh, true);
    return pShape;
}

std::vector<btVector3> tgImportGround::interpolateTriangles(std::vector<btVector3> verticies) {

    int vertLength = verticies.size();

    for (int i=0; i<vertLength/3; i++) {
        // Unpack verticies vector
        btVector3 v0 = verticies[i*3];
        btVector3 v1 = verticies[i*3+1];
        btVector3 v2 = verticies[i*3+2];

        // Find mid points
        btVector3 v01 = v0+(v1-v0)/2.0;
        btVector3 v02 = v0+(v2-v0)/2.0;
        btVector3 v12 = v1+(v2-v1)/2.0;

        verticies.push_back(v0);
        verticies.push_back(v01);
        verticies.push_back(v02);

        verticies.push_back(v1);
        verticies.push_back(v01);
        verticies.push_back(v12);

        verticies.push_back(v2);
        verticies.push_back(v12);
        verticies.push_back(v02);

        verticies.push_back(v01);
        verticies.push_back(v02);
        verticies.push_back(v12);
    }

    return verticies;

}

/*
void tgImportGround::setVertices(btVector3 vertices[]) {
    for (std::size_t i = 0; i < m_config.m_nx; i++)
    {
        for (std::size_t j = 0; j < m_config.m_ny; j++)
        {
            const btScalar x = (i - (m_config.m_nx * 0.5)) * m_config.m_triangleSize;
            const btScalar y = (m_config.m_waveHeight * sin((double)i) * cos((double)j) +
                    m_config.m_offset);
            const btScalar z = (j - (m_config.m_ny * 0.5)) * m_config.m_triangleSize;
            vertices[i + (j * m_config.m_nx)].setValue(x, y, z);
        }
    }
}
*/

/*
void tgImportGround::setIndices(int indices[]) {
    int index = 0;
    for (std::size_t i = 0; i < m_config.m_nx - 1; i++)
    {
        for (std::size_t j = 0; j < m_config.m_ny - 1; j++)
        {
            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = (j       * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;

            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i;
        }
    }
}
*/
