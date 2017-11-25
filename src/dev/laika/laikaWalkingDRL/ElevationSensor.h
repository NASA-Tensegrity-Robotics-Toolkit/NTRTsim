#ifndef ELEVATION_SENSOR_H
#define ELEVATION_SENSOR_H

#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <numeric/ublas/matrix.hpp>
#include <numeric/ublas/vector.hpp>
#include <numeric/ublas/assignment.hpp>
#include <numeric/ublas/operation.hpp>

#include "LinearMath/btVector3.h"

using namespace boost::numeric::ublas;

class ElevationSensor
{
public:
  ElevationSensor();
  ElevationSensor(double range, double resolution, double sf);

  virtual ~ElevationSensor() {}

  void setSensorParams(double range, double resolution, double sf);

  // Builds a boost matrix elevation map
  void buildElevationMap(std::string filename);

  // Gets a sensor reading of elevation points in a square with m_range/m_resolution points centered at the current position
  matrix<double> getSensorReading(btVector3 com_pos);

private:
  // Get verticies that are on the top surface of the terrain, only saves unique ones
  void getVertexArray(std::string filename);

  // Gets the "num" nearest verticies in m_vertexArray
  std::vector<btVector3> getPlaneVerticies(btVector3 com_pos);

  // Gets the x-z bounds on the terrain
  vector<double> getTerrainBounds();

  // Gets the elevation at a certain x-z position
  double getElevation(double x_pos, double z_pos);

  double m_range, m_resolution, m_sf;
  std::vector<btVector3> m_vertexArray;
  matrix<double> m_elevMap;
  vector<double> m_terrain_bounds;
};

#endif
