#include "ElevationSensor.h"

ElevationSensor::ElevationSensor() {}

ElevationSensor::ElevationSensor(double range, double resolution, double sf)
{
  m_range = range;
  m_resolution = resolution;
  m_sf = sf;
}

void ElevationSensor::setSensorParams(double range, double resolution, double sf)
{
  m_range = range;
  m_resolution = resolution;
  m_sf = sf;
}

void ElevationSensor::buildElevationMap(std::string filename)
{
  getVertexArray(filename);

  m_terrain_bounds.resize(4);
  m_terrain_bounds = getTerrainBounds();
  // std::cout << terrain_bounds(0) << "," << terrain_bounds(1) << "," << terrain_bounds(2) << "," << terrain_bounds(3) << std::endl;
  int num_x_elmts = int((m_terrain_bounds(1)-m_terrain_bounds(0))/m_resolution);
  int num_z_elmts = int((m_terrain_bounds(3)-m_terrain_bounds(2))/m_resolution);
  // std::cout << num_x_elmts << "," << num_z_elmts << std::endl;
  m_elevMap.resize(num_x_elmts, num_z_elmts);

  // btVector3 test_pos(180,0,-20);
  // std::vector<btVector3> nearest = getPlaneVerticies(test_pos);
  // std::cout << nearest.size() << std::endl;
  // std::cout << nearest[0].x() << "," << nearest[0].y() << "," << nearest[0].z() << std::endl;
  // std::cout << nearest[1].x() << "," << nearest[1].y() << "," << nearest[1].z() << std::endl;
  // std::cout << nearest[2].x() << "," << nearest[2].y() << "," << nearest[2].z() << std::endl;
  // std::cout << (nearest[1].y()-nearest[0].y())/(nearest[1].x()-nearest[0].x()) <<
  //           "," << (nearest[2].y()-nearest[0].y())/(nearest[2].z()-nearest[0].z()) << std::endl;

  double x_pos, z_pos, x_slope, z_slope;

  for (int i = 0; i < num_x_elmts; i++) {
    for (int j = 0; j < num_z_elmts; j++) {
      x_pos = i*m_resolution + m_terrain_bounds(0);
      z_pos = j*m_resolution + m_terrain_bounds(2);
      btVector3 pos(x_pos, 0, z_pos);
      std::vector<btVector3> nearest = getPlaneVerticies(pos);
      x_slope = (nearest[1].y()-nearest[0].y())/(nearest[1].x()-nearest[0].x());
      m_elevMap(i,j) = nearest[0].y()+x_slope*(pos-nearest[0]).x();
    }
  }
}

matrix<double> ElevationSensor::getSensorReading(btVector3 com_pos)
{
  btVector3 com_pos_unscaled = com_pos/10;
  std::cout << "Getting sensor reading at " << com_pos_unscaled.x() << "," << com_pos_unscaled.z() << std::endl;
  int x_elmnt_pos = round((com_pos_unscaled.x()-m_terrain_bounds(0))/m_resolution);
  int z_elmnt_pos = round((com_pos_unscaled.z()-m_terrain_bounds(2))/m_resolution);
  std::cout << x_elmnt_pos << "," << z_elmnt_pos << std::endl;
  int range_elmnts = int(m_range/m_resolution);
  matrix<double> reading(2*range_elmnts+1,2*range_elmnts+1);
  for (int i = 0; i < 2*range_elmnts+1; i++) {
    for (int j = 0; j < 2*range_elmnts+1; j++) {
      double elev = m_elevMap(x_elmnt_pos-range_elmnts+i,z_elmnt_pos-range_elmnts+j);
      reading(i,j) = elev;
      std::cout << elev << std::endl;
    }
  }
  std::cout << reading.size1() << "," << reading.size2() << std::endl;
  return reading;
}

void ElevationSensor::getVertexArray(std::string filename)
{
  // Clear contents of vector
  m_vertexArray.clear();

  std::ifstream file;
  std::string line;
  std::string value;

  std::string key = "vertex";
  std::string delimiter = " "; // Coordinates are space delimited in STL files
  int vertex_count = 0;
  int line_index = 0;
  int element_index = 0;

  file.open(filename.c_str());

  // Check for valid file header in first line
  getline(file, line);
  if (line.find("solid") == std::string::npos) {
    std::cout << "Incorrect file header, make sure the input file is an ASCII STL file" << std::endl;
    exit(EXIT_FAILURE);
  }
  else {
    std::cout << "Valid header found, building elevation map with file: " << filename <<  std::endl;
  }
  line_index += 1;

  while (file.good()) {
    getline(file, line);
    line_index += 1;

    // Find vertex tag in line, if it can't be found, skip line
    size_t found_key = line.find(key);
    if (found_key == std::string::npos) {
      continue;
    }

    btVector3 vertex;

    // Parse through line to get vertex values
    size_t delim_1_pos = line.find(delimiter, found_key)+1;;
    size_t delim_2_pos;

    for (int i = 0; i < 3; i++) {
      delim_2_pos = line.find(delimiter,delim_1_pos);
      if (delim_2_pos == std::string::npos) {
        value = line.substr(delim_1_pos);
      }
      else {
        value = line.substr(delim_1_pos,delim_2_pos-delim_1_pos);
      }
      if (i == 0) {
        vertex.setX(std::atof(value.c_str()));
      }
      else if (i == 1) {
        vertex.setY(std::atof(value.c_str()));
      }
      else {
        vertex.setZ(std::atof(value.c_str()));
      }
      delim_1_pos = delim_2_pos+1;
    }

    std::vector<btVector3>::iterator it;
    it = find(m_vertexArray.begin(), m_vertexArray.end(), vertex);
    if (it != m_vertexArray.end() || vertex.y() < 1e-10) {
      continue;
    }

    // std::cout << vertex.x() << "," << vertex.y() << "," << vertex.z() << std::endl;
    m_vertexArray.push_back(vertex);
    vertex_count += 1;
  }
  std::cout << "Number of unique top surface verticies: " << vertex_count << std::endl;
  file.close();
}

std::vector<btVector3> ElevationSensor::getPlaneVerticies(btVector3 com_pos)
{
  int nearest_idx;
  std::vector<btVector3> nearest;
  std::vector<btVector3> tmp;

  btVector3 first_ref;
  btVector3 x_ref;
  btVector3 z_ref;

  bool x_larger = false, z_larger = false;

  // Use a copy of m_vertexArray as we will be deleting elements once they are found
  tmp.assign(m_vertexArray.begin(), m_vertexArray.end());

  // Find nearest 3 vertecies
  for (int j = 0; j < 3; j++) {
    double min_dist = 10000;
    if (j == 0) {
      for (int i = 0; i < tmp.size(); i++) {
        double dist = com_pos.distance(tmp[i]);
        if (dist < min_dist) {
          min_dist = dist;
          nearest_idx = i;
        }
      }
      first_ref.setX(tmp[nearest_idx].x());
      first_ref.setY(tmp[nearest_idx].y());
      first_ref.setZ(tmp[nearest_idx].z());
      tmp.erase(tmp.begin()+nearest_idx);
      if (com_pos.x() >= first_ref.x()) {
        x_larger = true;
      }
      if (com_pos.z() >= first_ref.z()) {
        z_larger = true;
      }
    }
    else if (j == 1) {
      for (int i = 0; i < tmp.size(); i++) {
        if (abs(tmp[i].z()-first_ref.z()) < 0.1) {
          double dist = com_pos.distance(tmp[i]);
          if (x_larger) {
            if (dist < min_dist && tmp[i].x() > first_ref.x()) {
              min_dist = dist;
              nearest_idx = i;
            }
          }
          else {
            if (dist < min_dist && tmp[i].x() < first_ref.x()) {
              min_dist = dist;
              nearest_idx = i;
            }
          }
        }
      }
      x_ref.setX(tmp[nearest_idx].x());
      x_ref.setY(tmp[nearest_idx].y());
      x_ref.setZ(tmp[nearest_idx].z());
      tmp.erase(tmp.begin()+nearest_idx);
    }
    else if (j == 2) {
      for (int i = 0; i < tmp.size(); i++) {
        if (abs(tmp[i].x()-first_ref.x()) < 0.1) {
          double dist = com_pos.distance(tmp[i]);
          if (z_larger) {
            if (dist < min_dist && tmp[i].z() > com_pos.z()) {
              min_dist = dist;
              nearest_idx = i;
            }
          }
          else {
            if (dist < min_dist && tmp[i].z() < com_pos.z()) {
              min_dist = dist;
              nearest_idx = i;
            }
          }
        }
      }
      z_ref.setX(tmp[nearest_idx].x());
      z_ref.setY(tmp[nearest_idx].y());
      z_ref.setZ(tmp[nearest_idx].z());
      tmp.erase(tmp.begin()+nearest_idx);
    }
  }
  nearest.push_back(first_ref);
  nearest.push_back(x_ref);
  nearest.push_back(z_ref);
  return nearest;
}

vector<double> ElevationSensor::getTerrainBounds()
{
  double min_x = 10000, min_z = 10000, max_x = -10000, max_z = -10000;
  for (int i = 0; i < m_vertexArray.size(); i++) {
    double x = m_vertexArray[i].x();
    double z = m_vertexArray[i].z();
    if (x < min_x) {
      min_x = x;
    }
    else if (x > max_x) {
      max_x = x;
    }
    if (z < min_z) {
      min_z = z;
    }
    else if (z > max_z) {
      max_z = z;
    }
  }
  vector<double> bounds(4);
  bounds <<= min_x, max_x, min_z, max_z;
  return bounds;
}
