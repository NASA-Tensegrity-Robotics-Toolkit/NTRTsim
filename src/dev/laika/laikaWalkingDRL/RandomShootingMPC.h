#ifndef RANDOM_SHOOTING_MPC_H
#define RANDOM_SHOOTING_MPC_H

#include <numeric/ublas/matrix.hpp>
#include <numeric/ublas/vector.hpp>
#include "NeuralNetDynamics.h"
#include <string>
#include <vector>
#include <time.h>

using namespace boost::numeric::ublas;

class RandomShootingMPC
{
public:
  RandomShootingMPC();
  RandomShootingMPC(NeuralNetDynamics* dyn_model, int horizon, int num_paths);

  virtual ~RandomShootingMPC() {}

  void attachDynamicsModel(NeuralNetDynamics* dyn_model);

  void setMPCParams(int horizon, int num_paths, int num_cables, int num_legs);

  void setInputLims(vector<double> uL, vector<double> lL);

  vector<double> getAction(vector<double> state);

private:
  double costFunction(vector<double> state, vector<double> action, vector<double> next_state);

  vector<double> sampleAction();

  int m_horizon, m_num_paths, m_num_cables, m_num_legs, m_num_c, m_num_d;
  NeuralNetDynamics* m_dyn_model;
  vector<double> m_uL, m_lL;
};

#endif
