#ifndef RANDOM_SHOOTING_MPC_H
#define RANDOM_SHOOTING_MPC_H

#include <numeric/ublas/matrix.hpp>
#include <numeric/ublas/vector.hpp>
#include "NeuralNet.h"
#include <string>
#include <vector>

using namespace boost::numeric::ublas;

class RandomShootingMPC
{
public:
  RandomShootingMPC();
  RandomShootingMPC(NeuralNet* dyn_model, int horizon, int num_paths);

  virtual ~RandomShootingMPC() {}

  void attachModel(NeuralNet* dyn_model);

  void setMPCParams(int horizon, int num_paths);

  void setInputLims(std::vector<double> uL, std::vector<double> lL);

  std::vector<double> getAction(vector<double> state);

private:
  double costFunction(vector<double> state, vector<double> action, vector<double> next_state);
  
  int m_horizon, m_num_paths;
  NeuralNet* m_dyn_model;
  std::vector<double> m_uL, m_lL;
};

#endif
