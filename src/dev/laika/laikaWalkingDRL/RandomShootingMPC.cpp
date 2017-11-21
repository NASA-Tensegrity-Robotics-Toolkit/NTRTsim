#include "RandomShootingMPC.h"

RandomShootingMPC::RandomShootingMPC() {}

RandomShootingMPC::RandomShootingMPC(NeuralNet* dyn_model, int horizon, int num_paths)
{
  m_dyn_model = dyn_model;
  m_horizon = horizon;
  m_num_paths = num_paths;
}

void RandomShootingMPC::attachModel(NeuralNet* dyn_model)
{
  m_dyn_model = dyn_model;
}

void RandomShootingMPC::setMPCParams(int horizon, int num_paths)
{
  m_horizon = horizon;
  m_num_paths = num_paths;
}

void RandomShootingMPC::setInputLims(std::vector<double> uL, std::vector<double> lL)
{
  m_uL.clear();
  m_lL.clear();
  m_uL.assign(uL.begin(),uL.end());
  m_lL.assign(lL.begin(),lL.end());
}

std::vector<double> RandomShootingMPC::getAction(vector<double> state)
{

}
