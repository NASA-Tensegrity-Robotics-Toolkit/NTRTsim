#include "RandomShootingMPC.h"

RandomShootingMPC::RandomShootingMPC() {}

RandomShootingMPC::RandomShootingMPC(NeuralNetDynamics* dyn_model, int horizon, int num_paths)
{
  m_dyn_model = dyn_model;
  m_horizon = horizon;
  m_num_paths = num_paths;
}

void RandomShootingMPC::attachDynamicsModel(NeuralNetDynamics* dyn_model)
{
  m_dyn_model = dyn_model;
}

void RandomShootingMPC::setMPCParams(int horizon, int num_paths, int num_cables, int num_legs)
{
  m_horizon = horizon;
  m_num_paths = num_paths;
  m_num_cables = num_cables;
  m_num_legs = num_legs;
}

void RandomShootingMPC::setInputLims(vector<double> uL, vector<double> lL)
{
  int action_dim = m_num_cables + m_num_legs;
  if ((uL.size() != action_dim) || (lL.size() != action_dim)) {
    throw std::runtime_error("Dimension of input limits does not match NN input dimension");
  }
  m_uL = uL;
  m_lL = lL;
}

vector<double> RandomShootingMPC::getAction(vector<double> state)
{
  vector<double> s(state);
  vector<double> sp1(state);
  vector<double> a(m_num_cables+m_num_legs);
  vector<double> first_action(a);
  vector<double> best_action(a);
  int best_path;

  double cost;
  double min_cost = 1000000;

  // Search over n paths
  for (int i = 0; i < m_num_paths; i++) {
    std::cout << "Simulating path: " << i+1 << "/" << m_num_paths << std::endl;
    // Initialize new state using input
    s.clear();
    sp1.clear();
    s = state;
    cost = 0;
    // Propagate dynamics over horizon and calculate cost
    for (int j = 0; j < m_horizon; j++) {
      a.clear();
      a = sampleAction();
      if (j == 0) {
        first_action.clear();
        first_action = a;
      }
      sp1.clear();
      sp1 = m_dyn_model->getNNDynOutput(s, a);
      cost += costFunction(s, a, sp1);
      s.clear();
      s = sp1;
    }
    if (cost < min_cost) {
      min_cost = cost;
      best_action.clear();
      best_action = first_action;
      best_path = i+1;
    }
  }
  std::cout << "Best path: " << best_path << ", best cost: " << min_cost << std::endl;
  return best_action;
}

double RandomShootingMPC::costFunction(vector<double> state, vector<double> action, vector<double> next_state)
{
  double cost = 0;
  // Cost is the average x position difference between two timesteps for the 5 vertebrae
  for (int i = 0; i < 5; i++) {
    cost -= (next_state(int(i*12))-state(int(i*12)))/5.0;
  }
  return cost;
}

vector<double> RandomShootingMPC::sampleAction()
{
  vector<double> action(m_num_cables+m_num_legs);
  for (int i = 0; i < m_num_cables+m_num_legs; i++) {
    if (i < m_num_cables) {
      action.insert_element(i, rand()%(int(m_uL(i)-m_lL(i))+1)+int(m_lL(i)));
    }
    else {
      action.insert_element(i, (m_uL(i)-m_lL(i))*rand()/RAND_MAX+m_lL(i));
    }
  }
  return action;
}
