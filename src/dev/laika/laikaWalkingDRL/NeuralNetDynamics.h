#ifndef NEURAL_NET_DYNAMICS_H
#define NEURAL_NET_DYNAMICS_H

#include "NeuralNet.h"

using namespace boost::numeric::ublas;

class NeuralNetDynamics : public NeuralNet
{
public:
  NeuralNetDynamics();
  NeuralNetDynamics(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose);

  virtual ~NeuralNetDynamics() {}

  void setInputNormalization(std::string filename_mean, std::string filename_std);

  void setOuputNormalization(std::string filename_mean, std::string filename_std);


  vector<double> getNNDynOutput(vector<double> states, vector<double> actions);

private:
  vector<double> normalizeInput(vector<double> input);

  vector<double> unnormalizeOuput(vector<double> output);

  vector<double> m_input_normalization_mean;
  vector<double> m_input_normalization_std;
  vector<double> m_output_normalization_mean;
  vector<double> m_output_normalization_std;
};

#endif
