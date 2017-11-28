#ifndef NEURAL_NET_POLICY_H
#define NEURAL_NET_POLICY_H

#include "NeuralNet.h"

using namespace boost::numeric::ublas;

class NeuralNetPolicy : public NeuralNet
{
public:
  NeuralNetPolicy();
  NeuralNetPolicy(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose);

  virtual ~NeuralNetPolicy() {}

  void setInputNormalization(std::string filename_mean, std::string filename_std);

  void setOuputNormalization(std::string filename_mean, std::string filename_std);


  vector<double> getNNPolicyOutput(vector<double> states);

private:
  vector<double> normalizeInput(vector<double> input);

  vector<double> normalizeInputAlt(vector<double> input);

  vector<double> unnormalizeOuput(vector<double> output);

  vector<double> m_input_normalization_mean;
  vector<double> m_input_normalization_std;
  vector<double> m_output_normalization_mean;
  vector<double> m_output_normalization_std;
};

#endif
