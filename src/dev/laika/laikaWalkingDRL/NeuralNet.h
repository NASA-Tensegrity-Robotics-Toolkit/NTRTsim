#ifndef NEURAL_NET_H
#define NEURAL_NET_H

#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>

#include <numeric/ublas/matrix.hpp>
#include <numeric/ublas/vector.hpp>

using namespace boost::numeric::ublas;

class NeuralNet
{
public:
  // Constructor
  NeuralNet();
  NeuralNet(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose);

  // Destructor
  virtual ~NeuralNet() { }

  // Do forward pass
  vector<double> getNNOutput(vector<double> input);

  void setLayerWeights(int layer, std::string filename);

  void setNNParams(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose);

private:
  int m_in_dim, m_out_dim, m_hid_dim, m_n_layers;
  bool m_transpose;
  std::vector< matrix<double> > m_weights;
  // std::vector< vector<double> > m_biases;
};

#endif
