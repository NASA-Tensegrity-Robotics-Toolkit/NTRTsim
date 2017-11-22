#include "NeuralNet.h"

NeuralNet::NeuralNet() {}

NeuralNet::NeuralNet(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose)
{
  std::cout << "Initializing neural net with input dim: " << in_dim
    << ", output dim: " << out_dim << ", hidden dim: " << hid_dim
    << ", and " << n_layers << " layers" << std::endl;
  m_in_dim = in_dim;
  m_out_dim = out_dim;
  m_hid_dim = hid_dim;
  m_n_layers = n_layers;
  m_transpose = transpose;

  std::vector< matrix<double> > weights_tmp;

  for (int i = 0; i <= n_layers; i++) {
    matrix<double> weights;
    if (i == 0) {
      weights.resize(hid_dim, in_dim+1);
    }
    else if (i == n_layers) {
      weights.resize(out_dim, hid_dim+1);
    }
    else {
      weights.resize(hid_dim, hid_dim+1);
    }
    weights_tmp.push_back(weights);
  }

  m_weights.clear();
  m_weights.assign(weights_tmp.begin(), weights_tmp.end());
}

vector<double> NeuralNet::getNNOutput(vector<double> input)
{
  if (input.size() != m_in_dim) {
    throw std::runtime_error("Input dimension does not match neural net");
  }
  vector<double> out(input);

  for (int i = 0; i <= m_n_layers; i++) {
    // Create input vector and append bias term
    vector<double> vec(m_weights[i].size2());
    vec <<= out, 1;
    // Create output vector
    out.resize(m_weights[i].size1(),false);
    // Do matrix multiplication
    axpy_prod(m_weights[i], vec, out, true);
  }

  return out;
}

void NeuralNet::setLayerWeights(int layer, std::string filename)
{
  if (layer > m_n_layers) {
    throw std::runtime_error("Layer does not exist in neural net");
  }
  std::ifstream file;
  std::string line;
  std::string value;

  file.open(filename.c_str());
  std::cout << m_weights[layer].size1() << ',' << m_weights[layer].size2() << std::endl;
  if (m_transpose) {
    for (size_t j = 0; j < m_weights[layer].size2(); j++) {
      int comma_1_pos = 0;
      int comma_2_pos = 0;
      std::getline(file, line);
      for (size_t i = 0; i < m_weights[layer].size1(); i++) {
        comma_2_pos = line.find(',',comma_1_pos);
        if (comma_2_pos == std::string::npos) {
          value = line.substr(comma_1_pos);
        }
        else {
          value = line.substr(comma_1_pos,comma_2_pos-comma_1_pos);
        }
        // std::cout << value << std::endl;
        m_weights[layer](i,j) = std::atof(value.c_str());
        comma_1_pos = comma_2_pos+1;
      }
    }
  }
  else {
    for (size_t i = 0; i < m_weights[layer].size1(); i++) {
      int comma_1_pos = 0;
      int comma_2_pos = 0;
      std::getline(file, line);
      for (size_t j = 0; j < m_weights[layer].size2(); j++) {
        comma_2_pos = line.find(',',comma_1_pos);
        if (comma_2_pos == std::string::npos) {
          value = line.substr(comma_1_pos);
        }
        else {
          value = line.substr(comma_1_pos,comma_2_pos-comma_1_pos);
        }
        // std::cout << value << std::endl;
        m_weights[layer](i,j) = std::atof(value.c_str());
        comma_1_pos = comma_2_pos+1;
      }
    }
  }
  file.close();
}

void NeuralNet::setNNParams(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose) {
  std::cout << "Initializing neural net with input dim: " << in_dim
    << ", output dim: " << out_dim << ", hidden dim: " << hid_dim
    << ", and " << n_layers << " hidden layers" << std::endl;
  m_in_dim = in_dim;
  m_out_dim = out_dim;
  m_hid_dim = hid_dim;
  m_n_layers = n_layers;
  m_transpose = transpose;

  std::vector< matrix<double> > weights_tmp;

  // n_layers+1 iterations for n_layers of hidden layers and 1 output layer
  for (int i = 0; i <= n_layers; i++) {
    // Size the weight matricies and add column for biases, we use the convention out = W*in
    matrix<double> weights;
    if (i == 0) {
      weights.resize(hid_dim, in_dim+1);
    }
    else if (i == n_layers) {
      weights.resize(out_dim, hid_dim+1);
    }
    else {
      weights.resize(hid_dim, hid_dim+1);
    }
    weights_tmp.push_back(weights);
  }

  m_weights.clear();
  m_weights.assign(weights_tmp.begin(), weights_tmp.end());
}

std::vector<int> NeuralNet::getDims()
{
  std::vector<int> dims;

  dims.push_back(m_in_dim);
  dims.push_back(m_out_dim);
  dims.push_back(m_hid_dim);
  dims.push_back(m_n_layers);

  return dims;
}
