#include "NeuralNetDynamics.h"

NeuralNetDynamics::NeuralNetDynamics() {}

NeuralNetDynamics::NeuralNetDynamics(int in_dim, int out_dim, int hid_dim, int n_layers, bool transpose)
{
  setNNParams(in_dim, out_dim, hid_dim, n_layers, transpose);
}

void NeuralNetDynamics::setInputNormalization(std::string filename_mean, std::string filename_std)
{
  int in_dim = getDims()[0];
  m_input_normalization_mean.clear();
  m_input_normalization_mean.resize(in_dim);
  m_input_normalization_std.clear();
  m_input_normalization_std.resize(in_dim);

  std::ifstream file;
  std::string value;

  file.open(filename_mean.c_str());
  for (size_t i = 0; i < in_dim; i++) {
    std::getline(file, value);
    m_input_normalization_mean(i) = std::atof(value.c_str());
  }
  file.close();

  file.open(filename_std.c_str());
  for (size_t i = 0; i < in_dim; i++) {
    std::getline(file, value);
    m_input_normalization_std(i) = std::atof(value.c_str());
  }
  file.close();

  std::cout << "Input normalization size: " << m_input_normalization_mean.size() << std::endl;
  // std::cout << input_normalization_mean(in_dim-1) << ',' << input_normalization_std(in_dim-1) << std::endl;
}

void NeuralNetDynamics::setOuputNormalization(std::string filename_mean, std::string filename_std)
{
  int out_dim = getDims()[1];
  m_output_normalization_mean.clear();
  m_output_normalization_mean.resize(out_dim);
  m_output_normalization_std.clear();
  m_output_normalization_std.resize(out_dim);

  std::ifstream file;
  std::string value;

  file.open(filename_mean.c_str());
  for (size_t i = 0; i < out_dim; i++) {
    std::getline(file, value);
    m_output_normalization_mean(i) = std::atof(value.c_str());
  }
  file.close();

  file.open(filename_std.c_str());
  for (size_t i = 0; i < out_dim; i++) {
    std::getline(file, value);
    m_output_normalization_std(i) = std::atof(value.c_str());
  }
  file.close();
  std::cout << "Output normalization size: " << m_output_normalization_mean.size() << std::endl;
  // std::cout << output_normalization_mean(out_dim-1) << ',' << output_normalization_std(out_dim-1) << std::endl;
}

vector<double> NeuralNetDynamics::getNNDynOutput(vector<double> states, vector<double> actions)
{
  // Check if inputs match expected input dimension
  int in_dim = states.size() + actions.size();
  // std::cout << states.size() << "," << actions.size() << std::endl;
  if (in_dim != getDims()[0]) {
    throw std::runtime_error("Input dimension does not match expected from neural net");
  }

  vector<double> in_vec(in_dim);
  vector<double> out_vec(getDims()[1]);

  in_vec <<= states, actions;

  // Choose whether or not to normalize inputs and outputs based on size of normalization vectors
  if (m_input_normalization_mean.empty() && !m_output_normalization_mean.empty()) {
    out_vec = unnormalizeOuput(getNNOutput(in_vec));
  }
  else if (!m_input_normalization_mean.empty() && m_output_normalization_mean.empty()) {
    out_vec = getNNOutput(normalizeInput(in_vec));
  }
  else if (m_input_normalization_mean.empty() && m_output_normalization_mean.empty()) {
    out_vec = getNNOutput(in_vec);
  }
  else {
    out_vec = unnormalizeOuput(getNNOutput(normalizeInput(in_vec)));
  }

  return states + out_vec;
}

vector<double> NeuralNetDynamics::normalizeInput(vector<double> input)
{
  return element_div(input-m_input_normalization_mean, m_input_normalization_std);
}

vector<double> NeuralNetDynamics::unnormalizeOuput(vector<double> output)
{
  return m_output_normalization_mean + element_prod(output, m_output_normalization_std);
}
