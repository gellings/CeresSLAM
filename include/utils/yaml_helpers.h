#pragma once
#include <stdexcept>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <random>
#include <experimental/filesystem>
#include <boost/algorithm/string.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

inline bool file_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

inline std::string current_working_dir( void ) {
  char buff[FILENAME_MAX];
  char* ptr = getcwd( buff, FILENAME_MAX );
  (void)ptr;
  std::string current_working_dir(buff);
  return current_working_dir;
}

template <typename T>
bool get_yaml_node(const std::string key, const std::string filename, T& val, bool print_error = true) 
{
  // Try to load the YAML file
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(filename);
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file " << filename << std::endl;
  }

  // Throw error if unable to load a parameter
  if (node[key])
  {
    val = node[key].as<T>();
    return true;
  }
  else
  {
    if (print_error)
    {
      throw std::runtime_error("Unable to load " + key + " from " + filename);
    }
    return false;
  }
}
template <typename Derived1>
bool get_yaml_eigen(const std::string key, const std::string filename, Eigen::MatrixBase<Derived1>& val, bool print_error=true)
{
  YAML::Node node = YAML::LoadFile(filename);
  std::vector<double> vec;
  if (node[key])
  {
    vec = node[key].as<std::vector<double>>();
    if (vec.size() == (val.rows() * val.cols()))
    {
      int k = 0;
      for (int i = 0; i < val.rows(); i++)
      {
        for (int j = 0; j < val.cols(); j++)
        {
          val(i,j) = vec[k++];
        }
      }
      return true;
    }
    else
    {
      throw std::runtime_error("Eigen Matrix Size does not match parameter size for " + key + " in " + filename +
                               ". Requested " + std::to_string(Derived1::RowsAtCompileTime) + "x" + std::to_string(Derived1::ColsAtCompileTime) +
                               ", Found " + std::to_string(vec.size()));
      return false;
    }
  }
  else if (print_error)
  {
    throw std::runtime_error("Unable to load " + key + " from " + filename);
  }
  return false;
}

template <typename Derived>
bool get_yaml_diag(const std::string key, const std::string filename, Eigen::MatrixBase<Derived>& val, bool print_error=true)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> diag;
  if (get_yaml_eigen(key, filename, diag, print_error))
  {
    val = diag.asDiagonal();
    return true;
  }
  return false;
}

template <typename T>
bool get_yaml_priority(const std::string key, const std::string file1, const std::string file2, T& val)
{
  if (get_yaml_node(key, file1, val, false))
  {
    return true;
  }
  else
  {
    return get_yaml_node(key, file2, val, true);
  }
}

template <typename Derived1>
bool get_yaml_priority_eigen(const std::string key, const std::string file1, const std::string file2, Eigen::MatrixBase<Derived1>& val)
{
  if (get_yaml_eigen(key, file1, val, false))
  {
    return true;
  }
  else
  {
    return get_yaml_eigen(key, file2, val, true);
  }
}

inline bool createDirIfNotExist(const std::string& dir)
{
  if(!std::experimental::filesystem::exists(dir))
    return std::experimental::filesystem::create_directory(dir);
  else
    return false;
}

inline std::vector<std::string> split(const std::string& s, const char* delimeter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimeter[0]))
   {
      tokens.push_back(token);
   }
   return tokens;
}

inline std::string baseName(const std::string& path)
{
  std::string filename = split(path, "/").back();
  return split(filename, ".")[0];
}

template <typename T>
T sat(const T val, const T max, const T min)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

template <typename T>
T sat(const T max, const T val)
{
  if (val > max)
    return max;
  if (val < -1.0 * max)
    return -1.0 * max;
  return val;
}

template<typename Derived>
Derived randomNormal(double stdev, std::normal_distribution<typename Derived::Scalar>& dist, std::default_random_engine& gen)
{
  Derived vec;
  for (int i = 0; i < Derived::RowsAtCompileTime; ++i)
    vec(i) = stdev*dist(gen);
  return vec;
}

inline bool isNan(const Eigen::Ref<const Eigen::MatrixXd>& A)
{
  return (A.array() != A.array()).any();
}


