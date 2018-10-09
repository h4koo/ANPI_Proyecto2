/**
 * Copyright (C) 2017 
 * Área Académica de Ingeniería en Computadoras, TEC, Costa Rica
 *
 * This file is part of the CE3102 Numerical Analysis lecture at TEC
 *
 * @author Pablo Alvarado
 * @date   29.12.2017
 */

#include "benchmarkFramework.hpp"

namespace anpi
{
namespace benchmark
{

/**
     * Compute measurement statistics for each size
     */
void computeStats(const std::vector<size_t> &sizes,
                  const anpi::Matrix<std::chrono::duration<double>> &mat,
                  std::vector<measurement> &times)
{

  const size_t nums = sizes.size();
  times.resize(nums);
  for (size_t s = 0; s < nums; ++s)
  {
    measurement &m = times[s];
    m.size = sizes[s];

    double val = mat[s][0].count();
    m.average = val;
    m.stddev = sqr(val);

    m.min = val;
    m.max = val;

    for (size_t i = 1; i < mat.cols(); ++i)
    {
      val = (mat[s][i] - mat[s][i - 1]).count();
      m.average += val;
      m.stddev += sqr(val);
      m.min = std::min(m.min, val);
      m.max = std::max(m.max, val);
    }

    m.average /= mat.cols();
    m.stddev = std::sqrt(m.stddev / mat.cols() - sqr(m.average));
  }
}

/**
     * Save a file with each measurement in a row.
     *
     * The meaning of the columns is as follows:
     * # Size
     * # Average
     * # Standard deviation
     * # Minimum
     * # Maximum  
     */
void write(std::ostream &stream,
           const std::vector<measurement> &m)
{
  for (auto i : m)
  {
    stream << i.size << " \t";
    stream << i.average << " \t";
    stream << i.stddev << " \t";
    stream << i.min << " \t";
    stream << i.max << " \t" << std::endl;
  }
}

/**
     * Save a file with each measurement in a row
     */
void write(const std::string &filename,
           const std::vector<measurement> &m)
{
  std::ofstream os(filename.c_str());
  write(os, m);
  os.close();
}

/**
     * Plot measurements (average only)
     *
     * The meaning of the columns is as follows:
     * # Size
     * # Average
     * # Standard deviation
     * # Minimum
     * # Maximum  
     */
void plot(const std::vector<measurement> &m,
          const std::string &legend,
          const std::string &color)
{
  std::vector<double> x(m.size()), y(m.size());

  for (size_t i = 0; i < m.size(); ++i)
  {
    x[i] = m[i].size;
    y[i] = m[i].average;
  }
}

/**
     * Plot measurements (average only)
     *
     * The meaning of the columns is as follows:
     * # Size
     * # Average
     * # Standard deviation
     * # Minimum
     * # Maximum  
     */
void plotRange(const std::vector<measurement> &m,
               const std::string &legend,
               const std::string &color)
{
  std::vector<double> x(m.size()), y(m.size()), miny(m.size()), maxy(m.size());

  for (size_t i = 0; i < m.size(); ++i)
  {
    const measurement &mi = m[i];
    x[i] = mi.size;
    y[i] = mi.average;
    miny[i] = mi.min;
    maxy[i] = mi.max;
  }

  static anpi::Plot2d<double> plotter;
  plotter.initialize(1);
  plotter.plot(x, y, miny, maxy, legend, color);
}

void plotPath(const std::vector<float> &x,
              const std::vector<float> &y,
              const std::string &legend,
              const std::string &color)
{
  static anpi::Plot2d<float> plotter;
  plotter.initialize(1);
  plotter.plot(x, y, legend, color);
  plotter.show();
}

void show()
{
  static anpi::Plot2d<double> plotter;
  plotter.show();
}
} // namespace benchmark
} // namespace anpi
