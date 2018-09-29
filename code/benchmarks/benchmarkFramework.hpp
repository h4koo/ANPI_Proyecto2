/**
 * Copyright (C) 2017 
 * Área Académica de Ingeniería en Computadoras, TEC, Costa Rica
 *
 * This file is part of the CE3102 Numerical Analysis lecture at TEC
 *
 * @author Pablo Alvarado
 * @date   29.12.2017
 */

#ifndef ANPI_BENCHMARK_FRAMEWORK_HPP
#define ANPI_BENCHMARK_FRAMEWORK_HPP

#include <chrono>
#include <iostream>
#include <ostream>
#include <fstream>
#include <limits>

#include <Matrix.hpp>
#include <PlotPy.hpp>


namespace anpi {
  namespace benchmark {
    /**
     * Each measurement is composed by five attributes
     */
    struct measurement {
      inline measurement() : size(0u),average(0.),stddev(0.),min(0.),max(0.) {};
      
      size_t size;
      double average;
      double stddev;
      double min;
      double max;
    };

    template<typename T>
    inline T sqr(const T val) { return val*val; }
    
    /**
     * Compute measurement statistics for each size
     */
    void computeStats(const std::vector<size_t>& sizes,
                      const anpi::Matrix<std::chrono::duration<double> >& mat,
                      std::vector<measurement>& times);

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
    void write(std::ostream& stream,
               const std::vector<measurement>& m);

    /**
     * Save a file with each measurement in a row
     */
    void write(const std::string& filename,
               const std::vector<measurement>& m);

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
    void plot(const std::vector<measurement>& m,
              const std::string& legend,
              const std::string& color = "r");

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
    void plotRange(const std::vector<measurement>& m,
                   const std::string& legend,
                   const std::string& color);

    /**
     * Show all registered plots.
     */
    void show();
  } // namespace benchmark
} // namespace anpi
    
/**
 * Meassure the time for all given sizes.
 * @param sizes  vector with all sizes to be tested
 * @param rep    number of repetitions to meassure the time
 * @param times  measurement taken for each time
 *               its time must be anpi::Matrix<anpi::benchmark::measurement>
 * @param bench  benchmark instance.  See below for requirements
 *
 * The @bench is an instance of a class that must provide at least
 * the following:
 * - an void prepare(const size_t size) method, that initializes the
 *   state of the instance as required for the evaluation.  This
 *   method is called outside the performance measurements.
 * - an inline void eval() method that performs the evaluation.
 *
 * The following variables are available in PREFIX and CODE:
 * - size holds the current evaluated size (type size_t)
 * - s    holds the index of the current evaluated size
 * - i    index of the current repetition
 */
#define ANPI_BENCHMARK(sizes,rep,times,bench)                      \
{                                                                  \
  /* number of sizes to be tested */                               \
  const size_t _nums = sizes.size();                               \
                                                                   \
  typedef std::chrono::duration<double> durat;                     \
                                                                   \
  /* each row holds a particular size */                           \
  ::anpi::Matrix<durat> _mat(_nums,rep,::anpi::DoNotInitialize);   \
                                                                   \
  /* test each size */                                             \
  for (size_t s=0;s<_nums;++s ) {                                  \
    const size_t size = sizes[s];                                  \
                                                                   \
    std::cout << "Testing size " << size << std::endl;             \
                                                                   \
    durat* _row = _mat[s];                                         \
                                                                   \
    /* prefix code for initialization before measurement */        \
    bench.prepare(size);                                           \
                                                                   \
    const auto _start = std::chrono::high_resolution_clock::now(); \
                                                                   \
    for ( size_t i=0;i<rep;++i ) {                                 \
      /* the code to be benchmarked */                             \
      bench.eval();                                                \
                                                                   \
      _row[i] = std::chrono::high_resolution_clock::now()-_start;  \
    }                                                              \
  }                                                                \
                                                                   \
  ::anpi::benchmark::computeStats(sizes,_mat,times);               \
}


#endif
