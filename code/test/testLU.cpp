/**
 * Copyright (C) 2017 
 * Área Académica de Ingeniería en Computadoras, TEC, Costa Rica
 *
 * This file is part of the CE3102 Numerical Analysis lecture at TEC
 *
 * @Author: Pablo Alvarado
 * @Date  : 10.02.2018
 */

#include <boost/test/unit_test.hpp>

#include "LUCrout.hpp"
#include "LUDoolittle.hpp"
#include "Solver.hpp"
//#include "LU.hpp"
#include "MatrixUtils.hpp"

#include <iostream>
#include <exception>
#include <cstdlib>
#include <complex>

#include <functional>

#include <cmath>

namespace anpi
{
namespace test
{

/// Test the given closed root finder
template <typename T>
void luTest(const std::function<void(const Matrix<T> &,
                                     Matrix<T> &,
                                     std::vector<size_t> &)> &decomp,
            const std::function<void(const Matrix<T> &,
                                     Matrix<T> &,
                                     Matrix<T> &)> &unpack)
{

  // The result
  Matrix<T> LU;

  // Test if a non-square matrix is successfully detected
  {
    Matrix<T> A = {{1, 7, 6, 4}, {2, 17, 27, 17}};
    std::vector<size_t> p;
    try
    {
      decomp(A, LU, p);
      BOOST_CHECK_MESSAGE(false, "Rectangular matrix not properly catched");
    }
    catch (anpi::Exception &exc)
    {
      BOOST_CHECK_MESSAGE(true, "Rectangular matrix properly detected");
    }
  }

  // Test pivoting
  {
    anpi::Matrix<T> A = {{-1, -2, 1, 2}, {2, 0, 1, 2}, {-1, -1, 0, 1}, {1, 1, 1, 1}};
    std::vector<size_t> p;
    decomp(A, LU, p);

    std::vector<size_t> gp = {1, 0, 3, 2};
    BOOST_CHECK(gp == p);
  }

  // Test decomposition
  {
    // same matrix as before, but already permuted to force a
    // clean decomposition
    anpi::Matrix<T> A = {{2, 0, 1, 2}, {-1, -2, 1, 2}, {1, 1, 1, 1}, {-1, -1, 0, 1}};
    std::vector<size_t> p;
    decomp(A, LU, p);
    Matrix<T> L, U;
    unpack(LU, L, U);
    Matrix<T> Ar = L * U;

    std::cout << "the matrix LU is:\n";
    anpi::printMatrix(LU);
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "the matrix L is:\n";
    anpi::printMatrix(L);
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "the matrix U is:\n";
    anpi::printMatrix(U);
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "the matrix Ar= L * U is:\n";
    anpi::printMatrix(Ar);
    std::cout << std::endl;
    std::cout << std::endl;

    const T eps = std::numeric_limits<T>::epsilon();

    BOOST_CHECK(Ar.rows() == A.rows());
    BOOST_CHECK(Ar.cols() == A.cols());

    for (size_t i = 0; i < Ar.rows(); ++i)
    {
      for (size_t j = 0; j < Ar.cols(); ++j)
      {
        BOOST_CHECK(std::abs(Ar(i, j) - A(i, j)) < eps);
      }
    }
  }
  //other decomposition test
  {
    std::cout << "-------------------------------------------------------------:\n";
    std::vector<size_t> p;
    anpi::Matrix<T> LU, L, U, A = {{0, 2, 0, 1}, {2, 2, 3, 2}, {4, -3, 0, 1.}, {6, 1, -6, -5}}; //{{10, 24, -4, 15}, {30, 24, -34, 28}, {32, -23, 10, 18}, {6, 1, -6, -5}}; //
    anpi::Matrix<T> AA = {{}};
    anpi::luCrout(A, LU, p);
    unpack(LU, L, U);
    Matrix<T> Ar = L * U;

    std::cout << "BEFORE PERMUTE: the matrix Ar= L * U is:\n";
    anpi::printMatrix(Ar);
    std::cout << std::endl;
    std::cout << std::endl;

    //permutation
    anpi::permuteMatrix(Ar, p);

    std::cout
        << "the matrix A is:\n";
    anpi::printMatrix(A);
    std::cout << std::endl;
    std::cout << std::endl;

    std::cout << "El vector de permutacion es: \n";
    for (size_t i = 0; i < p.size(); ++i)
      std::cout << p[i] << "  ";
    std::cout << std::endl;

    std::cout << "AFTER PERMUTE: the matrix Ar= L * U is:\n";
    anpi::printMatrix(Ar);
    std::cout << std::endl;
    std::cout << std::endl;

    std::cout << " the matrix LU is:\n";
    anpi::printMatrix(LU);
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "the matrix L is:\n";
    anpi::printMatrix(L);
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "the matrix U is:\n";
    anpi::printMatrix(U);
    std::cout << std::endl;
    std::cout << std::endl;
  }

} //end luTest

template <typename T>
void invertTest()
{
  anpi::Matrix<T> Ai, exAi, A = {{4, -2}, {10, -3}}; //{{1, 2, -3}, {4, -5, 6}, {7, -8, 9}};
  // exAi = {{0.50000, 1.00000, -0.50000},
  //         {1.00000, 5.00000, -3.00000},
  //         {0.50000, 3.66667, -2.16667}};

  //simple test
  exAi = {{-0.375, 0.25}, {-1.25, 0.5}};

  //anpi::invert(A, Ai);

  std::cout << "the matrix A is:\n";
  anpi::printMatrix(A);
  std::cout << std::endl;
  std::cout << "the matrix Ai is:\n";
  anpi::printMatrix(Ai);
  std::cout << std::endl;

  BOOST_CHECK(Ai == exAi);
}

template <typename T>
void prueba()
{
  std::cout << (9 / 8) << endl;
}

template <typename T>
void solverTest()
{
  anpi::Matrix<T> LU, A = {{0, 2, 0, 1}, {2, 2, 3, 2}, {4, -3, 0, 1.}, {6, 1, -6, -5}}, r = {{1, -5}, {2, -2}}, Z = {{10, 2, 0, 1}, {2, 5, 3, 3}, {4, -3, 0, 1.}, {6, 1, -4, -5}}; //r = {{1, -5}, {2, -2}};
  std::vector<T> br, results, b = {0, -2, -7, 6}, y = {-2, 9}, exResults = {-0.5, 1.00000, 0.33333, -2.00000};                                                                     //{-3.5, -2.5}; //{-0.5, 1.00000, 0.33333, -2.00000},
  std::vector<size_t> p;

  // check LU
  // anpi::luCrout<T>(r, LU, p);
  anpi::luCrout<T>(Z, LU, p);

  std::cout << "the matrix LU is:\n";
  anpi::printMatrix(LU);
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "El vector de permutacion es: \n";
  for (size_t i = 0; i < p.size(); ++i)
    std::cout << p[i] << "  ";
  std::cout << std::endl;

  // anpi::solveLU(A, results, b);
  // br = A * results;

  // anpi::solveLU(r, results, y);
  // br = r * results;

  anpi::solveLU(Z, results, b);
  br = Z * results;

  std::cout << "El vector resultado X es: \n";
  for (size_t i = 0; i < results.size(); ++i)
    std::cout << results[i] << "  ";
  std::cout << std::endl;

  std::cout << "El vector b es: \n";
  for (size_t i = 0; i < results.size(); ++i)
    std::cout << b[i] << "  ";
  std::cout << std::endl;
  std::cout << "El vector A * x(calculado) = b es: \n";
  for (size_t i = 0; i < results.size(); ++i)
    std::cout << br[i] << "  ";
  std::cout << std::endl;

  BOOST_CHECK(results == exResults);
}

} // namespace test
} // namespace anpi

BOOST_AUTO_TEST_SUITE(LU)

BOOST_AUTO_TEST_CASE(Doolittle)
{
  anpi::test::luTest<float>(anpi::luDoolittle<float>,
                            anpi::unpackDoolittle<float>);
  anpi::test::luTest<double>(anpi::luDoolittle<double>,
                             anpi::unpackDoolittle<double>);
}

BOOST_AUTO_TEST_CASE(Crout)
{
  anpi::test::luTest<float>(anpi::luCrout<float>, anpi::unpackCrout<float>);
  anpi::test::luTest<double>(anpi::luCrout<double>, anpi::unpackCrout<double>);
}

BOOST_AUTO_TEST_CASE(Inversion)
{
  anpi::test::invertTest<float>();
  anpi::test::invertTest<double>();
}

BOOST_AUTO_TEST_CASE(SolveLU)
{
  anpi::test::solverTest<float>();
  anpi::test::solverTest<double>();
}

BOOST_AUTO_TEST_CASE(pruebasMate)
{
  anpi::test::prueba<float>();
}
BOOST_AUTO_TEST_SUITE_END()
