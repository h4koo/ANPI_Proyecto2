/**
 * Copyright (C) 2017
 * Área Académica de Ingeniería en Computadoras, ITCR, Costa Rica
 *
 * This file is part of the numerical analysis lecture CE3102 at TEC
 *
 * @author Pablo Alvarado
 * @date   15.12.2017
 */

#ifndef ANPI_SOLVER_HPP
#define ANPI_SOLVER_HPP
#include <iostream>
#include "LUDoolittle.hpp"

#include "LUCrout.hpp"
// #include "MatrixUtils.hpp"

using namespace std;
namespace anpi
{

/** faster method used for LU decomposition
   */
template <typename T>
inline void lu(const anpi::Matrix<T> &A,
               anpi::Matrix<T> &LU,
               std::vector<size_t> &p)
{
  anpi::luCrout(A, LU, p);
  // anpi::luDoolittle(A, LU, p);
}

/** method used to create  the permutation matrix given a
   * permutation vector
  **/
template <typename T>
void permutationMatrix(const std::vector<size_t> &p,
                       anpi::Matrix<T> &pMatrix)
{

  int n = p.size();
  anpi::Matrix<T> matrix(p.size(), p.size());

  for (int i = 0; i < n; i++)
  {
    matrix[i][p[i]] = T(1);
  }

  pMatrix = matrix;
}

/// method used to solve lower triangular matrices
template <typename T>
void forwardSubstitution(const anpi::Matrix<T> &L,
                         const std::vector<T> &b,
                         std::vector<T> &y)
{

  int n = L.rows();
  std::vector<T> x(n);
  for (int i = 0; i < n; i++)
  {
    x[i] = T(1);
  }

  T sum;

  for (int m = 0; m < n; m++)
  {
    sum = T(0);
    for (int i = 0; i < m; i++)
    {
      sum += L[m][i] * x[i];
    }
    x[m] = (b[m] - sum) / L[m][m];
  }

  y = x;
}

/// method used to solve upper triangular matrices
template <typename T> //Aqui esta el error
void backwardSubstitution(const anpi::Matrix<T> &U,
                          const std::vector<T> &y,
                          std::vector<T> &x)
{
  int n = U.cols();
  std::vector<T> w;
  w.resize(n);

  for (int i = 0; i < n; i++)
  {
    w[i] = T(1);
  }

  T sum;

  w[n - 1] = y[n - 1] / U[n - 1][n - 1];

  for (int i = (n - 2); i >= 0; i--)
  {
    sum = T(0);
    for (int j = (n - 1); j >= (i + 1); j--)
    {
      sum += U[i][j] * w[j];
    }
    w[i] = (y[i] - sum) / U[i][i];
  }

  x = w;
}

template <typename T>
void datosMatrix(anpi::Matrix<T> A)
{
  cout << "Columnas de la matriz" << A.cols() << endl;
  cout << "Filas de la Matriz" << A.rows() << endl;
  cout << "Matriz completa" << endl;
  int n = A.cols();
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      std::cout << A[i][j] << "  ";
    }
    std::cout << std::endl;
  }
  cout << "----------------------------------------" << endl;
}

template <typename T>
bool solveLU(const anpi::Matrix<T> &A,
             std::vector<T> &x,
             const std::vector<T> &b)
{
  cout << "Datos de la matriz A" << endl;
  datosMatrix(A);
  cout << "                    " << endl;
  anpi::Matrix<T> LU;
  std::vector<size_t> p;
  anpi::lu(A, LU, p);

  anpi::Matrix<T> L;
  anpi::Matrix<T> U;
  anpi::unpackDoolittle(LU, L, U);

  anpi::Matrix<T> P;
  anpi::permutationMatrix(p, P);
  // // anpi::Matrix<T>PB = P * b; //ERRROR
  /*cout << "Datos de la matriz A" << endl;
  datosMatrix(A);
  cout << "                    " << endl;
  cout << "Datos de la matriz L" << endl;
  datosMatrix(L);
  cout << "                    " << endl;
  cout << "Datos de la matriz U" << endl;
  datosMatrix(U);
  cout << "                    " << endl;
  cout << "Datos de la matriz P" << endl;
  datosMatrix(P);
  cout << "                    " << endl;
  */
  std::vector<T> Pb(P.rows());

  // for (int i = 0; i < PB.rows(); i++)
  // {
  //   Pb[i] = PB[i][0];
  // }
  // x=b;
  //we permute the b vector to match the permuted A matrix obtained from LU decomposition
  int n = P.rows();
  for (int i = 0; i < n; ++i)
  {
    int ip = p[i]; //index in the permutation vector
    Pb[i] = b[ip]; //move the current value to the permuted position
  }
  std::vector<T> y;
  anpi::forwardSubstitution(L, Pb, y);
  anpi::backwardSubstitution(U, y, x);

  //permute the result to have it ordered
  T temp;
  int pidx;
  for (int i = 0; i < n; ++i)
  {
    pidx = p[i];
    //if the index was permuted
    if (i != pidx)
    {
      //permute x value
      temp = x[i];
      x[i] = x[pidx];
      x[pidx] = temp;
      //permute index in permutation
      p[pidx] = pidx;
      p[i] = i;
    }
  }

  return 1;
}

} // namespace anpi

#endif
