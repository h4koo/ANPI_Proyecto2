/**
 * Copyright (C) 2018
 * Área Académica de Ingeniería en Computadoras, ITCR, Costa Rica
 *
 * This file is part of the numerical analysis lecture CE3102 at TEC
 *
 * @Author: 
 * @Date  : 03.03.2018
 */

#include <cmath>
#include <limits>
#include <functional>
#include <algorithm>
#include <iostream>

#include "Exception.hpp"
#include "Matrix.hpp"

#ifndef ANPI_LU_DOOLITTLE_HPP
#define ANPI_LU_DOOLITTLE_HPP
using namespace std;
namespace anpi
{

/**
   * Auxiliary method used to debug LU decomposition.
   *
   * It separates a packed LU matrix into the lower triangular matrix
   * L and the upper triangular matrix U, such that the diagonal of L
   * is composed by 1's.
   */
template <typename T>
void unpackDoolittle(const Matrix<T> &LU,
                     Matrix<T> &L,
                     Matrix<T> &U)
{

  L = LU;
  U = LU;
  int n = LU.rows();
  for (int fila = 0; fila < n; fila++)
  {
    for (int columna = 0; columna < n; columna++)
    {
      if (fila == columna)
      {
        L(fila, columna) = 1;
      }
      else if (fila >= columna)
      {
        L(fila, columna) = LU(fila, columna);
      }
      else
      {
        L(fila, columna) = 0;
      }
    }
  }
  for (int fila = 0; fila < n; fila++)
  {
    for (int columna = 0; columna < n; columna++)
    {
      if (fila <= columna)
      {

        U(fila, columna) = LU(fila, columna);
      }
      else
      {
        U(fila, columna) = 0;
      }
    }
  }
}

/**
   * Decompose the matrix A into a lower triangular matrix L and an
   * upper triangular matrix U.  The matrices L and U are packed into
   * a single matrix LU. 
   *
   * The L matrix will have in the Doolittle's LU decomposition a
   * diagonal of 1's
   *
   * @param[in] A a square matrix 
   * @param[out] LU matrix encoding the L and U matrices
   * @param[out] permut permutation vector, holding the indices of the
   *             original matrix falling into the corresponding element.
   *             For example if permut[5]==3 holds, then the fifth row
   *             of the LU decomposition in fact is dealing with the third
   *             row of the original matrix.
   *
   * @throws anpi::Exception if matrix cannot be decomposed, or input
   *         matrix is not square.
   */
template <typename T>
void luDoolittle(const Matrix<T> &A,
                 Matrix<T> &LU,
                 std::vector<size_t> &permut)
{

  if (A.rows() != A.cols())
  {
    throw anpi::Exception("Matrix for Doolittle LU decomposition must be square");
    return;
  }
  else
  {
    LU = A;
    int n = A.rows();
    permut.resize(n);
    int imax = 0;
    int temp;

    for (int pivot = 0; pivot < n; ++pivot)
    {
      permut[pivot] = pivot;
    }
    for (int k = 0; k < n; ++k)
    {
      for (int j = k; j < n; ++j)
      {
        //Nos encontramos arriba de la diagonal, se encuentran elementos de U
        double sum = 0.0;
        for (int p = 0; p < k; p++)
        {
          sum = sum + LU(k, p) * LU(p, j);
        }
        LU(k, j) = A(k, j) - sum;
      }
      //Calculo de la diagonal inferior, se hallan los elementos de L
      imax = k;
      for (int i = k + 1; i < n; i++)
      {
        double sum = 0.0;
        if (!(abs(A(imax, k)) >= abs(A(i, k))))
        {
          imax = i;
        }
        if (imax != k)
        {
          temp = permut[imax];
          permut[imax] = permut[k];
          permut[k] = temp;
        }
        for (int p = 0; p < k; p++)
        {
          sum = sum + LU(i, p) * LU(p, k);
        }
        LU(i, k) = (A(i, k) - sum) / LU(k, k);
      }
    }
  }
}
} // namespace anpi

#endif
