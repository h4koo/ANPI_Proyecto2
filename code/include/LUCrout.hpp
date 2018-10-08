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

#include "Exception.hpp"
#include "Matrix.hpp"

#ifndef ANPI_LU_CROUT_HPP
#define ANPI_LU_CROUT_HPP

namespace anpi
{

/**
   * Auxiliary method used to debug LU decomposition.
   *
   * It separates a packed LU matrix into the lower triangular matrix
   * L and the upper triangular matrix U, such that the diagonal of U
   * is composed by 1's.
   */
template <typename T>
void unpackCrout(const Matrix<T> &LU,
                 Matrix<T> &L,
                 Matrix<T> &U)
{
  if (LU.rows() != LU.cols())
  {
    throw anpi::Exception("Matrix for Crout unpacking must be square");
  }

  L = LU;
  U = LU;
  int size = LU.cols();
  //we make all upper values of L equal 0
  for (int i = 0; i < size; ++i)
  {
    L[i][i] = 1;
    for (int j = i + 1; j < size; ++j)
    {
      L[i][j] = 0;
    }
  }
  //we make all lower values of U equal 0, and the diagonal equal 1
  for (int i = 0; i < size; ++i)
  {

    for (int j = 0; j < i; ++j)
    {
      U[i][j] = 0;
    }
    // U[i][i] = 1;
  }

  // throw anpi::Exception("To be implemented yet");
}

/**
   * Decompose the matrix A into a lower triangular matrix L and an
   * upper triangular matrix U.  The matrices L and U are packed into
   * a single matrix LU.  
   *
   * Crout's way of packing assumes a diagonal of
   * 1's in the U matrix.
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
void luCrout(const Matrix<T> &A,
             Matrix<T> &LU,
             std::vector<size_t> &permut)
{

  if (A.rows() != A.cols())
  {
    throw anpi::Exception("Matrix for Crout LU decomposition must be square");
  }

  // LUdcmp::LUdcmp(MatDoub_I &a) : n(a.nrows()), lu(a), aref(a), indx(n) {
  // Given a matrix a[0..n-1][0..n-1], this routine replaces it by the LU decomposition of a
  // rowwise permutation of itself. a is input. On output, it is arranged as in equation (2.3.14)
  // above; indx[0..n-1] is an output vector that records the row permutation effected by the
  // partial pivoting; d is output as  ̇1 depending on whether the number of row interchanges
  // was even or odd, respectively. This routine is used in combination with solve to solve linear
  // equations or invert a matrix.

  //intialize the result matrix, permutation vector
  LU = A;
  int n = A.rows();
  permut.resize(n);
  int i, imax, j, k;
  T big, temp;
  std::vector<T> vv(n); //vv stores the implicit scaling of each row.

  //initialize the permutation vector
  for (int i = 0; i < n; ++i)
    permut[i] = i;

  //Loop over rows to get the implicit scaling info
  for (i = 0; i < n; i++)
  {
    big = 0.0;
    for (j = 0; j < n; j++)
      if ((temp = abs(LU[i][j])) > big)
        big = temp;
    if (big == T(0.0))
      throw anpi::Exception("A is a singular matrix, unable to decompose into LU");

    //No nonzero largest element.
    vv[i] = T(1.0) / big; //Save the scaling.
  }

  //This is the outermost  loop. K
  for (k = 0; k < n; k++)
  {
    big = 0.0;
    imax = k;
    //Search for largest pivot element.
    for (i = k; i < n; i++)
    {
      temp = vv[i] * abs(LU[i][k]);
      if (temp > big) //Is the figure of merit for the pivot better than the best so far?
      {
        big = temp;
        imax = i;
      }
    }

    //index of the largest element is different from the current index
    if (k != imax)
    { //we do pivot

      //interchange the rows in the Matrix
      for (j = 0; j < n; j++)
      {
        temp = LU[imax][j];
        LU[imax][j] = LU[k][j];
        LU[k][j] = temp;
      }
      //Interchange the scale factor.
      temp = vv[imax];
      vv[imax] = vv[k];
      vv[k] = temp;

      //Interchange the values in the permutation vector
      temp = permut[k];
      permut[k] = imax;
      permut[imax] = temp;
    } //end pivot

    //calculate the values for the LU matrix
    for (i = k + 1; i < n; i++)
    {
      if (LU[k][k] == 0)
        throw anpi::Exception("Singular Matrix, pivot element is zero");

      temp = LU[i][k] /= LU[k][k]; // Divide by the pivot element.
      for (j = k + 1; j < n; j++)
        LU[i][j] -= temp * LU[k][j];
    }

  } //end of k loop
} //end LUCrout

} // namespace anpi
// namespace anpi

#endif
