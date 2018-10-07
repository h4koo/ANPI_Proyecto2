#include <iostream>
#include "Matrix.hpp"

namespace anpi
{

//---------------------------------------helping functions Tarea 4-----------------------------------//
template <class T, typename Alloc = typename anpi::aligned_row_allocator<T>>
inline void printMatrix(anpi::Matrix<T, Alloc> &A)
{
    int r = A.rows(), c = A.cols();
    for (int i = 0; i < r; ++i)
    {
        for (int j = 0; j < c; ++j)
        {
            std::cout << A[i][j] << "  ";
        }
        std::cout << std::endl;
    }
}

template <class T>
inline void printMatrix(anpi::Matrix<T> &A)
{
    int n = A.cols(), height = A.rows();
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            std::cout << A[i][j] << "  ";
        }
        std::cout << std::endl;
    }
}

template <class T>
void transposeMatrix(anpi::Matrix<T> &A)
{
    T swap;
    int n = A.cols();
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            swap = A[i][j];
            A[i][j] = A[j][i];
            A[j][i] = swap;
        }
    }
}

template <class T>
void permuteMatrix(anpi::Matrix<T> &A, std::vector<size_t> permutations)
{

    int i, ip, n = permutations.size();

    anpi::Matrix<T> permuted(n, n);
    // T temp;
    //interchange the rows in the Matrix
    for (i = 0; i < n; ++i)
    {
        ip = permutations[i];

        for (int j = 0; j < n; j++)

            permuted[ip][j] = A[i][j];
    }
    A = permuted;
}

} // namespace anpi