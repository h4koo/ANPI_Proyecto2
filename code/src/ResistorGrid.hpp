#include <cstdlib>
#include <iostream>

#include <AnpiConfig.hpp>

#include "MatrixUtils.hpp"
#include <string>

#include <opencv2/core.hpp>    // For cv::Mat
#include <opencv2/highgui.hpp> // For cv::imread/imshow

#include <Matrix.hpp>
#include <Exception.hpp>

namespace anpi
{
/// Pack a  pair  of  indices  of  the  nodes  of  a  resistor
struct indexPair
{
    /// Row of  the  first node
    std::size_t row1;
    /// Column  of  the  first  node
    std::size_t col1;
    /// Row of  the  second  node
    std::size_t row2;
    /// Column  of  the  second  node
    std::size_t col2;

    inline bool operator==(indexPair &other)
    {
        return row1 == other.row1 && row2 == other.row2 && col1 == other.col1 && col2 == other.col2;
    }

    void print()
    {
        std::cout << "row1: " << row1 << std::endl;
        std::cout << "col1: " << col1 << std::endl;
        std::cout << "row2: " << row2 << std::endl;
        std::cout << "col2: " << col2 << std::endl;
    }
};
class ResistorGrid
{
  private:
    ///  Matrix  of  the  current  equation  system
    Matrix<float> A;
    ///  Vector  of  the  current  equation  system
    std::vector<float> b;
    /// Raw map data
    Matrix<float> rawMap;

  public:
    ///  . . .  constructors  and  other  methods

    //getters and setters
    inline void setA(Matrix<float> a)
    {
        A = a;
    }

    inline Matrix<float> getA()
    {
        return A;
    }

    /**
* Construct the grid from the given file
*
 *           @ return true if successful or false otherwise
*/
    bool build(const std::string filename);
    /**
∗ Compute the internal data to navigate between the given nodes
*/
    bool navigate(const indexPair &nodes);

    /**
∗ compute a number representig the resistor  located in the provided indices
*/
    std::size_t nodesToIndex(const std::size_t row1, const std::size_t col1, const std::size_t row2, const std::size_t col2);

    /**
∗ compute a number representig the resistor  located in the provided indices
*/
    inline std::size_t nodesToIndex(const indexPair idx)
    {
        return nodesToIndex(idx.row1, idx.col1, idx.row2, idx.col2);
    }

    /**
∗ compute the indices of the resistor given the numerical representation of it
*/
    indexPair indexToNodes(const std::size_t idx);
};
} // namespace anpi
