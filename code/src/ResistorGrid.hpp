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
∗ compute the indices of the resistor given the numerical representation of it
*/
    indexPair indexToNodes(const std::size_t idx);
};
} // namespace anpi
