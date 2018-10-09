#include <cstdlib>
#include <iostream>

#include <AnpiConfig.hpp>

#include "Solver.hpp"

#include "MatrixUtils.hpp"
#include <string>

#include <opencv2/core.hpp>    // For cv::Mat
#include <opencv2/highgui.hpp> // For cv::imread/imshow

#include <Matrix.hpp>
#include <Exception.hpp>

namespace anpi
{
//posible resistance values
const int MEGAOHM = 1000;
const int OHM = 1;
const int BLACK = 0;
const int WHITE = 1;

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
    Matrix<double> A;
    ///  Vector  of  the  current  equation  system
    std::vector<double> b;
    ///  Vector  of solutions for the  current  equation  system
    std::vector<double> x;
    /// Raw map data
    Matrix<float> rawMap;
    ///  Vector  with the nodes to follow simple path
    std::vector<int> simplePath;

  public:
    ///  . . .  constructors  and  other  methods

    // inline void initializeForTesting(int x, int y)
    // {
    //     rawMap.allocate(x, y);
    //     rawMap.fill(1);
    //     rawMap[3][3] = 0;
    //     rawMap[2][2] = 0;
    // }

    //getters and setters
    inline void setA(Matrix<double> a)
    {
        A = a;
    }
    inline void setRawMap(Matrix<float> a)
    {
        rawMap = Matrix<float>(a);
    }
    inline Matrix<double> getA()
    {
        return A;
    }

    inline void printA()
    {
        anpi::printMatrix(A);
        std::cout << std::endl;
    }
    inline void printRawMap()
    {
        anpi::printMatrix(rawMap);

        std::cout << std::endl;
    }
    inline void printB()
    {
        std::cout << " \n b vector is: \n";
        for (size_t i = 0; i < b.size(); ++i)
        {
            std::cout << b[i] << "  ";
        }
        std::cout << std::endl;
    }
    inline void printX()
    {
        std::cout << " \n x vector is: \n";
        for (size_t i = 0; i < x.size(); ++i)
        {
            std::cout << x[i] << "  ";
        }
        std::cout << std::endl;
    }
    inline void printSimplePath()
    {
        std::cout << " \n Simple Path is: \n";
        for (size_t i = 0; i < simplePath.size(); ++i)
        {
            std::cout << simplePath[i] << "  ";
        }
        std::cout << std::endl;
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

    /**
 * Compute the vale of the resitance given by the provided index
 * 
 * 
 * 
 **/

    int getResistanceValue(int indx);

    /**
    ∗ Compute a rute of more current and create a matrix for print.
    */
    void calculateSimplePath(const indexPair &nodes);

    /**
    ∗ Calculate a node in matrix.
    */
    int calcNode(int row, int col);

    /**
    ∗ Calculate more current number.
    */
    int calcCurrent(int Up, int Dowm, int Right, int Left);
    int calcCurrent(int down, int right, int left);
    int calcCurrent(int right, int left);

    int maxCurrent(std::vector<int> &currents);
};

} // namespace anpi
