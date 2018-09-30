#include "ResistorGrid.hpp"

namespace anpi
{

///... constructors  and  other  methods

/**
* Construct the grid from the given file
*
 *           @ return true if successful or false otherwise
*/
bool ResistorGrid::build(const std::string filename)
{

    return false;
}

/**
∗ Compute the internal data to navigate between the given nodes
*/
bool ResistorGrid::navigate(const indexPair &nodes)
{

    return false;
}

/**
∗ compute an index number representig the resistor located in the provided indices. 
* this method works when the indices row1 and col1 are equal or less than the row2
* and col2 indexes, that is the initial node is the one with the lowest indexes.
* this method assumes that the provided indexes are contiguos(i.e. they don't exceed
* +1 in only one of the indexes, row or column)
*/
std::size_t ResistorGrid::nodesToIndex(const std::size_t row1, const std::size_t col1, const std::size_t row2, const std::size_t col2)
{
    //horizontal
    if (row1 == row2)
    {
        if (col2 < col1) // we have the indexes ordered backwards
        {

            return nodesToIndex(row2, col2, row1, col1);
        }

        //if not contiguos
        if (col2 != col1 + 1)
        {
            //throw exception
        }

        return col1 + row1 * (2 * A.cols() - 1);
    }
    //vertical
    else if (col1 == col2)
    {
        if (row2 < row1) // we have the indexes ordered backwards
        {

            return nodesToIndex(row2, col2, row1, col1);
        }
        //if not contiguos
        if (row2 != row1 + 1)
        {
            //throw exception
        }

        return col1 + (A.cols() - 1) + row1 * (2 * A.cols() - 1);
    }
    //indexes are not contiguos
    else
    {
        //throw exception
    }
    return 0;
}

/**
∗ compute the indices of the resistor given the numerical representation of it
*/
indexPair ResistorGrid::indexToNodes(const std::size_t idx)
{
    //resistor does not exist in the grid
    if (idx > A.cols() * A.rows() * 2 - (A.cols() + A.rows()))
    {
        //throw exception
        }
    indexPair res;

    //the amount of resistors in a line of nodes, that is for n coumns
    //we have n-1 horizontal and n vertical resistors for a total of
    // 2n - 1 total resistors
    int blockSize = 2 * A.cols() - 1;

    //we use n-2 since we start counting resistors from 0
    int rowsize = A.cols() - 1;

    res.row1 = idx / blockSize;

    int posCol = idx % blockSize;

    //vertical
    if (posCol > (rowsize - 1)) //we use rowsize-1 becuse we start counting resistors from 0
    {
        res.col2 = res.col1 = posCol - rowsize;
        res.row2 = res.row1 + 1;
        return res;
    }
    //horizontal
    else
    {
        res.col1 = posCol;
        res.col2 = posCol + 1;
        res.row2 = res.row1;
        return res;
    }

    return res;
}

} // namespace anpi
