#include "ResistorGrid.hpp"

namespace anpi
{

///  . . .  constructors  and  other  methods
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
∗ compute a number representig the resistor  located in the provided indices
*/
std::size_t nodesToIndex(const std::size_t row1, const std::size_t col1, const std::size_t row2, const std::size_t col2)
{

    return 0;
}

/**
∗ compute the indices of the resistor given the numerical representation of it
*/
indexPair indexToNodes(const std::size_t idx)
{
    indexPair res;

    return res;
}

} // namespace anpi
