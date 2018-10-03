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
    // Read the image using the OpenCV
    cv::Mat_<float> map;
    try
    {
        cv::imread(filename.c_str(),
                   CV_LOAD_IMAGE_GRAYSCALE)
            .convertTo(map, CV_32FC1);
        map /= 255.0f; // normalize image range to 0 .. 255

        // Convert the OpenCV matrix into an anpi matrix
        // We have to use the std::allocator to avoid an exact stride
        anpi::Matrix<float, std::allocator<float>> amapTmp(map.rows,
                                                           map.cols,
                                                           map.ptr<float>());
        // And transform it to a SIMD-enabled matrix
        anpi::Matrix<float> amap(amapTmp);
        rawMap = amap;

        return true;
    }
    catch (Exception e)
    {
        return false;
    }
}

/**
∗ Compute the internal data to navigate between the given nodes
*/
bool ResistorGrid::navigate(const indexPair &nodes)
{
    int cols = rawMap.cols(), rows = rawMap.rows();
    int resistors = cols * rows * 2 - (cols + rows);       //total amount of resistors and equations
    int nodeEquationNum = cols * rows;                     // amount of node equations
    int gridEquationNum = cols * rows - (cols + rows) + 1; //amount of grid equations

    int startNode = nodes.row1 * cols + nodes.col1;
    int endNode = nodes.row2 * cols + nodes.col2;

    if (startNode == endNode)
    {
        //throw exception not possible to have same start and end node
        throw anpi::Exception("Start and End nodes are the same, no path to navigate\n");
        return false;
    }
    //initialize A & b
    ResistorGrid::A.allocate(resistors, resistors);
    ResistorGrid::A.fill(0.f);
    ResistorGrid::b.resize(resistors, 0);

    //************************************************* node equations ************************************************************************************
    int nodePtr;
    //if one of the start or finish nodes is the first node 0,0 we can't eliminate that one
    if ((startNode == 0) || (endNode == 0))
    {

        //if one of the start or finish nodes is the last node n,m we can't eliminate that one
        if ((startNode == nodeEquationNum) || (endNode == nodeEquationNum))
        {
            //so we eliminate node 0,1

            //since we are removing the second equation the indexes of the start and end nodes change depending if
            //one of them is the first node
            if (startNode == 0)
            {
                b[startNode] = 1;
                b[endNode - 1] = -1;
            }
            if (endNode == 0)
            {
                b[startNode - 1] = 1;
                b[endNode] = -1;
            }
            else
            {
                b[startNode - 1] = 1;
                b[endNode - 1] = -1;
            }

            nodePtr = 0;

            for (int i = 0; i < nodeEquationNum - 1; ++i)
            {
                // because we are skipping the  node 0,1 which is i=1
                if (i == 1)
                    ++nodePtr;

                //if we are at the top border
                if (nodePtr % cols == nodePtr)
                {
                    //if we are at the top left corner
                    if (i == 0)
                    {
                        //incoming right
                        A[i][nodePtr] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                    }

                    //we are at the top right corner
                    if (i == cols - 1)
                    {
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                    }
                    else
                    {
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                    }
                }
                //if we are at the left border
                else if (nodePtr % cols == 0)
                {
                    //if we are at the botton left corner
                    if (nodePtr / cols == rows)
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                    }
                    else
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                        //outfoing down
                        A[i][nodePtr + cols] = -1;
                    }
                }
                //if we are at the right border
                else if (nodePtr % cols == cols - 1)
                {

                    //if we are at the botton right corner
                    if (nodePtr / cols == rows)
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing left
                        A[i][nodePtr - 1] = -1;
                    }
                    else
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                    }
                }
                //if we are at the bottom border
                else if (nodePtr / cols == rows)
                {
                    //all borders have been checked
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                }

                else
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                    //outgoing down
                    A[i][nodePtr + cols] = -1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                }
                //increment pointer to current node
                ++nodePtr;
            } //end of for-------------------------------------------------------------------------------------------
        }     //end of eliminate 0,1

        //we eliminate equation for node n,m
        else
        {
            //since we are removing the last equation the indexes of the star andindex nodes remain the same
            b[startNode] = 1;
            b[endNode] = -1;

            // because we are skipping the last node n,m which is i=nodeEquationNum-1
            nodePtr = 0;

            for (int i = 0; i < nodeEquationNum - 1; ++i)
            {

                //if we are at the top border
                if (nodePtr % cols == nodePtr)
                {
                    //if we are at the top left corner
                    if (i == 0)
                    {
                        //incoming right
                        A[i][nodePtr] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                    }

                    //we are at the top right corner
                    if (i == cols - 1)
                    {
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                    }
                    else
                    {
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                    }
                }
                //if we are at the left border
                else if (nodePtr % cols == 0)
                {
                    //if we are at the botton left corner
                    if (nodePtr / cols == rows)
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                    }
                    else
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing right
                        A[i][nodePtr] = -1;
                        //outfoing down
                        A[i][nodePtr + cols] = -1;
                    }
                }
                //if we are at the right border
                else if (nodePtr % cols == cols - 1)
                {

                    // //if we are at the botton right corner
                    // if (nodePtr / cols == rows)
                    // {
                    //     //incoming up
                    //     A[i][nodePtr - cols - 1] = 1;
                    //     //outgoing left
                    //     A[i][nodePtr - 1] = -1;
                    // }
                    // else

                    //we don't check for node n,m since we are skipping it
                    {
                        //incoming up
                        A[i][nodePtr - cols] = 1;
                        //outgoing down
                        A[i][nodePtr + cols] = -1;
                        //incoming left
                        A[i][nodePtr - 1] = 1;
                    }
                }
                //if we are at the bottom border
                else if (nodePtr / cols == rows)
                {
                    //all borders have been checked
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                }

                else
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                    //outgoing down
                    A[i][nodePtr + cols] = -1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                }
                //increment pointer to current node
                ++nodePtr;
            } //end of for-----------------------------------------------------------------------------------
        }     //end of eliminate n,m
    }

    //we eliminate equation for node 0,0
    else
    {
        //since we are removing the first equation the indexes of the start and end nodes have to be reduced by 1
        b[startNode - 1] = 1;
        b[endNode - 1] = -1;

        for (int i = 0; i < nodeEquationNum - 1; ++i)
        {
            // because we are skipping node 0,0 which is i=0
            nodePtr = i + 1;
            //if we are at the top border
            if (nodePtr % cols == nodePtr)
            {
                //we are at the top right corner
                if (i == cols - 1)
                {
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                    //outgoing down
                    A[i][nodePtr + cols - 1] = -1;
                }
                else
                {
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                    //outgoing down
                    A[i][nodePtr + cols] = -1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                } //we don't check for node 0,0 since we are skipping it
            }
            //if we are at the left border
            else if (nodePtr % cols == 0)
            {
                //if we are at the botton left corner
                if (nodePtr / cols == rows)
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                }
                else
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing right
                    A[i][nodePtr] = -1;
                    //outfoing down
                    A[i][nodePtr + cols] = -1;
                }
            }
            //if we are at the right border
            else if (nodePtr % cols == cols - 1)
            {

                //if we are at the botton right corner
                if (nodePtr / cols == rows)
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing left
                    A[i][nodePtr - 1] = -1;
                }
                else
                {
                    //incoming up
                    A[i][nodePtr - cols] = 1;
                    //outgoing down
                    A[i][nodePtr + cols] = -1;
                    //incoming left
                    A[i][nodePtr - 1] = 1;
                }
            }
            //if we are at the bottom border
            else if (nodePtr / cols == rows)
            {
                //all corners have been checked
                //incoming up
                A[i][nodePtr - cols] = 1;
                //outgoing right
                A[i][nodePtr] = -1;
                //incoming left
                A[i][nodePtr - 1] = 1;
            }

            //not on a border and not a corner
            else
            {
                //incoming up
                A[i][nodePtr - cols] = 1;
                //incoming left
                A[i][nodePtr - 1] = 1;
                //outgoing down
                A[i][nodePtr + cols] = -1;
                //outgoing right
                A[i][nodePtr] = -1;
            }
        } //end of for
    }     //end of eliminate node 0,0

    //********************************** end of node equations *****************************************************************************************

    //############################### begin grid equations ###############################################
    int gridPtr = 0;
    int currBand, numGridBand, r1, r2, r3, r4;
    int bandSize = 2 * cols - 1;

    //fill the last part of A matrix, we skipped one equation from the nodes equations so we start from
    //nodeEquationNum - 1
    for (int i = nodeEquationNum - 1; i < nodeEquationNum + gridEquationNum - 1; ++i)
    {
        //the amount of grids per band is n-1 where n is the amount of columns
        currBand = gridPtr / (cols - 1);

        //the position of the grid witthin the band from 0 to n-2
        numGridBand = gridPtr % (cols - 1);

        r1 = currBand * (bandSize) + numGridBand;
        r2 = r1 + cols;
        r3 = r1 + (bandSize);
        r4 = r1 + cols - 1;

        A[i][r1] = getResistanceValue(r1);
        A[i][r2] = getResistanceValue(r2);
        A[i][r3] = getResistanceValue(r3) * (-1);
        A[i][r4] = getResistanceValue(r4) * (-1);

        //increment the current grid equation pointer
        ++gridPtr;
    }
    //############################## end grid equations #################################

    return true;
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
            throw anpi::Exception("Indexes provided are not contiguos\n");
        }

        return col1 + row1 * (2 * rawMap.cols() - 1);
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
            //throw
            throw anpi::Exception("Indexes provided are not contiguos\n");
        }

        return col1 + (rawMap.cols() - 1) + row1 * (2 * rawMap.cols() - 1);
    }
    //indexes are not contiguos
    else
    {
        //throw exception
        throw anpi::Exception("Indexes provided are not contiguos\n");
    }
    return 0;
}

/**
∗ compute the indices of the resistor given the numerical representation of it
*/
indexPair ResistorGrid::indexToNodes(const std::size_t idx)
{
    //resistor does not exist in the grid
    if (idx > rawMap.cols() * rawMap.rows() * 2 - (rawMap.cols() + rawMap.rows()))
    {
        //throw exception
        throw anpi::Exception("Resistor does not exist. Index exceeds the amount of resistors\n");
    }
    indexPair res;

    //the amount of resistors in a line of nodes, i.e. for n columns
    //we have n-1 horizontal and n vertical resistors for a total of
    // 2n - 1 total resistors
    int blockSize = 2 * rawMap.cols() - 1;

    //amount of horizontal resistors
    int rowsize = rawMap.cols() - 1;

    res.row1 = idx / blockSize;

    int posCol = idx % blockSize;

    //vertical
    if (posCol > (rowsize - 1)) //we use rowsize-1 becuase we start counting resistors from 0
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

/**
 * Compute the value of the resitance given by the provided index
 * 
 * 
 * 
 **/

int ResistorGrid::getResistanceValue(int indx)
{
    indexPair resIndex = indexToNodes(indx);

    //if one of the nodes is 0 there is a high resistance
    if ((rawMap[resIndex.row1][resIndex.col1] == BLACK) || (rawMap[resIndex.row2][resIndex.col2] == BLACK))
    {
        return MEGAOHM;
    }
    else
        return OHM;
}

/**
∗ Compute a rute of more current and create a matrix for print.
*/
void ResistorGrid::calculateSimplePath(const indexPair &nodes)
{
    int cols = rawMap.cols();
    int rows = rawMap.rows();
    int startNode = nodes.row1 * cols + nodes.col1;
    int endNode = nodes.row2 * cols + nodes.col2;
    int nodeEquationNum = cols * rows; // amount of node equations
    int nodePtr = startNode;
    std::vector<int> vectPath;
    int iUp, iDown, iRight, iLeft;
    indexPair moveRes;
    while (nodePtr != endNode)
    {
        //if we are at the top border
        if (nodePtr % cols == nodePtr)
        {
            //we are at the top right corner
            if (nodePtr == cols - 1)
            {
                //incoming left
                iLeft = nodePtr - 1;
                //outgoing down
                iDown = cols - 1;
            }
            else
            {
                //incoming left
                iLeft = nodePtr - 1;
                //outgoing down
                iDown = nodePtr + cols;
                //outgoing right
                iRight = nodePtr;
            } //we don't check for node 0,0 since we are skipping it
        }
        //if we are at the left border
        else if (nodePtr % cols == 0)
        {
            //if we are at the botton left corner
            if (nodePtr / cols == rows)
            {
                //incoming up
                iUp = nodePtr - cols;
                //outgoing right
                iRight = nodePtr;
            }
            else
            {
                //incoming up
                iUp = nodePtr - cols;
                //outgoing right
                iRight = nodePtr;
                //outfoing down
                iDown = nodePtr + cols;
            }
        }
        //if we are at the right border
        else if (nodePtr % cols == cols - 1)
        {

            //if we are at the botton right corner
            if (nodePtr / cols == rows)
            {
                //incoming up
                iUp = nodePtr - cols;
                //outgoing left
                iLeft = nodePtr - 1;
            }
            else
            {
                //incoming up
                iUp = nodePtr - cols;
                //outgoing down
                iDown = nodePtr + cols - 1;
                //incoming left
                iLeft = nodePtr - 1;
            }
        }
        //if we are at the bottom border
        else if (nodePtr / cols == rows)
        {
            //all corners have been checked
            //incoming up
            iUp = nodePtr - cols;
            //outgoing right
            iRight = nodePtr;
            //incoming left
            iLeft = nodePtr - 1;
        }

        //not on a border and not a corner
        else
        {
            //incoming up
            iUp = nodePtr - cols;
            //incoming left
            iLeft = nodePtr - 1;
            //outgoing down
            iDown = nodePtr + cols;
            //outgoing right
            iRight = nodePtr;
        }

        for (int j = 0; j <= 4; j++)
        {
        }
        //moveRes = indexToNodes(restencia mayor);
        nodePtr = calcNode(moveRes.row2, moveRes.col2);

    } //end while
}

int ResistorGrid::calcNode(int row, int col)
{
    int cols = rawMap.cols();
    int rows = rawMap.rows();
    return row * cols + col;
}

} // namespace anpi
