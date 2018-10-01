/**
 * Copyright (C) 2017 
 * Área Académica de Ingeniería en Computadoras, TEC, Costa Rica
 *
 * This file is part of the CE3102 Numerical Analysis lecture at TEC
 *
 * @Author: Pablo Alvarado
 * @Date  : 10.02.2018
 */

#include <boost/test/unit_test.hpp>

#include "ResistorGrid.hpp"
// #include "LUDoolittle.hpp"

#include <iostream>
#include <exception>
#include <cstdlib>
#include <complex>

#include <functional>

#include <cmath>

namespace anpi
{
namespace test
{

/// Test the index conversion functions
void indexTest()
{

    ResistorGrid rg;

    //create a 10x10 Matrix filled with ones
    Matrix<float> a(10, 10, 1);

    //set the Matrix in ResistorGrid class since methos use size of internal matrix
    rg.setRawMap(a);

    int calculatedResistor, testResistor = 10;

    indexPair calculatedIndex;

    // re0 = {0, 0, 0, 1};

    // //testing for 0 which is the first resistor
    // rest01 = rg.nodesToIndex(0, 0, 0, 1);
    // r0 = rg.indexToNodes(0);

    // BOOST_CHECK(rest01 == 0);
    // BOOST_CHECK(re0 == r0);

    // // testing for 10
    // calculatedIndex = rg.indexToNodes(testResistor);
    // calculatedIndex.print();

    // calculatedResistor = rg.nodesToIndex(calculatedIndex);
    // std::cout << calculatedResistor;

    // BOOST_CHECK(calculatedResistor == testResistor);
    // BOOST_CHECK(re0 == r0);

    // testing for 13
    testResistor = 13;
    calculatedIndex = rg.indexToNodes(testResistor);
    std::cout << "\ncalculated indexes: \n";
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 0
    testResistor = 0;
    calculatedIndex = rg.indexToNodes(testResistor);
    std::cout << "\ncalculated indexes: \n";
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);

    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 19
    testResistor = 19;
    calculatedIndex = rg.indexToNodes(testResistor);
    std::cout << "\ncalculated indexes: \n";
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 38
    testResistor = 38;
    calculatedIndex = rg.indexToNodes(testResistor);
    std::cout << "\ncalculated indexes: \n";
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);
} //END OF indexTest()

void testNavigate()
{

    ResistorGrid rg;

    //create a 3x4 Matrix filled with ones
    Matrix<float> a(3, 4, 1);
    a[2][1] = 0;

    indexPair test = {1, 0, 1, 3};
    //set the Matrix in ResistorGrid class since method uses size of internal matrix
    rg.setRawMap(a);

    rg.navigate(test);

    std::cout << "\nMatrix A is: \n";
    rg.printA();

    //test initial top right border
    test = {0, 0, 1, 3};
    rg.navigate(test);

    std::cout << "\nMatrix A when initial node is top left corner is: \n";
    rg.printA();

    // //test bottom right border
    // test = {0, 0, 2, 3};
    // rg.navigate(test);

    std::cout << "\nMatrix A when initial node is top left corner and final node is bottom right corner is: \n";
    rg.printA();
} //end test navigate

void testBuild()
{
    // Build the name of the image in the data path
    std::string mapPath = std::string(ANPI_DATA_PATH) + "/mapa.png";
    ResistorGrid rg;
    rg.build(mapPath);
    rg.printRawMap();
}

} // namespace test
} // namespace anpi

BOOST_AUTO_TEST_SUITE(ResistorGrid)

BOOST_AUTO_TEST_CASE(IndexConversion)
{

    anpi::test::indexTest();
}
BOOST_AUTO_TEST_CASE(Navigate)
{
    // anpi::test::testNavigate();
}
BOOST_AUTO_TEST_CASE(MapLoading)
{
    anpi::test::testBuild();
}

BOOST_AUTO_TEST_SUITE_END()