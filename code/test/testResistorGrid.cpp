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
    rg.setA(a);

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
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 0
    testResistor = 0;
    calculatedIndex = rg.indexToNodes(testResistor);
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 19
    testResistor = 19;
    calculatedIndex = rg.indexToNodes(testResistor);
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);

    // testing for 38
    testResistor = 38;
    calculatedIndex = rg.indexToNodes(testResistor);
    calculatedIndex.print();

    calculatedResistor = rg.nodesToIndex(calculatedIndex);
    std::cout << calculatedResistor;

    BOOST_CHECK(calculatedResistor == testResistor);
} //END OF indexTest()

} // namespace test
} // namespace anpi

BOOST_AUTO_TEST_SUITE(ResistorGrid)

BOOST_AUTO_TEST_CASE(IndexConversion)
{

    anpi::test::indexTest();
}

BOOST_AUTO_TEST_CASE(MapLoading)
{
    // anpi::test::luTest<float>(anpi::luCrout<float>, anpi::unpackCrout<float>);
    // anpi::test::luTest<double>(anpi::luCrout<double>, anpi::unpackCrout<double>);
}

BOOST_AUTO_TEST_SUITE_END()
