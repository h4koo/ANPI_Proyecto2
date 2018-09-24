# Copyright (C) 2018
# Área Académica de Ingeniería en Computadoras, ITCR, Costa Rica
#
# This file is part of the numerical analysis lecture CE3102 at TEC
#
# author: Pablo Alvarado
# date:   2018-09-07


find_package (Boost COMPONENTS system filesystem program_options REQUIRED)

if (ANPI_ENABLE_OpenMP)
  find_package(OpenMP)
  if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endif (ANPI_ENABLE_OpenMP)

find_package(OpenCV REQUIRED)

include_directories (${CMAKE_SOURCE_DIR}/include
                     ${Boost_INCLUDE_DIR})
##                   ${OpenCV_INCLUDE_DIR})
