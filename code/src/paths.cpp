/**
 * Copyright (C) 2018
 * Área Académica de Ingeniería en Computadoras, TEC, Costa Rica
 *
 * This file is part of the CE3102 Numerical Analysis lecture at TEC
 *
 * @Author: 
 * @Date  : 24.02.2018
 */

#include <cstdlib>
#include <iostream>

#include <AnpiConfig.hpp>

#include <string>

#include <opencv2/core.hpp>    // For cv::Mat
#include <opencv2/highgui.hpp> // For cv::imread/imshow

#include <Matrix.hpp>
#include <Exception.hpp>

int main() {
  // Build the name of the image in the data path
  std::string mapPath = std::string( ANPI_DATA_PATH ) + "/mapa.png";

  // Read the image using the OpenCV
  cv::Mat_<float> map;

  cv::imread(mapPath.c_str(),
             CV_LOAD_IMAGE_GRAYSCALE).convertTo(map,CV_32FC1);
  map /= 255.0f; // normalize image range to 0 .. 255

  // And create a window to show the image
  cv::namedWindow(mapPath,CV_WINDOW_NORMAL | CV_GUI_EXPANDED);
  cv::imshow(mapPath,map);

  // Convert the OpenCV matrix into an anpi matrix
  // We have to use the std::allocator to avoid an exact stride
  anpi::Matrix<float,std::allocator<float> > amapTmp(map.rows,
                                                     map.cols,
                                                     map.ptr<float>());
  // And transform it to a SIMD-enabled matrix
  anpi::Matrix<float> amap(amapTmp);
  
  cv::waitKey();
  
  return EXIT_SUCCESS;
}
  
