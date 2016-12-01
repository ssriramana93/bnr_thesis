/**
 * gtsam_ros.h
 */

#ifndef GTSAM_ROS_H
#define GTSAM_ROS_H

#include <gtsam/base/Value.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtest/gtest.h>
#include <iostream>

namespace gtsam {

// Implement 'ToString' functions for GTSAM classes that implement a 'print' for use with gtest
template <typename T>
std::string ToString(const T& value) {
  // Create a string stream buffer
  std::stringstream stream;
  // Store the current cout buffer
  std::streambuf* orig(std::cout.rdbuf());
  // Replace the cout buffer with the provided stream
  std::streambuf* target(stream.rdbuf());
  std::cout.rdbuf(target);
  // Use GTSAM print function to output on cout
  value.print();
  // Put the original cout buffer back
  std::cout.rdbuf(orig);
  // Return the string
  return stream.str();
}

} /// namespace gtsam

#endif // GTSAM_ROS_H
