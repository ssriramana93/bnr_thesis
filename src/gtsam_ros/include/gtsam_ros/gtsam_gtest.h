/**
 * gtsam_gtest.h
 */

#ifndef GTSAM_GTEST_H
#define GTSAM_GTEST_H

#include <gtsam_ros/gtsam_ros.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtest/gtest.h>
#include <iostream>

namespace testing {

// Internal helper function for implementing {EXPECT|ASSERT}_GTSAM_EQUAL.
// Don't use this in your code.
template <typename T>
AssertionResult AssertGtsamHelper(const char* e1,
                                  const char* e2,
                                  const T& v1,
                                  const T& v2,
                                  const double tol) {
  if (v1.equals(v2, tol)) return AssertionSuccess();

  return AssertionFailure() << e1 << ".equals("
                            << e2 << ", "
                            << tol << ") evaluates to false, where"
                            << "\n" << e1 << " evaluates to " << gtsam::ToString(v1)
                            << "\n" << e2 << " evaluates to " << gtsam::ToString(v2);
}

//template <>
//AssertionResult AssertGtsamHelper<gtsam::Vector>(const char* e1,
//                                  const char* e2,
//                                  const gtsam::Vector& v1,
//                                  const gtsam::Vector& v2,
//                                  const double tol) {
//  if (gtsam::equal_with_abs_tol(v1, v2, tol)) return AssertionSuccess();
//
//  return AssertionFailure() << e1 << ".equals("
//                            << e2 << ", "
//                            << tol << ") evaluates to false, where"
//                            << "\n" << e1 << " evaluates to [" << v1.transpose() << "] "
//                            << "\n" << e2 << " evaluates to [" << v2.transpose() << "]";
//}

template <>
AssertionResult AssertGtsamHelper<gtsam::Matrix>(const char* e1,
                                  const char* e2,
                                  const gtsam::Matrix& v1,
                                  const gtsam::Matrix& v2,
                                  const double tol) {
  if (gtsam::equal_with_abs_tol(v1, v2, tol)) return AssertionSuccess();

  return AssertionFailure() << e1 << ".equals("
                            << e2 << ", "
                            << tol << ") evaluates to false, where"
                            << "\n" << e1 << " evaluates to [" << v1 << "]"
                            << "\n" << e2 << " evaluates to [" << v2 << "]";
}

// Internal macro for implementing {EXPECT|ASSERT}_GTSAM_EQUAL.
// Don't use this in your code.
#define GTEST_GTSAM_EQUAL_(v1, v2, tol, on_failure)\
  GTEST_ASSERT_(::testing::AssertGtsamHelper(#v1, \
                                             #v2, \
                                             v1, \
                                             v2, \
                                             tol), on_failure)

// Define gtest macros for use with GTSAM
#define EXPECT_GTSAM_EQUAL(v1, v2, tol) \
		GTEST_GTSAM_EQUAL_(v1, v2, tol, GTEST_NONFATAL_FAILURE_)
#define ASSERT_GTSAM_EQUAL(v1, v2, tol) \
		GTEST_GTSAM_EQUAL_(v1, v2, tol, GTEST_FATAL_FAILURE_)

} /// namespace testing

#endif // GTSAM_GTEST_H
