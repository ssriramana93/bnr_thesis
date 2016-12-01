/**
 * test_gtsam_ros.cpp
 */

#include <gtsam_ros/gtsam_gtest.h>
#include <gtsam_ros/gtsam_serialization.h>
#include <boost/archive/polymorphic_xml_iarchive.hpp>
#include <boost/archive/polymorphic_xml_oarchive.hpp>
#include <gtest/gtest.h>

/* ************************************************************************* */
TEST(gtsam_ros, serialize_geometry){

  init_gtsam_serialization();

  // Create a set of geometry variables
  gtsam::Point2 expected_point2(1.0, 2.0);
  gtsam::Pose2 expected_pose2(1.0, 2.0, 3.0);
  gtsam::Point3 expected_point3(1.0, 2.0, 3.0);
  gtsam::Pose3 expected_pose3(gtsam::Rot3::RzRyRx(1.0, 2.0, 3.0), gtsam::Point3(1.0, 2.0, 3.0));

  // Serialize the variables
  std::ofstream ofs("/tmp/test_gtsam_ros__geometry.xml");
  //boost::archive::xml_oarchive oa(ofs);
  boost::archive::polymorphic_xml_oarchive oa(ofs);
  oa << boost::serialization::make_nvp("point2", expected_point2);
  oa << boost::serialization::make_nvp("pose2", expected_pose2);
  oa << boost::serialization::make_nvp("point3", expected_point3);
  oa << boost::serialization::make_nvp("pose3", expected_pose3);
  ofs.close();

  // Deserialize the variables
  std::ifstream ifs("/tmp/test_gtsam_ros__geometry.xml");
  //boost::archive::xml_iarchive ia(ifs);
  boost::archive::polymorphic_xml_iarchive ia(ifs);
  gtsam::Point2 actual_point2;
  gtsam::Pose2 actual_pose2;
  gtsam::Point3 actual_point3;
  gtsam::Pose3 actual_pose3;
  ia >> boost::serialization::make_nvp("point2", actual_point2);
  ia >> boost::serialization::make_nvp("pose2",  actual_pose2);
  ia >> boost::serialization::make_nvp("point3", actual_point3);
  ia >> boost::serialization::make_nvp("pose3",  actual_pose3);
  ifs.close();

  // Test
  EXPECT_GTSAM_EQUAL(expected_point2, actual_point2, 1.0e-9);
  EXPECT_GTSAM_EQUAL(expected_pose2, actual_pose2, 1.0e-9);
  EXPECT_GTSAM_EQUAL(expected_point3, actual_point3, 1.0e-9);
  EXPECT_GTSAM_EQUAL(expected_pose3, actual_pose3, 1.0e-9);
}

/* ************************************************************************* */
TEST(gtsam_ros, serialize_values){

  init_gtsam_serialization();

  // Create a set of geometry variables
  gtsam::Values expected_values;
  expected_values.insert(1, gtsam::Point2(1.0, 2.0));
  expected_values.insert(2, gtsam::Pose2(1.0, 2.0, 3.0));
  expected_values.insert(3, gtsam::Point3(1.0, 2.0, 3.0));
  expected_values.insert(4, gtsam::Pose3(gtsam::Rot3::RzRyRx(1.0, 2.0, 3.0), gtsam::Point3(1.0, 2.0, 3.0)));

  // Serialize the variables
  std::ofstream ofs("/tmp/test_gtsam_ros__values.xml");
  //boost::archive::xml_oarchive oa(ofs);
  boost::archive::polymorphic_xml_oarchive oa(ofs);
  oa << boost::serialization::make_nvp("values", expected_values);
  ofs.close();

  // Deserialize the variables
  std::ifstream ifs("/tmp/test_gtsam_ros__values.xml");
  //boost::archive::xml_iarchive ia(ifs);
  boost::archive::polymorphic_xml_iarchive ia(ifs);
  gtsam::Values actual_values;
  ia >> boost::serialization::make_nvp("values",  actual_values);
  ifs.close();

  // Test
  EXPECT_GTSAM_EQUAL(expected_values, actual_values, 1.0e-9);
}

/* ************************************************************************* */
TEST(gtsam_ros, serialize_noise_models){

  init_gtsam_serialization();

  // Create a set of geometry variables
  gtsam::noiseModel::Base::shared_ptr expected_gaussian = gtsam::noiseModel::Gaussian::SqrtInformation(gtsam::Matrix_(2, 2,  2.0, 0.0,   0.0, 3.0));
  gtsam::noiseModel::Base::shared_ptr expected_diagonal = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(2,  1.0, 2.0), true);
  gtsam::noiseModel::Base::shared_ptr expected_isotropic = gtsam::noiseModel::Isotropic::Sigma(2, 2.0, true);
  gtsam::noiseModel::Base::shared_ptr expected_unit = gtsam::noiseModel::Unit::Create(2);

  // Serialize the variables
  std::ofstream ofs("/tmp/test_gtsam_ros__noise_models.xml");
  //boost::archive::xml_oarchive oa(ofs);
  boost::archive::polymorphic_xml_oarchive oa(ofs);
  oa << boost::serialization::make_nvp("gaussian", expected_gaussian);
  oa << boost::serialization::make_nvp("diagonal", expected_diagonal);
  oa << boost::serialization::make_nvp("isotropic", expected_isotropic);
  oa << boost::serialization::make_nvp("unit", expected_unit);
  ofs.close();

  // Deserialize the variables
  std::ifstream ifs("/tmp/test_gtsam_ros__noise_models.xml");
  //boost::archive::xml_iarchive ia(ifs);
  boost::archive::polymorphic_xml_iarchive ia(ifs);
  gtsam::noiseModel::Base::shared_ptr actual_gaussian;
  gtsam::noiseModel::Base::shared_ptr actual_diagonal;
  gtsam::noiseModel::Base::shared_ptr actual_isotropic;
  gtsam::noiseModel::Base::shared_ptr actual_unit;
  ia >> boost::serialization::make_nvp("gaussian", actual_gaussian);
  ia >> boost::serialization::make_nvp("diagonal", actual_diagonal);
  ia >> boost::serialization::make_nvp("isotropic", actual_isotropic);
  ia >> boost::serialization::make_nvp("unit", actual_unit);
  ifs.close();

  // Test
  EXPECT_GTSAM_EQUAL(*expected_gaussian,  *actual_gaussian,  1.0e-9);
  EXPECT_GTSAM_EQUAL(*expected_diagonal,  *actual_diagonal,  1.0e-9);
  EXPECT_GTSAM_EQUAL(*expected_isotropic, *actual_isotropic, 1.0e-9);
  EXPECT_GTSAM_EQUAL(*expected_unit,      *actual_unit,      1.0e-9);
}

/* ************************************************************************* */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
