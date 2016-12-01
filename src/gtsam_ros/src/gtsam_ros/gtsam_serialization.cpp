/**
 * gtsam_serialization.cpp
 *
 * The Boost serialization library, while useful, requires quite a bit
 * of coding overhead to deal with derived classes. This header contains
 * all of the overhead needed to serialize/deserialize GTSAM classes from
 * base class pointers. This needs to be included wherever serialization
 * is performed.
 */

#include <gtsam_ros/gtsam_serialization.h>

/* Export Noise Models */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Base);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Constrained);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Diagonal);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Gaussian);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Unit);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Isotropic);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::Robust);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::mEstimator::Base);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::mEstimator::Null);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::mEstimator::Fair);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::mEstimator::Huber);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::noiseModel::mEstimator::Tukey);

/* Export geometry */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Value);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::LieVector);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::LieMatrix);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Point2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::StereoPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Point3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Rot2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Rot3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Pose2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Pose3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Cal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Cal3DS2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::Cal3_S2Stereo);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::CalibratedCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::SimpleCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::StereoCamera);

/* Export factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::JacobianFactor);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::HessianFactor);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorLieVector);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorLieMatrix);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorStereoPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorPoint3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorRot2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorRot3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorPose2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorPose3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorCal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorCal3DS2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorCalibratedCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorSimpleCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PriorFactorStereoCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorLieVector);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorLieMatrix);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorPoint3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorRot2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorRot3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorPose2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BetweenFactorPose3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityLieVector);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityLieMatrix);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityStereoPoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityPoint3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityRot2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityRot3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityPose2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityPose3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityCal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityCal3DS2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityCalibratedCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualitySimpleCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::NonlinearEqualityStereoCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorPosePoint2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorPosePoint3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorPose2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorPose3);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorCalibratedCameraPoint);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorSimpleCameraPoint);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorCalibratedCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::RangeFactorSimpleCamera);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BearingFactor2D);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::BearingRangeFactor2D);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::GenericProjectionFactorCal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::GenericProjectionFactorCal3DS2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::GeneralSFMFactorCal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::GeneralSFMFactor2Cal3_S2);
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::GenericStereoFactor3D);

/* ************************************************************************* */
void init_gtsam_serialization() {
  // This function is only required so that the compiler pulls in all of the above exports. Looking for a better way.
}
