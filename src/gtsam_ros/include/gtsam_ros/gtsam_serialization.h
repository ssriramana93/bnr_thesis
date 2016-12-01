/**
 * gtsam_serialization.h
 *
 * The Boost serialization library, while useful, requires quite a bit
 * of coding overhead to deal with derived classes. This header contains
 * all of the overhead needed to serialize/deserialize GTSAM classes from
 * base class pointers. This needs to be included wherever serialization
 * is performed.
 */

#ifndef GTSAM_SERIALIZATION_H__
#define GTSAM_SERIALIZATION_H__

// includes for the GTSAM datatypes
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/LieMatrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/StereoCamera.h>
// includes for standard serialization types
#include <boost/serialization/export.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/weak_ptr.hpp>
// includes for the different archive types
#include <boost/archive/polymorphic_iarchive.hpp>
#include <boost/archive/polymorphic_oarchive.hpp>

// Typedefs to define/instantiate templated factors
namespace gtsam {
typedef gtsam::PriorFactor<gtsam::LieVector>         PriorFactorLieVector;
typedef gtsam::PriorFactor<gtsam::LieMatrix>         PriorFactorLieMatrix;
typedef gtsam::PriorFactor<gtsam::Point2>            PriorFactorPoint2;
typedef gtsam::PriorFactor<gtsam::StereoPoint2>      PriorFactorStereoPoint2;
typedef gtsam::PriorFactor<gtsam::Point3>            PriorFactorPoint3;
typedef gtsam::PriorFactor<gtsam::Rot2>              PriorFactorRot2;
typedef gtsam::PriorFactor<gtsam::Rot3>              PriorFactorRot3;
typedef gtsam::PriorFactor<gtsam::Pose2>             PriorFactorPose2;
typedef gtsam::PriorFactor<gtsam::Pose3>             PriorFactorPose3;
typedef gtsam::PriorFactor<gtsam::Cal3_S2>           PriorFactorCal3_S2;
typedef gtsam::PriorFactor<gtsam::Cal3DS2>           PriorFactorCal3DS2;
typedef gtsam::PriorFactor<gtsam::CalibratedCamera>  PriorFactorCalibratedCamera;
typedef gtsam::PriorFactor<gtsam::SimpleCamera>      PriorFactorSimpleCamera;
typedef gtsam::PriorFactor<gtsam::StereoCamera>      PriorFactorStereoCamera;

typedef gtsam::BetweenFactor<gtsam::LieVector>       BetweenFactorLieVector;
typedef gtsam::BetweenFactor<gtsam::LieMatrix>       BetweenFactorLieMatrix;
typedef gtsam::BetweenFactor<gtsam::Point2>          BetweenFactorPoint2;
typedef gtsam::BetweenFactor<gtsam::Point3>          BetweenFactorPoint3;
typedef gtsam::BetweenFactor<gtsam::Rot2>            BetweenFactorRot2;
typedef gtsam::BetweenFactor<gtsam::Rot3>            BetweenFactorRot3;
typedef gtsam::BetweenFactor<gtsam::Pose2>           BetweenFactorPose2;
typedef gtsam::BetweenFactor<gtsam::Pose3>           BetweenFactorPose3;

typedef gtsam::NonlinearEquality<gtsam::LieVector>         NonlinearEqualityLieVector;
typedef gtsam::NonlinearEquality<gtsam::LieMatrix>         NonlinearEqualityLieMatrix;
typedef gtsam::NonlinearEquality<gtsam::Point2>            NonlinearEqualityPoint2;
typedef gtsam::NonlinearEquality<gtsam::StereoPoint2>      NonlinearEqualityStereoPoint2;
typedef gtsam::NonlinearEquality<gtsam::Point3>            NonlinearEqualityPoint3;
typedef gtsam::NonlinearEquality<gtsam::Rot2>              NonlinearEqualityRot2;
typedef gtsam::NonlinearEquality<gtsam::Rot3>              NonlinearEqualityRot3;
typedef gtsam::NonlinearEquality<gtsam::Pose2>             NonlinearEqualityPose2;
typedef gtsam::NonlinearEquality<gtsam::Pose3>             NonlinearEqualityPose3;
typedef gtsam::NonlinearEquality<gtsam::Cal3_S2>           NonlinearEqualityCal3_S2;
typedef gtsam::NonlinearEquality<gtsam::Cal3DS2>           NonlinearEqualityCal3DS2;
typedef gtsam::NonlinearEquality<gtsam::CalibratedCamera>  NonlinearEqualityCalibratedCamera;
typedef gtsam::NonlinearEquality<gtsam::SimpleCamera>      NonlinearEqualitySimpleCamera;
typedef gtsam::NonlinearEquality<gtsam::StereoCamera>      NonlinearEqualityStereoCamera;

typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>                             RangeFactorPosePoint2;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>                             RangeFactorPosePoint3;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>                              RangeFactorPose2;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>                              RangeFactorPose3;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>                  RangeFactorCalibratedCameraPoint;
typedef gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::Point3>                      RangeFactorSimpleCameraPoint;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>        RangeFactorCalibratedCamera;
typedef gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::SimpleCamera>                RangeFactorSimpleCamera;

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>              BearingFactor2D;
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Rot3>              BearingFactor3D;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>                      BearingRangeFactor2D;
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>                      BearingRangeFactor3D;

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> GenericProjectionFactorCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> GenericProjectionFactorCal3DS2;
typedef gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3>                 GeneralSFMFactorCal3_S2;
typedef gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>                                    GeneralSFMFactor2Cal3_S2;
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>                     GenericStereoFactor3D;
}

/* Export Noise Models */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Base);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Constrained);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Diagonal);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Gaussian);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Unit);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Isotropic);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Robust);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Base);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Null);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Fair);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Huber);
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Tukey);

/* Export geometry */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_KEY(gtsam::Value);
BOOST_CLASS_EXPORT_KEY(gtsam::LieVector);
BOOST_CLASS_EXPORT_KEY(gtsam::LieMatrix);
BOOST_CLASS_EXPORT_KEY(gtsam::Point2);
BOOST_CLASS_EXPORT_KEY(gtsam::StereoPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::Point3);
BOOST_CLASS_EXPORT_KEY(gtsam::Rot2);
BOOST_CLASS_EXPORT_KEY(gtsam::Rot3);
BOOST_CLASS_EXPORT_KEY(gtsam::Pose2);
BOOST_CLASS_EXPORT_KEY(gtsam::Pose3);
BOOST_CLASS_EXPORT_KEY(gtsam::Cal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::Cal3DS2);
BOOST_CLASS_EXPORT_KEY(gtsam::Cal3_S2Stereo);
BOOST_CLASS_EXPORT_KEY(gtsam::CalibratedCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::SimpleCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::StereoCamera);

/* Export factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_KEY(gtsam::JacobianFactor);
BOOST_CLASS_EXPORT_KEY(gtsam::HessianFactor);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorLieVector);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorLieMatrix);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorStereoPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorPoint3);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorRot2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorRot3);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorPose2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorPose3);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorCal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorCal3DS2);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorCalibratedCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorSimpleCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::PriorFactorStereoCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorLieVector);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorLieMatrix);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorPoint3);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorRot2);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorRot3);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorPose2);
BOOST_CLASS_EXPORT_KEY(gtsam::BetweenFactorPose3);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityLieVector);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityLieMatrix);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityStereoPoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityPoint3);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityRot2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityRot3);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityPose2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityPose3);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityCal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityCal3DS2);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityCalibratedCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualitySimpleCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::NonlinearEqualityStereoCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorPosePoint2);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorPosePoint3);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorPose2);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorPose3);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorCalibratedCameraPoint);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorSimpleCameraPoint);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorCalibratedCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::RangeFactorSimpleCamera);
BOOST_CLASS_EXPORT_KEY(gtsam::BearingFactor2D);
BOOST_CLASS_EXPORT_KEY(gtsam::BearingRangeFactor2D);
BOOST_CLASS_EXPORT_KEY(gtsam::GenericProjectionFactorCal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::GenericProjectionFactorCal3DS2);
BOOST_CLASS_EXPORT_KEY(gtsam::GeneralSFMFactorCal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::GeneralSFMFactor2Cal3_S2);
BOOST_CLASS_EXPORT_KEY(gtsam::GenericStereoFactor3D);

/* ************************************************************************* */
void init_gtsam_serialization();

#endif // GTSAM_SERIALIZATION_H__
