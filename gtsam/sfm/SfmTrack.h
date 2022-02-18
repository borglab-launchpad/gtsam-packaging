/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmTrack.h
 * @date January 2022
 * @author Frank Dellaert
 * @brief A simple data structure for a track in Structure from Motion
 */

#pragma once

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/// A measurement with its camera index
typedef std::pair<size_t, Point2> SfmMeasurement;

/// Sift index for SfmTrack
typedef std::pair<size_t, size_t> SiftIndex;

/**
 * @brief An SfmTrack stores SfM measurements grouped in a track
 * @addtogroup sfm
 */
struct GTSAM_EXPORT SfmTrack {
  Point3 p;       ///< 3D position of the point
  float r, g, b;  ///< RGB color of the 3D point

  /// The 2D image projections (id,(u,v))
  std::vector<SfmMeasurement> measurements;

  /// The feature descriptors
  std::vector<SiftIndex> siftIndices;

  /// @name Constructors
  /// @{

  explicit SfmTrack(float r = 0, float g = 0, float b = 0)
      : p(0, 0, 0), r(r), g(g), b(b) {}

  explicit SfmTrack(const gtsam::Point3& pt, float r = 0, float g = 0,
                    float b = 0)
      : p(pt), r(r), g(g), b(b) {}

  /// @}
  /// @name Standard Interface
  /// @{

  /// Add measurement (camera_idx, Point2) to track
  void addMeasurement(size_t idx, const gtsam::Point2& m) {
    measurements.emplace_back(idx, m);
  }

  /// Total number of measurements in this track
  size_t numberMeasurements() const { return measurements.size(); }

  /// Get the measurement (camera index, Point2) at pose index `idx`
  const SfmMeasurement& measurement(size_t idx) const {
    return measurements[idx];
  }

  /// Get the SIFT feature index corresponding to the measurement at `idx`
  const SiftIndex& siftIndex(size_t idx) const { return siftIndices[idx]; }

  /// Get 3D point
  const Point3& point3() const { return p; }

  /// Get RGB values describing 3d point
  Point3 rgb() const { return Point3(r, g, b); }

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const SfmTrack& sfmTrack, double tol = 1e-9) const;

  /// @}
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  /// @name Deprecated
  /// @{
  void GTSAM_DEPRECATED add_measurement(size_t idx, const gtsam::Point2& m) {
    measurements.emplace_back(idx, m);
  }

  size_t GTSAM_DEPRECATED number_measurements() const {
    return measurements.size();
  }
  /// @}
#endif
  /// @name Serialization
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(p);
    ar& BOOST_SERIALIZATION_NVP(r);
    ar& BOOST_SERIALIZATION_NVP(g);
    ar& BOOST_SERIALIZATION_NVP(b);
    ar& BOOST_SERIALIZATION_NVP(measurements);
    ar& BOOST_SERIALIZATION_NVP(siftIndices);
  }
  /// @}
};

template <typename T>
struct traits;

template <>
struct traits<SfmTrack> : public Testable<SfmTrack> {};

}  // namespace gtsam
