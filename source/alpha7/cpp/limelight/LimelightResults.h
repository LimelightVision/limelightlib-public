// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "limelight/BarcodeTarget.h"
#include "limelight/CameraIntrinsics.h"
#include "limelight/ClassifierTarget.h"
#include "limelight/CounterTarget.h"
#include "limelight/DetectorTarget.h"
#include "limelight/FiducialTarget.h"
#include "limelight/Geometry.h"
#include "limelight/HardwareData.h"
#include "limelight/IMUData.h"
#include "limelight/OCRTarget.h"
#include "limelight/PipelineConfigurationOverrideState.h"
#include "limelight/PoseEstimateType.h"
#include "limelight/RetroTarget.h"
#include "limelight/RewindData.h"
#include "limelight/RobotOrientationData.h"
#include "limelight/SharedMapState.h"
#include "limelight/Timebase.h"
#include "wpi/math/geometry/Pose3d.hpp"
#include "wpi/nt/ntcore_cpp.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
namespace limelight {

class Limelight;

/**
 * The complete decoded results for one frame. All fields come from one
 * MessagePack payload.
 */
class LimelightResults {
 public:
  /** True if at least one target is valid in this frame */
  bool valid = false;
  /** Legacy timestamp in milliseconds since vision-process boot */
  wpi::units::millisecond_t timestamp{0.0};
  /** Microseconds since vision-process boot */
  int64_t timestampMicros = 0;
  /** System wall-clock time in microseconds since epoch */
  int64_t systemTimestampMicros = 0;
  /** NetworkTables server time in microseconds. 0 if the camera is not
   *  connected to a server */
  int64_t ntTimestampMicros = 0;
  /** Frame counter. Starts at 0. -1 before the first frame */
  int64_t frameIndex = -1;
  /** Targeting/pipeline latency in milliseconds */
  wpi::units::millisecond_t targetingLatency{0.0};
  /** Capture latency in milliseconds */
  wpi::units::millisecond_t captureLatency{0.0};
  /** Active pipeline index */
  double pipelineIndex = -1;
  /** Active pipeline type (e.g. "pipe_fiducial") */
  std::string pipelineType;
  /** Focus metric for focus-assist */
  double focusMetric = 0;

  /** Camera-reported MegaTag1 standard deviations [x, y, z, roll, pitch,
   *  yaw]. The camera averages these over a long window. Use for telemetry
   *  only. Do not use for fusion */
  std::vector<double> stdDevsMT1 = std::vector<double>(kPoseArrayLength, 0.0);
  /** Camera-reported MegaTag2 standard deviations [x, y, z, roll, pitch,
   *  yaw]. The camera averages these over a long window. Use for telemetry
   *  only. Do not use for fusion */
  std::vector<double> stdDevsMT2 = std::vector<double>(kPoseArrayLength, 0.0);

  /** MegaTag1 robot pose in field space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> robotPoseMT1 = std::vector<double>(kPoseArrayLength, 0.0);
  /** MegaTag1 robot pose, WPILib red-alliance coordinate system */
  std::vector<double> robotPoseMT1WpiRed =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** MegaTag1 robot pose, WPILib blue-alliance coordinate system */
  std::vector<double> robotPoseMT1WpiBlue =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** MegaTag2 robot pose in field space */
  std::vector<double> robotPoseMT2 = std::vector<double>(kPoseArrayLength, 0.0);
  /** MegaTag2 robot pose, WPILib red-alliance coordinate system */
  std::vector<double> robotPoseMT2WpiRed =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** MegaTag2 robot pose, WPILib blue-alliance coordinate system */
  std::vector<double> robotPoseMT2WpiBlue =
      std::vector<double>(kPoseArrayLength, 0.0);

  /** Number of AprilTags that contribute to the robot pose estimates */
  int reportedTagCount = 0;
  /** Max distance between contributing tags in meters */
  wpi::units::meter_t tagSpan{0.0};
  /** Average distance to contributing tags in meters */
  wpi::units::meter_t avgTagDistance{0.0};
  /** Average area of contributing tags (percentage of image) */
  double avgTagAreaPercent = 0;

  /** Final camera pose in robot space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> cameraPoseRobotSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Data set by a python snapscript via llpython */
  std::vector<double> pythonOutput;

  /** Horizontal offset from crosshair to primary target in degrees */
  wpi::units::degree_t tx{0.0};
  /** Vertical offset from crosshair to primary target in degrees */
  wpi::units::degree_t ty{0.0};
  /** Horizontal offset from principal pixel to primary target in degrees */
  wpi::units::degree_t txNoCrosshair{0.0};
  /** Vertical offset from principal pixel to primary target in degrees */
  wpi::units::degree_t tyNoCrosshair{0.0};
  /** Primary target area (percentage of image, 0-100) */
  double targetAreaPercent = 0;
  /** 3D distance from camera to primary target in meters */
  wpi::units::meter_t targetDistance{0.0};

  std::vector<RetroTarget> retroTargets;
  std::vector<FiducialTarget> fiducialTargets;
  std::vector<DetectorTarget> detectorTargets;
  std::vector<ClassifierTarget> classifierTargets;
  /** Classifier results below the confidence threshold. For visualization
   *  and debugging */
  std::vector<ClassifierTarget> classifierTargetsRejected;
  std::vector<BarcodeTarget> barcodeTargets;
  std::vector<CounterTarget> counterTargets;
  std::vector<OCRTarget> ocrTargets;

  IMUData imu;
  HardwareData hardware;
  RewindData rewind;
  RobotOrientationData robotOrientation;
  CameraIntrinsics intrinsics;

  /** Image source index */
  int imageSource = 0;
  /** Hardware type identifier */
  int hardwareType = 0;
  /** UI refresh counter */
  int uiRefresh = 0;
  /** True if the camera is ignoring NetworkTables input */
  bool ignoreNT = false;
  /** True if the camera is connected to a NetworkTables server */
  bool ntConnected = false;
  /** True if the camera is running a robot-published pipeline configuration
   *  override */
  bool pipelineConfigurationOverrideActive = false;
  /** Pipeline configuration override state reported by the camera */
  PipelineConfigurationOverrideState pipelineConfigurationOverrideState =
      PipelineConfigurationOverrideState::OFF;
  /** True if the camera is localizing against the robot-published shared
   *  field map */
  bool sharedMapActive = false;
  /** Shared field map state reported by the camera */
  SharedMapState sharedMapState = SharedMapState::OFF;

  /** Time spent decoding this envelope in robot code, in milliseconds. It
   *  happens in this library, not in the camera, so it is not part of the
   *  camera's latency. Useful for logging. */
  wpi::units::millisecond_t parseLatency{0.0};
  /** Local NetworkTables time when this frame arrived, in seconds. 0 if this
   *  object did not come from the network */
  wpi::units::second_t receiveTimestamp{0.0};
  /** Non-empty if the envelope failed to decode */
  std::string error;

  /**
   * @return Seconds since this frame arrived. If this value keeps growing, the
   *         camera has stopped sending frames. Possible causes: disconnected,
   *         stopped, or renamed.
   */
  wpi::units::second_t GetAge() const {
    return wpi::units::second_t{wpi::nt::Now() / NT_TICKS_PER_SECOND} -
           receiveTimestamp;
  }

  /** @return Total number of retro, fiducial, detector, classifier, and
   *          barcode targets. Counter and OCR results are not included */
  int GetTargetCount() const {
    return static_cast<int>(retroTargets.size() + fiducialTargets.size() +
                            detectorTargets.size() + classifierTargets.size() +
                            barcodeTargets.size());
  }

  /** @return MegaTag1 robot pose in field space. The origin is the field center
   */
  wpi::math::Pose3d GetRobotPose_MT1() const { return ToPose3D(robotPoseMT1); }

  /** @return MegaTag2 robot pose in field space. The origin is the field center
   */
  wpi::math::Pose3d GetRobotPose_MT2() const { return ToPose3D(robotPoseMT2); }

  /** @return The full 3D robot pose for the given estimate type. The type
   *          selects the origin and the algorithm */
  wpi::math::Pose3d GetRobotPose(PoseEstimateType type) const;

  /** @return Camera pose in robot space (meters, degrees) */

  wpi::math::Pose3d GetCameraPose_RobotSpace() const {
    return ToPose3D(cameraPoseRobotSpace);
  }

 private:
  friend class limelight::Limelight;
  bool fromLiveSubscriber = false;
};

namespace detail {

inline const std::vector<double>& PoseArray(const LimelightResults& results,
                                            PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
      return results.robotPoseMT1WpiBlue;
    case PoseEstimateType::MT1_WPIRED:
      return results.robotPoseMT1WpiRed;
    case PoseEstimateType::MT2_WPIBLUE:
      return results.robotPoseMT2WpiBlue;
    case PoseEstimateType::MT2_WPIRED:
    default:
      return results.robotPoseMT2WpiRed;
  }
}

inline const std::vector<double>& ReportedStdDevs(
    const LimelightResults& results, PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
    case PoseEstimateType::MT1_WPIRED:
      return results.stdDevsMT1;
    case PoseEstimateType::MT2_WPIBLUE:
    case PoseEstimateType::MT2_WPIRED:
    default:
      return results.stdDevsMT2;
  }
}

}  // namespace detail

inline wpi::math::Pose3d LimelightResults::GetRobotPose(
    PoseEstimateType type) const {
  return ToPose3D(detail::PoseArray(*this, type));
}

}  // namespace limelight
