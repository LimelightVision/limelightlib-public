// LimelightLib v2.0.0. Requires Limelight OS 2027.0 or later.

// Copy this file to src/main/include/Limelight.h.

#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Pose3d.hpp"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/geometry/Rotation3d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/geometry/Translation3d.hpp"
#include "wpi/math/geometry/struct/Pose2dStruct.hpp"
#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/DoubleArrayTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/NetworkTable.hpp"
#include "wpi/nt/NetworkTableEntry.hpp"
#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/RawTopic.hpp"
#include "wpi/nt/StringTopic.hpp"
#include "wpi/nt/StructArrayTopic.hpp"
#include "wpi/nt/ntcore_cpp.hpp"
#include "wpi/system/Filesystem.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/util/array.hpp"

namespace limelight {

class Limelight;

namespace detail {

constexpr double DegreesToRadians(double degrees) {
  return degrees * std::numbers::pi / 180.0;
}

constexpr double RadiansToDegrees(double radians) {
  return radians * 180.0 / std::numbers::pi;
}

constexpr double ClampArg(double value, double min, double max,
                          double nanFallback) {
  if (std::isnan(value)) {
    return nanFallback;
  }
  return std::max(min, std::min(max, value));
}

constexpr double ClampArg(double value, double min, double max) {
  return ClampArg(value, min, max, min);
}

}  // namespace detail

/**
 * Converts a pose array to a Pose3d. The array has 6 values: [x, y, z, roll,
 * pitch, yaw]. Units are meters and degrees.
 * @return The pose. Returns an empty Pose3d if the array is not valid
 */
inline wpi::math::Pose3d ToPose3D(std::span<const double> inData) {
  if (inData.size() < 6) {
    return wpi::math::Pose3d{};
  }
  return wpi::math::Pose3d{
      wpi::math::Translation3d{wpi::units::meter_t{inData[0]},
                               wpi::units::meter_t{inData[1]},
                               wpi::units::meter_t{inData[2]}},
      wpi::math::Rotation3d{
          wpi::units::radian_t{detail::DegreesToRadians(inData[3])},
          wpi::units::radian_t{detail::DegreesToRadians(inData[4])},
          wpi::units::radian_t{detail::DegreesToRadians(inData[5])}}};
}

/**
 * Converts a pose array to a Pose2d. The array has 6 values: [x, y, z, roll,
 * pitch, yaw]. Units are meters and degrees. Uses only the x, y, and yaw
 * values.
 * @return The pose. Returns an empty Pose2d if the array is not valid
 */
inline wpi::math::Pose2d ToPose2D(std::span<const double> inData) {
  if (inData.size() < 6) {
    return wpi::math::Pose2d{};
  }
  return wpi::math::Pose2d{
      wpi::math::Translation2d{wpi::units::meter_t{inData[0]},
                               wpi::units::meter_t{inData[1]}},
      wpi::math::Rotation2d{
          wpi::units::radian_t{detail::DegreesToRadians(inData[5])}}};
}

/** NetworkTables timestamp ticks per second for the WPILib build this library
 *  targets. WPILib 2027 alpha-7 and later use 1e9. Alpha-6 and earlier use
 *  1e6. Divide a NetworkTables timestamp by this value to get seconds. */
inline constexpr double NT_TICKS_PER_SECOND = 1.0e6;

/**
 * Health of a camera.
 */
enum class Status {
  /** A decodable frame has been received recently */
  OK,
  /** The camera has not sent a results envelope. Possible causes: the camera
   *  is off, the name is wrong, or MsgPack output is disabled */
  NO_DATA,
  /** The newest frame is older than the stale threshold. The camera is
   *  disconnected or has stopped */
  STALE,
  /** The newest frame did not decode. See LimelightResults::error */
  DECODE_ERROR
};

/** Pipeline configuration override state reported by the camera. */
enum class PipelineConfigurationOverrideState {
  /** No override requested */
  OFF,
  /** The override is applied */
  ACTIVE,
  /** The camera web interface forced the override off */
  FORCED_OFF,
  /** The override is enabled but no configuration is published */
  NO_STRING,
  /** The published configuration failed to parse or was too large */
  PARSE_ERROR,
  /** The camera reported a state this library does not know */
  UNKNOWN
};

/** Shared field map state reported by the camera. */
enum class SharedMapState {
  /** No shared map published */
  OFF,
  /** The shared map is in use */
  ACTIVE,
  /** The published map failed to parse or was too large */
  PARSE_ERROR,
  /** The camera reported a state this library does not know */
  UNKNOWN
};

/** Result of loading a Limelight::PipelineConfiguration or
 *  Limelight::FieldMap. */
enum class LoadStatus {
  /** Loaded and within the size cap */
  OK,
  /** No contents or no file name were given (empty) */
  NO_CONTENTS,
  /** The deploy-folder file could not be read */
  READ_FAILED,
  /** The contents exceed the size cap */
  TOO_LARGE
};

inline PipelineConfigurationOverrideState
ParsePipelineConfigurationOverrideState(std::string_view s) {
  if (s == "off")
    return PipelineConfigurationOverrideState::OFF;
  if (s == "active")
    return PipelineConfigurationOverrideState::ACTIVE;
  if (s == "forcedOff")
    return PipelineConfigurationOverrideState::FORCED_OFF;
  if (s == "noString")
    return PipelineConfigurationOverrideState::NO_STRING;
  if (s == "parseError")
    return PipelineConfigurationOverrideState::PARSE_ERROR;
  return PipelineConfigurationOverrideState::UNKNOWN;
}

inline SharedMapState ParseSharedMapState(std::string_view s) {
  if (s == "off")
    return SharedMapState::OFF;
  if (s == "active")
    return SharedMapState::ACTIVE;
  if (s == "parseError")
    return SharedMapState::PARSE_ERROR;
  return SharedMapState::UNKNOWN;
}

/**
 * The robot pose estimate outputs. Names use the form {algorithm}_{origin}.
 *
 * MegaTag1 (MT1) computes the full robot pose from tag geometry only.
 * MegaTag2 (MT2) also uses the robot orientation from
 * Limelight::SetRobotOrientation() or Limelight::SetSharedRobotOrientation().
 * MT2 is usually more stable. MT2 requires you to publish the orientation
 * every loop.
 */
enum class PoseEstimateType {
  /** MegaTag1, WPILib blue-alliance-corner origin (botpose_wpiblue) */
  MT1_WPIBLUE,
  /** MegaTag1, WPILib red-alliance-corner origin (botpose_wpired) */
  MT1_WPIRED,
  /** MegaTag2, WPILib blue-alliance-corner origin (botpose_orb_wpiblue) */
  MT2_WPIBLUE,
  /** MegaTag2, WPILib red-alliance-corner origin (botpose_orb_wpired) */
  MT2_WPIRED
};

/** Number of PoseEstimateType values. */
inline constexpr size_t kPoseEstimateTypeCount = 4;

/**
 * Fields common to all image-space targets.
 */
struct LimelightTarget {
  /** Horizontal offset from crosshair to target in degrees */
  double txDegrees = 0;
  /** Vertical offset from crosshair to target in degrees */
  double tyDegrees = 0;
  /** Horizontal offset from principal pixel to target in degrees */
  double txDegreesNoCrosshair = 0;
  /** Vertical offset from principal pixel to target in degrees */
  double tyDegreesNoCrosshair = 0;
  /** Target x position in pixels */
  double txPixels = 0;
  /** Target y position in pixels */
  double tyPixels = 0;
  /** Target area (percentage of image, 0-100) */
  double targetAreaPercent = 0;
  /** Target corners as [x, y] pixel pairs. Empty unless "send corners" is
   *  enabled */
  std::vector<std::vector<double>> corners;
};

/**
 * A color/retroreflective target.
 */
struct RetroTarget : public LimelightTarget {
  /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> cameraPoseTargetSpace;
  /** Target pose in camera space */
  std::vector<double> targetPoseCameraSpace;
  /** Target pose in robot space */
  std::vector<double> targetPoseRobotSpace;
  /** Robot pose in target space */
  std::vector<double> robotPoseTargetSpace;
  /** Robot pose in field space */
  std::vector<double> robotPoseFieldSpace;

  /** @return Camera pose in target space as a Pose3d */
  wpi::math::Pose3d GetCameraPose_TargetSpace() const {
    return ToPose3D(cameraPoseTargetSpace);
  }

  /** @return Target pose in camera space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_CameraSpace() const {
    return ToPose3D(targetPoseCameraSpace);
  }

  /** @return Target pose in robot space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_RobotSpace() const {
    return ToPose3D(targetPoseRobotSpace);
  }

  /** @return Robot pose in target space as a Pose3d */
  wpi::math::Pose3d GetRobotPose_TargetSpace() const {
    return ToPose3D(robotPoseTargetSpace);
  }

  /** @return Robot pose in field space as a Pose3d */
  wpi::math::Pose3d GetRobotPose_FieldSpace() const {
    return ToPose3D(robotPoseFieldSpace);
  }
};

/**
 * An AprilTag/fiducial target.
 */
struct FiducialTarget : public LimelightTarget {
  /** Fiducial/AprilTag ID */
  int fiducialId = 0;
  /** Fiducial family (e.g. "36h11") */
  std::string family;
  /** Target skew values */
  std::vector<double> skew;
  /** Pose ambiguity. Lower is better. Values above 0.7 are usually not
   *  reliable */
  double ambiguity = 0;
  /** True if this tag contributed to pose estimation. The tag matched the
   *  field map and the ID filters did not exclude it. */
  bool fielded = false;

  /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> cameraPoseTargetSpace;
  /** Target pose in camera space */
  std::vector<double> targetPoseCameraSpace;
  /** Target pose in robot space */
  std::vector<double> targetPoseRobotSpace;
  /** Robot pose in target space */
  std::vector<double> robotPoseTargetSpace;
  /** Robot pose in field space (MegaTag1, this tag only) */
  std::vector<double> robotPoseFieldSpace;
  /** Robot pose in field space (MegaTag2, this tag only) */
  std::vector<double> robotPoseFieldSpaceMT2;

  /** @return 3D distance from the camera to this tag in meters */
  double GetDistanceToCamera() const {
    return Distance3d(targetPoseCameraSpace);
  }

  /** @return 3D distance from the robot center to this tag in meters */
  double GetDistanceToRobot() const { return Distance3d(targetPoseRobotSpace); }

  /** @return Camera pose in target space as a Pose3d */
  wpi::math::Pose3d GetCameraPose_TargetSpace() const {
    return ToPose3D(cameraPoseTargetSpace);
  }

  /** @return Target pose in camera space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_CameraSpace() const {
    return ToPose3D(targetPoseCameraSpace);
  }

  /** @return Target pose in robot space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_RobotSpace() const {
    return ToPose3D(targetPoseRobotSpace);
  }

  /** @return Robot pose in target space as a Pose3d */
  wpi::math::Pose3d GetRobotPose_TargetSpace() const {
    return ToPose3D(robotPoseTargetSpace);
  }

  /** @return MegaTag1 robot pose in field space from this tag only, as a Pose3d
   */
  wpi::math::Pose3d GetRobotPose_FieldSpace() const {
    return ToPose3D(robotPoseFieldSpace);
  }

  /** @return MegaTag2 robot pose in field space from this tag only, as a Pose3d
   */
  wpi::math::Pose3d GetRobotPose_FieldSpace_MT2() const {
    return ToPose3D(robotPoseFieldSpaceMT2);
  }

 private:
  static double Distance3d(std::span<const double> pose) {
    if (pose.size() < 3) {
      return 0;
    }
    return std::sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
  }
};

/**
 * A neural detector target.
 */
struct DetectorTarget : public LimelightTarget {
  /** Class index from the class list */
  int classId = -1;
  /** Human-readable class name */
  std::string className;
  /** Detection confidence (0-1) */
  double confidence = 0;
  /** Tracking ID for object counting pipelines. -1 means no tracking */
  int trackId = -1;
};

/**
 * A neural classifier result.
 */
struct ClassifierTarget {
  /** Class index from the class list */
  int classId = -1;
  /** Human-readable class name */
  std::string className;
  /** Classification confidence (0-1) */
  double confidence = 0;
};

/**
 * A barcode/QR code target.
 */
struct BarcodeTarget : public LimelightTarget {
  /** Barcode family (e.g. "QRCODE") */
  std::string family;
  /** Decoded barcode data string */
  std::string data;
};

/**
 * A per-class object counter result from the neural counter pipeline.
 */
struct CounterTarget {
  int classId = -1;
  std::string className;
  int count = 0;
};

/**
 * An OCR (text recognition) result.
 */
struct OCRTarget {
  bool valid = false;
  int regionId = -1;
  /** Recognized text */
  std::string text;
  double confidence = 0;
  double numericValue = 0;
  bool hasNumericValue = false;
  int digitCount = 0;
  std::string unit;
  bool isCounter = false;
  double ratePerMin = 0;
  int alarmState = 0;
  std::string alarmString;
  /** Bounding box [x, y, width, height] in pixels */
  std::vector<double> boundingBox;
  double processingTimeMs = 0;
};

/**
 * Live camera intrinsics for the running pipeline. The camera matrix is scaled
 * to the current processing resolution.
 */
struct CameraIntrinsics {
  /** True when the camera uses a user-uploaded calibration file. False when
   *  the camera uses a default calibration */
  bool customCalibration = false;
  /** Horizontal field of view in degrees */
  double hfovDegrees = 0;
  /** Vertical field of view in degrees */
  double vfovDegrees = 0;
  /** Processing-resolution width in pixels. The camera matrix is scaled to
   *  this width */
  double resolutionWidthPixels = 0;
  /** Processing-resolution height in pixels */
  double resolutionHeightPixels = 0;
  /** Row-major 3x3 camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1], scaled to
   *  the processing resolution. Empty until the camera publishes */
  std::vector<double> cameraMatrix;
  /** OpenCV distortion coefficients. Empty until the camera publishes */
  std::vector<double> distortionCoefficients;
};

/**
 * The state of the camera's internal IMU.
 */
struct IMUData {
  /** Orientation quaternion [w, x, y, z] */
  std::vector<double> quaternion{1, 0, 0, 0};
  /** Final fused robot yaw in degrees. MegaTag2 uses this value. Includes
   *  yawOffsetDegrees */
  double robotYawDegrees = 0;
  /** Yaw offset in degrees currently applied to the internal IMU */
  double yawOffsetDegrees = 0;
  /** Internal IMU roll in degrees */
  double rollDegrees = 0;
  /** Internal IMU pitch in degrees */
  double pitchDegrees = 0;
  /** Raw internal IMU yaw in degrees, before yawOffsetDegrees. See
   *  robotYawDegrees */
  double yawDegrees = 0;
  /** Gyro angular velocity about X in degrees per second */
  double gyroXDegreesPerSecond = 0;
  /** Gyro angular velocity about Y in degrees per second */
  double gyroYDegreesPerSecond = 0;
  /** Gyro angular velocity about Z in degrees per second */
  double gyroZDegreesPerSecond = 0;
  /** Accelerometer X (forward) axis */
  double accelX = 0;
  /** Accelerometer Y (left) axis */
  double accelY = 0;
  /** Accelerometer Z (up) axis */
  double accelZ = 0;
};

/**
 * Camera hardware and system stats.
 */
struct HardwareData {
  /** CPU temperature in degrees Celsius */
  double cpuTempCelsius = 0;
  /** CPU usage percentage */
  double cpuUsagePercent = 0;
  /** RAM usage percentage */
  double ramUsagePercent = 0;
  /** Free disk space in MB */
  int64_t diskFreeMB = 0;
  /** Total disk space in MB */
  int64_t diskTotalMB = 0;
  /** Camera sensor ID */
  std::string cameraId;
  /** True if an AI accelerator is present */
  bool accelPresent = false;
  /** AI accelerator type */
  std::string accelType;
  /** AI accelerator temperature in degrees Celsius */
  double accelTempCelsius = 0;
  /** AI accelerator power draw in watts */
  double accelPowerWatts = 0;
  /** True if the AI accelerator is throttling */
  bool accelThrottling = false;
};

/**
 * Rewind buffer state.
 */
struct RewindData {
  bool enabled = false;
  double storedSeconds = 0;
  int64_t frameCount = 0;
  double bufferUsage = 0;
  bool flushing = false;
  /** Main-thread latency penalty in microseconds */
  int latencyPenaltyMicros = 0;
};

/**
 * IMU sources for the robot yaw that MegaTag2 uses. The external orientation
 * comes from Limelight::SetRobotOrientation() or
 * Limelight::SetSharedRobotOrientation(). Only cameras with a built-in IMU use
 * this.
 */
enum class IMUMode {
  /** Use the external orientation directly */
  EXTERNAL = 0,
  /** Uses the external orientation. Continuously seeds the internal IMU with
   *  it */
  EXTERNAL_SEED_INTERNAL = 1,
  /** Use the internal IMU only */
  INTERNAL = 2,
  /** Internal IMU with a complementary filter. The filter converges on the
   *  MegaTag1 yaw */
  INTERNAL_MT1_ASSIST = 3,
  /** Internal IMU with a complementary filter. The filter converges on the
   *  external orientation */
  INTERNAL_EXTERNAL_ASSIST = 4,
  /** The camera reported a mode this library does not know (never valid to
   *  set) */
  UNKNOWN = -1
};

inline IMUMode IMUModeFromNT(int value) {
  switch (value) {
    case 0:
      return IMUMode::EXTERNAL;
    case 1:
      return IMUMode::EXTERNAL_SEED_INTERNAL;
    case 2:
      return IMUMode::INTERNAL;
    case 3:
      return IMUMode::INTERNAL_MT1_ASSIST;
    case 4:
      return IMUMode::INTERNAL_EXTERNAL_ASSIST;
    default:
      return IMUMode::UNKNOWN;
  }
}

/**
 * The robot-orientation state the camera is using for MegaTag2.
 */
struct RobotOrientationData {
  /** Active IMU mode (IMUMode::UNKNOWN before the first frame) */
  IMUMode imuMode = IMUMode::UNKNOWN;
  /** IMU assist alpha */
  double alpha = 0;
  /** Last interpolated robot yaw used for botpose, in degrees */
  double interpolatedYawDegrees = 0;
};

/**
 * The complete decoded results for one frame. All fields come from one
 * MessagePack payload.
 */
class LimelightResults {
 public:
  /** True if at least one target is valid in this frame */
  bool valid = false;
  /** Legacy timestamp in milliseconds since vision-process boot */
  double timestampMillis = 0;
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
  double targetingLatencyMillis = 0;
  /** Capture latency in milliseconds */
  double captureLatencyMillis = 0;
  /** Active pipeline index */
  double pipelineIndex = -1;
  /** Active pipeline type (e.g. "pipe_fiducial") */
  std::string pipelineType;
  /** Focus metric for focus-assist */
  double focusMetric = 0;

  /** Camera-reported MegaTag1 standard deviations [x, y, z, roll, pitch,
   *  yaw]. The camera averages these over a long window. Use for telemetry
   *  only. Do not use for fusion */
  std::vector<double> stdDevsMT1;
  /** Camera-reported MegaTag2 standard deviations [x, y, z, roll, pitch,
   *  yaw]. The camera averages these over a long window. Use for telemetry
   *  only. Do not use for fusion */
  std::vector<double> stdDevsMT2;

  /** MegaTag1 robot pose in field space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> robotPoseMT1;
  /** MegaTag1 robot pose, WPILib red-alliance coordinate system */
  std::vector<double> robotPoseMT1WpiRed;
  /** MegaTag1 robot pose, WPILib blue-alliance coordinate system */
  std::vector<double> robotPoseMT1WpiBlue;
  /** MegaTag2 robot pose in field space */
  std::vector<double> robotPoseMT2;
  /** MegaTag2 robot pose, WPILib red-alliance coordinate system */
  std::vector<double> robotPoseMT2WpiRed;
  /** MegaTag2 robot pose, WPILib blue-alliance coordinate system */
  std::vector<double> robotPoseMT2WpiBlue;

  /** Number of AprilTags that contribute to the robot pose estimates */
  int reportedTagCount = 0;
  /** Max distance between contributing tags in meters */
  double tagSpanMeters = 0;
  /** Average distance to contributing tags in meters */
  double avgTagDistanceMeters = 0;
  /** Average area of contributing tags (percentage of image) */
  double avgTagAreaPercent = 0;

  /** Final camera pose in robot space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> cameraPoseRobotSpace;
  /** Data set by a python snapscript via llpython */
  std::vector<double> pythonOutput;

  /** Horizontal offset from crosshair to primary target in degrees */
  double txDegrees = 0;
  /** Vertical offset from crosshair to primary target in degrees */
  double tyDegrees = 0;
  /** Horizontal offset from principal pixel to primary target in degrees */
  double txDegreesNoCrosshair = 0;
  /** Vertical offset from principal pixel to primary target in degrees */
  double tyDegreesNoCrosshair = 0;
  /** Primary target area (percentage of image, 0-100) */
  double targetAreaPercent = 0;
  /** 3D distance from camera to primary target in meters */
  double targetDistanceMeters = 0;

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

  /** Time to decode this envelope on the robot, in milliseconds. This is
   *  library overhead for logging. It is not part of the camera latency. */
  double parseLatencyMillis = 0;
  /** Local NetworkTables time when this frame arrived, in seconds. 0 if this
   *  object did not come from the network */
  double receiveTimestampSeconds = 0;
  /** Non-empty if the envelope failed to decode */
  std::string error;

  /**
   * @return Seconds since this frame arrived. If this value keeps growing, the
   *         camera has stopped sending frames. Possible causes: disconnected,
   *         stopped, or renamed.
   */
  double GetAgeSeconds() const {
    return wpi::nt::Now() / NT_TICKS_PER_SECOND - receiveTimestampSeconds;
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

constexpr bool UsesMT2(PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
    case PoseEstimateType::MT1_WPIRED:
      return false;
    case PoseEstimateType::MT2_WPIBLUE:
    case PoseEstimateType::MT2_WPIRED:
    default:
      return true;
  }
}

constexpr bool CenteredOrigin(PoseEstimateType) {
  return false;
}

constexpr std::string_view PoseEstimateTypeName(PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
      return "MT1_WPIBLUE";
    case PoseEstimateType::MT1_WPIRED:
      return "MT1_WPIRED";
    case PoseEstimateType::MT2_WPIBLUE:
      return "MT2_WPIBLUE";
    case PoseEstimateType::MT2_WPIRED:
    default:
      return "MT2_WPIRED";
  }
}

}  // namespace detail

inline wpi::math::Pose3d LimelightResults::GetRobotPose(
    PoseEstimateType type) const {
  return ToPose3D(detail::PoseArray(*this, type));
}

class PoseEstimate;

/**
 * A reusable configuration object for rejection filters and standard
 * deviation scaling. Start from a factory (DefaultMT1(), DefaultMT2(),
 * NoFiltering()) or from a new instance. Chain the With* methods. Attach the
 * result with Limelight::WithPoseEstimateConfig_MT1() or
 * Limelight::WithPoseEstimateConfig_MT2().
 *
 * A new instance accepts every structurally valid estimate that has at least
 * one fielded tag. It uses the same standard deviation model as DefaultMT2().
 *
 * @code
 * limelight::Limelight camera{"limelight"};
 * camera
 *     .WithPoseEstimateConfig_MT1(limelight::PoseEstimateConfig::DefaultMT1()
 *         .WithMinTagCount(2)
 *         .WithFieldBounds(17.55, 8.05))
 *     .WithPoseEstimateConfig_MT2(limelight::PoseEstimateConfig::DefaultMT2()
 *         .WithMaxAvgTagDistance(4.0)
 *         .WithFieldBounds(17.55, 8.05));
 * @endcode
 */
class PoseEstimateConfig {
 public:
  /** Rejection flag: too few contributing tags */
  static constexpr int REJECT_TAG_COUNT = 1 << 0;
  /** Rejection flag: single-tag ambiguity above the configured maximum */
  static constexpr int REJECT_AMBIGUITY = 1 << 1;
  /** Rejection flag: tag distance above a configured maximum */
  static constexpr int REJECT_TAG_DISTANCE = 1 << 2;
  /** Rejection flag: average tag area below the configured minimum */
  static constexpr int REJECT_TAG_AREA = 1 << 3;
  /** Rejection flag: pose outside the configured field bounds */
  static constexpr int REJECT_FIELD_BOUNDS = 1 << 4;

  /** Rejection flag set by the library: the source envelope had a decode
   *  error */
  static constexpr int REJECT_DECODE_ERROR = 1 << 5;

  /** Rejection flag set by the library: a value needed for fusion was NaN or
   *  infinite */
  static constexpr int REJECT_NONFINITE = 1 << 6;

  /** Rejection flag set by the library: no usable capture timestamp. For
   *  example, an envelope decoded with Limelight::Decode(bytes) has no receive
   *  time */
  static constexpr int REJECT_NO_TIMESTAMP = 1 << 7;

  /** Rejection flag set by the library: the pose array was absent or too
   *  short, it was the all-zero value the camera sends when it has no
   *  estimate, or the estimate type is not known */
  static constexpr int REJECT_MISSING_POSE = 1 << 8;

  /** Rejection flag set by the library: tags contributed but the reported tag
   *  distance was zero or negative */
  static constexpr int REJECT_BAD_METADATA = 1 << 9;

  /** Rejection flag set by the library: fiducials were visible but none
   *  contributed to pose estimation */
  static constexpr int REJECT_NO_FIELDED_TAGS = 1 << 11;

  /** A standard deviation so large that a pose estimator ignores the
   *  measurement */
  static constexpr double UNTRUSTED = 9999999;
  /** Smallest heading standard deviation this configuration produces, in
   *  radians */
  static constexpr double MIN_THETA_STD_DEV = 0.01;

  /**
   * Requires at least this many contributing tags. The count uses the
   * FiducialTarget::fielded flags. At least one fielded tag is always required.
   * Use WithMinTagCount(2) to ignore single-tag MT1 estimates.
   */
  PoseEstimateConfig& WithMinTagCount(int minTagCount) {
    m_minTagCount = std::max(1, minTagCount);
    return *this;
  }

  /**
   * Rejects single-tag estimates when the ambiguity of the fielded tag is above
   * this value (0-1). A value of 1 disables this check.
   */
  PoseEstimateConfig& WithMaxSingleTagAmbiguity(double maxSingleTagAmbiguity) {
    m_maxSingleTagAmbiguity = detail::ClampArg(maxSingleTagAmbiguity, 0, 1, 1);
    return *this;
  }

  /**
   * Rejects single-tag estimates when the tag is farther than this distance in
   * meters. Single-tag estimates lose accuracy with distance faster than
   * multi-tag estimates. Set this value tighter than WithMaxAvgTagDistance().
   * For example, allow multi-tag estimates to 6 m and single-tag estimates to
   * 2.5 m. Applies only when exactly one tag contributes. 0 disables this
   * check.
   */
  PoseEstimateConfig& WithMaxSingleTagDistance(
      double maxSingleTagDistanceMeters) {
    m_maxSingleTagDistance = detail::ClampArg(
        maxSingleTagDistanceMeters, 0, std::numeric_limits<double>::max());
    return *this;
  }

  /** Rejects estimates when the average tag distance is above this value in
   *  meters. A value of 0 disables this check. */
  PoseEstimateConfig& WithMaxAvgTagDistance(double maxAvgTagDistanceMeters) {
    m_maxAvgTagDistance = detail::ClampArg(maxAvgTagDistanceMeters, 0,
                                           std::numeric_limits<double>::max());
    return *this;
  }

  /** Rejects estimates when the average tag area (percentage of image) is
   *  below this value. A value of 0 disables this check. */
  PoseEstimateConfig& WithMinAvgTagArea(double minAvgTagArea) {
    m_minAvgTagArea = detail::ClampArg(minAvgTagArea, 0, 100);
    return *this;
  }

  /**
   * Rejects estimates outside the field. The bounds depend on the coordinate
   * origin of the estimate. Corner origins (wpiblue, wpired) span [0, length] x
   * [0, width]. Centered origins span +/-length/2 x +/-width/2.
   *
   * A zero length or width disables this check.
   *
   * @param fieldLengthMeters Field length (x extent) in meters
   * @param fieldWidthMeters Field width (y extent) in meters
   */
  PoseEstimateConfig& WithFieldBounds(double fieldLengthMeters,
                                      double fieldWidthMeters) {
    m_fieldLengthMeters = detail::ClampArg(fieldLengthMeters, 0,
                                           std::numeric_limits<double>::max());
    m_fieldWidthMeters = detail::ClampArg(fieldWidthMeters, 0,
                                          std::numeric_limits<double>::max());
    return *this;
  }

  /**
   * Sets the margin for the field bounds check. The default is 0.5. A positive
   * margin accepts poses up to that distance outside the field. This allows
   * small localization errors at the walls. A negative margin requires poses
   * at least that distance inside the field walls.
   *
   * @param marginMeters Margin in meters. Negative values are an inset. NaN
   *        becomes 0. Infinite values clamp to the largest finite margin
   */
  PoseEstimateConfig& WithFieldBoundsMargin(double marginMeters) {
    m_fieldBoundsMarginMeters =
        std::isnan(marginMeters)
            ? 0
            : std::clamp(marginMeters, -std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
    return *this;
  }

  /**
   * The MegaTag1 defaults. The library uses these when you do not provide an
   * MT1 configuration. Single-tag estimates must pass the ambiguity gate (0.7)
   * and the distance gate (3 m). MT1 computes the full pose from tag geometry
   * only. Heading is not fused. XY standard deviation = 0.5 * tagDistanceMeters
   * / sqrt(fieldedTagCount), with a minimum of 0.0001 m. These are untuned
   * initial values. Tune them on your robot. Start from this configuration and
   * chain changes: PoseEstimateConfig::DefaultMT1().WithFieldBounds(...).
   */
  static PoseEstimateConfig DefaultMT1() {
    PoseEstimateConfig config;
    config.WithMaxSingleTagAmbiguity(0.7)
        .WithMaxSingleTagDistance(3.0)
        .WithStdDevXY(0.5);
    return config;
  }

  /**
   * The MegaTag2 defaults. The library uses these when you do not provide an
   * MT2 configuration. There are no acceptance gates. Gyro-fused MT2 is robust,
   * and structural validation still applies. Heading is not fused. XY standard
   * deviation = 0.3 * tagDistanceMeters / sqrt(fieldedTagCount), with a minimum
   * of 0.0001 m. These are untuned initial values. Tune them on your robot.
   * Start from this configuration and chain changes.
   */
  static PoseEstimateConfig DefaultMT2() {
    PoseEstimateConfig config;
    config.WithStdDevXY(0.3);
    return config;
  }

  /**
   * Accepts every structurally valid pose. Uses a fixed 0.5 m XY standard
   * deviation. Heading is untrusted.
   */
  static PoseEstimateConfig NoFiltering() {
    return NoFiltering(0.5, UNTRUSTED);
  }

  /**
   * Accepts every structurally valid pose. Uses the fixed standard deviations
   * that you select.
   *
   * @param xyStdDevMeters Fixed XY standard deviation in meters
   * @param thetaStdDevRadians Fixed heading standard deviation in radians. Pass
   *        UNTRUSTED to exclude vision heading from fusion
   */
  static PoseEstimateConfig NoFiltering(double xyStdDevMeters,
                                        double thetaStdDevRadians) {
    PoseEstimateConfig config;
    config.WithStdDevXY(xyStdDevMeters)
        .WithStdDevTheta(thetaStdDevRadians)
        .WithStdDevDistanceScaling(0)
        .WithStdDevTagCountDivision(0);
    return config;
  }

  /** Sets the base XY standard deviation in meters. When distance scaling is
   *  active, this value is per meter of average tag distance. */
  PoseEstimateConfig& WithStdDevXY(double baseMeters) {
    m_xyStdDev =
        detail::ClampArg(baseMeters, kMinXYStdDev, UNTRUSTED, UNTRUSTED);
    return *this;
  }

  /** Sets the base XY standard deviation. Clamps the computed value to
   *  [min, max] meters. */
  PoseEstimateConfig& WithStdDevXY(double baseMeters, double minMeters,
                                   double maxMeters) {
    WithStdDevXY(baseMeters);
    double lo =
        detail::ClampArg(minMeters, kMinXYStdDev, UNTRUSTED, kMinXYStdDev);
    double hi = detail::ClampArg(maxMeters, kMinXYStdDev, UNTRUSTED, UNTRUSTED);
    m_minXYStdDev = std::min(lo, hi);
    m_maxXYStdDev = std::max(lo, hi);
    return *this;
  }

  /** Sets the base heading standard deviation in radians. The default excludes
   *  vision heading from fusion. Lower this value only to fuse vision heading.
   *  The minimum is MIN_THETA_STD_DEV radians. */
  PoseEstimateConfig& WithStdDevTheta(double baseRadians) {
    m_thetaStdDev =
        detail::ClampArg(baseRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    return *this;
  }

  /** Sets the base heading standard deviation. Clamps the computed value to
   *  [min, max] radians. The minimum is MIN_THETA_STD_DEV radians. */
  PoseEstimateConfig& WithStdDevTheta(double baseRadians, double minRadians,
                                      double maxRadians) {
    WithStdDevTheta(baseRadians);
    double lo = detail::ClampArg(minRadians, MIN_THETA_STD_DEV, UNTRUSTED,
                                 MIN_THETA_STD_DEV);
    double hi =
        detail::ClampArg(maxRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    m_minThetaStdDev = std::min(lo, hi);
    m_maxThetaStdDev = std::max(lo, hi);
    return *this;
  }

  /**
   * Scales the standard deviations by avgTagDistance^exponent. 1 = linear
   * (default). 2 = quadratic. 0 = no distance scaling.
   */
  PoseEstimateConfig& WithStdDevDistanceScaling(double exponent) {
    m_distanceExponent = detail::ClampArg(exponent, 0, 10, 1);
    return *this;
  }

  /** Same as WithStdDevDistanceScaling(double). Clamps the tag distance to
   *  [min, max] meters before the exponent applies. Tags closer than min scale
   *  as if at min. Tags farther than max scale as if at max. */
  PoseEstimateConfig& WithStdDevDistanceScaling(double exponent,
                                                double minMeters,
                                                double maxMeters) {
    WithStdDevDistanceScaling(exponent);
    double lo =
        detail::ClampArg(minMeters, 0, std::numeric_limits<double>::max(), 0);
    double hi =
        detail::ClampArg(maxMeters, 0, std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
    m_minScalingDistance = std::min(lo, hi);
    m_maxScalingDistance = std::max(lo, hi);
    return *this;
  }

  /** Divides the standard deviations by fieldedTagCount^exponent. The default
   *  0.5 divides by the square root of the fielded tag count. 0 disables this.
   */
  PoseEstimateConfig& WithStdDevTagCountDivision(double exponent) {
    m_tagCountExponent = detail::ClampArg(exponent, 0, 10, 0.5);
    return *this;
  }

  /**
   * @return The rejection flags as readable text, for example
   *         "TAG_COUNT|AMBIGUITY". Empty if the estimate was accepted
   */
  static std::string DescribeRejection(int flags);

 private:
  friend class limelight::Limelight;

  void ApplyTo(PoseEstimate& estimate) const;
  int Evaluate(const PoseEstimate& estimate) const;
  wpi::util::array<double, 3> Compute(const PoseEstimate& estimate) const;

  static constexpr double Clamp(double value, double min, double max) {
    return std::max(min, std::min(max, value));
  }

  static constexpr double kMinXYStdDev = 0.0001;

  int m_minTagCount = 1;
  double m_maxSingleTagAmbiguity = 1.0;
  double m_maxSingleTagDistance = 0;
  double m_maxAvgTagDistance = 0;
  double m_minAvgTagArea = 0;
  double m_fieldLengthMeters = 0;
  double m_fieldWidthMeters = 0;
  double m_fieldBoundsMarginMeters = 0.5;

  // xy = clamp(xyStdDev * distance^distExp / fieldedTagCount^tagExp,
  //            minXY, maxXY)
  double m_xyStdDev = 0.3;
  double m_thetaStdDev = UNTRUSTED;
  double m_distanceExponent = 1.0;
  double m_minScalingDistance = 0.0;
  double m_maxScalingDistance = std::numeric_limits<double>::max();
  double m_tagCountExponent = 0.5;
  double m_minXYStdDev = kMinXYStdDev;
  double m_maxXYStdDev = std::numeric_limits<double>::max();
  double m_minThetaStdDev = MIN_THETA_STD_DEV;
  double m_maxThetaStdDev = std::numeric_limits<double>::max();
};

/**
 * A robot pose estimate with the metadata needed for pose-estimator fusion.
 */
class PoseEstimate {
 public:
  wpi::math::Pose2d pose;
  /**
   * Latency-compensated capture timestamp in the local NetworkTables timebase.
   * Pass this value directly to AddVisionMeasurement in WPILib.
   */
  wpi::units::second_t timestampSeconds{0.0};
  /** Total latency (capture + targeting) in milliseconds */
  double latencyMillis = 0;
  /** Camera-reported botpose tag count, telemetry only */
  int reportedTagCount = 0;
  /** Number of fielded tags. Used for validity, filtering, and standard
   *  deviation scaling */
  int fieldedTagCount = 0;
  /** Max distance between contributing tags in meters */
  double tagSpanMeters = 0;
  /** Average distance to contributing tags in meters */
  double avgTagDistanceMeters = 0;
  /** Average area of contributing tags (percentage of image) */
  double avgTagAreaPercent = 0;
  /**
   * Fusion-ready standard deviations [x meters, y meters, theta radians]. The
   * PoseEstimateConfig of this camera computes them from tag distance and tag
   * count. Pass this value as the third argument to AddVisionMeasurement.
   */
  wpi::util::array<double, 3> stdDevs{PoseEstimateConfig::UNTRUSTED,
                                      PoseEstimateConfig::UNTRUSTED,
                                      PoseEstimateConfig::UNTRUSTED};
  /**
   * Standard deviations [x, y, z, roll, pitch, yaw] reported by the camera. The
   * camera averages them over a long window. Use them for telemetry only. Do
   * not give them to a pose estimator. Use stdDevs instead.
   */
  std::vector<double> reportedStdDevs;
  /** The fiducials visible in this frame */
  std::vector<FiducialTarget> rawFiducials;
  /** Which pose estimate output this is (origin + algorithm) */
  PoseEstimateType type = PoseEstimateType::MT1_WPIBLUE;
  /**
   * The frame index of the source camera. Estimates from the same camera with
   * the same index came from one frame. Do not fuse more than one pose type
   * from one frame. The index can reset when the camera restarts.
   */
  int64_t frameIndex = -1;
  /**
   * OR of the PoseEstimateConfig REJECT_* flags for every check this estimate
   * failed. 0 = accepted. Decode with PoseEstimateConfig::DescribeRejection.
   * The flags reflect the filter configured for the algorithm of this estimate.
   */
  int rejectionFlags = 0;

  /**
   * @return The visible fiducials that contributed to pose estimation. These
   *         have FiducialTarget::fielded set
   */
  std::vector<FiducialTarget> GetFieldedFiducials() const {
    std::vector<FiducialTarget> out;
    for (const FiducialTarget& fiducial : rawFiducials) {
      if (fiducial.fielded) {
        out.push_back(fiducial);
      }
    }
    return out;
  }

  /** @return True if this estimate was produced by the MegaTag2 algorithm */
  bool IsMT2() const { return detail::UsesMT2(type); }

  /** @return True if at least one fielded tag produced this estimate and no
   *          check rejected it. This does not show current camera health. It
   *          does not show whether the frame was already read. */
  bool IsValid() const { return fieldedTagCount > 0 && rejectionFlags == 0; }

  std::string ToString() const {
    std::string rejection;
    if (rejectionFlags != 0) {
      rejection =
          " REJECTED:" + PoseEstimateConfig::DescribeRejection(rejectionFlags);
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "PoseEstimate(%.*s x=%.2f y=%.2f deg=%.1f fieldedTags=%d "
                  "reportedTags=%d avgDist=%.2f ts=%.3f%s)",
                  static_cast<int>(detail::PoseEstimateTypeName(type).size()),
                  detail::PoseEstimateTypeName(type).data(), pose.X().value(),
                  pose.Y().value(), pose.Rotation().Degrees().value(),
                  fieldedTagCount, reportedTagCount, avgTagDistanceMeters,
                  timestampSeconds.value(), rejection.c_str());
    return std::string{buf};
  }
};

namespace detail {

inline constexpr std::array<int, 11> kRejectionFlags = {
    PoseEstimateConfig::REJECT_TAG_COUNT,
    PoseEstimateConfig::REJECT_AMBIGUITY,
    PoseEstimateConfig::REJECT_TAG_DISTANCE,
    PoseEstimateConfig::REJECT_TAG_AREA,
    PoseEstimateConfig::REJECT_FIELD_BOUNDS,
    PoseEstimateConfig::REJECT_DECODE_ERROR,
    PoseEstimateConfig::REJECT_NONFINITE,
    PoseEstimateConfig::REJECT_NO_TIMESTAMP,
    PoseEstimateConfig::REJECT_MISSING_POSE,
    PoseEstimateConfig::REJECT_BAD_METADATA,
    PoseEstimateConfig::REJECT_NO_FIELDED_TAGS};

inline constexpr std::array<std::string_view, 11> kRejectionNames = {
    "TAG_COUNT",    "AMBIGUITY",    "TAG_DISTANCE",   "TAG_AREA",
    "FIELD_BOUNDS", "DECODE_ERROR", "NONFINITE",      "NO_TIMESTAMP",
    "MISSING_POSE", "BAD_METADATA", "NO_FIELDED_TAGS"};

}  // namespace detail

inline std::string PoseEstimateConfig::DescribeRejection(int flags) {
  if (flags == 0) {
    return "";
  }
  std::string out;
  for (size_t i = 0; i < detail::kRejectionFlags.size(); i++) {
    if ((flags & detail::kRejectionFlags[i]) != 0) {
      if (!out.empty()) {
        out.push_back('|');
      }
      out.append(detail::kRejectionNames[i]);
    }
  }
  return out;
}

inline void PoseEstimateConfig::ApplyTo(PoseEstimate& estimate) const {
  estimate.stdDevs = Compute(estimate);
  estimate.rejectionFlags |= Evaluate(estimate);
}

inline int PoseEstimateConfig::Evaluate(const PoseEstimate& estimate) const {
  int flags = 0;
  double maxFieldedAmbiguity = 0;
  for (const FiducialTarget& fiducial : estimate.rawFiducials) {
    if (fiducial.fielded) {
      maxFieldedAmbiguity = std::max(maxFieldedAmbiguity, fiducial.ambiguity);
    }
  }
  int contributingTags = estimate.fieldedTagCount;

  if (contributingTags < m_minTagCount) {
    flags |= REJECT_TAG_COUNT;
  }
  if (m_maxSingleTagAmbiguity < 1.0 && contributingTags == 1) {
    if (maxFieldedAmbiguity > m_maxSingleTagAmbiguity) {
      flags |= REJECT_AMBIGUITY;
    }
  }
  if (m_maxSingleTagDistance > 0 && contributingTags == 1 &&
      estimate.avgTagDistanceMeters > m_maxSingleTagDistance) {
    flags |= REJECT_TAG_DISTANCE;
  }
  if (m_maxAvgTagDistance > 0 &&
      estimate.avgTagDistanceMeters > m_maxAvgTagDistance) {
    flags |= REJECT_TAG_DISTANCE;
  }
  if (m_minAvgTagArea > 0 && estimate.avgTagAreaPercent < m_minAvgTagArea) {
    flags |= REJECT_TAG_AREA;
  }
  if (m_fieldLengthMeters > 0 && m_fieldWidthMeters > 0) {
    double x = estimate.pose.X().value();
    double y = estimate.pose.Y().value();
    bool outOfBounds;
    if (detail::CenteredOrigin(estimate.type)) {
      outOfBounds =
          std::abs(x) > m_fieldLengthMeters / 2 + m_fieldBoundsMarginMeters ||
          std::abs(y) > m_fieldWidthMeters / 2 + m_fieldBoundsMarginMeters;
    } else {
      outOfBounds = x < -m_fieldBoundsMarginMeters ||
                    x > m_fieldLengthMeters + m_fieldBoundsMarginMeters ||
                    y < -m_fieldBoundsMarginMeters ||
                    y > m_fieldWidthMeters + m_fieldBoundsMarginMeters;
    }
    if (outOfBounds) {
      flags |= REJECT_FIELD_BOUNDS;
    }
  }
  return flags;
}

inline wpi::util::array<double, 3> PoseEstimateConfig::Compute(
    const PoseEstimate& estimate) const {
  double scale = 1.0;
  if (m_distanceExponent != 0) {
    if (!std::isfinite(estimate.avgTagDistanceMeters) ||
        estimate.avgTagDistanceMeters <= 0) {
      return {UNTRUSTED, UNTRUSTED, UNTRUSTED};
    }
    double distance = Clamp(estimate.avgTagDistanceMeters, m_minScalingDistance,
                            m_maxScalingDistance);
    scale *= std::pow(distance, m_distanceExponent);
  }
  if (m_tagCountExponent != 0 && estimate.fieldedTagCount > 1) {
    scale /= std::pow(estimate.fieldedTagCount, m_tagCountExponent);
  }
  double xy = Clamp(m_xyStdDev * scale, m_minXYStdDev, m_maxXYStdDev);
  double theta =
      Clamp(m_thetaStdDev * scale, m_minThetaStdDev, m_maxThetaStdDev);
  return {xy, xy, theta};
}

namespace detail {

// Minimal, allocation-light MessagePack reader covering the full msgpack spec
// as produced by the camera (nil, bool, all int/uint widths, float32/64,
// str, bin, array, map, with skippable ext types). Nil is read leniently as
// 0 / "" / empty so absent optional data never throws. Unlike the Java
// implementation, which relies on array bounds exceptions for truncated
// envelopes, every read is explicitly bounds-checked.
class MsgPackReader {
 public:
  explicit MsgPackReader(std::span<const uint8_t> buf) : m_buf{buf} {}

  // Peeks at the next token without consuming it
  bool NextIsMap() const {
    RequireBytes(1);
    int b = m_buf[m_pos];
    return (b & 0xF0) == 0x80 || b == 0xDE || b == 0xDF;
  }

  // The envelope root must be a map. Anything else is a decode error.
  int ExpectMapHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xDE:
        return CheckedCount(U16(), 2, "map");
      case 0xDF:
        return CheckedCount(static_cast<int>(U32()), 2, "map");
      default:
        throw BadToken("map", b);
    }
  }

  // The typed readers below are lenient about the value type. A known key
  // whose value has an unexpected type is skipped and reads as its default
  // (0, 0.0, "", or empty) so one surprising field never fails the frame.
  int ReadMapHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil -> empty map
      case 0xDE:
        return CheckedCount(U16(), 2, "map");
      case 0xDF:
        return CheckedCount(static_cast<int>(U32()), 2, "map");
      default:
        SkipMismatched();
        return 0;
    }
  }

  // Rewinds to the token that did not match and skips the whole value
  void SkipMismatched() {
    m_pos--;
    SkipValue();
  }

  int ReadArrayHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x90) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil -> empty array
      case 0xDC:
        return CheckedCount(U16(), 1, "array");
      case 0xDD:
        return CheckedCount(static_cast<int>(U32()), 1, "array");
      default:
        SkipMismatched();
        return 0;
    }
  }

  std::string_view ReadString() {
    int b = U8();
    int len;
    if (b >= 0xA0 && b <= 0xBF) {
      len = b & 0x1F;
    } else {
      switch (b) {
        case 0xC0:
          return {};  // nil -> empty string
        case 0xD9:
          len = U8();
          break;
        case 0xDA:
          len = U16();
          break;
        case 0xDB:
          len = static_cast<int>(U32());
          break;
        default:
          SkipMismatched();
          return {};
      }
    }
    CheckedCount(len, 1, "string");
    std::string_view s{reinterpret_cast<const char*>(m_buf.data()) + m_pos,
                       static_cast<size_t>(len)};
    m_pos += static_cast<size_t>(len);
    return s;
  }

  int64_t ReadLong() {
    int b = U8();
    if (b <= 0x7F) {
      return b;  // positive fixint
    }
    if (b >= 0xE0) {
      return static_cast<int8_t>(b);  // negative fixint
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil
      case 0xC2:
        return 0;  // false
      case 0xC3:
        return 1;  // true
      case 0xCA:
        return FloatToInt64(Float32());
      case 0xCB:
        return FloatToInt64(Float64());
      case 0xCC:
        return U8();
      case 0xCD:
        return U16();
      case 0xCE:
        return static_cast<int64_t>(U32());
      case 0xCF:
        // uint64: values > INT64_MAX wrap, never produced in practice
        return static_cast<int64_t>(U64());
      case 0xD0:
        return static_cast<int8_t>(U8());
      case 0xD1:
        return static_cast<int16_t>(U16());
      case 0xD2:
        return static_cast<int32_t>(U32());
      case 0xD3:
        return static_cast<int64_t>(U64());
      default:
        SkipMismatched();
        return 0;
    }
  }

  double ReadDouble() {
    int b = U8();
    if (b <= 0x7F) {
      return b;
    }
    if (b >= 0xE0) {
      return static_cast<int8_t>(b);
    }
    switch (b) {
      case 0xC0:
        return 0.0;
      case 0xC2:
        return 0.0;
      case 0xC3:
        return 1.0;
      case 0xCA:
        return Float32();
      case 0xCB:
        return Float64();
      case 0xCC:
        return U8();
      case 0xCD:
        return U16();
      case 0xCE:
        return static_cast<double>(U32());
      case 0xCF:
        return static_cast<double>(U64());
      case 0xD0:
        return static_cast<int8_t>(U8());
      case 0xD1:
        return static_cast<int16_t>(U16());
      case 0xD2:
        return static_cast<int32_t>(U32());
      case 0xD3:
        return static_cast<double>(static_cast<int64_t>(U64()));
      default:
        SkipMismatched();
        return 0.0;
    }
  }

  std::vector<double> ReadDoubleArray() {
    int n = ReadArrayHeader();
    std::vector<double> out;
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      out.push_back(ReadDouble());
    }
    return out;
  }

  std::vector<std::vector<double>> ReadPointArray() {
    int n = ReadArrayHeader();
    std::vector<std::vector<double>> out;
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      out.push_back(ReadDoubleArray());
    }
    return out;
  }

  void SkipValue() {
    // A corrupt envelope of nested container headers would otherwise recurse
    // once per byte and overflow the stack past Decode()'s never-throws catch
    if (++m_skipDepth > kMaxSkipDepth) {
      m_skipDepth--;
      throw std::runtime_error("msgpack nested deeper than " +
                               std::to_string(kMaxSkipDepth));
    }
    try {
      SkipValueInner();
    } catch (...) {
      m_skipDepth--;
      throw;
    }
    m_skipDepth--;
  }

 private:
  static constexpr int kMaxSkipDepth = 64;

  int U8() {
    RequireBytes(1);
    return m_buf[m_pos++];
  }

  int U16() {
    RequireBytes(2);
    int v = (m_buf[m_pos] << 8) | m_buf[m_pos + 1];
    m_pos += 2;
    return v;
  }

  uint32_t U32() {
    RequireBytes(4);
    uint32_t v = (static_cast<uint32_t>(m_buf[m_pos]) << 24) |
                 (static_cast<uint32_t>(m_buf[m_pos + 1]) << 16) |
                 (static_cast<uint32_t>(m_buf[m_pos + 2]) << 8) |
                 static_cast<uint32_t>(m_buf[m_pos + 3]);
    m_pos += 4;
    return v;
  }

  uint64_t U64() {
    RequireBytes(8);
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
      v = (v << 8) | m_buf[m_pos + static_cast<size_t>(i)];
    }
    m_pos += 8;
    return v;
  }

  float Float32() {
    uint32_t bits = U32();
    float out;
    static_assert(sizeof(out) == sizeof(bits));
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  double Float64() {
    uint64_t bits = U64();
    double out;
    static_assert(sizeof(out) == sizeof(bits));
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  void RequireBytes(size_t count) const {
    if (count > m_buf.size() - m_pos) {
      throw std::runtime_error("truncated msgpack value near byte " +
                               std::to_string(m_pos));
    }
  }

  // Rejects corrupt length prefixes before they drive a huge allocation or a
  // silent cursor desync (n items need >= n * minBytesPerItem remaining bytes)
  int CheckedCount(int count, int minBytesPerItem, const char* what) const {
    if (count < 0 ||
        static_cast<uint64_t>(count) * static_cast<uint64_t>(minBytesPerItem) >
            m_buf.size() - m_pos) {
      throw std::runtime_error("corrupt " + std::string{what} + " length " +
                               std::to_string(count) + " near byte " +
                               std::to_string(m_pos));
    }
    return count;
  }

  void SkipValueInner() {
    int b = U8();
    if (b <= 0x7F || b >= 0xE0) {
      return;  // fixint
    }
    if (b >= 0xA0 && b <= 0xBF) {
      SkipBytes(b & 0x1F);  // fixstr
      return;
    }
    if (b >= 0x90 && b <= 0x9F) {
      SkipValues(b & 0x0F);  // fixarray
      return;
    }
    if (b >= 0x80 && b <= 0x8F) {
      SkipValues(2 * (b & 0x0F));  // fixmap
      return;
    }
    switch (b) {
      case 0xC0:
      case 0xC2:
      case 0xC3:
        return;
      case 0xCC:
      case 0xD0:
        SkipBytes(1);
        return;
      case 0xCD:
      case 0xD1:
        SkipBytes(2);
        return;
      case 0xCE:
      case 0xD2:
      case 0xCA:
        SkipBytes(4);
        return;
      case 0xCF:
      case 0xD3:
      case 0xCB:
        SkipBytes(8);
        return;
      case 0xD9:
      case 0xC4:
        SkipBytes(U8());
        return;
      case 0xDA:
      case 0xC5:
        SkipBytes(U16());
        return;
      case 0xDB:
      case 0xC6:
        SkipBytes(static_cast<int64_t>(U32()));
        return;
      case 0xD4:
        SkipBytes(2);
        return;  // fixext1
      case 0xD5:
        SkipBytes(3);
        return;  // fixext2
      case 0xD6:
        SkipBytes(5);
        return;  // fixext4
      case 0xD7:
        SkipBytes(9);
        return;  // fixext8
      case 0xD8:
        SkipBytes(17);
        return;  // fixext16
      case 0xC7:
        SkipBytes(1 + static_cast<int64_t>(U8()));
        return;
      case 0xC8:
        SkipBytes(1 + static_cast<int64_t>(U16()));
        return;
      case 0xC9:
        SkipBytes(1 + static_cast<int64_t>(U32()));
        return;
      case 0xDC:
        SkipValues(CheckedCount(U16(), 1, "skipped array"));
        return;
      case 0xDD:
        SkipValues(CheckedCount(static_cast<int>(U32()), 1, "skipped array"));
        return;
      case 0xDE:
        SkipValues(2 * CheckedCount(U16(), 2, "skipped map"));
        return;
      case 0xDF:
        SkipValues(2 * CheckedCount(static_cast<int>(U32()), 2, "skipped map"));
        return;
      default:
        throw BadToken("value", b);
    }
  }

  void SkipBytes(int64_t count) {
    if (count < 0 || static_cast<uint64_t>(count) > m_buf.size() - m_pos) {
      throw std::runtime_error("truncated msgpack value near byte " +
                               std::to_string(m_pos));
    }
    m_pos += static_cast<size_t>(count);
  }

  void SkipValues(int count) {
    for (int i = 0; i < count; i++) {
      SkipValue();
    }
  }

  // Matches Java's (long) cast: NaN reads as 0, out-of-range values saturate.
  // A raw static_cast would be undefined behavior for those inputs.
  static int64_t FloatToInt64(double v) {
    if (std::isnan(v)) {
      return 0;
    }
    if (v >= 9223372036854775808.0) {
      return std::numeric_limits<int64_t>::max();
    }
    if (v <= -9223372036854775808.0) {
      return std::numeric_limits<int64_t>::min();
    }
    return static_cast<int64_t>(v);
  }

  std::runtime_error BadToken(const char* expected, int token) const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "expected %s, got 0x%x at byte %zu",
                  expected, token, m_pos == 0 ? m_pos : m_pos - 1);
    return std::runtime_error(buf);
  }

  std::span<const uint8_t> m_buf;
  size_t m_pos = 0;
  int m_skipDepth = 0;
};

inline bool DecodeCommonTargetKey(LimelightTarget& target, std::string_view key,
                                  MsgPackReader& r) {
  if (key == "tx") {
    target.txDegrees = r.ReadDouble();
  } else if (key == "ty") {
    target.tyDegrees = r.ReadDouble();
  } else if (key == "tx_nocross") {
    target.txDegreesNoCrosshair = r.ReadDouble();
  } else if (key == "ty_nocross") {
    target.tyDegreesNoCrosshair = r.ReadDouble();
  } else if (key == "txp") {
    target.txPixels = r.ReadDouble();
  } else if (key == "typ") {
    target.tyPixels = r.ReadDouble();
  } else if (key == "ta") {
    target.targetAreaPercent = r.ReadDouble();
  } else if (key == "pts") {
    target.corners = r.ReadPointArray();
  } else {
    return false;
  }
  return true;
}

inline std::vector<RetroTarget> DecodeRetroTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<RetroTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    RetroTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (DecodeCommonTargetKey(t, key, r)) {
        continue;
      }
      if (key == "t6c_ts") {
        t.cameraPoseTargetSpace = r.ReadDoubleArray();
      } else if (key == "t6t_cs") {
        t.targetPoseCameraSpace = r.ReadDoubleArray();
      } else if (key == "t6t_rs") {
        t.targetPoseRobotSpace = r.ReadDoubleArray();
      } else if (key == "t6r_ts") {
        t.robotPoseTargetSpace = r.ReadDoubleArray();
      } else if (key == "t6r_fs") {
        t.robotPoseFieldSpace = r.ReadDoubleArray();
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<FiducialTarget> DecodeFiducialTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<FiducialTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    FiducialTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (DecodeCommonTargetKey(t, key, r)) {
        continue;
      }
      if (key == "fID") {
        t.fiducialId = static_cast<int>(r.ReadLong());
      } else if (key == "fam") {
        t.family = r.ReadString();
      } else if (key == "skew") {
        t.skew = r.ReadDoubleArray();
      } else if (key == "ambig") {
        t.ambiguity = r.ReadDouble();
      } else if (key == "fielded") {
        t.fielded = r.ReadLong() != 0;
      } else if (key == "t6c_ts") {
        t.cameraPoseTargetSpace = r.ReadDoubleArray();
      } else if (key == "t6t_cs") {
        t.targetPoseCameraSpace = r.ReadDoubleArray();
      } else if (key == "t6t_rs") {
        t.targetPoseRobotSpace = r.ReadDoubleArray();
      } else if (key == "t6r_ts") {
        t.robotPoseTargetSpace = r.ReadDoubleArray();
      } else if (key == "t6r_fs") {
        t.robotPoseFieldSpace = r.ReadDoubleArray();
      } else if (key == "t6r_fs_orb") {
        t.robotPoseFieldSpaceMT2 = r.ReadDoubleArray();
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<DetectorTarget> DecodeDetectorTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<DetectorTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    DetectorTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (DecodeCommonTargetKey(t, key, r)) {
        continue;
      }
      if (key == "classID") {
        t.classId = static_cast<int>(r.ReadLong());
      } else if (key == "class") {
        t.className = r.ReadString();
      } else if (key == "conf") {
        t.confidence = r.ReadDouble();
      } else if (key == "tID") {
        t.trackId = static_cast<int>(r.ReadLong());
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<ClassifierTarget> DecodeClassifierTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<ClassifierTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    ClassifierTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (key == "classID") {
        t.classId = static_cast<int>(r.ReadLong());
      } else if (key == "class") {
        t.className = r.ReadString();
      } else if (key == "conf") {
        t.confidence = r.ReadDouble();
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<BarcodeTarget> DecodeBarcodeTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<BarcodeTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    BarcodeTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (DecodeCommonTargetKey(t, key, r)) {
        continue;
      }
      if (key == "fam") {
        t.family = r.ReadString();
      } else if (key == "data") {
        t.data = r.ReadString();
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<CounterTarget> DecodeCounterTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<CounterTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    CounterTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (key == "classID") {
        t.classId = static_cast<int>(r.ReadLong());
      } else if (key == "class") {
        t.className = r.ReadString();
      } else if (key == "count") {
        t.count = static_cast<int>(r.ReadLong());
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline std::vector<OCRTarget> DecodeOCRTargets(MsgPackReader& r) {
  int n = r.ReadArrayHeader();
  std::vector<OCRTarget> out;
  for (int i = 0; i < n; i++) {
    if (!r.NextIsMap()) {
      r.SkipValue();  // not a target map, skip the element
      continue;
    }
    OCRTarget t;
    int fields = r.ReadMapHeader();
    for (int f = 0; f < fields; f++) {
      std::string_view key = r.ReadString();
      if (key == "valid") {
        t.valid = r.ReadLong() != 0;
      } else if (key == "regionId") {
        t.regionId = static_cast<int>(r.ReadLong());
      } else if (key == "text") {
        t.text = r.ReadString();
      } else if (key == "confidence") {
        t.confidence = r.ReadDouble();
      } else if (key == "numericValue") {
        t.numericValue = r.ReadDouble();
      } else if (key == "hasNumericValue") {
        t.hasNumericValue = r.ReadLong() != 0;
      } else if (key == "digitCount") {
        t.digitCount = static_cast<int>(r.ReadLong());
      } else if (key == "unit") {
        t.unit = r.ReadString();
      } else if (key == "isCounter") {
        t.isCounter = r.ReadLong() != 0;
      } else if (key == "ratePerMin") {
        t.ratePerMin = r.ReadDouble();
      } else if (key == "alarmState") {
        t.alarmState = static_cast<int>(r.ReadLong());
      } else if (key == "alarmString") {
        t.alarmString = r.ReadString();
      } else if (key == "bbox") {
        t.boundingBox = r.ReadDoubleArray();
      } else if (key == "processingTimeMs") {
        t.processingTimeMs = r.ReadDouble();
      } else {
        r.SkipValue();
      }
    }
    out.push_back(std::move(t));
  }
  return out;
}

inline void DecodeIMU(MsgPackReader& r, IMUData& imu) {
  int n = r.ReadMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "quat") {
      imu.quaternion = r.ReadDoubleArray();
    } else if (key == "yaw_offset") {
      imu.yawOffsetDegrees = r.ReadDouble();
    } else if (key == "data") {
      // The "yaw" key carries the same fused yaw as data[0]. The array is the
      // single source.
      std::vector<double> d = r.ReadDoubleArray();
      if (d.size() >= 10) {
        imu.robotYawDegrees = d[0];
        imu.rollDegrees = d[1];
        imu.pitchDegrees = d[2];
        imu.yawDegrees = d[3];
        imu.gyroXDegreesPerSecond = d[4];
        imu.gyroYDegreesPerSecond = d[5];
        imu.gyroZDegreesPerSecond = d[6];
        imu.accelX = d[7];
        imu.accelY = d[8];
        imu.accelZ = d[9];
      }
    } else {
      r.SkipValue();
    }
  }
}

inline void DecodeHardware(MsgPackReader& r, HardwareData& hw) {
  int n = r.ReadMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "temp") {
      hw.cpuTempCelsius = r.ReadDouble();
    } else if (key == "cpu") {
      hw.cpuUsagePercent = r.ReadDouble();
    } else if (key == "ram") {
      hw.ramUsagePercent = r.ReadDouble();
    } else if (key == "dfree") {
      hw.diskFreeMB = r.ReadLong();
    } else if (key == "dtot") {
      hw.diskTotalMB = r.ReadLong();
    } else if (key == "cid") {
      hw.cameraId = r.ReadString();
    } else if (key == "hailo") {
      int m = r.ReadMapHeader();
      for (int j = 0; j < m; j++) {
        std::string_view hkey = r.ReadString();
        if (hkey == "present") {
          hw.accelPresent = r.ReadLong() != 0;
        } else if (hkey == "type") {
          hw.accelType = r.ReadString();
        } else if (hkey == "temp") {
          hw.accelTempCelsius = r.ReadDouble();
        } else if (hkey == "power") {
          hw.accelPowerWatts = r.ReadDouble();
        } else if (hkey == "throttle") {
          hw.accelThrottling = r.ReadLong() != 0;
        } else {
          r.SkipValue();
        }
      }
    } else {
      r.SkipValue();
    }
  }
}

inline void DecodeRewind(MsgPackReader& r, RewindData& rw) {
  int n = r.ReadMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "enabled") {
      rw.enabled = r.ReadLong() != 0;
    } else if (key == "storedSeconds") {
      rw.storedSeconds = r.ReadDouble();
    } else if (key == "frameCount") {
      rw.frameCount = r.ReadLong();
    } else if (key == "bufferUsage") {
      rw.bufferUsage = r.ReadDouble();
    } else if (key == "flushing") {
      rw.flushing = r.ReadLong() != 0;
    } else if (key == "latpen") {
      rw.latencyPenaltyMicros = static_cast<int>(r.ReadLong());
    } else {
      r.SkipValue();
    }
  }
}

inline void DecodeIntrinsics(MsgPackReader& r, CameraIntrinsics& intrinsics) {
  int n = r.ReadMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "customcal") {
      intrinsics.customCalibration = r.ReadLong() != 0;
    } else if (key == "hfov") {
      intrinsics.hfovDegrees = r.ReadDouble();
    } else if (key == "vfov") {
      intrinsics.vfovDegrees = r.ReadDouble();
    } else if (key == "resw") {
      intrinsics.resolutionWidthPixels = r.ReadDouble();
    } else if (key == "resh") {
      intrinsics.resolutionHeightPixels = r.ReadDouble();
    } else if (key == "cameramatrix") {
      intrinsics.cameraMatrix = r.ReadDoubleArray();
    } else if (key == "distcoeffs") {
      intrinsics.distortionCoefficients = r.ReadDoubleArray();
    } else {
      r.SkipValue();
    }
  }
}

inline void DecodeBotOrientation(MsgPackReader& r, RobotOrientationData& bo) {
  int n = r.ReadMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "imumode") {
      bo.imuMode = IMUModeFromNT(static_cast<int>(r.ReadLong()));
    } else if (key == "alpha") {
      bo.alpha = r.ReadDouble();
    } else if (key == "interpbotyaw") {
      bo.interpolatedYawDegrees = r.ReadDouble();
    } else {
      r.SkipValue();
    }
  }
}

inline void DecodeResults(MsgPackReader& r, LimelightResults& out) {
  int n = r.ExpectMapHeader();
  for (int i = 0; i < n; i++) {
    std::string_view key = r.ReadString();
    if (key == "v") {
      out.valid = r.ReadLong() != 0;
    } else if (key == "ts") {
      out.timestampMillis = r.ReadDouble();
    } else if (key == "ts_us") {
      out.timestampMicros = r.ReadLong();
    } else if (key == "ts_sys") {
      out.systemTimestampMicros = r.ReadLong();
    } else if (key == "ts_nt") {
      out.ntTimestampMicros = r.ReadLong();
    } else if (key == "fidx") {
      out.frameIndex = r.ReadLong();
    } else if (key == "tl") {
      out.targetingLatencyMillis = r.ReadDouble();
    } else if (key == "cl") {
      out.captureLatencyMillis = r.ReadDouble();
    } else if (key == "pID") {
      out.pipelineIndex = r.ReadDouble();
    } else if (key == "pTYPE") {
      out.pipelineType = r.ReadString();
    } else if (key == "focus_metric") {
      out.focusMetric = r.ReadDouble();
    } else if (key == "stdev_mt1") {
      out.stdDevsMT1 = r.ReadDoubleArray();
    } else if (key == "stdev_mt2") {
      out.stdDevsMT2 = r.ReadDoubleArray();
    } else if (key == "botpose") {
      out.robotPoseMT1 = r.ReadDoubleArray();
    } else if (key == "botpose_wpired") {
      out.robotPoseMT1WpiRed = r.ReadDoubleArray();
    } else if (key == "botpose_wpiblue") {
      out.robotPoseMT1WpiBlue = r.ReadDoubleArray();
    } else if (key == "botpose_orb") {
      out.robotPoseMT2 = r.ReadDoubleArray();
    } else if (key == "botpose_orb_wpired") {
      out.robotPoseMT2WpiRed = r.ReadDoubleArray();
    } else if (key == "botpose_orb_wpiblue") {
      out.robotPoseMT2WpiBlue = r.ReadDoubleArray();
    } else if (key == "botpose_tagcount") {
      out.reportedTagCount = static_cast<int>(r.ReadLong());
    } else if (key == "botpose_span") {
      out.tagSpanMeters = r.ReadDouble();
    } else if (key == "botpose_avgdist") {
      out.avgTagDistanceMeters = r.ReadDouble();
    } else if (key == "botpose_avgarea") {
      out.avgTagAreaPercent = r.ReadDouble();
    } else if (key == "t6c_rs") {
      out.cameraPoseRobotSpace = r.ReadDoubleArray();
    } else if (key == "PythonOut") {
      out.pythonOutput = r.ReadDoubleArray();
    } else if (key == "tx") {
      out.txDegrees = r.ReadDouble();
    } else if (key == "ty") {
      out.tyDegrees = r.ReadDouble();
    } else if (key == "txnc") {
      out.txDegreesNoCrosshair = r.ReadDouble();
    } else if (key == "tync") {
      out.tyDegreesNoCrosshair = r.ReadDouble();
    } else if (key == "ta") {
      out.targetAreaPercent = r.ReadDouble();
    } else if (key == "tdist") {
      out.targetDistanceMeters = r.ReadDouble();
    } else if (key == "Retro") {
      out.retroTargets = DecodeRetroTargets(r);
    } else if (key == "Fiducial") {
      out.fiducialTargets = DecodeFiducialTargets(r);
    } else if (key == "Detector") {
      out.detectorTargets = DecodeDetectorTargets(r);
    } else if (key == "Classifier") {
      out.classifierTargets = DecodeClassifierTargets(r);
    } else if (key == "ClassifierRejected") {
      out.classifierTargetsRejected = DecodeClassifierTargets(r);
    } else if (key == "Barcode") {
      out.barcodeTargets = DecodeBarcodeTargets(r);
    } else if (key == "Counter") {
      out.counterTargets = DecodeCounterTargets(r);
    } else if (key == "OCR") {
      out.ocrTargets = DecodeOCRTargets(r);
    } else if (key == "imu") {
      DecodeIMU(r, out.imu);
    } else if (key == "hw") {
      DecodeHardware(r, out.hardware);
    } else if (key == "rewind") {
      DecodeRewind(r, out.rewind);
    } else if (key == "botorient") {
      DecodeBotOrientation(r, out.robotOrientation);
    } else if (key == "intrinsics") {
      DecodeIntrinsics(r, out.intrinsics);
    } else if (key == "imgsrc") {
      out.imageSource = static_cast<int>(r.ReadLong());
    } else if (key == "hwtype") {
      out.hardwareType = static_cast<int>(r.ReadLong());
    } else if (key == "uirefresh") {
      out.uiRefresh = static_cast<int>(r.ReadLong());
    } else if (key == "ignorent") {
      out.ignoreNT = r.ReadLong() != 0;
    } else if (key == "ntconnected") {
      out.ntConnected = r.ReadLong() != 0;
    } else if (key == "codepipeline") {
      out.pipelineConfigurationOverrideActive = r.ReadLong() != 0;
    } else if (key == "codepipelinestate") {
      out.pipelineConfigurationOverrideState =
          ParsePipelineConfigurationOverrideState(r.ReadString());
    } else if (key == "codemap") {
      out.sharedMapActive = r.ReadLong() != 0;
    } else if (key == "codemapstate") {
      out.sharedMapState = ParseSharedMapState(r.ReadString());
    } else {
      r.SkipValue();
    }
  }
}

}  // namespace detail

/**
 * LED behavior modes.
 */
enum class LEDMode {
  /** LED behavior is controlled by the current pipeline */
  PIPELINE_CONTROL = 0,
  /** LEDs forced off */
  FORCE_OFF = 1,
  /** LEDs forced to blink */
  FORCE_BLINK = 2,
  /** LEDs forced on */
  FORCE_ON = 3
};

/**
 * AprilTag detector downscaling factors. More downscaling improves
 * performance. It can reduce the detection range.
 */
enum class DownscaleOverride {
  /** Use the downscale configured in the current pipeline */
  PIPELINE_CONTROL = 0,
  /** No downscaling */
  X1 = 1,
  /** 1.5x downscale */
  X1_5 = 2,
  /** 2x downscale */
  X2 = 3,
  /** 3x downscale */
  X3 = 4,
  /** 4x downscale */
  X4 = 5
};

namespace detail {

// Publishes Pose2d structs for AdvantageScope and Field2d arrays for
// Glass/Elastic.
class PoseTelemetry {
 public:
  PoseTelemetry(wpi::nt::NetworkTable& telemetryTable,
                wpi::nt::NetworkTable& fieldTable,
                const std::string& cameraName, PoseEstimateType type) {
    std::string typeName{PoseEstimateTypeName(type)};
    std::string prefix = typeName + "/";
    m_accepted =
        telemetryTable
            .GetStructArrayTopic<wpi::math::Pose2d>(prefix + "accepted")
            .Publish();
    m_rejected =
        telemetryTable
            .GetStructArrayTopic<wpi::math::Pose2d>(prefix + "rejected")
            .Publish();
    m_rejectionReasons =
        telemetryTable.GetStringTopic(prefix + "rejectionReasons").Publish();
    m_fieldAccepted =
        fieldTable.GetDoubleArrayTopic(cameraName + "-" + typeName).Publish();
    m_fieldRejected =
        fieldTable
            .GetDoubleArrayTopic(cameraName + "-" + typeName + "-rejected")
            .Publish();
  }

  void Publish(const PoseEstimate& estimate) {
    bool drawRejected =
        estimate.fieldedTagCount > 0 && estimate.rejectionFlags != 0 &&
        (estimate.rejectionFlags & PoseEstimateConfig::REJECT_MISSING_POSE) ==
            0 &&
        std::isfinite(estimate.pose.X().value()) &&
        std::isfinite(estimate.pose.Y().value()) &&
        std::isfinite(estimate.pose.Rotation().Radians().value());
    std::span<const wpi::math::Pose2d> onePose{&estimate.pose, 1};
    std::span<const wpi::math::Pose2d> noPoses{};
    m_accepted.Set(estimate.IsValid() ? onePose : noPoses);
    m_rejected.Set(drawRejected ? onePose : noPoses);
    m_rejectionReasons.Set(
        PoseEstimateConfig::DescribeRejection(estimate.rejectionFlags));
    std::array<double, 3> fieldPose = FieldPose(estimate.pose);
    std::span<const double> noDoubles{};
    m_fieldAccepted.Set(estimate.IsValid() ? std::span<const double>{fieldPose}
                                           : noDoubles);
    m_fieldRejected.Set(drawRejected ? std::span<const double>{fieldPose}
                                     : noDoubles);
  }

  void Clear() {
    m_accepted.Set({});
    m_rejected.Set(std::span<const wpi::math::Pose2d>{});
    m_rejectionReasons.Set("");
    m_fieldAccepted.Set({});
    m_fieldRejected.Set(std::span<const double>{});
  }

 private:
  static std::array<double, 3> FieldPose(const wpi::math::Pose2d& pose) {
    return {pose.X().value(), pose.Y().value(),
            pose.Rotation().Degrees().value()};
  }

  wpi::nt::StructArrayPublisher<wpi::math::Pose2d> m_accepted;
  wpi::nt::StructArrayPublisher<wpi::math::Pose2d> m_rejected;
  wpi::nt::StringPublisher m_rejectionReasons;
  wpi::nt::DoubleArrayPublisher m_fieldAccepted;
  wpi::nt::DoubleArrayPublisher m_fieldRejected;
};

class PoseCountsTelemetry {
 public:
  explicit PoseCountsTelemetry(wpi::nt::NetworkTable& telemetryTable) {
    std::shared_ptr<wpi::nt::NetworkTable> countsTable =
        telemetryTable.GetSubTable("counts");
    m_processed = countsTable->GetIntegerTopic("processed").Publish();
    m_accepted = countsTable->GetIntegerTopic("accepted").Publish();
    m_rejected = countsTable->GetIntegerTopic("rejected").Publish();
    m_processed.Set(0);
    m_accepted.Set(0);
    m_rejected.Set(0);
    for (size_t i = 0; i < kRejectionFlags.size(); i++) {
      m_rejectionCounts[i] =
          countsTable->GetIntegerTopic(std::string{kRejectionNames[i]})
              .Publish();
      m_rejectionCounts[i].Set(0);
    }
  }

  void Count(const PoseEstimate& estimate) {
    m_processed.Set(++m_processedTotal);
    if (estimate.IsValid()) {
      m_accepted.Set(++m_acceptedTotal);
      return;
    }
    m_rejected.Set(++m_rejectedTotal);
    for (size_t i = 0; i < kRejectionFlags.size(); i++) {
      if ((estimate.rejectionFlags & kRejectionFlags[i]) != 0) {
        m_rejectionCounts[i].Set(++m_rejectionTotals[i]);
      }
    }
  }

 private:
  wpi::nt::IntegerPublisher m_processed;
  wpi::nt::IntegerPublisher m_accepted;
  wpi::nt::IntegerPublisher m_rejected;
  std::array<wpi::nt::IntegerPublisher, kRejectionFlags.size()>
      m_rejectionCounts;
  std::array<int64_t, kRejectionFlags.size()> m_rejectionTotals{};
  int64_t m_processedTotal = 0;
  int64_t m_acceptedTotal = 0;
  int64_t m_rejectedTotal = 0;
};

}  // namespace detail

/**
 * <h2>Quick start</h2>
 *
 * <h3>Visual servoing</h3>
 * @code
 * double turnKp = -0.02;
 * limelight::Limelight camera{"limelight"};
 * //limelight::Limelight camera{limelight::Limelight::SYSTEMCORE_USB0};
 * //limelight::Limelight camera{limelight::Limelight::SYSTEMCORE_USB1};
 *
 * // Each loop
 * double turn = joystick.GetRightX();
 * if (aimingEnabled && camera.HasTarget()) {
 *   turn = turnKp * camera.GetTXDegrees();
 * }
 * drivetrain.ArcadeDrive(forward, turn);
 * @endcode
 *
 * <h3>MegaTag1 localization</h3>
 * @code
 * wpi::math::Pose3d cameraPoseRobotSpace{0.30_m, 0.0_m, 0.20_m,
 *                                        wpi::math::Rotation3d{}};
 * limelight::Limelight camera{"limelight", cameraPoseRobotSpace};
 *
 * // Each robot loop
 * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(
 *          limelight::PoseEstimateType::MT1_WPIBLUE)) {
 *   poseEstimator.AddVisionMeasurement(estimate.pose,
 *                                      estimate.timestampSeconds,
 *                                      estimate.stdDevs);
 * }
 * @endcode
 *
 * <h3>MegaTag2 localization</h3>
 * Publish the robot yaw before you read the queue in each robot loop.
 * @code
 * wpi::math::Pose3d cameraPoseRobotSpace{0.30_m, 0.0_m, 0.20_m,
 *                                        wpi::math::Rotation3d{}};
 * limelight::Limelight camera{"limelight", cameraPoseRobotSpace};
 *
 * // Each robot loop
 * limelight::Limelight::SetSharedRobotOrientation(robotYawDegrees);
 * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(
 *          limelight::PoseEstimateType::MT2_WPIBLUE)) {
 *   poseEstimator.AddVisionMeasurement(estimate.pose,
 *                                      estimate.timestampSeconds,
 *                                      estimate.stdDevs);
 * }
 * @endcode
 *
 * <h3>Configured MegaTag1 and MegaTag2</h3>
 * This example configures every filter and standard deviation scaling term.
 * MT1 scales XY uncertainty linearly with distance and clamps it to 0.05-2.0
 * meters. MT2 scales it by the square root of distance and clamps it to
 * 0.0001-2.0 meters. Both divide by the square root of the fielded tag count.
 * Vision heading is not fused. The field bounds are for the 2026 welded field.
 * Tune the other thresholds on your robot.
 * @code
 * double untrusted = limelight::PoseEstimateConfig::UNTRUSTED;
 * // Starting with the default means you are not required to set every
 * // single configuration value
 * limelight::PoseEstimateConfig mt1Config =
 *     limelight::PoseEstimateConfig::DefaultMT1()
 *         .WithMinTagCount(1)
 *         .WithMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity
 * perspectives .WithMaxSingleTagDistance(3.0) // If we only see one tag, don't
 * trust it unless we are at most 3m away from it .WithMaxAvgTagDistance(6.0) //
 * If we see multiple tags, max avg distance must be less than 6 meters.
 *         .WithMinAvgTagArea(0.05)
 *         .WithFieldBounds(16.541, 8.069) // Reject pose estimates that are out
 * of bounds. .WithFieldBoundsMargin(0.5) // Reject pose estimates that are out
 * of bounds +.5m .WithStdDevXY(0.5, 0.05, 2.0) // .5 base, absolute min .05,
 * absolute max 2.0 .WithStdDevTheta(untrusted, untrusted, untrusted) // never
 * incorporate pose estimate rotation. You may want to incorporate rotation by
 * setting these to other values .WithStdDevDistanceScaling(1.0, 0.0, 6.0) //
 * linear scaling .WithStdDevTagCountDivision(0.5);
 *
 * limelight::PoseEstimateConfig mt2Config =
 *     limelight::PoseEstimateConfig::DefaultMT2()
 *         .WithMinTagCount(1)
 *         .WithMaxSingleTagAmbiguity(1.0) // MT2 can handle maximally ambiguous
 * perspectives. Accept tags regardless of ambiguity value.
 *         .WithMaxSingleTagDistance(0.0) // 0 disables this check
 *         .WithMaxAvgTagDistance(8.0)
 *         .WithMinAvgTagArea(0.02)
 *         .WithFieldBounds(16.541, 8.069)
 *         .WithFieldBoundsMargin(0.5)
 *         .WithStdDevXY(0.3, 0.0001, 2.0)
 *         .WithStdDevTheta(untrusted, untrusted, untrusted)
 *         .WithStdDevDistanceScaling(0.5, 0.0, 8.0) // Less aggressive STDDev
 * scaling for MT2. Scale by sqrt(distance) rather than distance^1.
 *         .WithStdDevTagCountDivision(0.5);
 *
 * wpi::math::Pose3d cameraPoseRobotSpace{0.30_m, 0.0_m, 0.20_m,
 *                                        wpi::math::Rotation3d{}};
 * limelight::Limelight camera =
 *     limelight::Limelight{"limelight", cameraPoseRobotSpace}
 *         .WithPoseEstimateConfig_MT1(mt1Config)
 *         .WithPoseEstimateConfig_MT2(mt2Config)
 *         .WithTelemetry(true); // Keep automatic telemetry enabled. All
 * accepted and rejected poses will remain easy to visualize in standard
 * dashboards bool useMegaTag2 = true;
 *
 * // Each robot loop
 * limelight::PoseEstimateType type =
 *     useMegaTag2 ? limelight::PoseEstimateType::MT2_WPIBLUE
 *                 : limelight::PoseEstimateType::MT1_WPIBLUE;
 * if (useMegaTag2) {
 *   // Use your robot's pose here, not your raw IMU reading.
 *   limelight::Limelight::SetSharedRobotOrientation(robotYawDegrees);
 * }
 * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(type)) {
 *   poseEstimator.AddVisionMeasurement(estimate.pose,
 *                                      estimate.timestampSeconds,
 *                                      estimate.stdDevs);
 * }
 * @endcode
 *
 * All values from one GetLatestResults() call come from the same frame. This
 * includes tx, botpose, fiducials, latencies, and timestamps.
 *
 * All 3D data uses the right-handed NWU convention in every space (field,
 * robot, camera, target): x = forward, y = left, z = up. Raw pose arrays are
 * [x, y, z, roll, pitch, yaw] in meters and degrees.
 *
 * Getters decode at most one new frame. Otherwise they return cached data.
 * After a disconnect, they return the values of the last frame. Use
 * GetStatus() or LimelightResults::GetAgeSeconds() to detect a stale camera.
 *
 * Use this class from one thread, the robot loop.
 */
class Limelight {
 public:
  /** Default stale threshold. GetStatus() reports Status::STALE for a frame
   *  older than this. Override per instance with WithStaleFrameThreshold(). */
  static constexpr double STALE_FRAME_SECONDS = 0.25;

  /** Maximum size in bytes of a pipeline configuration override. Both this
   *  library and the camera reject larger publishes. */
  static constexpr size_t MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES = 4096;

  /** Maximum size in bytes of a shared field map. Both this library and the
   *  camera reject larger publishes. */
  static constexpr size_t MAX_SHARED_MAP_BYTES = 262144;

  /** The highest camera protocol version ("protover") this library
   *  understands. If a camera publishes a higher version, the library prints
   *  one warning. */
  static constexpr int SUPPORTED_PROTOCOL_VERSION = 1;

  /** Name of the vision instance running on Systemcore USB port 0:
   *  `limelight::Limelight{limelight::Limelight::SYSTEMCORE_USB0}` */
  static constexpr std::string_view SYSTEMCORE_USB0 = "limelightsc0";
  /** Name of the vision instance running on Systemcore USB port 1 */
  static constexpr std::string_view SYSTEMCORE_USB1 = "limelightsc1";
  /** Name of the vision instance running on Systemcore USB port 2 */
  static constexpr std::string_view SYSTEMCORE_USB2 = "limelightsc2";
  /** Name of the vision instance running on Systemcore USB port 3 */
  static constexpr std::string_view SYSTEMCORE_USB3 = "limelightsc3";

  /** Root telemetry table. Each camera uses `limelight_telemetry/<name>/`. The
   *  shared Field2d table is limelight_telemetry/Field. */
  static constexpr std::string_view TELEMETRY_TABLE = "limelight_telemetry";

  /**
   * Creates an interface to the Limelight with the default name
   * ("limelight").
   */
  Limelight() : Limelight("limelight") {}

  /**
   * Creates an interface to the given Limelight.
   * @param name The camera or vision instance's NetworkTables name
   *        (e.g. "limelight" or "limelight-left")
   */
  explicit Limelight(std::string_view name) {
    if (name.empty()) {
      name = "limelight";
    }
    m_name = std::string{name};
    m_latestResults.fromLiveSubscriber = true;
    m_table = wpi::nt::NetworkTableInstance::GetDefault().GetTable(m_name);
    m_telemetryTable = wpi::nt::NetworkTableInstance::GetDefault().GetTable(
        std::string{TELEMETRY_TABLE} + "/" + m_name);
    wpi::nt::PubSubOptions options;
    options.periodic = 0.005;
    options.sendAll = true;
    options.pollStorage = 20;
    m_resultsSubscriber = m_table->GetRawTopic("results_msgpack")
                              .Subscribe("msgpack", {}, options);
    m_protocolVersionSubscriber =
        m_table->GetIntegerTopic("protover").Subscribe(0);
  }

  /**
   * Creates an interface to the given Limelight and sets its camera pose in
   * robot space.
   *
   * @param name The NetworkTables name of the camera, for example "limelight"
   *        or "limelight-left"
   * @param cameraPoseRobotSpace The pose of the camera relative to the robot
   *        center (x = forward, y = left, z = up, in meters). This overrides
   *        the camera pose configured in the web interface. Call
   *        ClearCameraPose_RobotSpaceOverride() to return to it. An all-zero
   *        pose is the clear value, so the camera keeps the pose from the web
   *        interface.
   */
  Limelight(std::string_view name,
            const wpi::math::Pose3d& cameraPoseRobotSpace)
      : Limelight(name) {
    PublishCameraPose(
        cameraPoseRobotSpace.X().value(), cameraPoseRobotSpace.Y().value(),
        cameraPoseRobotSpace.Z().value(),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().X().value()),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().Y().value()),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().Z().value()));
    FlushNT();
  }

  /**
   * Creates an interface to the given Limelight and sets its camera pose in
   * robot space. This overrides the camera pose configured in the web
   * interface. Call ClearCameraPose_RobotSpaceOverride() to return to it. An
   * all-zero pose is the clear value, so the camera keeps the pose from the web
   * interface.
   *
   * @param name The NetworkTables name of the camera, for example "limelight"
   *        or "limelight-left"
   * @param forward Forward (x) offset in meters
   * @param left Left (y) offset in meters
   * @param up Up (z) offset in meters
   * @param rollDegrees Roll angle in degrees
   * @param pitchDegrees Pitch angle in degrees
   * @param yawDegrees Yaw angle in degrees
   */
  Limelight(std::string_view name, double forward, double left, double up,
            double rollDegrees, double pitchDegrees, double yawDegrees)
      : Limelight(
            name,
            wpi::math::Pose3d{
                wpi::units::meter_t{forward}, wpi::units::meter_t{left},
                wpi::units::meter_t{up},
                wpi::math::Rotation3d{
                    wpi::units::radian_t{detail::DegreesToRadians(rollDegrees)},
                    wpi::units::radian_t{
                        detail::DegreesToRadians(pitchDegrees)},
                    wpi::units::radian_t{
                        detail::DegreesToRadians(yawDegrees)}}}) {}

  Limelight(const Limelight&) = delete;
  Limelight& operator=(const Limelight&) = delete;
  Limelight(Limelight&&) = default;
  Limelight& operator=(Limelight&&) = default;

  /**
   * Releases the NetworkTables subscriptions of this instance. Removes its
   * telemetry topics.
   */
  ~Limelight() = default;

  /**
   * Sets the maximum age of the newest frame. Above this age, GetStatus()
   * reports Status::STALE and HasTarget() returns false. Raise this value when
   * you use SetThrottle().
   *
   * @param seconds Stale threshold in seconds. Positive infinity disables stale
   *        detection.
   * @return this, for chaining with the constructor
   */
  Limelight& WithStaleFrameThreshold(double seconds) & {
    m_staleFrameSeconds = detail::ClampArg(
        seconds, 0.01, std::numeric_limits<double>::max(), STALE_FRAME_SECONDS);
    return *this;
  }

  /** @copydoc WithStaleFrameThreshold(double)& */
  Limelight&& WithStaleFrameThreshold(double seconds) && {
    WithStaleFrameThreshold(seconds);
    return std::move(*this);
  }

  /**
   * Configures filtering and fusion standard deviations for every MegaTag1 pose
   * estimate. MegaTag1 computes heading from tag geometry only. It usually
   * needs stricter ambiguity and tag-count gates than MegaTag2. Rejected
   * estimates report {@code IsValid() == false}. PoseEstimate::rejectionFlags
   * lists the failed checks.
   *
   * @param config The MegaTag1 configuration
   * @return this, for chaining with the constructor
   */
  Limelight& WithPoseEstimateConfig_MT1(const PoseEstimateConfig& config) & {
    m_megaTag1Config = config;
    return *this;
  }

  /** @copydoc WithPoseEstimateConfig_MT1(const PoseEstimateConfig&)& */
  Limelight&& WithPoseEstimateConfig_MT1(const PoseEstimateConfig& config) && {
    WithPoseEstimateConfig_MT1(config);
    return std::move(*this);
  }

  /**
   * Configures filtering and fusion standard deviations for every MegaTag2
   * pose estimate.
   *
   * @param config The MegaTag2 configuration
   * @return this, for chaining with the constructor
   */
  Limelight& WithPoseEstimateConfig_MT2(const PoseEstimateConfig& config) & {
    m_megaTag2Config = config;
    return *this;
  }

  /** @copydoc WithPoseEstimateConfig_MT2(const PoseEstimateConfig&)& */
  Limelight&& WithPoseEstimateConfig_MT2(const PoseEstimateConfig& config) && {
    WithPoseEstimateConfig_MT2(config);
    return std::move(*this);
  }

  /**
   * Enables or disables automatic pose-estimate telemetry. Enabled by default.
   * Pose estimates publish under {@code limelight_telemetry/<name>/<type>/}:
   * <ul>
   * <li>{@code accepted} and {@code rejected}: Pose2d struct arrays for field
   * views</li>
   * <li>{@code rejectionReasons}: for example {@code TAG_COUNT|AMBIGUITY}</li>
   * </ul>
   * The camera-level {@code counts/} topics aggregate all estimate types that
   * ReadPoseEstimateQueue() and ReadAcceptedPoseEstimates() process. One
   * estimate can increment several rejection-reason totals. The same estimates
   * appear on the shared {@code limelight_telemetry/Field} Field2d table for
   * Glass and Elastic. Camera health publishes as {@code connected},
   * {@code customCalibration}, and {@code status}. Pose displays clear when
   * the camera is unhealthy. Disabling telemetry unpublishes everything and
   * resets the counters.
   *
   * @return this, for chaining with the constructor
   */
  Limelight& WithTelemetry(bool enabled) & {
    if (m_telemetryEnabled && !enabled) {
      UnpublishTelemetry();
    }
    m_telemetryEnabled = enabled;
    return *this;
  }

  /** @copydoc WithTelemetry(bool)& */
  Limelight&& WithTelemetry(bool enabled) && {
    WithTelemetry(enabled);
    return std::move(*this);
  }

  /**
   * @return The camera's NetworkTables name
   */
  const std::string& GetName() const { return m_name; }

  /**
   * @return The msgpack envelope protocol version from the camera. 0 if the
   *         camera has not connected or runs old software
   */
  int GetProtocolVersion() {
    return static_cast<int>(m_protocolVersionSubscriber.Get());
  }

  // ---- Results ----

  /**
   * Returns the latest results envelope. Decodes the newest MessagePack frame
   * if one arrived since the last call. This method is cheap to call many times
   * per loop. The decoded envelope is cached until a new frame arrives. This
   * method never consumes the frame queue. You can use it together with
   * ReadResultsQueue().
   *
   * @return The latest LimelightResults. Use GetStatus() for camera health.
   *         Use LimelightResults::valid for target validity. The reference
   *         stays valid for the life of this Limelight. It updates in place on
   *         the next getter call.
   */
  const LimelightResults& GetLatestResults() {
    WarnIfProtocolNewer();
    wpi::nt::TimestampedRaw raw = m_resultsSubscriber.GetAtomic();
    if (raw.time != m_lastDecodedTimestamp && !raw.value.empty()) {
      m_latestResults = DecodeFrame(raw.value, raw.time);
      m_lastDecodedTimestamp = raw.time;
    }
    PublishHealthTelemetry();
    return m_latestResults;
  }

  /**
   * Decodes a results envelope from raw MessagePack bytes. Decode failures are
   * reported in LimelightResults::error. This method never throws. Use it for
   * unit tests.
   *
   * {@code receiveTimestampSeconds} stays 0. Pose estimates built from the
   * result are rejected as NO_TIMESTAMP. Use
   * Decode(std::span<const uint8_t>, int64_t) for pose estimate tests.
   *
   * @param envelope The raw MessagePack results dump
   * @return The decoded results. {@code receiveTimestampSeconds} stays 0
   */
  static LimelightResults Decode(std::span<const uint8_t> envelope) {
    LimelightResults results;
    auto start = std::chrono::steady_clock::now();
    try {
      detail::MsgPackReader reader{envelope};
      detail::DecodeResults(reader, results);
    } catch (const std::exception& e) {
      results.error = std::string{"llmsgpack decode error: "} + e.what();
    }
    results.parseLatencyMillis = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
    return results;
  }

  /**
   * Decodes a results envelope and stamps it with its NetworkTables receive
   * time. This matches live decoding and timestamp handling. For log replay:
   * record raw frames with GetLatestRawFrame() or ReadRawFrameQueue(). Then
   * pass the bytes and the timestamp to this method. Estimates built from the
   * result have latency-compensated timestamps.
   *
   * @param envelope The raw MessagePack results dump
   * @param receiveTimestampMicros NetworkTables receive time in microseconds
   *        (local NetworkTables timebase, WPILib alpha-6 or older), for
   *        example wpi::nt::TimestampedRaw::time
   */
  static LimelightResults Decode(std::span<const uint8_t> envelope,
                                 int64_t receiveTimestampMicros) {
    LimelightResults results = Decode(envelope);
    results.receiveTimestampSeconds =
        receiveTimestampMicros / NT_TICKS_PER_SECOND;
    return results;
  }

  /**
   * @return The current health of the camera from the point of view of this
   *         consumer. Status::OK means a decodable frame arrived within the
   *         stale threshold. The threshold is STALE_FRAME_SECONDS unless you
   *         override it with WithStaleFrameThreshold().
   */
  Status GetStatus() {
    GetLatestResults();
    return ComputeStatus();
  }

  /**
   * @return True if the camera is reachable. The status is not Status::NO_DATA
   *         and not Status::STALE
   */
  bool IsConnected() {
    Status status = GetStatus();
    return status == Status::OK || status == Status::DECODE_ERROR;
  }

  /**
   * @return True if the camera is connected, sends fresh frames, and has at
   *         least one valid target. Returns false when the newest frame is
   *         older than the stale threshold.
   */
  bool HasTarget() {
    const LimelightResults& results = GetLatestResults();
    return ComputeStatus() == Status::OK && results.valid;
  }

  /**
   * @return Horizontal offset from crosshair to target in degrees. This value
   *         holds the last received value after a disconnect. Check HasTarget()
   *         every loop.
   */
  double GetTXDegrees() { return GetLatestResults().txDegrees; }

  /**
   * @return Vertical offset from crosshair to target in degrees. This value
   *         holds the last received value after a disconnect. Check HasTarget()
   *         every loop.
   */
  double GetTYDegrees() { return GetLatestResults().tyDegrees; }

  /**
   * @return Horizontal offset from principal pixel to target in degrees
   *         (crosshair-independent)
   */
  double GetTXDegreesNoCrosshair() {
    return GetLatestResults().txDegreesNoCrosshair;
  }

  /**
   * @return Vertical offset from principal pixel to target in degrees
   *         (crosshair-independent)
   */
  double GetTYDegreesNoCrosshair() {
    return GetLatestResults().tyDegreesNoCrosshair;
  }

  /**
   * @return Target area as a percentage of the image (0-100). This value holds
   *         the last received value after a disconnect. Check HasTarget() every
   *         loop.
   */
  double GetTargetAreaPercent() { return GetLatestResults().targetAreaPercent; }

  /**
   * @return 3D distance from the camera to the primary fiducial target in
   *         meters. 0 if not available
   */
  double GetTargetDistanceMeters() {
    return GetLatestResults().targetDistanceMeters;
  }

  /**
   * @return Total number of retro, fiducial, detector, classifier, and
   *         barcode targets in the latest frame
   */
  int GetTargetCount() { return GetLatestResults().GetTargetCount(); }

  /**
   * @return Active pipeline index (0-9). -1 before the first frame arrives
   */
  int GetCurrentPipelineIndex() {
    return static_cast<int>(GetLatestResults().pipelineIndex);
  }

  /**
   * @return Active pipeline type, for example "pipe_fiducial", "pipe_color",
   *         or "pipe_detector"
   */
  const std::string& GetCurrentPipelineType() {
    return GetLatestResults().pipelineType;
  }

  /**
   * @return Targeting/pipeline latency in milliseconds
   */
  double GetTargetingLatencyMillis() {
    return GetLatestResults().targetingLatencyMillis;
  }

  /**
   * @return Capture latency in milliseconds
   */
  double GetCaptureLatencyMillis() {
    return GetLatestResults().captureLatencyMillis;
  }

  /**
   * @return IMU state from the latest frame
   */
  const IMUData& GetIMUData() { return GetLatestResults().imu; }

  /**
   * @return Hardware/system stats from the latest frame
   */
  const HardwareData& GetHardwareData() { return GetLatestResults().hardware; }

  /**
   * @return The camera intrinsics that the running pipeline uses. Includes the
   *         camera matrix scaled to the processing resolution, the OpenCV
   *         distortion coefficients, and the FOV
   */
  const CameraIntrinsics& GetCameraIntrinsics() {
    return GetLatestResults().intrinsics;
  }

  /**
   * @return True if the running pipeline uses a user-uploaded camera
   *         calibration instead of a built-in default. See
   *         CameraIntrinsics::customCalibration. False before the first frame
   *         arrives.
   */
  bool IsUsingCustomCalibration() {
    return GetLatestResults().intrinsics.customCalibration;
  }

  /**
   * @return Data set by a python snapscript via llpython
   */
  const std::vector<double>& GetPythonScriptData() {
    return GetLatestResults().pythonOutput;
  }

  // ---- Pose Estimates ----

  /**
   * Gets the pose estimate of the given type from the newest frame. This getter
   * can return the same frame many times, also after a disconnect. Use
   * ReadAcceptedPoseEstimates() for fusion. Use GetStatus() for current camera
   * health.
   *
   * @param type Which pose estimate to produce
   */
  PoseEstimate GetPoseEstimate(PoseEstimateType type) {
    return GetPoseEstimate(GetLatestResults(), type);
  }

  // ---- Queue reads ----

  /**
   * Decodes and returns every buffered results envelope received since the last
   * queue read. The queue holds 20 frames. If more frames arrive between reads,
   * the oldest frames are discarded. The newest frame also becomes the result
   * of GetLatestResults().
   *
   * Use only one queue reading method per camera. The queue reading methods are
   * this method, ReadPoseEstimateQueue(), ReadAcceptedPoseEstimates(), and
   * ReadRawFrameQueue(). To get several values from each frame, read the queue
   * here one time. Then call GetPoseEstimate(const LimelightResults&,
   * PoseEstimateType) for each frame. Other getters only read the newest frame.
   * They do not consume the queue.
   *
   * @return All buffered frames, oldest first. Empty if no new frame arrived
   */
  std::vector<LimelightResults> ReadResultsQueue() {
    WarnIfProtocolNewer();
    std::vector<wpi::nt::TimestampedRaw> frames =
        m_resultsSubscriber.ReadQueue();
    std::vector<LimelightResults> out;
    out.reserve(frames.size());
    for (const wpi::nt::TimestampedRaw& frame : frames) {
      if (frame.value.empty()) {
        continue;
      }
      out.push_back(DecodeFrame(frame.value, frame.time));
      m_lastDecodedTimestamp = frame.time;
    }
    if (!out.empty()) {
      m_latestResults = out.back();
    }
    PublishHealthTelemetry();
    return out;
  }

  /**
   * Reads every frame received since the last queue read. Returns one pose
   * estimate of the given type for each frame. This lets your pose estimator
   * use every vision update. Check PoseEstimate::IsValid() on each estimate
   * before you fuse it.
   *
   * This method consumes the queue. Do not use another queue reading method
   * for this camera.
   *
   * @param type Which pose estimate to produce for each frame
   * @return One pose estimate per frame, oldest first
   */
  std::vector<PoseEstimate> ReadPoseEstimateQueue(PoseEstimateType type) {
    std::vector<LimelightResults> frames = ReadResultsQueue();
    std::vector<PoseEstimate> out;
    out.reserve(frames.size());
    for (const LimelightResults& frame : frames) {
      out.push_back(GetPoseEstimate(frame, type));
      CountPoseEstimate(out.back());
    }
    return out;
  }

  /**
   * Reads every frame received since the last queue read. Returns only the pose
   * estimates that passed validation and filtering. Each queued frame is
   * processed one time. The pose estimator decides if the timestamp is still
   * in range.
   *
   * @code
   * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(
   *          limelight::PoseEstimateType::MT2_WPIBLUE)) {
   *   poseEstimator.AddVisionMeasurement(estimate.pose,
   *                                      estimate.timestampSeconds,
   *                                      estimate.stdDevs);
   * }
   * @endcode
   *
   * Telemetry shows the newest estimate and keeps rejection counters. Use
   * ReadPoseEstimateQueue() to inspect every rejected estimate.
   *
   * This method consumes the queue. Do not use another queue reading method
   * for this camera.
   *
   * @param type Which pose estimate to produce for each frame
   * @return The accepted estimates, oldest first. Empty if no new frame
   *         arrived or no estimate passed
   */
  std::vector<PoseEstimate> ReadAcceptedPoseEstimates(PoseEstimateType type) {
    std::vector<PoseEstimate> all = ReadPoseEstimateQueue(type);
    std::erase_if(
        all, [](const PoseEstimate& estimate) { return !estimate.IsValid(); });
    return all;
  }

  /**
   * Returns the newest raw results envelope without decoding it. The result has
   * the MessagePack bytes and the NetworkTables receive timestamp in
   * microseconds (local NetworkTables timebase). This method does not consume
   * the frame queue. Use it with Decode(std::span<const uint8_t>, int64_t) for
   * logging and replay.
   *
   * @return The newest raw frame. The value is empty if no frame has arrived
   */
  wpi::nt::TimestampedRaw GetLatestRawFrame() {
    WarnIfProtocolNewer();
    return m_resultsSubscriber.GetAtomic();
  }

  /**
   * Reads every raw envelope received since the last queue read, without
   * decoding. Use this for logging and replay systems. These systems record raw
   * frames and decode them with Decode(std::span<const uint8_t>, int64_t), live
   * or from a log.
   *
   * This method consumes the queue. Do not use another queue reading method
   * for this camera.
   *
   * @return All buffered raw frames, oldest first. Empty if no new frame
   *         arrived
   */
  std::vector<wpi::nt::TimestampedRaw> ReadRawFrameQueue() {
    WarnIfProtocolNewer();
    return m_resultsSubscriber.ReadQueue();
  }

  /**
   * @return The full 3D robot pose for the given estimate type. The type
   *         selects the origin and the algorithm
   */
  wpi::math::Pose3d GetRobotPose(PoseEstimateType type) {
    return GetLatestResults().GetRobotPose(type);
  }

  /**
   * @return Camera pose in robot space (meters, degrees) as a Pose3d
   */
  wpi::math::Pose3d GetCameraPose_RobotSpace() {
    return ToPose3D(GetLatestResults().cameraPoseRobotSpace);
  }

  /**
   * Builds a pose estimate of the given type from one results envelope. Applies
   * the PoseEstimateConfig for the algorithm of the estimate. Use this with
   * ReadResultsQueue() to get several estimate types (for example MT1 and MT2)
   * from one queue read.
   *
   * @param results The envelope to build from
   * @param type Which pose estimate to produce
   */
  PoseEstimate GetPoseEstimate(const LimelightResults& results,
                               PoseEstimateType type) {
    PoseEstimate estimate;
    estimate.type = type;
    if (static_cast<size_t>(type) >= kPoseEstimateTypeCount) {
      // An enum value outside the known set has no pose array. Reject it here
      // so nothing downstream indexes by type.
      estimate.rejectionFlags = PoseEstimateConfig::REJECT_MISSING_POSE;
      return estimate;
    }
    const std::vector<double>& poseArray = detail::PoseArray(results, type);

    estimate.frameIndex = results.frameIndex;
    estimate.latencyMillis =
        results.captureLatencyMillis + results.targetingLatencyMillis;
    estimate.timestampSeconds = wpi::units::second_t{
        results.receiveTimestampSeconds - (estimate.latencyMillis / 1000.0)};
    estimate.reportedTagCount = results.reportedTagCount;
    estimate.fieldedTagCount = CountFieldedFiducials(results.fiducialTargets);
    estimate.tagSpanMeters = results.tagSpanMeters;
    estimate.avgTagDistanceMeters = results.avgTagDistanceMeters;
    estimate.avgTagAreaPercent = results.avgTagAreaPercent;
    estimate.reportedStdDevs = detail::ReportedStdDevs(results, type);
    estimate.rawFiducials = results.fiducialTargets;

    if (!results.error.empty()) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_DECODE_ERROR;
    }
    if (results.receiveTimestampSeconds <= 0 ||
        estimate.timestampSeconds.value() <= 0) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_NO_TIMESTAMP;
    }
    if (estimate.fieldedTagCount == 0 && !estimate.rawFiducials.empty()) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_NO_FIELDED_TAGS;
    }

    // A missing pose array or the all-zero sentinel means no estimate.
    if (poseArray.size() < 6 ||
        (poseArray[0] == 0 && poseArray[1] == 0 && poseArray[2] == 0 &&
         poseArray[3] == 0 && poseArray[4] == 0 && poseArray[5] == 0)) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_MISSING_POSE;
      PublishPoseTelemetry(estimate, results.fromLiveSubscriber);
      return estimate;
    }

    estimate.pose = ToPose2D(poseArray);
    const PoseEstimateConfig& config = ConfigFor(type);
    config.ApplyTo(estimate);

    if (HasNonfiniteData(estimate)) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_NONFINITE;
    }
    if (estimate.fieldedTagCount > 0 && estimate.avgTagDistanceMeters <= 0) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_BAD_METADATA;
    }
    PublishPoseTelemetry(estimate, results.fromLiveSubscriber);
    return estimate;
  }

  // ---- Control ----

  /**
   * Switches to the given pipeline.
   * @param pipelineIndex Pipeline index (0-9)
   */
  void SetPipelineIndex(int pipelineIndex) {
    m_table->GetEntry("pipeline")
        .SetDouble(static_cast<int>(detail::ClampArg(pipelineIndex, 0, 9)));
  }

  /**
   * An immutable, validated pipeline configuration override. It holds the
   * contents of a .vpr file. Create it one time during robot initialization.
   * All file IO and size validation happen at that time. Then publish it at
   * any time with SetPipelineConfigurationOverride(const
   * PipelineConfiguration&).
   * @code
   * // RobotInit
   * auto aiming =
   * limelight::Limelight::PipelineConfiguration::FromDeployFolder("aiming");
   * auto intake =
   * limelight::Limelight::PipelineConfiguration::FromDeployFolder("intake");
   * // mid-match
   * camera.SetPipelineConfigurationOverride(aiming);
   * @endcode
   *
   * A failed load produces an instance where IsValid() is false.
   */
  class PipelineConfiguration {
   public:
    /**
     * Wraps .vpr contents. Checks the MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES
     * size cap one time.
     *
     * @param contents The .vpr pipeline file contents
     * @return The wrapped configuration. Not valid when the contents are empty
     *         or too large
     */
    static PipelineConfiguration FromString(std::string_view contents) {
      if (contents.empty()) {
        WarnLoader("pipeline configuration rejected: no contents");
        return PipelineConfiguration{};
      }
      if (contents.size() > MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES) {
        WarnLoader("pipeline configuration rejected: larger than " +
                   std::to_string(MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES) +
                   " bytes");
        PipelineConfiguration bad;
        bad.m_status = LoadStatus::TOO_LARGE;
        return bad;
      }
      PipelineConfiguration config;
      config.m_contents = std::string{contents};
      config.m_status = LoadStatus::OK;
      return config;
    }

    /**
     * Reads a .vpr pipeline file from the deploy directory of the robot program
     * (src/main/deploy in the robot project). If the read fails, this method
     * prints a warning and returns a configuration that is not valid.
     *
     * @param deployRelativePath The pipeline file relative to the deploy
     *        directory, for example "aiming" or "pipelines/aiming.vpr". ".vpr"
     * is added when missing
     * @return The loaded configuration. Check IsValid()
     */
    static PipelineConfiguration FromDeployFolder(
        std::string_view deployRelativePath) {
      if (deployRelativePath.empty()) {
        WarnLoader("no pipeline configuration file name given");
        return PipelineConfiguration{};
      }
      std::optional<std::string> contents = ReadDeployFileForNT(
          deployRelativePath, ".vpr", "pipeline configuration");
      if (!contents) {
        PipelineConfiguration bad;
        bad.m_status = LoadStatus::READ_FAILED;
        return bad;
      }
      return FromString(*contents);
    }

    /** @return True if this holds a usable configuration */
    bool IsValid() const { return m_status == LoadStatus::OK; }

    /** @return Why this configuration is or is not usable */
    LoadStatus GetLoadStatus() const { return m_status; }

    /** @return The configuration's size in bytes, or 0 when invalid */
    size_t SizeBytes() const { return m_contents.size(); }

   private:
    friend class Limelight;
    std::string m_contents;
    LoadStatus m_status = LoadStatus::NO_CONTENTS;
  };

  /**
   * Publishes a pipeline configuration override to this camera. Flushes
   * NetworkTables immediately. The camera runs the override while it is enabled
   * with SetUsePipelineConfigurationOverride(). The ten pipelines on the camera
   * do not change. You can switch between them and the override at any time.
   *
   * Create the PipelineConfiguration during robot initialization. Publishing is
   * only a NetworkTables write. It does not access the disk. You can switch
   * between several prepared configurations during a match:
   * @code
   * // RobotInit
   * auto aiming =
   * limelight::Limelight::PipelineConfiguration::FromDeployFolder("aiming");
   * // whenever
   * camera.SetPipelineConfigurationOverride(aiming);
   * camera.SetUsePipelineConfigurationOverride(true);
   * @endcode
   *
   * @param config The configuration to publish. The method ignores a
   *        configuration that is not valid and prints a warning
   */
  void SetPipelineConfigurationOverride(const PipelineConfiguration& config) {
    if (!config.IsValid()) {
      WarnLoader("ignoring invalid pipeline configuration override");
      return;
    }
    m_table->GetEntry("codepipeline_set").SetString(config.m_contents);
    FlushNT();
  }

  /**
   * Clears the published pipeline configuration override. Flushes NetworkTables
   * immediately. If the override was running, the camera returns immediately to
   * the pipeline selected by SetPipelineIndex(). Otherwise, it returns when the
   * override is next disabled.
   */
  void ClearPipelineConfigurationOverride() {
    m_table->GetEntry("codepipeline_set").SetString("");
    FlushNT();
  }

  /**
   * Enables or disables the pipeline configuration override. While enabled, the
   * camera runs the pipeline published with SetPipelineConfigurationOverride().
   * While disabled, the camera runs the pipeline selected by
   * SetPipelineIndex(). The published override stays on the camera in both
   * states. You can switch between the two at any time.
   *
   * @param use True to run the override. False to run the indexed pipeline
   */
  void SetUsePipelineConfigurationOverride(bool use) {
    m_table->GetEntry("codepipeline_enable_set").SetInteger(use ? 1 : 0);
  }

  /**
   * Reads back whether the camera runs the pipeline configuration override. The
   * value comes from the latest results frame. It can differ from the value
   * requested with SetUsePipelineConfigurationOverride(). For example, the
   * camera web interface can force the override off.
   *
   * @return True if the camera reports that the override is running
   */
  bool IsPipelineConfigurationOverrideEnabled() {
    return GetLatestResults().pipelineConfigurationOverrideActive;
  }

  /**
   * @return The camera's pipeline configuration override state from the
   *         latest results frame
   */
  PipelineConfigurationOverrideState GetPipelineConfigurationOverrideState() {
    return GetLatestResults().pipelineConfigurationOverrideState;
  }

  /**
   * @return True if the camera localizes with the shared field map published
   *         with SetSharedMap(). The value comes from the latest results frame
   */
  bool IsSharedMapActive() { return GetLatestResults().sharedMapActive; }

  /**
   * @return The camera's shared field map state from the latest results
   *         frame
   */
  SharedMapState GetSharedMapState() {
    return GetLatestResults().sharedMapState;
  }

  /**
   * An immutable, validated shared field map. It holds the contents of an .fmap
   * file. Create it one time during robot initialization. All file IO and size
   * validation happen at that time. Then publish it at any time with
   * SetSharedMap(const FieldMap&):
   * @code
   * // RobotInit
   * auto fieldMap = limelight::Limelight::FieldMap::FromDeployFolder("field");
   * limelight::Limelight::SetSharedMap(fieldMap);
   * @endcode
   *
   * A failed load produces an instance where IsValid() is false.
   */
  class FieldMap {
   public:
    /**
     * Wraps .fmap contents. Checks the MAX_SHARED_MAP_BYTES size cap one time.
     *
     * @param contents The .fmap field map file contents
     * @return The wrapped field map. Not valid when the contents are empty or
     *         too large
     */
    static FieldMap FromString(std::string_view contents) {
      if (contents.empty()) {
        WarnLoader("field map rejected: no contents");
        return FieldMap{};
      }
      if (contents.size() > MAX_SHARED_MAP_BYTES) {
        WarnLoader("field map rejected: larger than " +
                   std::to_string(MAX_SHARED_MAP_BYTES) + " bytes");
        FieldMap bad;
        bad.m_status = LoadStatus::TOO_LARGE;
        return bad;
      }
      FieldMap map;
      map.m_contents = std::string{contents};
      map.m_status = LoadStatus::OK;
      return map;
    }

    /**
     * Reads an .fmap field map file from the deploy directory of the robot
     * program (src/main/deploy in the robot project). If the read fails, this
     * method prints a warning and returns a field map that is not valid.
     *
     * @param deployRelativePath The field map file relative to the deploy
     *        directory, for example "field" or "maps/field.fmap". ".fmap" is
     *        added when missing
     * @return The loaded field map. Check IsValid()
     */
    static FieldMap FromDeployFolder(std::string_view deployRelativePath) {
      if (deployRelativePath.empty()) {
        WarnLoader("no field map file name given");
        return FieldMap{};
      }
      std::optional<std::string> contents =
          ReadDeployFileForNT(deployRelativePath, ".fmap", "field map");
      if (!contents) {
        FieldMap bad;
        bad.m_status = LoadStatus::READ_FAILED;
        return bad;
      }
      return FromString(*contents);
    }

    /** @return True if this holds a usable field map */
    bool IsValid() const { return m_status == LoadStatus::OK; }

    /** @return Why this field map is or is not usable */
    LoadStatus GetLoadStatus() const { return m_status; }

    /** @return The field map's size in bytes, or 0 when invalid */
    size_t SizeBytes() const { return m_contents.size(); }

   private:
    friend class Limelight;
    std::string m_contents;
    LoadStatus m_status = LoadStatus::NO_CONTENTS;
  };

  /**
   * Publishes a shared field map on the "limelightshared" table. Flushes
   * NetworkTables immediately. Publishing is only a NetworkTables write. All
   * file IO and validation happened when the FieldMap was created. While the
   * shared map is not empty, every Limelight on the network localizes with it
   * instead of its uploaded map.
   *
   * @param fieldMap The field map to publish. The method ignores a field map
   *        that is not valid and prints a warning
   */
  static void SetSharedMap(const FieldMap& fieldMap) {
    if (!fieldMap.IsValid()) {
      WarnLoader("ignoring invalid shared field map");
      return;
    }
    wpi::nt::NetworkTableInstance::GetDefault()
        .GetTable("limelightshared")
        ->GetEntry("map_set")
        .SetString(fieldMap.m_contents);
    FlushNT();
  }

  /**
   * Clears the shared field map. Flushes NetworkTables immediately. Every
   * Limelight on the network returns to its own uploaded map.
   */
  static void ClearSharedMap() {
    wpi::nt::NetworkTableInstance::GetDefault()
        .GetTable("limelightshared")
        ->GetEntry("map_set")
        .SetString("");
    FlushNT();
  }

  /**
   * Sets the priority AprilTag ID for tx/ty targeting.
   * @param id Priority tag ID
   */
  void SetPriorityTagIDOverride(int id) {
    m_table->GetEntry("priorityid")
        .SetDouble(static_cast<int>(
            detail::ClampArg(id, -1, std::numeric_limits<int>::max())));
  }

  /** Clears the priority AprilTag ID override. tx/ty targeting returns to the
   *  target selection of the pipeline. */
  void ClearPriorityTagIDOverride() {
    m_table->GetEntry("priorityid").SetDouble(-1);
  }

  /**
   * Sets the LED behavior.
   */
  void SetLEDMode(LEDMode mode) {
    m_table->GetEntry("ledMode").SetDouble(static_cast<int>(mode));
  }

  /**
   * Sets the crop window. The crop window in the web interface must be fully
   * open (as large as possible).
   * @param cropXMin Minimum X value (-1 to 1)
   * @param cropXMax Maximum X value (-1 to 1)
   * @param cropYMin Minimum Y value (-1 to 1)
   * @param cropYMax Maximum Y value (-1 to 1)
   */
  void SetCropWindowOverride(double cropXMin, double cropXMax, double cropYMin,
                             double cropYMax) {
    double xMin = detail::ClampArg(std::min(cropXMin, cropXMax), -1, 1, -1);
    double xMax = detail::ClampArg(std::max(cropXMin, cropXMax), -1, 1, 1);
    double yMin = detail::ClampArg(std::min(cropYMin, cropYMax), -1, 1, -1);
    double yMax = detail::ClampArg(std::max(cropYMin, cropYMax), -1, 1, 1);
    std::array<double, 4> crop{xMin, xMax, yMin, yMax};
    m_table->GetEntry("crop").SetDoubleArray(crop);
  }

  /** Clears the crop window override, returning to the full image. */
  void ClearCropWindowOverride() {
    std::array<double, 4> full{-1, 1, -1, 1};
    m_table->GetEntry("crop").SetDoubleArray(full);
  }

  /**
   * Sets the keystone modification for the crop window.
   * @param horizontal Horizontal keystone value (-0.95 to 0.95)
   * @param vertical Vertical keystone value (-0.95 to 0.95)
   */
  void SetKeystoneOverride(double horizontal, double vertical) {
    std::array<double, 2> keystone{detail::ClampArg(horizontal, -0.95, 0.95, 0),
                                   detail::ClampArg(vertical, -0.95, 0.95, 0)};
    m_table->GetEntry("keystone_set").SetDoubleArray(keystone);
  }

  /** Clears the keystone override. */
  void ClearKeystoneOverride() {
    std::array<double, 2> zeros{0, 0};
    m_table->GetEntry("keystone_set").SetDoubleArray(zeros);
  }

  /**
   * Moves the 3D targeting point away from the center of the primary in-view
   * fiducial. The offset is in target space (the coordinate system of the
   * target: x = forward, y = left, z = up).
   *
   * @param forward Forward (x) offset from the target in meters
   * @param left Left (y) offset from the target in meters
   * @param up Up (z) offset from the target in meters
   */
  void SetFiducial3DOffsetOverride(double forward, double left, double up) {
    if (!(std::isfinite(forward) && std::isfinite(left) && std::isfinite(up))) {
      return;
    }
    std::array<double, 3> offset{forward, left, up};
    m_table->GetEntry("fiducial_offset_set").SetDoubleArray(offset);
  }

  /**
   * Moves the 3D targeting point away from the center of the primary in-view
   * fiducial. The offset is in target space (the coordinate system of the
   * target). Same as SetFiducial3DOffsetOverride(double, double, double) with a
   * Translation3d.
   *
   * @param offset Offset from the target in meters (x = forward, y = left,
   *        z = up)
   */
  void SetFiducial3DOffsetOverride(const wpi::math::Translation3d& offset) {
    SetFiducial3DOffsetOverride(offset.X().value(), offset.Y().value(),
                                offset.Z().value());
  }

  /** Clears the fiducial 3D offset override. The 3D targeting point returns to
   *  the center of the primary in-view fiducial. */
  void ClearFiducial3DOffsetOverride() {
    std::array<double, 3> zeros{0, 0, 0};
    m_table->GetEntry("fiducial_offset_set").SetDoubleArray(zeros);
  }

  /**
   * (ADVANCED) Sets the individual robot orientation of this camera for the
   * MegaTag2 algorithm. Call this every loop. Most robots should use
   * SetSharedRobotOrientation() instead. It updates every camera at one time.
   *
   * Every call makes this camera ignore the shared orientation from
   * SetSharedRobotOrientation(). Use SetUseSharedOrientation() to make it use
   * the shared orientation again.
   *
   * @param yawDegrees Robot yaw in degrees.
   * @param flush True to flush NetworkTables immediately. Pass false when you
   *        update several cameras in one loop. Then call FlushNT() one time.
   */
  void SetRobotOrientation(double yawDegrees, bool flush) {
    SetRobotOrientationInternal(yawDegrees, 0, 0, 0, 0, 0, flush);
  }

  /**
   * (ADVANCED) Sets the full individual robot orientation of this camera for
   * the MegaTag2 algorithm. Every call makes this camera ignore the shared
   * orientation (see SetUseSharedOrientation()).
   *
   * @param yaw Robot yaw in degrees.
   * @param yawRate (optional, may be 0) Angular velocity of robot yaw in
   *        degrees per second
   * @param pitch (optional, may be 0) Robot pitch in degrees
   * @param pitchRate (optional, may be 0) Angular velocity of robot pitch in
   *        degrees per second
   * @param roll (optional, may be 0) Robot roll in degrees
   * @param rollRate (optional, may be 0) Angular velocity of robot roll in
   *        degrees per second
   * @param flush True to flush NetworkTables immediately. Pass false when you
   *        update several cameras in one loop. Then call FlushNT() one time.
   */
  void SetRobotOrientation(double yaw, double yawRate, double pitch,
                           double pitchRate, double roll, double rollRate,
                           bool flush) {
    SetRobotOrientationInternal(yaw, yawRate, pitch, pitchRate, roll, rollRate,
                                flush);
  }

  /**
   * (ADVANCED) Controls whether this camera reads the shared orientation from
   * SetSharedRobotOrientation(). Every SetRobotOrientation() call makes the
   * camera ignore the shared orientation. Call this method with true to use
   * the shared orientation again. For example, call SetRobotOrientation one
   * time to seed the camera. Then let the shared orientation drive MegaTag2.
   *
   * @param useShared True to follow the shared orientation. False to use only
   *        the individual orientation of this camera
   */
  void SetUseSharedOrientation(bool useShared) {
    m_table->GetEntry("robot_orientation_ignoreshared_set")
        .SetInteger(useShared ? 0 : 1);
  }

  /**
   * Sets the robot orientation for MegaTag2 on the shared "limelightshared"
   * table. Flushes NetworkTables immediately. Every Limelight on the network
   * reads this table. One call updates all cameras. You do not need to call
   * SetRobotOrientation() for each instance.
   *
   * Each camera follows this shared orientation unless it was opted out. Every
   * SetRobotOrientation() call opts a camera out (see
   * SetUseSharedOrientation()). For example, a turret camera can use its own
   * orientation while every other camera follows the shared value.
   *
   * @param yawDegrees Robot yaw in degrees.
   */
  static void SetSharedRobotOrientation(double yawDegrees) {
    SetSharedRobotOrientation(yawDegrees, 0, 0, 0, 0, 0);
  }

  /**
   * Sets the full shared robot orientation for MegaTag2 on the
   * "limelightshared" table. Flushes NetworkTables immediately. Each camera
   * follows this shared orientation unless it was opted out with
   * SetRobotOrientation() or SetUseSharedOrientation().
   *
   * @param yaw Robot yaw in degrees.
   * @param yawRate (optional, may be 0) Angular velocity of robot yaw in
   *        degrees per second
   * @param pitch (optional, may be 0) Robot pitch in degrees
   * @param pitchRate (optional, may be 0) Angular velocity of robot pitch in
   *        degrees per second
   * @param roll (optional, may be 0) Robot roll in degrees
   * @param rollRate (optional, may be 0) Angular velocity of robot roll in
   *        degrees per second
   */
  static void SetSharedRobotOrientation(double yaw, double yawRate,
                                        double pitch, double pitchRate,
                                        double roll, double rollRate) {
    if (!(std::isfinite(yaw) && std::isfinite(yawRate) &&
          std::isfinite(pitch) && std::isfinite(pitchRate) &&
          std::isfinite(roll) && std::isfinite(rollRate))) {
      return;
    }
    std::array<double, 6> orientation{yaw,       yawRate, pitch,
                                      pitchRate, roll,    rollRate};
    wpi::nt::NetworkTableInstance::GetDefault()
        .GetTable("limelightshared")
        ->GetEntry("robot_orientation_set")
        .SetDoubleArray(orientation);
    FlushNT();
  }

  /**
   * Configures the robot-yaw source for MegaTag2 localization.
   */
  void SetIMUMode(IMUMode mode) {
    if (mode == IMUMode::UNKNOWN) {
      return;
    }
    m_table->GetEntry("imumode_set").SetDouble(static_cast<int>(mode));
  }

  /**
   * Configures the complementary filter alpha for the IMU assist modes (modes 3
   * and 4).
   * @param alpha Default .001. Higher values converge on the assist source
   *        faster.
   */
  void SetIMUAssistAlpha(double alpha) {
    m_table->GetEntry("imuassistalpha_set")
        .SetDouble(detail::ClampArg(alpha, 0.0001, 1, 0.001));
  }

  /**
   * Configures the throttle value. The Limelight skips {@code throttle} frames
   * between processed frames. Set 100-200 while the robot is disabled to reduce
   * heat. Use WithStaleFrameThreshold() with this so GetStatus() does not
   * report the camera as stale.
   *
   * @param throttle Default 0. The camera processes one frame, then skips this
   *        many frames.
   */
  void SetThrottle(int throttle) {
    m_table->GetEntry("throttle_set").SetDouble(std::max(0, throttle));
  }

  /**
   * Overrides the valid AprilTag IDs for localization. Tags not in this list
   * are ignored for robot pose estimation. They do not get the "fielded" flag.
   * @param validIDs Valid AprilTag IDs to track
   */
  void SetFiducialIDFiltersOverride(std::span<const int> validIDs) {
    std::vector<double> validIDsDouble;
    validIDsDouble.reserve(validIDs.size());
    for (int id : validIDs) {
      validIDsDouble.push_back(std::max(0, id));
    }
    m_table->GetEntry("fiducial_id_filters_set").SetDoubleArray(validIDsDouble);
  }

  /** Clears the AprilTag ID filter override. The camera returns to the ID
   *  filters of the pipeline. */
  void ClearFiducialIDFiltersOverride() {
    m_table->GetEntry("fiducial_id_filters_set").SetDoubleArray({});
  }

  /**
   * Overrides the AprilTag detector's downscaling factor.
   */
  void SetFiducialDownscalingOverride(DownscaleOverride downscale) {
    m_table->GetEntry("fiducial_downscale_set")
        .SetDouble(static_cast<int>(downscale));
  }

  /** Clears the AprilTag downscaling override. The camera returns to the
   *  downscale configured in the current pipeline. */
  void ClearFiducialDownscalingOverride() {
    SetFiducialDownscalingOverride(DownscaleOverride::PIPELINE_CONTROL);
  }

  /**
   * Sets the camera pose relative to the robot. You can call this every loop.
   * Use it to track a camera on a moving mechanism (elevator, turret). An
   * all-zero pose is the clear value, so the camera returns to the pose from
   * the web interface.
   *
   * @param forward Forward (x) offset in meters
   * @param left Left (y) offset in meters
   * @param up Up (z) offset in meters
   * @param roll Roll angle in degrees
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   * @param flush True to flush NetworkTables immediately. Pass false when you
   *        update several cameras in one loop. Then call FlushNT() one time.
   */
  void SetCameraPose_RobotSpaceOverride(double forward, double left, double up,
                                        double roll, double pitch, double yaw,
                                        bool flush) {
    PublishCameraPose(forward, left, up, roll, pitch, yaw);
    if (flush) {
      FlushNT();
    }
  }

  /**
   * Sets the camera pose relative to the robot. The camera applies updates
   * live. See SetCameraPose_RobotSpaceOverride(double, double, double, double,
   * double, double, bool).
   *
   * @param cameraPoseRobotSpace The pose of the camera relative to the robot
   *        center (x = forward, y = left, z = up, in meters). An all-zero pose
   *        is the clear value
   * @param flush True to flush NetworkTables immediately. Pass false when you
   *        update several cameras in one loop. Then call FlushNT() one time.
   */
  void SetCameraPose_RobotSpaceOverride(
      const wpi::math::Pose3d& cameraPoseRobotSpace, bool flush) {
    SetCameraPose_RobotSpaceOverride(
        cameraPoseRobotSpace.X().value(), cameraPoseRobotSpace.Y().value(),
        cameraPoseRobotSpace.Z().value(),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().X().value()),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().Y().value()),
        detail::RadiansToDegrees(cameraPoseRobotSpace.Rotation().Z().value()),
        flush);
  }

  /**
   * Clears the camera pose override. Flushes NetworkTables immediately. The
   * camera returns to the camera pose configured in its web interface.
   */
  void ClearCameraPose_RobotSpaceOverride() {
    std::array<double, 6> zeros{0, 0, 0, 0, 0, 0};
    m_table->GetEntry("camerapose_robotspace_set").SetDoubleArray(zeros);
    FlushNT();
  }

  /**
   * Sends data to a running python snapscript via llrobot.
   */
  void SetPythonScriptData(std::span<const double> outgoingPythonData) {
    m_table->GetEntry("llrobot").SetDoubleArray(outgoingPythonData);
  }

  /**
   * Triggers a snapshot capture. The Limelight firmware rate-limits this.
   */
  void TriggerSnapshot() {
    double current = m_table->GetEntry("snapshot").GetDouble(0);
    m_table->GetEntry("snapshot").SetDouble(current + 1);
  }

  /**
   * Enables or pauses the rewind buffer recording.
   * @param enabled True to enable recording. False to pause. Default true on
   *        supported platforms.
   */
  void SetRewindEnabled(bool enabled) {
    m_table->GetEntry("rewind_enable_set").SetDouble(enabled ? 1 : 0);
  }

  /**
   * Triggers a rewind capture with the given duration. The maximum duration is
   * 165 seconds. The camera rate-limits this.
   * @param durationSeconds Duration of the rewind capture in seconds (maximum
   *        165)
   */
  void TriggerRewindCapture(double durationSeconds) {
    std::vector<double> currentArray =
        m_table->GetEntry("capture_rewind").GetDoubleArray({});
    double counter = !currentArray.empty() ? currentArray[0] : 0;
    std::array<double, 2> capture{counter + 1,
                                  detail::ClampArg(durationSeconds, 1, 165)};
    m_table->GetEntry("capture_rewind").SetDoubleArray(capture);
  }

  /**
   * Flushes NetworkTables immediately. These methods call it automatically: the
   * pose-setting constructors, SetSharedRobotOrientation(),
   * SetPipelineConfigurationOverride(), ClearPipelineConfigurationOverride(),
   * SetSharedMap(), ClearSharedMap(), ClearCameraPose_RobotSpaceOverride(), and
   * SetRobotOrientation() / SetCameraPose_RobotSpaceOverride() when their
   * flush argument is true. The other setters and Clear methods do not flush.
   * Call it yourself after a group of them.
   */
  static void FlushNT() { wpi::nt::NetworkTableInstance::GetDefault().Flush(); }

 private:
  static LimelightResults DecodeFrame(std::span<const uint8_t> envelope,
                                      int64_t receiveTimestampMicros) {
    LimelightResults results = Decode(envelope, receiveTimestampMicros);
    results.fromLiveSubscriber = true;
    return results;
  }

  void WarnIfProtocolNewer() {
    int64_t version = m_protocolVersionSubscriber.Get();
    if (version > SUPPORTED_PROTOCOL_VERSION && !m_protocolWarningPrinted) {
      m_protocolWarningPrinted = true;
      std::fprintf(
          stderr,
          "Limelight - %s: camera protocol version %" PRId64
          " is newer than this library supports (%d). Update Limelight.h. "
          "Results may be missing or wrong.\n",
          m_name.c_str(), static_cast<int64_t>(version),
          SUPPORTED_PROTOCOL_VERSION);
    }
  }

  // Status of the already-decoded latest results. Kept separate from
  // GetStatus() so health telemetry can run inside GetLatestResults() without
  // recursing into it.
  Status ComputeStatus() const {
    if (m_latestResults.receiveTimestampSeconds == 0) {
      return Status::NO_DATA;
    }
    if (m_latestResults.GetAgeSeconds() > m_staleFrameSeconds) {
      return Status::STALE;
    }
    if (!m_latestResults.error.empty()) {
      return Status::DECODE_ERROR;
    }
    return Status::OK;
  }

  const PoseEstimateConfig& ConfigFor(PoseEstimateType type) const {
    return detail::UsesMT2(type) ? m_megaTag2Config : m_megaTag1Config;
  }

  static int CountFieldedFiducials(
      const std::vector<FiducialTarget>& fiducials) {
    int count = 0;
    for (const FiducialTarget& fiducial : fiducials) {
      if (fiducial.fielded) {
        count++;
      }
    }
    return count;
  }

  static bool HasNonfiniteData(const PoseEstimate& estimate) {
    return !(std::isfinite(estimate.pose.X().value()) &&
             std::isfinite(estimate.pose.Y().value()) &&
             std::isfinite(estimate.pose.Rotation().Radians().value()) &&
             std::isfinite(estimate.timestampSeconds.value()) &&
             std::isfinite(estimate.latencyMillis) &&
             std::isfinite(estimate.avgTagDistanceMeters) &&
             IsUsableStdDev(estimate.stdDevs[0]) &&
             IsUsableStdDev(estimate.stdDevs[1]) &&
             IsUsableStdDev(estimate.stdDevs[2]));
  }

  static bool IsUsableStdDev(double value) {
    return std::isfinite(value) && value > 0;
  }

  // Loader failures print a single line to stdout (flushed, so it shows up in
  // robot service logs) and never throw.
  static void WarnLoader(const std::string& message) {
    std::fprintf(stdout, "Limelight - %s\n", message.c_str());
    std::fflush(stdout);
  }

  static bool EndsWithIgnoreCase(std::string_view text,
                                 std::string_view suffix) {
    if (suffix.size() > text.size()) {
      return false;
    }
    std::string_view tail = text.substr(text.size() - suffix.size());
    for (size_t i = 0; i < suffix.size(); i++) {
      if (std::tolower(static_cast<unsigned char>(tail[i])) !=
          std::tolower(static_cast<unsigned char>(suffix[i]))) {
        return false;
      }
    }
    return true;
  }

  static std::optional<std::string> ReadDeployFileForNT(
      std::string_view deployRelativePath, std::string_view extension,
      const char* what) {
    try {
      std::string name{deployRelativePath};
      if (!EndsWithIgnoreCase(name, extension)) {
        name.append(extension);
      }
      std::filesystem::path relative{name};
      if (relative.is_absolute()) {
        WarnLoader(std::string{what} +
                   " path must be relative to the deploy directory: " + name);
        return std::nullopt;
      }
      return ReadFileForNT(
          std::filesystem::path{wpi::filesystem::GetDeployDirectory()} /
              relative,
          what);
    } catch (const std::exception& e) {
      WarnLoader(std::string{"could not resolve the deploy directory: "} +
                 e.what());
      return std::nullopt;
    }
  }

  static std::optional<std::string> ReadFileForNT(
      const std::filesystem::path& path, const char* what) {
    try {
      std::ifstream file{path, std::ios::binary};
      if (!file) {
        WarnLoader(std::string{"could not read "} + what + " file " +
                   path.string());
        return std::nullopt;
      }
      std::string contents{std::istreambuf_iterator<char>{file},
                           std::istreambuf_iterator<char>{}};
      if (file.bad()) {
        WarnLoader(std::string{"could not read "} + what + " file " +
                   path.string());
        return std::nullopt;
      }
      return contents;
    } catch (const std::exception& e) {
      WarnLoader(std::string{"could not read "} + what + " file: " + e.what());
      return std::nullopt;
    }
  }

  void PublishCameraPose(double forward, double left, double up, double roll,
                         double pitch, double yaw) {
    if (!(std::isfinite(forward) && std::isfinite(left) && std::isfinite(up) &&
          std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw))) {
      return;
    }
    std::array<double, 6> pose{forward, left, up, roll, pitch, yaw};
    m_table->GetEntry("camerapose_robotspace_set").SetDoubleArray(pose);
  }

  void SetRobotOrientationInternal(double yaw, double yawRate, double pitch,
                                   double pitchRate, double roll,
                                   double rollRate, bool flush) {
    if (!(std::isfinite(yaw) && std::isfinite(yawRate) &&
          std::isfinite(pitch) && std::isfinite(pitchRate) &&
          std::isfinite(roll) && std::isfinite(rollRate))) {
      return;
    }
    SetUseSharedOrientation(false);
    std::array<double, 6> orientation{yaw,       yawRate, pitch,
                                      pitchRate, roll,    rollRate};
    m_table->GetEntry("robot_orientation_set").SetDoubleArray(orientation);
    if (flush) {
      FlushNT();
    }
  }

  void UnpublishTelemetry() {
    for (std::unique_ptr<detail::PoseTelemetry>& telemetry : m_poseTelemetry) {
      telemetry.reset();
    }
    m_poseCountsTelemetry.reset();
    m_fieldTypeTelemetry.reset();
    m_connectedTelemetry.reset();
    m_customCalibrationTelemetry.reset();
    m_statusTelemetry.reset();
  }

  void PublishPoseTelemetry(const PoseEstimate& estimate, bool liveFrame) {
    if (!m_telemetryEnabled) {
      return;
    }
    if (liveFrame && ComputeStatus() != Status::OK) {
      PublishHealthTelemetry();
      return;
    }
    size_t i = static_cast<size_t>(estimate.type);
    if (i >= kPoseEstimateTypeCount) {
      return;
    }
    if (!m_poseTelemetry[i]) {
      std::shared_ptr<wpi::nt::NetworkTable> fieldTable = SharedFieldTable();
      if (!m_fieldTypeTelemetry) {
        m_fieldTypeTelemetry = fieldTable->GetStringTopic(".type").Publish();
        m_fieldTypeTelemetry->Set("Field2d");
      }
      m_poseTelemetry[i] = std::make_unique<detail::PoseTelemetry>(
          *m_telemetryTable, *fieldTable, m_name, estimate.type);
    }
    m_poseTelemetry[i]->Publish(estimate);
  }

  void CountPoseEstimate(const PoseEstimate& estimate) {
    if (!m_telemetryEnabled) {
      return;
    }
    if (!m_poseCountsTelemetry) {
      m_poseCountsTelemetry =
          std::make_unique<detail::PoseCountsTelemetry>(*m_telemetryTable);
    }
    m_poseCountsTelemetry->Count(estimate);
  }

  void PublishHealthTelemetry() {
    if (!m_telemetryEnabled) {
      return;
    }
    if (!m_connectedTelemetry) {
      m_connectedTelemetry =
          m_telemetryTable->GetBooleanTopic("connected").Publish();
      m_customCalibrationTelemetry =
          m_telemetryTable->GetBooleanTopic("customCalibration").Publish();
      m_statusTelemetry = m_telemetryTable->GetStringTopic("status").Publish();
    }
    Status status = ComputeStatus();
    bool connected = status == Status::OK || status == Status::DECODE_ERROR;
    m_connectedTelemetry->Set(connected);
    m_customCalibrationTelemetry->Set(
        m_latestResults.intrinsics.customCalibration);
    m_statusTelemetry->Set(StatusName(status));
    if (status != Status::OK) {
      for (std::unique_ptr<detail::PoseTelemetry>& telemetry :
           m_poseTelemetry) {
        if (telemetry) {
          telemetry->Clear();
        }
      }
    }
  }

  static std::string_view StatusName(Status status) {
    switch (status) {
      case Status::OK:
        return "OK";
      case Status::NO_DATA:
        return "NO_DATA";
      case Status::STALE:
        return "STALE";
      case Status::DECODE_ERROR:
      default:
        return "DECODE_ERROR";
    }
  }

  static std::shared_ptr<wpi::nt::NetworkTable> SharedFieldTable() {
    return wpi::nt::NetworkTableInstance::GetDefault().GetTable(
        std::string{TELEMETRY_TABLE} + "/Field");
  }

  std::string m_name;
  std::shared_ptr<wpi::nt::NetworkTable> m_table;
  std::shared_ptr<wpi::nt::NetworkTable> m_telemetryTable;
  wpi::nt::RawSubscriber m_resultsSubscriber;
  wpi::nt::IntegerSubscriber m_protocolVersionSubscriber;

  LimelightResults m_latestResults;
  int64_t m_lastDecodedTimestamp = -1;

  double m_staleFrameSeconds = STALE_FRAME_SECONDS;
  bool m_protocolWarningPrinted = false;
  PoseEstimateConfig m_megaTag1Config = PoseEstimateConfig::DefaultMT1();
  PoseEstimateConfig m_megaTag2Config = PoseEstimateConfig::DefaultMT2();
  bool m_telemetryEnabled = true;
  std::array<std::unique_ptr<detail::PoseTelemetry>, kPoseEstimateTypeCount>
      m_poseTelemetry;
  std::unique_ptr<detail::PoseCountsTelemetry> m_poseCountsTelemetry;
  std::optional<wpi::nt::StringPublisher> m_fieldTypeTelemetry;
  std::optional<wpi::nt::BooleanPublisher> m_connectedTelemetry;
  std::optional<wpi::nt::BooleanPublisher> m_customCalibrationTelemetry;
  std::optional<wpi::nt::StringPublisher> m_statusTelemetry;
};

}  // namespace limelight
