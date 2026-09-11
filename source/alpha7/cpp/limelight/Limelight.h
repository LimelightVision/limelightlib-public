// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "limelight/BarcodeTarget.h"
#include "limelight/CameraIntrinsics.h"
#include "limelight/ClassifierTarget.h"
#include "limelight/CounterTarget.h"
#include "limelight/DetectorTarget.h"
#include "limelight/DownscaleOverride.h"
#include "limelight/FiducialTarget.h"
#include "limelight/FieldMap.h"
#include "limelight/Geometry.h"
#include "limelight/HardwareData.h"
#include "limelight/IMUData.h"
#include "limelight/IMUMode.h"
#include "limelight/LEDMode.h"
#include "limelight/LimelightResults.h"
#include "limelight/LimelightTarget.h"
#include "limelight/LoadStatus.h"
#include "limelight/OCRTarget.h"
#include "limelight/PipelineConfiguration.h"
#include "limelight/PipelineConfigurationOverrideState.h"
#include "limelight/PoseEstimate.h"
#include "limelight/PoseEstimateConfig.h"
#include "limelight/PoseEstimateType.h"
#include "limelight/RetroTarget.h"
#include "limelight/RewindData.h"
#include "limelight/RobotOrientationData.h"
#include "limelight/SharedMapState.h"
#include "limelight/Status.h"
#include "limelight/Timebase.h"
#include "limelight/detail/Decode.h"
#include "limelight/detail/Loaders.h"
#include "limelight/detail/Math.h"
#include "limelight/detail/MsgPackReader.h"
#include "limelight/detail/PoseCountsTelemetry.h"
#include "limelight/detail/PoseTelemetry.h"
#include "wpi/math/geometry/Pose3d.hpp"
#include "wpi/math/geometry/Rotation3d.hpp"
#include "wpi/math/geometry/Translation3d.hpp"
#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/NetworkTable.hpp"
#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/RawTopic.hpp"
#include "wpi/nt/StringTopic.hpp"
#include "wpi/nt/ntcore_cpp.hpp"
#include "wpi/system/Errors.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/util/UsageReporting.hpp"
namespace limelight {

/**
 * <h2>Quick start</h2>
 *
 * <h3>Visual servoing</h3>
 * @code
 * double turnKp = -0.02;
 * using limelight::Limelight;
 * Limelight camera{"limelight"};
 * // Systemcore USB vision instances:
 * // Limelight camera{Limelight::SYSTEMCORE_USB0};
 * // Limelight camera{Limelight::SYSTEMCORE_USB1};
 *
 * // Each loop
 * double turn = joystick.GetRightX();
 * if (aimingEnabled && camera.HasTarget()) {
 *   turn = turnKp * camera.GetTX().value();
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
 *                                      estimate.timestamp,
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
 * limelight::Limelight::SetSharedRobotOrientation(robotYaw);
 * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(
 *          limelight::PoseEstimateType::MT2_WPIBLUE)) {
 *   poseEstimator.AddVisionMeasurement(estimate.pose,
 *                                      estimate.timestamp,
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
 * using namespace wpi::units::literals;
 * using limelight::Limelight;
 * using limelight::PoseEstimateConfig;
 * using limelight::PoseEstimateType;
 * wpi::units::radian_t untrusted{PoseEstimateConfig::UNTRUSTED};
 *
 * // Start from the default so you do not have to set every value.
 * PoseEstimateConfig mt1Config =
 *     PoseEstimateConfig::DefaultMT1()
 *         .WithMinTagCount(1)
 *         // MT1 needs low-ambiguity perspectives.
 *         .WithMaxSingleTagAmbiguity(0.7)
 *         // Trust a single tag only within 3 m.
 *         .WithMaxSingleTagDistance(3_m)
 *         // With several tags, the average distance must be under 6 m.
 *         .WithMaxAvgTagDistance(6_m)
 *         .WithMinAvgTagArea(0.05)
 *         // Reject estimates outside the field, with 0.5 m of margin.
 *         .WithFieldBounds(16.541_m, 8.069_m)
 *         .WithFieldBoundsMargin(0.5_m)
 *         // 0.5 m base, minimum 0.05 m, maximum 2 m.
 *         .WithStdDevXY(0.5_m, 0.05_m, 2_m)
 *         // Never fuse vision heading.
 *         .WithStdDevTheta(untrusted, untrusted, untrusted)
 *         // Linear scaling with distance.
 *         .WithStdDevDistanceScaling(1.0, 0_m, 6_m)
 *         .WithStdDevTagCountDivision(0.5);
 *
 * PoseEstimateConfig mt2Config =
 *     PoseEstimateConfig::DefaultMT2()
 *         .WithMinTagCount(1)
 *         // MT2 handles ambiguous perspectives, so accept every tag.
 *         .WithMaxSingleTagAmbiguity(1.0)
 *         // 0 disables this check.
 *         .WithMaxSingleTagDistance(0_m)
 *         .WithMaxAvgTagDistance(8_m)
 *         .WithMinAvgTagArea(0.02)
 *         .WithFieldBounds(16.541_m, 8.069_m)
 *         .WithFieldBoundsMargin(0.5_m)
 *         .WithStdDevXY(0.3_m, 0.0001_m, 2_m)
 *         .WithStdDevTheta(untrusted, untrusted, untrusted)
 *         // Square root of distance for MT2.
 *         .WithStdDevDistanceScaling(0.5, 0_m, 8_m)
 *         .WithStdDevTagCountDivision(0.5);
 *
 * wpi::math::Pose3d cameraPoseRobotSpace{0.30_m, 0.0_m, 0.20_m,
 *                                        wpi::math::Rotation3d{}};
 * Limelight camera = Limelight{"limelight", cameraPoseRobotSpace}
 *                        .WithPoseEstimateConfig_MT1(mt1Config)
 *                        .WithPoseEstimateConfig_MT2(mt2Config)
 *                        // Keep accepted and rejected poses on dashboards.
 *                        .WithTelemetry(true);
 * bool useMegaTag2 = true;
 *
 * // Each robot loop
 * PoseEstimateType type = useMegaTag2 ? PoseEstimateType::MT2_WPIBLUE
 *                                     : PoseEstimateType::MT1_WPIBLUE;
 * if (useMegaTag2) {
 *   // Use your robot pose heading here, not your raw IMU reading.
 *   Limelight::SetSharedRobotOrientation(robotYaw);
 * }
 * for (const auto& estimate : camera.ReadAcceptedPoseEstimates(type)) {
 *   poseEstimator.AddVisionMeasurement(estimate.pose, estimate.timestamp,
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
 * GetStatus() or LimelightResults::GetAge() to detect a stale camera.
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
  static constexpr size_t MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES =
      detail::kMaxPipelineConfigurationOverrideBytes;

  /** Maximum size in bytes of a shared field map. Both this library and the
   *  camera reject larger publishes. */
  static constexpr size_t MAX_SHARED_MAP_BYTES = detail::kMaxSharedMapBytes;

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
    wpi::util::ReportUsage("Limelight", m_name);
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
    PublishCameraPose(cameraPoseRobotSpace.X(), cameraPoseRobotSpace.Y(),
                      cameraPoseRobotSpace.Z(),
                      cameraPoseRobotSpace.Rotation().X(),
                      cameraPoseRobotSpace.Rotation().Y(),
                      cameraPoseRobotSpace.Rotation().Z());
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
   * @param roll Roll angle in degrees
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   */
  Limelight(std::string_view name, wpi::units::meter_t forward,
            wpi::units::meter_t left, wpi::units::meter_t up,
            wpi::units::degree_t roll, wpi::units::degree_t pitch,
            wpi::units::degree_t yaw)
      : Limelight(name, wpi::math::Pose3d{
                            wpi::units::meter_t{forward},
                            wpi::units::meter_t{left}, wpi::units::meter_t{up},
                            wpi::math::Rotation3d{roll, pitch, yaw}}) {}

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
  Limelight& WithStaleFrameThreshold(wpi::units::second_t seconds) & {
    m_staleFrameSeconds = detail::ClampArg(seconds.value(), 0.01,
                                           std::numeric_limits<double>::max(),
                                           STALE_FRAME_SECONDS);
    return *this;
  }

  /** @copydoc WithStaleFrameThreshold(wpi::units::second_t)& */
  Limelight&& WithStaleFrameThreshold(wpi::units::second_t seconds) && {
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
   * The camera-level {@code counts/} topics add up every estimate type that
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
   * {@code receiveTimestamp} stays 0. Pose estimates built from the
   * result are rejected as NO_TIMESTAMP. Use
   * Decode(std::span<const uint8_t>, int64_t) for pose estimate tests.
   *
   * @param envelope The raw MessagePack results dump
   * @return The decoded results. {@code receiveTimestamp} stays 0
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
    results.parseLatency =
        wpi::units::millisecond_t{std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - start)
                                      .count()};
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
   * @param receiveTimestampNanos NetworkTables receive time in nanoseconds
   *        (local NetworkTables timebase, WPILib alpha-7 or newer), for
   *        example wpi::nt::TimestampedRaw::time
   */
  static LimelightResults Decode(std::span<const uint8_t> envelope,
                                 int64_t receiveTimestampNanos) {
    LimelightResults results = Decode(envelope);
    results.receiveTimestamp =
        wpi::units::second_t{receiveTimestampNanos / NT_TICKS_PER_SECOND};
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
  wpi::units::degree_t GetTX() { return GetLatestResults().tx; }

  /**
   * @return Vertical offset from crosshair to target in degrees. This value
   *         holds the last received value after a disconnect. Check HasTarget()
   *         every loop.
   */
  wpi::units::degree_t GetTY() { return GetLatestResults().ty; }

  /**
   * @return Horizontal offset from principal pixel to target in degrees
   *         (crosshair-independent)
   */
  wpi::units::degree_t GetTXNoCrosshair() {
    return GetLatestResults().txNoCrosshair;
  }

  /**
   * @return Vertical offset from principal pixel to target in degrees
   *         (crosshair-independent)
   */
  wpi::units::degree_t GetTYNoCrosshair() {
    return GetLatestResults().tyNoCrosshair;
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
  wpi::units::meter_t GetTargetDistance() {
    return GetLatestResults().targetDistance;
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
  wpi::units::millisecond_t GetTargetingLatency() {
    return GetLatestResults().targetingLatency;
  }

  /**
   * @return Capture latency in milliseconds
   */
  wpi::units::millisecond_t GetCaptureLatency() {
    return GetLatestResults().captureLatency;
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
   * @return The active camera intrinsics, scaled to the current processing
   *         resolution: the camera matrix, the OpenCV distortion coefficients,
   *         and the FOV
   */
  const CameraIntrinsics& GetCameraIntrinsics() {
    return GetLatestResults().intrinsics;
  }

  /**
   * @return True if the camera uses a user-uploaded calibration instead of a
   *         built-in default. See CameraIntrinsics::customCalibration. False
   *         before the first frame arrives.
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
   *                                      estimate.timestamp,
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
   * nanoseconds (local NetworkTables timebase). This method does not consume
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
   * @param results The results envelope to build from
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
    estimate.latency = results.captureLatency + results.targetingLatency;
    estimate.timestamp = results.receiveTimestamp - estimate.latency;
    estimate.reportedTagCount = results.reportedTagCount;
    estimate.fieldedTagCount = CountFieldedFiducials(results.fiducialTargets);
    estimate.tagSpan = results.tagSpan;
    estimate.avgTagDistance = results.avgTagDistance;
    estimate.avgTagAreaPercent = results.avgTagAreaPercent;
    estimate.reportedStdDevs = detail::ReportedStdDevs(results, type);
    estimate.rawFiducials = results.fiducialTargets;

    if (!results.error.empty()) {
      estimate.rejectionFlags |= PoseEstimateConfig::REJECT_DECODE_ERROR;
    }
    if (results.receiveTimestamp.value() <= 0 ||
        estimate.timestamp.value() <= 0) {
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
    if (estimate.fieldedTagCount > 0 && estimate.avgTagDistance.value() <= 0) {
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
   * using limelight::Limelight;
   * auto aiming = PipelineConfiguration::FromDeployFolder("aiming");
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
      detail::WarnLoader("ignoring invalid pipeline configuration override");
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
   * value comes from the latest results envelope. It can differ from the value
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
   *         latest results envelope
   */
  PipelineConfigurationOverrideState GetPipelineConfigurationOverrideState() {
    return GetLatestResults().pipelineConfigurationOverrideState;
  }

  /**
   * @return True if the camera localizes with the shared field map published
   *         with SetSharedMap(). The value comes from the latest results
   * envelope
   */
  bool IsSharedMapActive() { return GetLatestResults().sharedMapActive; }

  /**
   * @return The camera's shared field map state from the latest results
   *         envelope
   */
  SharedMapState GetSharedMapState() {
    return GetLatestResults().sharedMapState;
  }

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
      detail::WarnLoader("ignoring invalid shared field map");
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
  void SetFiducial3DOffsetOverride(wpi::units::meter_t forward,
                                   wpi::units::meter_t left,
                                   wpi::units::meter_t up) {
    std::array<double, 3> offset{forward.value(), left.value(), up.value()};
    if (!std::ranges::all_of(offset,
                             [](double v) { return std::isfinite(v); })) {
      return;
    }
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
    SetFiducial3DOffsetOverride(offset.X(), offset.Y(), offset.Z());
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
   * @param yaw Robot yaw in degrees.
   * @param flush True to flush NetworkTables immediately. Pass false when you
   *        update several cameras in one loop. Then call FlushNT() one time.
   */
  void SetRobotOrientation(wpi::units::degree_t yaw, bool flush) {
    SetRobotOrientationInternal(
        yaw, wpi::units::degrees_per_second_t{0}, wpi::units::degree_t{0},
        wpi::units::degrees_per_second_t{0}, wpi::units::degree_t{0},
        wpi::units::degrees_per_second_t{0}, flush);
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
  void SetRobotOrientation(wpi::units::degree_t yaw,
                           wpi::units::degrees_per_second_t yawRate,
                           wpi::units::degree_t pitch,
                           wpi::units::degrees_per_second_t pitchRate,
                           wpi::units::degree_t roll,
                           wpi::units::degrees_per_second_t rollRate,
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
   * @param yaw Robot yaw in degrees.
   */
  static void SetSharedRobotOrientation(wpi::units::degree_t yaw) {
    SetSharedRobotOrientation(
        yaw, wpi::units::degrees_per_second_t{0}, wpi::units::degree_t{0},
        wpi::units::degrees_per_second_t{0}, wpi::units::degree_t{0},
        wpi::units::degrees_per_second_t{0});
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
  static void SetSharedRobotOrientation(
      wpi::units::degree_t yaw, wpi::units::degrees_per_second_t yawRate,
      wpi::units::degree_t pitch, wpi::units::degrees_per_second_t pitchRate,
      wpi::units::degree_t roll, wpi::units::degrees_per_second_t rollRate) {
    std::array<double, 6> orientation{yaw.value(),   yawRate.value(),
                                      pitch.value(), pitchRate.value(),
                                      roll.value(),  rollRate.value()};
    if (!std::ranges::all_of(orientation,
                             [](double v) { return std::isfinite(v); })) {
      return;
    }

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
  void SetCameraPose_RobotSpaceOverride(wpi::units::meter_t forward,
                                        wpi::units::meter_t left,
                                        wpi::units::meter_t up,
                                        wpi::units::degree_t roll,
                                        wpi::units::degree_t pitch,
                                        wpi::units::degree_t yaw, bool flush) {
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
        cameraPoseRobotSpace.X(), cameraPoseRobotSpace.Y(),
        cameraPoseRobotSpace.Z(), cameraPoseRobotSpace.Rotation().X(),
        cameraPoseRobotSpace.Rotation().Y(),
        cameraPoseRobotSpace.Rotation().Z(), flush);
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
   * Resets the object counter of a counter pipeline to zero. The camera resets
   * when the value changes, so calling this every loop resets every loop.
   */
  void ResetCounter() {
    double current = m_table->GetEntry("reset_counter_set").GetDouble(0);
    m_table->GetEntry("reset_counter_set").SetDouble(current + 1);
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
   * @param duration Duration of the rewind capture in seconds (maximum
   *        165)
   */
  void TriggerRewindCapture(wpi::units::second_t duration) {
    std::vector<double> currentArray =
        m_table->GetEntry("capture_rewind").GetDoubleArray({});
    double counter = !currentArray.empty() ? currentArray[0] : 0;
    std::array<double, 2> capture{counter + 1,
                                  detail::ClampArg(duration.value(), 1, 165)};
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
                                      int64_t receiveTimestampNanos) {
    LimelightResults results = Decode(envelope, receiveTimestampNanos);
    results.fromLiveSubscriber = true;
    return results;
  }

  void WarnIfProtocolNewer() {
    int64_t version = m_protocolVersionSubscriber.Get();
    if (version > SUPPORTED_PROTOCOL_VERSION && !m_protocolWarningPrinted) {
      m_protocolWarningPrinted = true;
      WPILIB_ReportWarning(
          "Limelight - {}: camera protocol version {} is newer than this "
          "library supports ({}). Update Limelight.h. Results may be missing "
          "or wrong.",
          m_name, version, SUPPORTED_PROTOCOL_VERSION);
    }
  }

  // Status of the already-decoded latest results. Kept separate from
  // GetStatus() so health telemetry can run inside GetLatestResults() without
  // recursing into it.
  Status ComputeStatus() const {
    if (m_latestResults.receiveTimestamp.value() == 0) {
      return Status::NO_DATA;
    }
    if (m_latestResults.GetAge().value() > m_staleFrameSeconds) {
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
             std::isfinite(estimate.timestamp.value()) &&
             std::isfinite(estimate.latency.value()) &&
             std::isfinite(estimate.avgTagDistance.value()) &&
             IsUsableStdDev(estimate.stdDevs[0]) &&
             IsUsableStdDev(estimate.stdDevs[1]) &&
             IsUsableStdDev(estimate.stdDevs[2]));
  }

  static bool IsUsableStdDev(double value) {
    return std::isfinite(value) && value > 0;
  }

  void PublishCameraPose(wpi::units::meter_t forward, wpi::units::meter_t left,
                         wpi::units::meter_t up, wpi::units::degree_t roll,
                         wpi::units::degree_t pitch, wpi::units::degree_t yaw) {
    std::array<double, 6> pose{forward.value(), left.value(),  up.value(),
                               roll.value(),    pitch.value(), yaw.value()};
    if (!std::ranges::all_of(pose, [](double v) { return std::isfinite(v); })) {
      return;
    }
    m_table->GetEntry("camerapose_robotspace_set").SetDoubleArray(pose);
  }

  void SetRobotOrientationInternal(wpi::units::degree_t yaw,
                                   wpi::units::degrees_per_second_t yawRate,
                                   wpi::units::degree_t pitch,
                                   wpi::units::degrees_per_second_t pitchRate,
                                   wpi::units::degree_t roll,
                                   wpi::units::degrees_per_second_t rollRate,
                                   bool flush) {
    std::array<double, 6> orientation{yaw.value(),   yawRate.value(),
                                      pitch.value(), pitchRate.value(),
                                      roll.value(),  rollRate.value()};
    if (!std::ranges::all_of(orientation,
                             [](double v) { return std::isfinite(v); })) {
      return;
    }
    SetUseSharedOrientation(false);

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
