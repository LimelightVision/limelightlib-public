// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.RawSubscriber;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.TimestampedRaw;
import org.wpilib.system.Filesystem;

/**
 * Interface to one Limelight camera or Systemcore vision instance.
 *
 * <h2>Quick start</h2>
 *
 * <h3>Visual servoing</h3>
 *
 * {@snippet :
 * double turnKp = -0.02;
 * Limelight camera = new Limelight("limelight");
 * //Limelight camera = new Limelight(Limelight.SYSTEMCORE_USB0);
 * //Limelight camera = new Limelight(Limelight.SYSTEMCORE_USB1);
 *
 * // Each loop
 * double turn = joystick.getRightX();
 * if(aimingEnabled && camera.hasTarget()){
 *      turn = turnKp * camera.getTXDegrees();
 * }
 * drivetrain.arcadeDrive(forward, turn);
 * }
 *
 * <h3>MegaTag1 localization</h3>
 *
 * {@snippet :
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);
 *
 * // Each robot loop
 * PoseEstimateType type = PoseEstimateType.MT1_WPIBLUE;
 * for (var estimate : camera.readAcceptedPoseEstimates(type)) {
 *   poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <h3>MegaTag2 localization</h3>
 *
 * <p>Publish robot yaw before reading the queue each robot loop.
 *
 * {@snippet :
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);
 *
 * // Each robot loop
 * Limelight.setSharedRobotOrientation(robotYawDegrees);
 * PoseEstimateType type = PoseEstimateType.MT2_WPIBLUE;
 * for (var estimate : camera.readAcceptedPoseEstimates(type)) {
 *   poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <h3>Configured MegaTag1 and MegaTag2</h3>
 *
 * <p>This example configures every filter and standard-deviation scaling term. MT1 scales XY
 * uncertainty linearly with distance and clamps it to 0.05-2.0 meters. MT2 scales it by the square
 * root of distance and clamps it to 0.0001-2.0 meters. Both divide by the square root of the
 * fielded tag count. Vision heading is not fused. The field bounds are for the 2026 welded field.
 * Tune the other thresholds on your robot.
 *
 * {@snippet :
 * double untrusted = PoseEstimateConfig.UNTRUSTED;
 * // Start from the default so you do not have to set every value.
 * PoseEstimateConfig mt1Config = PoseEstimateConfig.defaultMT1()
 *     .withMinTagCount(1)
 *     .withMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity perspectives
 *     .withMaxSingleTagDistance(3.0) // Trust a single tag only within 3 m
 *     .withMaxAvgTagDistance(6.0) // With several tags, average distance under 6 m
 *     .withMinAvgTagArea(0.05)
 *     .withFieldBounds(16.541, 8.069) // Reject estimates outside the field
 *     .withFieldBoundsMargin(0.5) // Allow 0.5 m past the field walls
 *     .withStdDevXY(0.5, 0.05, 2.0) // 0.5 base, minimum 0.05, maximum 2.0
 *     .withStdDevTheta(untrusted, untrusted, untrusted) // Never fuse vision heading
 *     .withStdDevDistanceScaling(1.0, 0.0, 6.0) // Linear scaling
 *     .withStdDevTagCountDivision(0.5);
 *
 * PoseEstimateConfig mt2Config = PoseEstimateConfig.defaultMT2()
 *     .withMinTagCount(1)
 *     .withMaxSingleTagAmbiguity(1.0) // MT2 handles ambiguous perspectives, accept all
 *     .withMaxSingleTagDistance(0.0) // 0 disables this check
 *     .withMaxAvgTagDistance(8.0)
 *     .withMinAvgTagArea(0.02)
 *     .withFieldBounds(16.541, 8.069)
 *     .withFieldBoundsMargin(0.5)
 *     .withStdDevXY(0.3, 0.0001, 2.0)
 *     .withStdDevTheta(untrusted, untrusted, untrusted)
 *     .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Square root of distance for MT2
 *     .withStdDevTagCountDivision(0.5);
 *
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace)
 *     .withPoseEstimateConfig_MT1(mt1Config)
 *     .withPoseEstimateConfig_MT2(mt2Config)
 *     .withTelemetry(true); // Keep accepted and rejected poses visible on dashboards
 * boolean useMegaTag2 = true;
 *
 * // Each robot loop
 * PoseEstimateType type = PoseEstimateType.MT1_WPIBLUE;
 * if (useMegaTag2) {
 *   type = PoseEstimateType.MT2_WPIBLUE;
 *   // Use your robot pose heading here, not your raw IMU reading.
 *   Limelight.setSharedRobotOrientation(robotYawDegrees);
 * }
 * for (var estimate : camera.readAcceptedPoseEstimates(type)) {
 *   poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <p>All values from one {@link #getLatestResults()} call come from the same frame. This includes
 * tx, botpose, fiducials, latencies, and timestamps.
 *
 * <p>All 3D data uses the right-handed NWU convention in every space (field, robot, camera,
 * target): x = forward, y = left, z = up. Raw pose arrays are [x, y, z, roll, pitch, yaw] in meters
 * and degrees.
 *
 * <p>Getters decode at most one new frame. Otherwise they return cached data. After a disconnect,
 * they return the values of the last frame. Use {@link #getStatus()} or {@link
 * LimelightResults#getAgeSeconds()} to detect a stale camera.
 *
 * <p>Use this class from one thread, the robot loop.
 */
public class Limelight implements AutoCloseable {
  static final double[] EMPTY = new double[0];

  /**
   * Length of every pose array: [x, y, z, roll, pitch, yaw]. The camera always sends six values,
   * all zero when there is no pose. Pose and standard deviation arrays default to six zeros, so
   * index access is safe before the first frame.
   */
  public static final int POSE_ARRAY_LENGTH = 6;

  static final double[][] EMPTY_CORNERS = new double[0][];
  static final Pose2d[] EMPTY_POSES = new Pose2d[0];
  static final int[] REJECTION_FLAGS = {
    PoseEstimateConfig.REJECT_TAG_COUNT,
    PoseEstimateConfig.REJECT_AMBIGUITY,
    PoseEstimateConfig.REJECT_TAG_DISTANCE,
    PoseEstimateConfig.REJECT_TAG_AREA,
    PoseEstimateConfig.REJECT_FIELD_BOUNDS,
    PoseEstimateConfig.REJECT_DECODE_ERROR,
    PoseEstimateConfig.REJECT_NONFINITE,
    PoseEstimateConfig.REJECT_NO_TIMESTAMP,
    PoseEstimateConfig.REJECT_MISSING_POSE,
    PoseEstimateConfig.REJECT_BAD_METADATA,
    PoseEstimateConfig.REJECT_NO_FIELDED_TAGS
  };
  static final String[] REJECTION_NAMES = {
    "TAG_COUNT",
    "AMBIGUITY",
    "TAG_DISTANCE",
    "TAG_AREA",
    "FIELD_BOUNDS",
    "DECODE_ERROR",
    "NONFINITE",
    "NO_TIMESTAMP",
    "MISSING_POSE",
    "BAD_METADATA",
    "NO_FIELDED_TAGS"
  };

  /**
   * Default stale threshold. {@link #getStatus} reports {@link Status#STALE} for a frame older than
   * this. Override per instance with {@link #withStaleFrameThreshold}.
   */
  public static final double STALE_FRAME_SECONDS = 0.25;

  /**
   * NetworkTables timestamp ticks per second for the WPILib build this library targets. WPILib 2027
   * alpha-7 and later use 1e9. Alpha-6 and earlier use 1e6. Divide a NetworkTables timestamp by
   * this value to get seconds.
   */
  public static final double NT_TICKS_PER_SECOND = 1.0e6;

  /**
   * The highest camera protocol version ("protover") this library understands. If a camera
   * publishes a higher version, the library prints one warning.
   */
  public static final int SUPPORTED_PROTOCOL_VERSION = 1;

  private static PipelineConfigurationOverrideState parseOverrideState(String s) {
    switch (s) {
      case "off":
        return PipelineConfigurationOverrideState.OFF;
      case "active":
        return PipelineConfigurationOverrideState.ACTIVE;
      case "forcedOff":
        return PipelineConfigurationOverrideState.FORCED_OFF;
      case "noString":
        return PipelineConfigurationOverrideState.NO_STRING;
      case "parseError":
        return PipelineConfigurationOverrideState.PARSE_ERROR;
      default:
        return PipelineConfigurationOverrideState.UNKNOWN;
    }
  }

  private static SharedMapState parseSharedMapState(String s) {
    switch (s) {
      case "off":
        return SharedMapState.OFF;
      case "active":
        return SharedMapState.ACTIVE;
      case "parseError":
        return SharedMapState.PARSE_ERROR;
      default:
        return SharedMapState.UNKNOWN;
    }
  }

  /**
   * Maximum size in bytes of a pipeline configuration override. Both this library and the camera
   * reject larger publishes.
   */
  public static final int MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES = 4096;

  /**
   * Maximum size in bytes of a shared field map. Both this library and the camera reject larger
   * publishes.
   */
  public static final int MAX_SHARED_MAP_BYTES = 262144;

  /**
   * Name of the vision instance running on Systemcore USB port 0: {@code new
   * Limelight(Limelight.SYSTEMCORE_USB0)}.
   */
  public static final String SYSTEMCORE_USB0 = "limelightsc0";

  /** Name of the vision instance running on Systemcore USB port 1. */
  public static final String SYSTEMCORE_USB1 = "limelightsc1";

  /** Name of the vision instance running on Systemcore USB port 2. */
  public static final String SYSTEMCORE_USB2 = "limelightsc2";

  /** Name of the vision instance running on Systemcore USB port 3. */
  public static final String SYSTEMCORE_USB3 = "limelightsc3";

  /**
   * Root telemetry table. Each camera uses {@code limelight_telemetry/<name>/}. The shared Field2d
   * table is {@code limelight_telemetry/Field}.
   */
  public static final String TELEMETRY_TABLE = "limelight_telemetry";

  private final String name;
  private final NetworkTable table;
  private final NetworkTable telemetryTable;
  private final RawSubscriber resultsSubscriber;
  private final IntegerSubscriber protocolVersionSubscriber;

  private LimelightResults latestResults = new LimelightResults();
  private long lastDecodedTimestamp = -1;

  private double staleFrameSeconds = STALE_FRAME_SECONDS;
  private boolean protocolWarningPrinted;
  private PoseEstimateConfig megaTag1Config = PoseEstimateConfig.defaultMT1();
  private PoseEstimateConfig megaTag2Config = PoseEstimateConfig.defaultMT2();
  private boolean telemetryEnabled = true;
  private final PoseTelemetry[] poseTelemetry = new PoseTelemetry[PoseEstimateType.values().length];
  private PoseCountsTelemetry poseCountsTelemetry;
  private StringPublisher fieldTypeTelemetry;
  private BooleanPublisher connectedTelemetry;
  private BooleanPublisher customCalibrationTelemetry;
  private StringPublisher statusTelemetry;

  /** Creates an interface to the Limelight with the default name ("limelight"). */
  public Limelight() {
    this("limelight");
  }

  /**
   * Creates an interface to the given Limelight.
   *
   * @param name The camera or vision instance's NetworkTables name (e.g. "limelight" or
   *     "limelight-left")
   */
  public Limelight(String name) {
    if (name == null || name.isEmpty()) {
      name = "limelight";
    }
    this.name = name;
    this.latestResults.fromLiveSubscriber = true;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.telemetryTable = NetworkTableInstance.getDefault().getTable(TELEMETRY_TABLE + "/" + name);
    this.resultsSubscriber =
        table
            .getRawTopic("results_msgpack")
            .subscribe(
                "msgpack",
                new byte[0],
                PubSubOption.periodic(0.005),
                PubSubOption.SEND_ALL,
                PubSubOption.pollStorage(20));
    this.protocolVersionSubscriber = table.getIntegerTopic("protover").subscribe(0);
  }

  /**
   * Creates an interface to the given Limelight and sets its camera pose in robot space.
   *
   * @param name The NetworkTables name of the camera, for example "limelight" or "limelight-left"
   * @param cameraPoseRobotSpace The pose of the camera relative to the robot center (x = forward, y
   *     = left, z = up, in meters). This overrides the camera pose configured in the web interface.
   *     Call {@link #clearCameraPose_RobotSpaceOverride} to return to it. Null publishes nothing.
   *     An all-zero pose is the clear value, so the camera keeps the pose from the web interface.
   */
  public Limelight(String name, Pose3d cameraPoseRobotSpace) {
    this(name);
    if (cameraPoseRobotSpace == null) {
      return;
    }
    publishCameraPose(
        cameraPoseRobotSpace.getX(),
        cameraPoseRobotSpace.getY(),
        cameraPoseRobotSpace.getZ(),
        Units.radiansToDegrees(cameraPoseRobotSpace.getRotation().getX()),
        Units.radiansToDegrees(cameraPoseRobotSpace.getRotation().getY()),
        Units.radiansToDegrees(cameraPoseRobotSpace.getRotation().getZ()));
    flushNT();
  }

  /**
   * Creates an interface to the given Limelight and sets its camera pose in robot space. This
   * overrides the camera pose configured in the web interface. Call {@link
   * #clearCameraPose_RobotSpaceOverride} to return to it. An all-zero pose is the clear value, so
   * the camera keeps the pose from the web interface.
   *
   * @param name The NetworkTables name of the camera, for example "limelight" or "limelight-left"
   * @param forward Forward (x) offset in meters
   * @param left Left (y) offset in meters
   * @param up Up (z) offset in meters
   * @param rollDegrees Roll angle in degrees
   * @param pitchDegrees Pitch angle in degrees
   * @param yawDegrees Yaw angle in degrees
   */
  public Limelight(
      String name,
      double forward,
      double left,
      double up,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    this(
        name,
        new Pose3d(
            forward,
            left,
            up,
            new Rotation3d(
                Units.degreesToRadians(rollDegrees),
                Units.degreesToRadians(pitchDegrees),
                Units.degreesToRadians(yawDegrees))));
  }

  /**
   * Sets the maximum age of the newest frame. Above this age, {@link #getStatus} reports {@link
   * Status#STALE} and {@link #hasTarget} returns false. Raise this value when you use {@link
   * #setThrottle}.
   *
   * @param seconds Stale threshold in seconds. Positive infinity disables stale detection.
   * @return this, for chaining with the constructor
   */
  public Limelight withStaleFrameThreshold(double seconds) {
    this.staleFrameSeconds = clampArg(seconds, 0.01, Double.MAX_VALUE, STALE_FRAME_SECONDS);
    return this;
  }

  /**
   * Configures filtering and fusion standard deviations for every MegaTag1 pose estimate. MegaTag1
   * computes heading from tag geometry only. It usually needs stricter ambiguity and tag-count
   * gates than MegaTag2. Rejected estimates report {@code isValid() == false}. {@link
   * PoseEstimate#rejectionFlags} lists the failed checks.
   *
   * @param config The MegaTag1 configuration. The configuration is copied. Later changes to the
   *     object have no effect until you attach it again. Null resets to {@link
   *     PoseEstimateConfig#defaultMT1()}
   * @return this, for chaining with the constructor
   */
  public Limelight withPoseEstimateConfig_MT1(PoseEstimateConfig config) {
    this.megaTag1Config =
        (config != null) ? new PoseEstimateConfig(config) : PoseEstimateConfig.defaultMT1();
    return this;
  }

  /**
   * Configures filtering and fusion standard deviations for every MegaTag2 pose estimate.
   *
   * @param config The MegaTag2 configuration. The configuration is copied. Later changes to the
   *     object have no effect until you attach it again. Null resets to {@link
   *     PoseEstimateConfig#defaultMT2()}
   * @return this, for chaining with the constructor
   */
  public Limelight withPoseEstimateConfig_MT2(PoseEstimateConfig config) {
    this.megaTag2Config =
        (config != null) ? new PoseEstimateConfig(config) : PoseEstimateConfig.defaultMT2();
    return this;
  }

  /**
   * Enables or disables automatic pose-estimate telemetry. Enabled by default. Pose estimates
   * publish under {@code limelight_telemetry/<name>/<type>/}:
   *
   * <ul>
   *   <li>{@code accepted} and {@code rejected}: Pose2d struct arrays for field views
   *   <li>{@code rejectionReasons}: for example {@code TAG_COUNT|AMBIGUITY}
   * </ul>
   *
   * <p>The camera-level {@code counts/} topics add up every estimate type that {@link
   * #readPoseEstimateQueue} and {@link #readAcceptedPoseEstimates} process. One estimate can
   * increment several rejection-reason totals. The same estimates appear on the shared {@code
   * limelight_telemetry/Field} Field2d table for Glass and Elastic. Camera health publishes as
   * {@code connected}, {@code customCalibration}, and {@code status}. Pose displays clear when the
   * camera is unhealthy. Disabling telemetry unpublishes everything and resets the counters.
   *
   * @return this, for chaining with the constructor
   */
  public Limelight withTelemetry(boolean enabled) {
    if (telemetryEnabled && !enabled) {
      unpublishTelemetry();
    }
    this.telemetryEnabled = enabled;
    return this;
  }

  private void unpublishTelemetry() {
    for (int i = 0; i < poseTelemetry.length; i++) {
      if (poseTelemetry[i] != null) {
        poseTelemetry[i].unpublish();
        poseTelemetry[i] = null;
      }
    }
    if (poseCountsTelemetry != null) {
      poseCountsTelemetry.unpublish();
      poseCountsTelemetry = null;
    }
    if (fieldTypeTelemetry != null) {
      fieldTypeTelemetry.close();
      fieldTypeTelemetry = null;
    }
    if (connectedTelemetry != null) {
      connectedTelemetry.close();
      customCalibrationTelemetry.close();
      statusTelemetry.close();
      connectedTelemetry = null;
      customCalibrationTelemetry = null;
      statusTelemetry = null;
    }
  }

  /**
   * Returns the camera's NetworkTables name.
   *
   * @return The camera's NetworkTables name
   */
  public String getName() {
    return name;
  }

  /**
   * Returns the msgpack envelope protocol version from the camera.
   *
   * @return The msgpack envelope protocol version from the camera. 0 if the camera has not
   *     connected or runs old software
   */
  public int getProtocolVersion() {
    return (int) protocolVersionSubscriber.get();
  }

  // ---- Results ----

  /**
   * Returns the latest results envelope. Decodes the newest MessagePack frame if one arrived since
   * the last call. This method is cheap to call many times per loop. The decoded envelope is cached
   * until a new frame arrives. This method never consumes the frame queue. You can use it together
   * with {@link #readResultsQueue}.
   *
   * @return The latest LimelightResults, never null. Use {@link #getStatus} for camera health. Use
   *     {@link LimelightResults#valid} for target validity. Each new frame produces a new object.
   *     Call this getter every loop instead of keeping the returned object.
   */
  public LimelightResults getLatestResults() {
    warnIfProtocolNewer();
    TimestampedRaw raw = resultsSubscriber.getAtomic();
    if (raw.timestamp != lastDecodedTimestamp && raw.value.length > 0) {
      latestResults = decodeFrame(raw.value, raw.timestamp);
      lastDecodedTimestamp = raw.timestamp;
    }
    publishHealthTelemetry();
    return latestResults;
  }

  private void warnIfProtocolNewer() {
    long version = protocolVersionSubscriber.get();
    if (version > SUPPORTED_PROTOCOL_VERSION && !protocolWarningPrinted) {
      protocolWarningPrinted = true;
      reportWarning(
          "Limelight - "
              + name
              + ": camera protocol version "
              + version
              + " is newer than this library supports ("
              + SUPPORTED_PROTOCOL_VERSION
              + "). Update Limelight.java. Results may be missing or wrong.");
    }
  }

  /**
   * Decodes a results envelope from raw MessagePack bytes. Decode failures are reported in {@link
   * LimelightResults#error}. This method never throws. Use it for unit tests.
   *
   * <p>{@code receiveTimestampSeconds} stays 0. Pose estimates built from the result are rejected
   * as NO_TIMESTAMP. Use {@link #decode(byte[], long)} for pose estimate tests.
   *
   * @param envelope The raw MessagePack results dump
   * @return The decoded results. {@code receiveTimestampSeconds} stays 0
   */
  // Every decoder failure, including index and cast errors, becomes results.error.
  @SuppressWarnings("PMD.AvoidCatchingGenericException")
  public static LimelightResults decode(byte[] envelope) {
    LimelightResults results = new LimelightResults();
    long start = System.nanoTime();
    try {
      decodeResults(new MsgPackReader(envelope), results);
    } catch (RuntimeException e) {
      results.error = "llmsgpack decode error: " + e.getMessage();
    }
    results.parseLatencyMillis = (System.nanoTime() - start) / 1.0e6;
    return results;
  }

  /**
   * Decodes a results envelope and stamps it with its NetworkTables receive time. This matches live
   * decoding and timestamp handling. For log replay: record raw frames with {@link
   * #getLatestRawFrame()} or {@link #readRawFrameQueue()}. Then pass the bytes and the timestamp to
   * this method. Estimates built from the result have latency-compensated timestamps.
   *
   * @param envelope The raw MessagePack results dump
   * @param receiveTimestampMicros NetworkTables receive time in microseconds (local NetworkTables
   *     timebase, WPILib alpha-6 or older), for example {@link TimestampedRaw#timestamp}
   */
  public static LimelightResults decode(byte[] envelope, long receiveTimestampMicros) {
    LimelightResults results = decode(envelope);
    results.receiveTimestampSeconds = receiveTimestampMicros / NT_TICKS_PER_SECOND;
    return results;
  }

  private static LimelightResults decodeFrame(byte[] envelope, long receiveTimestampMicros) {
    LimelightResults results = decode(envelope, receiveTimestampMicros);
    results.fromLiveSubscriber = true;
    return results;
  }

  /**
   * Returns the current health of the camera as seen by this robot program.
   *
   * @return The current health of the camera as seen by this robot program. {@link Status#OK} means
   *     a decodable frame arrived within the stale threshold. The threshold is {@link
   *     #STALE_FRAME_SECONDS} unless you override it with {@link #withStaleFrameThreshold}.
   */
  public Status getStatus() {
    getLatestResults();
    return computeStatus();
  }

  // Status of the already-decoded latestResults. Kept separate from getStatus() so
  // health telemetry can run inside getLatestResults() without recursing into it.
  private Status computeStatus() {
    if (latestResults.receiveTimestampSeconds == 0) {
      return Status.NO_DATA;
    }
    if (latestResults.getAgeSeconds() > staleFrameSeconds) {
      return Status.STALE;
    }
    if (!latestResults.error.isEmpty()) {
      return Status.DECODE_ERROR;
    }
    return Status.OK;
  }

  /**
   * Returns true if the camera is reachable.
   *
   * @return True if the camera is reachable. The status is not {@link Status#NO_DATA} and not
   *     {@link Status#STALE}
   */
  public boolean isConnected() {
    Status status = getStatus();
    return status == Status.OK || status == Status.DECODE_ERROR;
  }

  /**
   * Releases this instance's NetworkTables subscriptions and removes its telemetry topics. Call
   * from test teardown. Robot code normally never needs this.
   */
  @Override
  public void close() {
    resultsSubscriber.close();
    protocolVersionSubscriber.close();
    unpublishTelemetry();
    telemetryEnabled = false;
  }

  /**
   * Returns true if the camera is connected, sends fresh frames, and has at least one valid target.
   *
   * @return True if the camera is connected, sends fresh frames, and has at least one valid target.
   *     Returns false when the newest frame is older than the stale threshold.
   */
  public boolean hasTarget() {
    LimelightResults results = getLatestResults();
    return computeStatus() == Status.OK && results.valid;
  }

  /**
   * Returns horizontal offset from crosshair to target in degrees.
   *
   * @return Horizontal offset from crosshair to target in degrees. This value holds the last
   *     received value after a disconnect. Check {@link #hasTarget} every loop.
   */
  public double getTXDegrees() {
    return getLatestResults().txDegrees;
  }

  /**
   * Returns vertical offset from crosshair to target in degrees.
   *
   * @return Vertical offset from crosshair to target in degrees. This value holds the last received
   *     value after a disconnect. Check {@link #hasTarget} every loop.
   */
  public double getTYDegrees() {
    return getLatestResults().tyDegrees;
  }

  /**
   * Returns horizontal offset from principal pixel to target in degrees (crosshair-independent).
   *
   * @return Horizontal offset from principal pixel to target in degrees (crosshair-independent)
   */
  public double getTXDegreesNoCrosshair() {
    return getLatestResults().txDegreesNoCrosshair;
  }

  /**
   * Returns vertical offset from principal pixel to target in degrees (crosshair-independent).
   *
   * @return Vertical offset from principal pixel to target in degrees (crosshair-independent)
   */
  public double getTYDegreesNoCrosshair() {
    return getLatestResults().tyDegreesNoCrosshair;
  }

  /**
   * Returns target area as a percentage of the image (0-100).
   *
   * @return Target area as a percentage of the image (0-100). This value holds the last received
   *     value after a disconnect. Check {@link #hasTarget} every loop.
   */
  public double getTargetAreaPercent() {
    return getLatestResults().targetAreaPercent;
  }

  /**
   * Returns 3D distance from the camera to the primary fiducial target in meters.
   *
   * @return 3D distance from the camera to the primary fiducial target in meters. 0 if not
   *     available
   */
  public double getTargetDistanceMeters() {
    return getLatestResults().targetDistanceMeters;
  }

  /**
   * Returns total number of retro, fiducial, detector, classifier, and barcode targets in the
   * latest frame.
   *
   * @return Total number of retro, fiducial, detector, classifier, and barcode targets in the
   *     latest frame
   */
  public int getTargetCount() {
    return getLatestResults().getTargetCount();
  }

  /**
   * Returns active pipeline index (0-9).
   *
   * @return Active pipeline index (0-9). -1 before the first frame arrives
   */
  public int getCurrentPipelineIndex() {
    return (int) getLatestResults().pipelineIndex;
  }

  /**
   * Returns active pipeline type, for example "pipe_fiducial", "pipe_color", or "pipe_detector".
   *
   * @return Active pipeline type, for example "pipe_fiducial", "pipe_color", or "pipe_detector"
   */
  public String getCurrentPipelineType() {
    return getLatestResults().pipelineType;
  }

  /**
   * Returns targeting/pipeline latency in milliseconds.
   *
   * @return Targeting/pipeline latency in milliseconds
   */
  public double getTargetingLatencyMillis() {
    return getLatestResults().targetingLatencyMillis;
  }

  /**
   * Returns capture latency in milliseconds.
   *
   * @return Capture latency in milliseconds
   */
  public double getCaptureLatencyMillis() {
    return getLatestResults().captureLatencyMillis;
  }

  /**
   * Returns iMU state from the latest frame.
   *
   * @return IMU state from the latest frame
   */
  public IMUData getIMUData() {
    return getLatestResults().imu;
  }

  /**
   * Returns hardware/system stats from the latest frame.
   *
   * @return Hardware/system stats from the latest frame
   */
  public HardwareData getHardwareData() {
    return getLatestResults().hardware;
  }

  /**
   * Returns the active camera intrinsics, scaled to the current processing resolution.
   *
   * @return The active camera intrinsics: the camera matrix, the OpenCV distortion coefficients,
   *     and the FOV
   */
  public CameraIntrinsics getCameraIntrinsics() {
    return getLatestResults().intrinsics;
  }

  /**
   * Returns true if the camera uses a user-uploaded calibration instead of a built-in default.
   *
   * @return True if the camera uses a user-uploaded calibration instead of a built-in default. See
   *     {@link CameraIntrinsics#customCalibration}. False before the first frame arrives.
   */
  public boolean isUsingCustomCalibration() {
    return getLatestResults().intrinsics.customCalibration;
  }

  /**
   * Returns data set by a python snapscript via llpython.
   *
   * @return Data set by a python snapscript via llpython
   */
  public double[] getPythonScriptData() {
    return getLatestResults().pythonOutput;
  }

  // ---- Pose Estimates ----

  static double[] poseArray(LimelightResults results, PoseEstimateType type) {
    return switch (type) {
      case MT1_WPIBLUE -> results.robotPoseMT1WpiBlue;
      case MT1_WPIRED -> results.robotPoseMT1WpiRed;
      case MT2_WPIBLUE -> results.robotPoseMT2WpiBlue;
      case MT2_WPIRED -> results.robotPoseMT2WpiRed;
    };
  }

  private static double[] reportedStdDevs(LimelightResults results, PoseEstimateType type) {
    return switch (type) {
      case MT1_WPIBLUE, MT1_WPIRED -> results.stdDevsMT1;
      case MT2_WPIBLUE, MT2_WPIRED -> results.stdDevsMT2;
    };
  }

  static boolean usesMT2(PoseEstimateType type) {
    return switch (type) {
      case MT1_WPIBLUE, MT1_WPIRED -> false;
      case MT2_WPIBLUE, MT2_WPIRED -> true;
    };
  }

  static boolean centeredOrigin(PoseEstimateType type) {
    return switch (type) {
      case MT1_WPIBLUE, MT1_WPIRED, MT2_WPIBLUE, MT2_WPIRED -> false;
    };
  }

  private PoseEstimateConfig configFor(PoseEstimateType type) {
    return switch (type) {
      case MT1_WPIBLUE, MT1_WPIRED -> megaTag1Config;
      case MT2_WPIBLUE, MT2_WPIRED -> megaTag2Config;
    };
  }

  /**
   * Gets the pose estimate of the given type from the newest frame. This getter can return the same
   * frame many times, also after a disconnect. Use {@link #readAcceptedPoseEstimates} for fusion.
   * Use {@link #getStatus} for current camera health.
   *
   * @param type Which pose estimate to produce
   */
  public PoseEstimate getPoseEstimate(PoseEstimateType type) {
    return getPoseEstimate(getLatestResults(), type);
  }

  /**
   * Builds a pose estimate of the given type from one results envelope. Applies the {@link
   * PoseEstimateConfig} for the algorithm of the estimate. Use this with {@link #readResultsQueue}
   * to get several estimate types (for example MT1 and MT2) from one queue read.
   *
   * @param results The results envelope to build from
   * @param type Which pose estimate to produce
   */
  public PoseEstimate getPoseEstimate(LimelightResults results, PoseEstimateType type) {
    PoseEstimate estimate = new PoseEstimate();
    estimate.type = type;
    estimate.frameIndex = results.frameIndex;
    estimate.latencyMillis = results.captureLatencyMillis + results.targetingLatencyMillis;
    estimate.timestampSeconds = results.receiveTimestampSeconds - (estimate.latencyMillis / 1000.0);
    estimate.reportedTagCount = results.reportedTagCount;
    estimate.fieldedTagCount = countFieldedFiducials(results.fiducialTargets);
    estimate.tagSpanMeters = results.tagSpanMeters;
    estimate.avgTagDistanceMeters = results.avgTagDistanceMeters;
    estimate.avgTagAreaPercent = results.avgTagAreaPercent;
    estimate.reportedStdDevs = reportedStdDevs(results, type);
    estimate.rawFiducials = results.fiducialTargets;

    if (!results.error.isEmpty()) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_DECODE_ERROR;
    }
    if (results.receiveTimestampSeconds <= 0 || estimate.timestampSeconds <= 0) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_NO_TIMESTAMP;
    }
    if (estimate.fieldedTagCount == 0 && estimate.rawFiducials.length > 0) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_NO_FIELDED_TAGS;
    }

    // A missing pose array or the all-zero sentinel means no estimate.
    double[] poseArray = poseArray(results, type);
    if (poseArray.length < 6
        || poseArray[0] == 0
            && poseArray[1] == 0
            && poseArray[2] == 0
            && poseArray[3] == 0
            && poseArray[4] == 0
            && poseArray[5] == 0) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_MISSING_POSE;
      publishPoseTelemetry(estimate, results.fromLiveSubscriber);
      return estimate;
    }

    estimate.pose = toPose2D(poseArray);
    PoseEstimateConfig config = configFor(type);
    config.applyTo(estimate);

    if (hasNonfiniteData(estimate)) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_NONFINITE;
    }
    if (estimate.fieldedTagCount > 0 && estimate.avgTagDistanceMeters <= 0) {
      estimate.rejectionFlags |= PoseEstimateConfig.REJECT_BAD_METADATA;
    }
    publishPoseTelemetry(estimate, results.fromLiveSubscriber);
    return estimate;
  }

  // ---- Queue reads ----

  /**
   * Decodes and returns every buffered results envelope received since the last queue read. The
   * queue holds 20 frames. If more frames arrive between reads, the oldest frames are discarded.
   * The newest frame also becomes the result of {@link #getLatestResults()}.
   *
   * <p>Use only one queue reading method per camera. The queue reading methods are this method,
   * {@link #readPoseEstimateQueue}, {@link #readAcceptedPoseEstimates}, and {@link
   * #readRawFrameQueue}. To get several values from each frame, read the queue here one time. Then
   * call {@link #getPoseEstimate(LimelightResults, PoseEstimateType)} for each frame. Other getters
   * only read the newest frame. They do not consume the queue.
   *
   * @return All buffered frames, oldest first. Empty if no new frame arrived
   */
  public LimelightResults[] readResultsQueue() {
    warnIfProtocolNewer();
    TimestampedRaw[] frames = resultsSubscriber.readQueue();
    LimelightResults[] out = new LimelightResults[frames.length];
    int count = 0;
    for (TimestampedRaw frame : frames) {
      if (frame.value.length == 0) {
        continue;
      }
      LimelightResults results = decodeFrame(frame.value, frame.timestamp);
      out[count++] = results;
      latestResults = results;
      lastDecodedTimestamp = frame.timestamp;
    }
    if (count != out.length) {
      LimelightResults[] trimmed = new LimelightResults[count];
      System.arraycopy(out, 0, trimmed, 0, count);
      out = trimmed;
    }
    publishHealthTelemetry();
    return out;
  }

  /**
   * Reads every frame received since the last queue read. Returns one pose estimate of the given
   * type for each frame. This lets your pose estimator use every vision update. Check {@link
   * PoseEstimate#isValid} on each estimate before you fuse it.
   *
   * <p>This method consumes the queue. Do not use another queue reading method for this camera.
   *
   * @param type Which pose estimate to produce for each frame
   * @return One pose estimate per frame, oldest first
   */
  public PoseEstimate[] readPoseEstimateQueue(PoseEstimateType type) {
    LimelightResults[] frames = readResultsQueue();
    PoseEstimate[] out = new PoseEstimate[frames.length];
    for (int i = 0; i < frames.length; i++) {
      out[i] = getPoseEstimate(frames[i], type);
      countPoseEstimate(out[i]);
    }
    return out;
  }

  /**
   * Reads every frame received since the last queue read. Returns only the pose estimates that
   * passed validation and filtering. Each queued frame is processed one time. The pose estimator
   * decides if the timestamp is still in range.
   *
   * <pre>
   * PoseEstimateType type = PoseEstimateType.MT2_WPIBLUE;
   * for (PoseEstimate estimate : camera.readAcceptedPoseEstimates(type)) {
   *   poseEstimator.addVisionMeasurement(
   *       estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
   * }
   * </pre>
   *
   * <p>Telemetry shows the newest estimate and keeps rejection counters. Use {@link
   * #readPoseEstimateQueue} to inspect every rejected estimate.
   *
   * <p>This method consumes the queue. Do not use another queue reading method for this camera.
   *
   * @param type Which pose estimate to produce for each frame
   * @return The accepted estimates, oldest first. Empty if no new frame arrived or no estimate
   *     passed
   */
  public PoseEstimate[] readAcceptedPoseEstimates(PoseEstimateType type) {
    PoseEstimate[] all = readPoseEstimateQueue(type);
    int count = 0;
    for (PoseEstimate estimate : all) {
      if (estimate.isValid()) {
        all[count++] = estimate;
      }
    }
    if (count == all.length) {
      return all;
    }
    PoseEstimate[] accepted = new PoseEstimate[count];
    System.arraycopy(all, 0, accepted, 0, count);
    return accepted;
  }

  /**
   * Returns the newest raw results envelope without decoding it. The result has the MessagePack
   * bytes and the NetworkTables receive timestamp in microseconds (local NetworkTables timebase).
   * This method does not consume the frame queue. Use it with {@link #decode(byte[], long)} for
   * logging and replay.
   *
   * @return The newest raw frame. The value is empty if no frame has arrived
   */
  public TimestampedRaw getLatestRawFrame() {
    warnIfProtocolNewer();
    return resultsSubscriber.getAtomic();
  }

  /**
   * Reads every raw envelope received since the last queue read, without decoding. Use this for
   * logging and replay systems. These systems record raw frames and decode them with {@link
   * #decode(byte[], long)}, live or from a log.
   *
   * <p>This method consumes the queue. Do not use another queue reading method for this camera.
   *
   * @return All buffered raw frames, oldest first. Empty if no new frame arrived
   */
  public TimestampedRaw[] readRawFrameQueue() {
    warnIfProtocolNewer();
    return resultsSubscriber.readQueue();
  }

  /**
   * Returns the full 3D robot pose for the given estimate type.
   *
   * @return The full 3D robot pose for the given estimate type. The type selects the origin and the
   *     algorithm
   */
  public Pose3d getRobotPose(PoseEstimateType type) {
    return getLatestResults().getRobotPose(type);
  }

  /**
   * Returns camera pose in robot space (meters, degrees) as a Pose3d.
   *
   * @return Camera pose in robot space (meters, degrees) as a Pose3d
   */
  public Pose3d getCameraPose_RobotSpace() {
    return toPose3D(getLatestResults().cameraPoseRobotSpace);
  }

  private static int countFieldedFiducials(FiducialTarget[] fiducials) {
    int count = 0;
    for (FiducialTarget fiducial : fiducials) {
      if (fiducial.fielded) {
        count++;
      }
    }
    return count;
  }

  private static boolean hasNonfiniteData(PoseEstimate estimate) {
    return !(Double.isFinite(estimate.pose.getX())
        && Double.isFinite(estimate.pose.getY())
        && Double.isFinite(estimate.pose.getRotation().getRadians())
        && Double.isFinite(estimate.timestampSeconds)
        && Double.isFinite(estimate.latencyMillis)
        && Double.isFinite(estimate.avgTagDistanceMeters)
        && isUsableStdDev(estimate.stdDevs.get(0))
        && isUsableStdDev(estimate.stdDevs.get(1))
        && isUsableStdDev(estimate.stdDevs.get(2)));
  }

  private static boolean isUsableStdDev(double value) {
    return Double.isFinite(value) && value > 0;
  }

  private void publishPoseTelemetry(PoseEstimate estimate, boolean liveFrame) {
    if (!telemetryEnabled) {
      return;
    }
    if (liveFrame && computeStatus() != Status.OK) {
      publishHealthTelemetry();
      return;
    }
    int i = estimate.type.ordinal();
    if (poseTelemetry[i] == null) {
      NetworkTable fieldTable = sharedFieldTable();
      if (fieldTypeTelemetry == null) {
        fieldTypeTelemetry = fieldTable.getStringTopic(".type").publish();
        fieldTypeTelemetry.set("Field2d");
      }
      poseTelemetry[i] = new PoseTelemetry(telemetryTable, fieldTable, name, estimate.type);
    }
    poseTelemetry[i].publish(estimate);
  }

  private void countPoseEstimate(PoseEstimate estimate) {
    if (!telemetryEnabled) {
      return;
    }
    if (poseCountsTelemetry == null) {
      poseCountsTelemetry = new PoseCountsTelemetry(telemetryTable);
    }
    poseCountsTelemetry.count(estimate);
  }

  private void publishHealthTelemetry() {
    if (!telemetryEnabled) {
      return;
    }
    if (connectedTelemetry == null) {
      connectedTelemetry = telemetryTable.getBooleanTopic("connected").publish();
      customCalibrationTelemetry = telemetryTable.getBooleanTopic("customCalibration").publish();
      statusTelemetry = telemetryTable.getStringTopic("status").publish();
    }
    Status status = computeStatus();
    boolean connected = status == Status.OK || status == Status.DECODE_ERROR;
    connectedTelemetry.set(connected);
    customCalibrationTelemetry.set(latestResults.intrinsics.customCalibration);
    statusTelemetry.set(status.toString());
    if (status != Status.OK) {
      for (PoseTelemetry telemetry : poseTelemetry) {
        if (telemetry != null) {
          telemetry.clear();
        }
      }
    }
  }

  // Publishes Pose2d structs for AdvantageScope and Field2d arrays for Glass/Elastic.

  private static NetworkTable sharedFieldTable() {
    return NetworkTableInstance.getDefault().getTable(TELEMETRY_TABLE + "/Field");
  }

  static double[] fieldPose(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }

  // ---- Control ----

  /**
   * Switches to the given pipeline.
   *
   * @param pipelineIndex Pipeline index (0-9)
   */
  public void setPipelineIndex(int pipelineIndex) {
    table.getEntry("pipeline").setDouble((int) clampArg(pipelineIndex, 0, 9));
  }

  /**
   * Publishes a pipeline configuration override to this camera. Flushes NetworkTables immediately.
   * The camera runs the override while it is enabled with {@link
   * #setUsePipelineConfigurationOverride}. The ten pipelines on the camera do not change. You can
   * switch between them and the override at any time.
   *
   * <p>Create the {@link PipelineConfiguration} during robot initialization. Publishing is only a
   * NetworkTables write. It does not access the disk. You can switch between several prepared
   * configurations during a match:
   *
   * {@snippet :
   * // robotInit
   * PipelineConfiguration aiming = PipelineConfiguration.fromDeployFolder("aiming");
   * // whenever
   * camera.setPipelineConfigurationOverride(aiming);
   * camera.setUsePipelineConfigurationOverride(true);
   * }
   *
   * @param config The configuration to publish. The method ignores null or a configuration that is
   *     not valid and prints a warning
   */
  public void setPipelineConfigurationOverride(PipelineConfiguration config) {
    if (config == null || !config.isValid()) {
      reportWarning("Limelight - ignoring invalid pipeline configuration override");
      return;
    }
    table.getEntry("codepipeline_set").setString(config.contents);
    flushNT();
  }

  /**
   * Clears the published pipeline configuration override. Flushes NetworkTables immediately. If the
   * override was running, the camera returns immediately to the pipeline selected by {@link
   * #setPipelineIndex}. Otherwise, it returns when the override is next disabled.
   */
  public void clearPipelineConfigurationOverride() {
    table.getEntry("codepipeline_set").setString("");
    flushNT();
  }

  /**
   * Enables or disables the pipeline configuration override. While enabled, the camera runs the
   * pipeline published with {@link #setPipelineConfigurationOverride}. While disabled, the camera
   * runs the pipeline selected by {@link #setPipelineIndex}. The published override stays on the
   * camera in both states. You can switch between the two at any time.
   *
   * @param use True to run the override. False to run the indexed pipeline
   */
  public void setUsePipelineConfigurationOverride(boolean use) {
    table.getEntry("codepipeline_enable_set").setInteger(use ? 1 : 0);
  }

  /**
   * Reads back whether the camera runs the pipeline configuration override. The value comes from
   * the latest results envelope. It can differ from the value requested with {@link
   * #setUsePipelineConfigurationOverride}. For example, the camera web interface can force the
   * override off.
   *
   * @return True if the camera reports that the override is running
   */
  public boolean isPipelineConfigurationOverrideEnabled() {
    return getLatestResults().pipelineConfigurationOverrideActive;
  }

  /**
   * Returns the camera's pipeline configuration override state from the latest results envelope.
   *
   * @return The camera's pipeline configuration override state from the latest results envelope
   */
  public PipelineConfigurationOverrideState getPipelineConfigurationOverrideState() {
    return getLatestResults().pipelineConfigurationOverrideState;
  }

  /**
   * Returns true if the camera localizes with the shared field map published with {@link
   * #setSharedMap}.
   *
   * @return True if the camera localizes with the shared field map published with {@link
   *     #setSharedMap}. The value comes from the latest results envelope
   */
  public boolean isSharedMapActive() {
    return getLatestResults().sharedMapActive;
  }

  /**
   * Returns the camera's shared field map state from the latest results envelope.
   *
   * @return The camera's shared field map state from the latest results envelope
   */
  public SharedMapState getSharedMapState() {
    return getLatestResults().sharedMapState;
  }

  /**
   * Publishes a shared field map on the "limelightshared" table. Flushes NetworkTables immediately.
   * Publishing is only a NetworkTables write. All file IO and validation happened when the {@link
   * FieldMap} was created. While the shared map is not empty, every Limelight on the network
   * localizes with it instead of its uploaded map.
   *
   * @param fieldMap The field map to publish. The method ignores null or a field map that is not
   *     valid and prints a warning
   */
  public static void setSharedMap(FieldMap fieldMap) {
    if (fieldMap == null || !fieldMap.isValid()) {
      reportWarning("Limelight - ignoring invalid shared field map");
      return;
    }
    NetworkTableInstance.getDefault()
        .getTable("limelightshared")
        .getEntry("map_set")
        .setString(fieldMap.contents);
    flushNT();
  }

  /**
   * Clears the shared field map. Flushes NetworkTables immediately. Every Limelight on the network
   * returns to its own uploaded map.
   */
  public static void clearSharedMap() {
    NetworkTableInstance.getDefault().getTable("limelightshared").getEntry("map_set").setString("");
    flushNT();
  }

  // Loader failures print a single line to stdout so they show up in robot
  // service logs, and never throw.
  // Loader failures never throw; any exception becomes a warning and an invalid wrapper.
  @SuppressWarnings("PMD.AvoidCatchingGenericException")
  static Path deployPath(String deployRelativePath, String extension, String what) {
    String name =
        deployRelativePath.toLowerCase(java.util.Locale.ROOT).endsWith(extension)
            ? deployRelativePath
            : deployRelativePath + extension;
    try {
      Path relative = Path.of(name);
      if (relative.isAbsolute()) {
        reportWarning(
            "Limelight - " + what + " path must be relative to the deploy directory: " + name);
        return null;
      }
      return Filesystem.getDeployDirectory().toPath().resolve(relative);
    } catch (Exception e) {
      reportWarning("Limelight - could not resolve the deploy directory: " + e);
      return null;
    }
  }

  @SuppressWarnings("PMD.AvoidCatchingGenericException")
  static String readFileForNT(Path path, String what) {
    if (path == null) {
      return null;
    }
    try {
      return Files.readString(path, StandardCharsets.UTF_8);
    } catch (Exception e) {
      reportWarning("Limelight - could not read " + what + " file " + path + ": " + e);
      return null;
    }
  }

  /**
   * Sets the priority AprilTag ID for tx/ty targeting.
   *
   * @param id Priority tag ID
   */
  public void setPriorityTagIDOverride(int id) {
    table.getEntry("priorityid").setDouble((int) clampArg(id, -1, Integer.MAX_VALUE));
  }

  /**
   * Clears the priority AprilTag ID override. tx/ty targeting returns to the target selection of
   * the pipeline.
   */
  public void clearPriorityTagIDOverride() {
    table.getEntry("priorityid").setDouble(-1);
  }

  /** Sets the LED behavior. */
  public void setLEDMode(LEDMode mode) {
    if (mode == null) {
      return;
    }
    table.getEntry("ledMode").setDouble(mode.ntValue);
  }

  /**
   * Sets the crop window. The crop window in the web interface must be fully open (as large as
   * possible).
   *
   * @param cropXMin Minimum X value (-1 to 1)
   * @param cropXMax Maximum X value (-1 to 1)
   * @param cropYMin Minimum Y value (-1 to 1)
   * @param cropYMax Maximum Y value (-1 to 1)
   */
  public void setCropWindowOverride(
      double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double xMin = clampArg(Math.min(cropXMin, cropXMax), -1, 1, -1);
    double xMax = clampArg(Math.max(cropXMin, cropXMax), -1, 1, 1);
    double yMin = clampArg(Math.min(cropYMin, cropYMax), -1, 1, -1);
    double yMax = clampArg(Math.max(cropYMin, cropYMax), -1, 1, 1);
    table.getEntry("crop").setDoubleArray(new double[] {xMin, xMax, yMin, yMax});
  }

  /** Clears the crop window override, returning to the full image. */
  public void clearCropWindowOverride() {
    table.getEntry("crop").setDoubleArray(new double[] {-1, 1, -1, 1});
  }

  /**
   * Sets the keystone modification for the crop window.
   *
   * @param horizontal Horizontal keystone value (-0.95 to 0.95)
   * @param vertical Vertical keystone value (-0.95 to 0.95)
   */
  public void setKeystoneOverride(double horizontal, double vertical) {
    table
        .getEntry("keystone_set")
        .setDoubleArray(
            new double[] {
              clampArg(horizontal, -0.95, 0.95, 0), clampArg(vertical, -0.95, 0.95, 0)
            });
  }

  /** Clears the keystone override. */
  public void clearKeystoneOverride() {
    table.getEntry("keystone_set").setDoubleArray(new double[] {0, 0});
  }

  /**
   * Moves the 3D targeting point away from the center of the primary in-view fiducial. The offset
   * is in target space (the coordinate system of the target: x = forward, y = left, z = up).
   *
   * @param forward Forward (x) offset from the target in meters
   * @param left Left (y) offset from the target in meters
   * @param up Up (z) offset from the target in meters
   */
  public void setFiducial3DOffsetOverride(double forward, double left, double up) {
    if (!(Double.isFinite(forward) && Double.isFinite(left) && Double.isFinite(up))) {
      return;
    }
    table.getEntry("fiducial_offset_set").setDoubleArray(new double[] {forward, left, up});
  }

  /**
   * Moves the 3D targeting point away from the center of the primary in-view fiducial. The offset
   * is in target space (the coordinate system of the target). Same as {@link
   * #setFiducial3DOffsetOverride(double, double, double)} with a Translation3d.
   *
   * @param offset Offset from the target in meters (x = forward, y = left, z = up). Null is ignored
   */
  public void setFiducial3DOffsetOverride(Translation3d offset) {
    if (offset == null) {
      return;
    }
    setFiducial3DOffsetOverride(offset.getX(), offset.getY(), offset.getZ());
  }

  /**
   * Clears the fiducial 3D offset override. The 3D targeting point returns to the center of the
   * primary in-view fiducial.
   */
  public void clearFiducial3DOffsetOverride() {
    table.getEntry("fiducial_offset_set").setDoubleArray(new double[] {0, 0, 0});
  }

  /**
   * (ADVANCED) Sets the individual robot orientation of this camera for the MegaTag2 algorithm.
   * Call this every loop. Most robots should use {@link #setSharedRobotOrientation} instead. It
   * updates every camera at one time.
   *
   * <p>Every call makes this camera ignore the shared orientation from {@link
   * #setSharedRobotOrientation}. Use {@link #setUseSharedOrientation} to make it use the shared
   * orientation again.
   *
   * @param yawDegrees Robot yaw in degrees.
   * @param flush True to flush NetworkTables immediately. Pass false when you update several
   *     cameras in one loop. Then call {@link #flushNT} one time.
   */
  public void setRobotOrientation(double yawDegrees, boolean flush) {
    setRobotOrientationInternal(yawDegrees, 0, 0, 0, 0, 0, flush);
  }

  /**
   * (ADVANCED) Sets the full individual robot orientation of this camera for the MegaTag2
   * algorithm. Every call makes this camera ignore the shared orientation (see {@link
   * #setUseSharedOrientation}).
   *
   * @param yaw Robot yaw in degrees.
   * @param yawRate (optional, may be 0) Angular velocity of robot yaw in degrees per second
   * @param pitch (optional, may be 0) Robot pitch in degrees
   * @param pitchRate (optional, may be 0) Angular velocity of robot pitch in degrees per second
   * @param roll (optional, may be 0) Robot roll in degrees
   * @param rollRate (optional, may be 0) Angular velocity of robot roll in degrees per second
   * @param flush True to flush NetworkTables immediately. Pass false when you update several
   *     cameras in one loop. Then call {@link #flushNT} one time.
   */
  public void setRobotOrientation(
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate,
      boolean flush) {
    setRobotOrientationInternal(yaw, yawRate, pitch, pitchRate, roll, rollRate, flush);
  }

  private void setRobotOrientationInternal(
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate,
      boolean flush) {
    if (!(Double.isFinite(yaw)
        && Double.isFinite(yawRate)
        && Double.isFinite(pitch)
        && Double.isFinite(pitchRate)
        && Double.isFinite(roll)
        && Double.isFinite(rollRate))) {
      return;
    }
    setUseSharedOrientation(false);
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
    if (flush) {
      flushNT();
    }
  }

  /**
   * (ADVANCED) Controls whether this camera reads the shared orientation from {@link
   * #setSharedRobotOrientation}. Every {@link #setRobotOrientation} call makes the camera ignore
   * the shared orientation. Call this method with true to use the shared orientation again. For
   * example, call {@link #setRobotOrientation} one time to seed the camera. Then let the shared
   * orientation drive MegaTag2.
   *
   * @param useShared True to follow the shared orientation. False to use only the individual
   *     orientation of this camera
   */
  public void setUseSharedOrientation(boolean useShared) {
    table.getEntry("robot_orientation_ignoreshared_set").setInteger(useShared ? 0 : 1);
  }

  /**
   * Sets the robot orientation for MegaTag2 on the shared "limelightshared" table. Flushes
   * NetworkTables immediately. Every Limelight on the network reads this table. One call updates
   * all cameras. You do not need to call {@link #setRobotOrientation} for each instance.
   *
   * <p>Each camera follows this shared orientation unless it was opted out. Every {@link
   * #setRobotOrientation} call opts a camera out (see {@link #setUseSharedOrientation}). For
   * example, a turret camera can use its own orientation while every other camera follows the
   * shared value.
   *
   * @param yawDegrees Robot yaw in degrees.
   */
  public static void setSharedRobotOrientation(double yawDegrees) {
    setSharedRobotOrientation(yawDegrees, 0, 0, 0, 0, 0);
  }

  /**
   * Sets the full shared robot orientation for MegaTag2 on the "limelightshared" table. Flushes
   * NetworkTables immediately. Each camera follows this shared orientation unless it was opted out
   * with {@link #setRobotOrientation} or {@link #setUseSharedOrientation}.
   *
   * @param yaw Robot yaw in degrees.
   * @param yawRate (optional, may be 0) Angular velocity of robot yaw in degrees per second
   * @param pitch (optional, may be 0) Robot pitch in degrees
   * @param pitchRate (optional, may be 0) Angular velocity of robot pitch in degrees per second
   * @param roll (optional, may be 0) Robot roll in degrees
   * @param rollRate (optional, may be 0) Angular velocity of robot roll in degrees per second
   */
  public static void setSharedRobotOrientation(
      double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
    if (!(Double.isFinite(yaw)
        && Double.isFinite(yawRate)
        && Double.isFinite(pitch)
        && Double.isFinite(pitchRate)
        && Double.isFinite(roll)
        && Double.isFinite(rollRate))) {
      return;
    }
    NetworkTableInstance.getDefault()
        .getTable("limelightshared")
        .getEntry("robot_orientation_set")
        .setDoubleArray(new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
    flushNT();
  }

  /** Configures the robot-yaw source for MegaTag2 localization. */
  public void setIMUMode(IMUMode mode) {
    if (mode == null || mode == IMUMode.UNKNOWN) {
      return;
    }
    table.getEntry("imumode_set").setDouble(mode.ntValue);
  }

  /**
   * Configures the complementary filter alpha for the IMU assist modes (modes 3 and 4).
   *
   * @param alpha Default .001. Higher values converge on the assist source faster.
   */
  public void setIMUAssistAlpha(double alpha) {
    table.getEntry("imuassistalpha_set").setDouble(clampArg(alpha, 0.0001, 1, 0.001));
  }

  /**
   * Configures the throttle value. The Limelight skips {@code throttle} frames between processed
   * frames. Set 100-200 while the robot is disabled to reduce heat. Use {@link
   * #withStaleFrameThreshold} with this so {@link #getStatus()} does not report the camera as
   * stale.
   *
   * @param throttle Default 0. The camera processes one frame, then skips this many frames.
   */
  public void setThrottle(int throttle) {
    table.getEntry("throttle_set").setDouble(Math.max(0, throttle));
  }

  /**
   * Overrides the valid AprilTag IDs for localization. Tags not in this list are ignored for robot
   * pose estimation. They do not get the "fielded" flag.
   *
   * @param validIDs Valid AprilTag IDs to track
   */
  public void setFiducialIDFiltersOverride(int[] validIDs) {
    if (validIDs == null) {
      return;
    }
    double[] validIDsDouble = new double[validIDs.length];
    for (int i = 0; i < validIDs.length; i++) {
      validIDsDouble[i] = Math.max(0, validIDs[i]);
    }
    table.getEntry("fiducial_id_filters_set").setDoubleArray(validIDsDouble);
  }

  /**
   * Clears the AprilTag ID filter override. The camera returns to the ID filters of the pipeline.
   */
  public void clearFiducialIDFiltersOverride() {
    table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[0]);
  }

  /** Overrides the AprilTag detector's downscaling factor. */
  public void setFiducialDownscalingOverride(DownscaleOverride downscale) {
    if (downscale == null) {
      return;
    }
    table.getEntry("fiducial_downscale_set").setDouble(downscale.ntValue);
  }

  /**
   * Clears the AprilTag downscaling override. The camera returns to the downscale configured in the
   * current pipeline.
   */
  public void clearFiducialDownscalingOverride() {
    setFiducialDownscalingOverride(DownscaleOverride.PIPELINE_CONTROL);
  }

  /**
   * Sets the camera pose relative to the robot. You can call this every loop. Use it to track a
   * camera on a moving mechanism (elevator, turret). An all-zero pose is the clear value, so the
   * camera returns to the pose from the web interface.
   *
   * @param forward Forward (x) offset in meters
   * @param left Left (y) offset in meters
   * @param up Up (z) offset in meters
   * @param roll Roll angle in degrees
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   * @param flush True to flush NetworkTables immediately. Pass false when you update several
   *     cameras in one loop. Then call {@link #flushNT} one time.
   */
  public void setCameraPose_RobotSpaceOverride(
      double forward,
      double left,
      double up,
      double roll,
      double pitch,
      double yaw,
      boolean flush) {
    publishCameraPose(forward, left, up, roll, pitch, yaw);
    if (flush) {
      flushNT();
    }
  }

  /**
   * Sets the camera pose relative to the robot. The camera applies updates live. See {@link
   * #setCameraPose_RobotSpaceOverride(double, double, double, double, double, double, boolean)}.
   *
   * @param cameraPoseRobotSpace The pose of the camera relative to the robot center (x = forward, y
   *     = left, z = up, in meters). Null is ignored. An all-zero pose is the clear value
   * @param flush True to flush NetworkTables immediately. Pass false when you update several
   *     cameras in one loop. Then call {@link #flushNT} one time.
   */
  public void setCameraPose_RobotSpaceOverride(Pose3d cameraPoseRobotSpace, boolean flush) {
    if (cameraPoseRobotSpace == null) {
      return;
    }
    setCameraPose_RobotSpaceOverride(
        cameraPoseRobotSpace.getX(),
        cameraPoseRobotSpace.getY(),
        cameraPoseRobotSpace.getZ(),
        Math.toDegrees(cameraPoseRobotSpace.getRotation().getX()),
        Math.toDegrees(cameraPoseRobotSpace.getRotation().getY()),
        Math.toDegrees(cameraPoseRobotSpace.getRotation().getZ()),
        flush);
  }

  /**
   * Clears the camera pose override. Flushes NetworkTables immediately. The camera returns to the
   * camera pose configured in its web interface.
   */
  public void clearCameraPose_RobotSpaceOverride() {
    table.getEntry("camerapose_robotspace_set").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
    flushNT();
  }

  private void publishCameraPose(
      double forward, double left, double up, double roll, double pitch, double yaw) {
    if (!(Double.isFinite(forward)
        && Double.isFinite(left)
        && Double.isFinite(up)
        && Double.isFinite(roll)
        && Double.isFinite(pitch)
        && Double.isFinite(yaw))) {
      return;
    }
    table
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(new double[] {forward, left, up, roll, pitch, yaw});
  }

  /** Sends data to a running python snapscript via llrobot. */
  public void setPythonScriptData(double[] outgoingPythonData) {
    if (outgoingPythonData == null) {
      return;
    }
    table.getEntry("llrobot").setDoubleArray(outgoingPythonData);
  }

  /** Triggers a snapshot capture. The Limelight firmware rate-limits this. */
  public void triggerSnapshot() {
    double current = table.getEntry("snapshot").getDouble(0);
    table.getEntry("snapshot").setDouble(current + 1);
  }

  /**
   * Resets the object counter of a counter pipeline to zero. The camera resets when the value
   * changes, so calling this every loop resets every loop.
   */
  public void resetCounter() {
    double current = table.getEntry("reset_counter_set").getDouble(0);
    table.getEntry("reset_counter_set").setDouble(current + 1);
  }

  /**
   * Enables or pauses the rewind buffer recording.
   *
   * @param enabled True to enable recording. False to pause. Default true on supported platforms.
   */
  public void setRewindEnabled(boolean enabled) {
    table.getEntry("rewind_enable_set").setDouble(enabled ? 1 : 0);
  }

  /**
   * Triggers a rewind capture with the given duration. The maximum duration is 165 seconds. The
   * camera rate-limits this.
   *
   * @param durationSeconds Duration of the rewind capture in seconds (maximum 165)
   */
  public void triggerRewindCapture(double durationSeconds) {
    double[] currentArray = table.getEntry("capture_rewind").getDoubleArray(EMPTY);
    double counter = (currentArray.length > 0) ? currentArray[0] : 0;
    table
        .getEntry("capture_rewind")
        .setDoubleArray(new double[] {counter + 1, clampArg(durationSeconds, 1, 165)});
  }

  // Loader and protocol warnings go to the Driver Station console like other WPILib warnings.
  static void reportWarning(String message) {
    DriverStationErrors.reportWarning(message, false);
  }

  /**
   * Flushes NetworkTables immediately. These methods call it automatically: the pose-setting
   * constructors, {@link #setSharedRobotOrientation}, {@link #setPipelineConfigurationOverride},
   * {@link #clearPipelineConfigurationOverride}, {@link #setSharedMap}, {@link #clearSharedMap},
   * {@link #clearCameraPose_RobotSpaceOverride}, and {@link #setRobotOrientation} / {@link
   * #setCameraPose_RobotSpaceOverride} when their flush argument is true. The other setters and
   * clear methods do not flush. Call it yourself after a group of them.
   */
  public static void flushNT() {
    NetworkTableInstance.getDefault().flush();
  }

  static double clampArg(double value, double min, double max) {
    return clampArg(value, min, max, min);
  }

  static double clampArg(double value, double min, double max, double nanFallback) {
    if (Double.isNaN(value)) {
      return nanFallback;
    }
    return Math.max(min, Math.min(max, value));
  }

  // ---- Geometry utilities ----

  /**
   * Converts a pose array to a Pose3d. The array has 6 values: [x, y, z, roll, pitch, yaw]. Units
   * are meters and degrees.
   *
   * @return The pose. Returns an empty Pose3d if the array is not valid
   */
  public static Pose3d toPose3D(double[] inData) {
    if (inData == null || inData.length < 6) {
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  /**
   * Converts a pose array to a Pose2d. The array has 6 values: [x, y, z, roll, pitch, yaw]. Units
   * are meters and degrees. Uses only the x, y, and yaw values.
   *
   * @return The pose. Returns an empty Pose2d if the array is not valid
   */
  public static Pose2d toPose2D(double[] inData) {
    if (inData == null || inData.length < 6) {
      return new Pose2d();
    }
    return new Pose2d(
        new Translation2d(inData[0], inData[1]), new Rotation2d(Units.degreesToRadians(inData[5])));
  }

  // ---- Result classes ----

  /** Fields common to all image-space targets. */

  // ---- Decoding ----

  private static void decodeResults(MsgPackReader r, LimelightResults out) {
    int n = r.expectMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "v":
          out.valid = r.readLong() != 0;
          break;
        case "ts":
          out.timestampMillis = r.readDouble();
          break;
        case "ts_us":
          out.timestampMicros = r.readLong();
          break;
        case "ts_sys":
          out.systemTimestampMicros = r.readLong();
          break;
        case "ts_nt":
          out.ntTimestampMicros = r.readLong();
          break;
        case "fidx":
          out.frameIndex = r.readLong();
          break;
        case "tl":
          out.targetingLatencyMillis = r.readDouble();
          break;
        case "cl":
          out.captureLatencyMillis = r.readDouble();
          break;
        case "pID":
          out.pipelineIndex = r.readDouble();
          break;
        case "pTYPE":
          out.pipelineType = r.readString();
          break;
        case "focus_metric":
          out.focusMetric = r.readDouble();
          break;
        case "stdev_mt1":
          out.stdDevsMT1 = r.readPoseArray();
          break;
        case "stdev_mt2":
          out.stdDevsMT2 = r.readPoseArray();
          break;
        case "botpose":
          out.robotPoseMT1 = r.readPoseArray();
          break;
        case "botpose_wpired":
          out.robotPoseMT1WpiRed = r.readPoseArray();
          break;
        case "botpose_wpiblue":
          out.robotPoseMT1WpiBlue = r.readPoseArray();
          break;
        case "botpose_orb":
          out.robotPoseMT2 = r.readPoseArray();
          break;
        case "botpose_orb_wpired":
          out.robotPoseMT2WpiRed = r.readPoseArray();
          break;
        case "botpose_orb_wpiblue":
          out.robotPoseMT2WpiBlue = r.readPoseArray();
          break;
        case "botpose_tagcount":
          out.reportedTagCount = (int) r.readLong();
          break;
        case "botpose_span":
          out.tagSpanMeters = r.readDouble();
          break;
        case "botpose_avgdist":
          out.avgTagDistanceMeters = r.readDouble();
          break;
        case "botpose_avgarea":
          out.avgTagAreaPercent = r.readDouble();
          break;
        case "t6c_rs":
          out.cameraPoseRobotSpace = r.readPoseArray();
          break;
        case "PythonOut":
          out.pythonOutput = r.readDoubleArray();
          break;
        case "tx":
          out.txDegrees = r.readDouble();
          break;
        case "ty":
          out.tyDegrees = r.readDouble();
          break;
        case "txnc":
          out.txDegreesNoCrosshair = r.readDouble();
          break;
        case "tync":
          out.tyDegreesNoCrosshair = r.readDouble();
          break;
        case "ta":
          out.targetAreaPercent = r.readDouble();
          break;
        case "tdist":
          out.targetDistanceMeters = r.readDouble();
          break;
        case "Retro":
          out.retroTargets = decodeRetroTargets(r);
          break;
        case "Fiducial":
          out.fiducialTargets = decodeFiducialTargets(r);
          break;
        case "Detector":
          out.detectorTargets = decodeDetectorTargets(r);
          break;
        case "Classifier":
          out.classifierTargets = decodeClassifierTargets(r);
          break;
        case "ClassifierRejected":
          out.classifierTargetsRejected = decodeClassifierTargets(r);
          break;
        case "Barcode":
          out.barcodeTargets = decodeBarcodeTargets(r);
          break;
        case "Counter":
          out.counterTargets = decodeCounterTargets(r);
          break;
        case "OCR":
          out.ocrTargets = decodeOCRTargets(r);
          break;
        case "imu":
          decodeIMU(r, out.imu);
          break;
        case "hw":
          decodeHardware(r, out.hardware);
          break;
        case "rewind":
          decodeRewind(r, out.rewind);
          break;
        case "botorient":
          decodeBotOrientation(r, out.robotOrientation);
          break;
        case "intrinsics":
          decodeIntrinsics(r, out.intrinsics);
          break;
        case "imgsrc":
          out.imageSource = (int) r.readLong();
          break;
        case "hwtype":
          out.hardwareType = (int) r.readLong();
          break;
        case "uirefresh":
          out.uiRefresh = (int) r.readLong();
          break;
        case "ignorent":
          out.ignoreNT = r.readLong() != 0;
          break;
        case "ntconnected":
          out.ntConnected = r.readLong() != 0;
          break;
        case "codepipeline":
          out.pipelineConfigurationOverrideActive = r.readLong() != 0;
          break;
        case "codepipelinestate":
          out.pipelineConfigurationOverrideState = parseOverrideState(r.readString());
          break;
        case "codemap":
          out.sharedMapActive = r.readLong() != 0;
          break;
        case "codemapstate":
          out.sharedMapState = parseSharedMapState(r.readString());
          break;
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static RetroTarget[] decodeRetroTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    RetroTarget[] out = new RetroTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      RetroTarget t = new RetroTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        if (decodeCommonTargetKey(t, key, r)) {
          continue;
        }
        switch (key) {
          case "t6c_ts":
            t.cameraPoseTargetSpace = r.readPoseArray();
            break;
          case "t6t_cs":
            t.targetPoseCameraSpace = r.readPoseArray();
            break;
          case "t6t_rs":
            t.targetPoseRobotSpace = r.readPoseArray();
            break;
          case "t6r_ts":
            t.robotPoseTargetSpace = r.readPoseArray();
            break;
          case "t6r_fs":
            t.robotPoseFieldSpace = r.readPoseArray();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static FiducialTarget[] decodeFiducialTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    FiducialTarget[] out = new FiducialTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      FiducialTarget t = new FiducialTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        if (decodeCommonTargetKey(t, key, r)) {
          continue;
        }
        switch (key) {
          case "fID":
            t.fiducialId = (int) r.readLong();
            break;
          case "fam":
            t.family = r.readString();
            break;
          case "skew":
            t.skew = r.readDoubleArray();
            break;
          case "ambig":
            t.ambiguity = r.readDouble();
            break;
          case "fielded":
            t.fielded = r.readLong() != 0;
            break;
          case "t6c_ts":
            t.cameraPoseTargetSpace = r.readPoseArray();
            break;
          case "t6t_cs":
            t.targetPoseCameraSpace = r.readPoseArray();
            break;
          case "t6t_rs":
            t.targetPoseRobotSpace = r.readPoseArray();
            break;
          case "t6r_ts":
            t.robotPoseTargetSpace = r.readPoseArray();
            break;
          case "t6r_fs":
            t.robotPoseFieldSpace = r.readPoseArray();
            break;
          case "t6r_fs_orb":
            t.robotPoseFieldSpaceMT2 = r.readPoseArray();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static DetectorTarget[] decodeDetectorTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    DetectorTarget[] out = new DetectorTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      DetectorTarget t = new DetectorTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        if (decodeCommonTargetKey(t, key, r)) {
          continue;
        }
        switch (key) {
          case "classID":
            t.classId = (int) r.readLong();
            break;
          case "class":
            t.className = r.readString();
            break;
          case "conf":
            t.confidence = r.readDouble();
            break;
          case "tID":
            t.trackId = (int) r.readLong();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static ClassifierTarget[] decodeClassifierTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    ClassifierTarget[] out = new ClassifierTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      ClassifierTarget t = new ClassifierTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        switch (key) {
          case "classID":
            t.classId = (int) r.readLong();
            break;
          case "class":
            t.className = r.readString();
            break;
          case "conf":
            t.confidence = r.readDouble();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static BarcodeTarget[] decodeBarcodeTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    BarcodeTarget[] out = new BarcodeTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      BarcodeTarget t = new BarcodeTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        if (decodeCommonTargetKey(t, key, r)) {
          continue;
        }
        switch (key) {
          case "fam":
            t.family = r.readString();
            break;
          case "data":
            t.data = r.readString();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static CounterTarget[] decodeCounterTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    CounterTarget[] out = new CounterTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      CounterTarget t = new CounterTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        switch (key) {
          case "classID":
            t.classId = (int) r.readLong();
            break;
          case "class":
            t.className = r.readString();
            break;
          case "count":
            t.count = (int) r.readLong();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static OCRTarget[] decodeOCRTargets(MsgPackReader r) {
    int n = r.readArrayHeader();
    OCRTarget[] out = new OCRTarget[n];
    int count = 0;
    for (int i = 0; i < n; i++) {
      if (!r.nextIsMap()) {
        r.skipValue(); // not a target map, skip the element
        continue;
      }
      OCRTarget t = new OCRTarget();
      int fields = r.readMapHeader();
      for (int f = 0; f < fields; f++) {
        String key = r.readString();
        switch (key) {
          case "valid":
            t.valid = r.readLong() != 0;
            break;
          case "regionId":
            t.regionId = (int) r.readLong();
            break;
          case "text":
            t.text = r.readString();
            break;
          case "confidence":
            t.confidence = r.readDouble();
            break;
          case "numericValue":
            t.numericValue = r.readDouble();
            break;
          case "hasNumericValue":
            t.hasNumericValue = r.readLong() != 0;
            break;
          case "digitCount":
            t.digitCount = (int) r.readLong();
            break;
          case "unit":
            t.unit = r.readString();
            break;
          case "isCounter":
            t.isCounter = r.readLong() != 0;
            break;
          case "ratePerMin":
            t.ratePerMin = r.readDouble();
            break;
          case "alarmState":
            t.alarmState = (int) r.readLong();
            break;
          case "alarmString":
            t.alarmString = r.readString();
            break;
          case "bbox":
            t.boundingBox = r.readDoubleArray();
            break;
          case "processingTimeMs":
            t.processingTimeMs = r.readDouble();
            break;
          default:
            r.skipValue();
            break;
        }
      }
      out[count++] = t;
    }
    return count == n ? out : java.util.Arrays.copyOf(out, count);
  }

  private static void decodeIMU(MsgPackReader r, IMUData imu) {
    int n = r.readMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "quat":
          imu.quaternion = r.readDoubleArray();
          break;
        case "yaw_offset":
          imu.yawOffsetDegrees = r.readDouble();
          break;
        // The "yaw" key carries the same fused yaw as data[0]. The array is the single source.
        case "data":
          {
            double[] d = r.readDoubleArray();
            if (d.length >= 10) {
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
            break;
          }
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static void decodeHardware(MsgPackReader r, HardwareData hw) {
    int n = r.readMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "temp":
          hw.cpuTempCelsius = r.readDouble();
          break;
        case "cpu":
          hw.cpuUsagePercent = r.readDouble();
          break;
        case "ram":
          hw.ramUsagePercent = r.readDouble();
          break;
        case "dfree":
          hw.diskFreeMB = r.readLong();
          break;
        case "dtot":
          hw.diskTotalMB = r.readLong();
          break;
        case "cid":
          hw.cameraId = r.readString();
          break;
        case "hailo":
          {
            int m = r.readMapHeader();
            for (int j = 0; j < m; j++) {
              String hkey = r.readString();
              switch (hkey) {
                case "present":
                  hw.accelPresent = r.readLong() != 0;
                  break;
                case "type":
                  hw.accelType = r.readString();
                  break;
                case "temp":
                  hw.accelTempCelsius = r.readDouble();
                  break;
                case "power":
                  hw.accelPowerWatts = r.readDouble();
                  break;
                case "throttle":
                  hw.accelThrottling = r.readLong() != 0;
                  break;
                default:
                  r.skipValue();
                  break;
              }
            }
            break;
          }
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static void decodeRewind(MsgPackReader r, RewindData rw) {
    int n = r.readMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "enabled":
          rw.enabled = r.readLong() != 0;
          break;
        case "storedSeconds":
          rw.storedSeconds = r.readDouble();
          break;
        case "frameCount":
          rw.frameCount = r.readLong();
          break;
        case "bufferUsage":
          rw.bufferUsage = r.readDouble();
          break;
        case "flushing":
          rw.flushing = r.readLong() != 0;
          break;
        case "latpen":
          rw.latencyPenaltyMicros = (int) r.readLong();
          break;
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static void decodeIntrinsics(MsgPackReader r, CameraIntrinsics intrinsics) {
    int n = r.readMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "customcal":
          intrinsics.customCalibration = r.readLong() != 0;
          break;
        case "hfov":
          intrinsics.hfovDegrees = r.readDouble();
          break;
        case "vfov":
          intrinsics.vfovDegrees = r.readDouble();
          break;
        case "resw":
          intrinsics.resolutionWidthPixels = r.readDouble();
          break;
        case "resh":
          intrinsics.resolutionHeightPixels = r.readDouble();
          break;
        case "cameramatrix":
          intrinsics.cameraMatrix = r.readDoubleArray();
          break;
        case "distcoeffs":
          intrinsics.distortionCoefficients = r.readDoubleArray();
          break;
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static void decodeBotOrientation(MsgPackReader r, RobotOrientationData bo) {
    int n = r.readMapHeader();
    for (int i = 0; i < n; i++) {
      String key = r.readString();
      switch (key) {
        case "imumode":
          bo.imuMode = IMUMode.fromNtValue((int) r.readLong());
          break;
        case "alpha":
          bo.alpha = r.readDouble();
          break;
        case "interpbotyaw":
          bo.interpolatedYawDegrees = r.readDouble();
          break;
        default:
          r.skipValue();
          break;
      }
    }
  }

  private static boolean decodeCommonTargetKey(
      LimelightTarget target, String key, MsgPackReader r) {
    switch (key) {
      case "tx":
        target.txDegrees = r.readDouble();
        return true;
      case "ty":
        target.tyDegrees = r.readDouble();
        return true;
      case "tx_nocross":
        target.txDegreesNoCrosshair = r.readDouble();
        return true;
      case "ty_nocross":
        target.tyDegreesNoCrosshair = r.readDouble();
        return true;
      case "txp":
        target.txPixels = r.readDouble();
        return true;
      case "typ":
        target.tyPixels = r.readDouble();
        return true;
      case "ta":
        target.targetAreaPercent = r.readDouble();
        return true;
      case "pts":
        target.corners = r.readPointArray();
        return true;
      default:
        return false;
    }
  }

  // ---- MessagePack decoder ----

}
