// LimelightLib v2.0.0. Requires Limelight OS 2027.0 or later.
// Copy this file to src/main/java/first/Limelight.java.

package first;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.RawSubscriber;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.TimestampedRaw;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import org.wpilib.system.Filesystem;

/**
 *
 * <h2>Quick start</h2>
 *
 * <h3>Visual servoing</h3>
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
 * {@snippet :
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);
 *
 * // Each robot loop
 * for (var estimate : camera.readAcceptedPoseEstimates(Limelight.PoseEstimateType.MT1_WPIBLUE)) {
 *     poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <h3>MegaTag2 localization</h3>
 * <p>Publish robot yaw before reading the queue each robot loop.
 * {@snippet :
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace);
 *
 * // Each robot loop
 * Limelight.setSharedRobotOrientation(robotYawDegrees);
 * for (var estimate : camera.readAcceptedPoseEstimates(Limelight.PoseEstimateType.MT2_WPIBLUE)) {
 *     poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <h3>Configured MegaTag1 and MegaTag2</h3>
 * <p>This example configures every filter and standard-deviation scaling term.
 * MT1 scales XY uncertainty linearly with distance and clamps it to 0.05-2.0
 * meters. MT2 scales it by the square root of distance and clamps it to
 * 0.0001-2.0 meters. Both divide by the square root of the fielded tag count.
 * Vision heading is not fused. The field bounds are for the 2026 welded field.
 * Tune the other thresholds on your robot.
 * {@snippet :
 * double untrusted = Limelight.PoseEstimateConfig.UNTRUSTED;
 * Limelight.PoseEstimateConfig mt1Config = Limelight.PoseEstimateConfig.defaultMT1() // starting with the default means you are not required to set every single configuration value
 *         .withMinTagCount(1)
 *         .withMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity perspectives
 *         .withMaxSingleTagDistance(3.0) // If we only see one tag, don't trust it unless we are at most 3m away from it
 *         .withMaxAvgTagDistance(6.0) // If we see multiple tags, max avg distance must be less than 6 meters.
 *         .withMinAvgTagArea(0.05)
 *         .withFieldBounds(16.541, 8.069) // Reject pose estimates that are out of bounds.
 *         .withFieldBoundsMargin(0.5) // Reject pose estimates that are out of bounds +.5m
 *         .withStdDevXY(0.5, 0.05, 2.0) // .5 base, absolute min .05, absolute max 2.0 
 *         .withStdDevTheta(untrusted, untrusted, untrusted) // never incorporate pose estimate rotation. You may want to incorporate rotation by setting these to other values
 *         .withStdDevDistanceScaling(1.0, 0.0, 6.0) // linear scaling
 *         .withStdDevTagCountDivision(0.5);
 *
 * Limelight.PoseEstimateConfig mt2Config = Limelight.PoseEstimateConfig.defaultMT2()
 *         .withMinTagCount(1)
 *         .withMaxSingleTagAmbiguity(1.0) // MT2 can handle maximally ambiguous perspectives. Accept tags regardless of ambiguity value.
 *         .withMaxSingleTagDistance(0.0) // 0 disables this check
 *         .withMaxAvgTagDistance(8.0)
 *         .withMinAvgTagArea(0.02)
 *         .withFieldBounds(16.541, 8.069)
 *         .withFieldBoundsMargin(0.5)
 *         .withStdDevXY(0.3, 0.0001, 2.0)
 *         .withStdDevTheta(untrusted, untrusted, untrusted)
 *         .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Less aggressive STDDev scaling for MT2. Scale by sqrt(distance) rather than distance^1.
 *         .withStdDevTagCountDivision(0.5);
 *
 * Pose3d cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d());
 * Limelight camera = new Limelight("limelight", cameraPoseRobotSpace)
 *         .withPoseEstimateConfig_MT1(mt1Config)
 *         .withPoseEstimateConfig_MT2(mt2Config)
 *         .withTelemetry(true); // Keep automatic telemetry enabled. All accepted and rejected poses will remain easy to visualize in standard dashboards
 * boolean useMegaTag2 = true;
 *
 * // Each robot loop
 * Limelight.PoseEstimateType type = useMegaTag2 ? Limelight.PoseEstimateType.MT2_WPIBLUE : Limelight.PoseEstimateType.MT1_WPIBLUE;
 * if (useMegaTag2) {
 *     Limelight.setSharedRobotOrientation(robotYawDegrees); // Use your robot's pose here, not your raw IMU reading.
 * }
 * for (var estimate : camera.readAcceptedPoseEstimates(type)) {
 *     poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
 * }
 * }
 *
 * <p>All values from one {@link #getLatestResults()} call come from the same frame.
 * This includes tx, botpose, fiducials, latencies, and timestamps.
 *
 * <p>All 3D data uses the right-handed NWU convention in every space (field, robot,
 * camera, target): x = forward, y = left, z = up. Raw pose arrays are
 * [x, y, z, roll, pitch, yaw] in meters and degrees.
 *
 * <p>Getters decode at most one new frame. Otherwise they return cached data.
 * After a disconnect, they return the values of the last frame. Use
 * {@link #getStatus()} or {@link LimelightResults#getAgeSeconds()} to detect a
 * stale camera.
 *
 * <p>Use this class from one thread, the robot loop.
 */

public class Limelight implements AutoCloseable {

    private static final double[] EMPTY = new double[0];
    private static final double[][] EMPTY_CORNERS = new double[0][];
    private static final Pose2d[] EMPTY_POSES = new Pose2d[0];
    private static final int[] REJECTION_FLAGS = {
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
    private static final String[] REJECTION_NAMES = {
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

    /** Default stale threshold. {@link #getStatus} reports {@link Status#STALE} for a frame
     *  older than this. Override per instance with {@link #withStaleFrameThreshold}. */
    public static final double STALE_FRAME_SECONDS = 0.25;

    /** NetworkTables timestamp ticks per second for the WPILib build this library
     *  targets. WPILib 2027 alpha-7 and later use 1e9. Alpha-6 and earlier use
     *  1e6. Divide a NetworkTables timestamp by this value to get seconds. */
    public static final double NT_TICKS_PER_SECOND = 1.0e6;

    /** The highest camera protocol version ("protover") this library
     *  understands. If a camera publishes a higher version, the library prints
     *  one warning. */
    public static final int SUPPORTED_PROTOCOL_VERSION = 1;


    /** Pipeline configuration override state reported by the camera. */
    public enum PipelineConfigurationOverrideState {
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
    }

    /** Shared field map state reported by the camera. */
    public enum SharedMapState {
        /** No shared map published */
        OFF,
        /** The shared map is in use */
        ACTIVE,
        /** The published map failed to parse or was too large */
        PARSE_ERROR,
        /** The camera reported a state this library does not know */
        UNKNOWN
    }

    /** Result of loading a {@link PipelineConfiguration} or {@link FieldMap}. */
    public enum LoadStatus {
        /** Loaded and within the size cap */
        OK,
        /** No contents or no file name were given (null or empty) */
        NO_CONTENTS,
        /** The deploy-folder file could not be read */
        READ_FAILED,
        /** The contents exceed the size cap */
        TOO_LARGE
    }

    private static PipelineConfigurationOverrideState parseOverrideState(String s) {
        switch (s) {
            case "off": return PipelineConfigurationOverrideState.OFF;
            case "active": return PipelineConfigurationOverrideState.ACTIVE;
            case "forcedOff": return PipelineConfigurationOverrideState.FORCED_OFF;
            case "noString": return PipelineConfigurationOverrideState.NO_STRING;
            case "parseError": return PipelineConfigurationOverrideState.PARSE_ERROR;
            default: return PipelineConfigurationOverrideState.UNKNOWN;
        }
    }

    private static SharedMapState parseSharedMapState(String s) {
        switch (s) {
            case "off": return SharedMapState.OFF;
            case "active": return SharedMapState.ACTIVE;
            case "parseError": return SharedMapState.PARSE_ERROR;
            default: return SharedMapState.UNKNOWN;
        }
    }

    /** Maximum size in bytes of a pipeline configuration override. Both this
     *  library and the camera reject larger publishes. */
    public static final int MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES = 4096;

    /** Maximum size in bytes of a shared field map. Both this library and the
     *  camera reject larger publishes. */
    public static final int MAX_SHARED_MAP_BYTES = 262144;

    /** Name of the vision instance running on Systemcore USB port 0:
     *  {@code new Limelight(Limelight.SYSTEMCORE_USB0)} */
    public static final String SYSTEMCORE_USB0 = "limelightsc0";
    /** Name of the vision instance running on Systemcore USB port 1 */
    public static final String SYSTEMCORE_USB1 = "limelightsc1";
    /** Name of the vision instance running on Systemcore USB port 2 */
    public static final String SYSTEMCORE_USB2 = "limelightsc2";
    /** Name of the vision instance running on Systemcore USB port 3 */
    public static final String SYSTEMCORE_USB3 = "limelightsc3";

    /** Root telemetry table. Each camera uses {@code limelight_telemetry/<name>/}. The
     *  shared Field2d table is {@code limelight_telemetry/Field}. */
    public static final String TELEMETRY_TABLE = "limelight_telemetry";

    /**
     * Health of a camera.
     */
    public enum Status {
        /** A decodable frame has been received recently */
        OK,
        /** The camera has not sent a results envelope. Possible causes: the camera
         *  is off, the name is wrong, or MsgPack output is disabled */
        NO_DATA,
        /** The newest frame is older than the stale threshold. The camera is
         *  disconnected or has stopped. The default threshold is
         *  {@link #STALE_FRAME_SECONDS}. See {@link #withStaleFrameThreshold} */
        STALE,
        /** The newest frame did not decode. See {@link LimelightResults#error} */
        DECODE_ERROR
    }

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
    private final PoseTelemetry[] poseTelemetry =
            new PoseTelemetry[PoseEstimateType.values().length];
    private PoseCountsTelemetry poseCountsTelemetry;
    private StringPublisher fieldTypeTelemetry;
    private BooleanPublisher connectedTelemetry;
    private BooleanPublisher customCalibrationTelemetry;
    private StringPublisher statusTelemetry;

    /**
     * Creates an interface to the Limelight with the default name ("limelight").
     */
    public Limelight() {
        this("limelight");
    }

    /**
     * Creates an interface to the given Limelight.
     * @param name The camera or vision instance's NetworkTables name (e.g. "limelight" or "limelight-left")
     */
    public Limelight(String name) {
        if (name == null || name.isEmpty()) {
            name = "limelight";
        }
        this.name = name;
        this.latestResults.fromLiveSubscriber = true;
        this.table = NetworkTableInstance.getDefault().getTable(name);
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable(TELEMETRY_TABLE + "/" + name);
        this.resultsSubscriber = table.getRawTopic("results_msgpack")
                .subscribe("msgpack", new byte[0],
                        PubSubOption.periodic(0.005), PubSubOption.SEND_ALL,
                        PubSubOption.pollStorage(20));
        this.protocolVersionSubscriber = table.getIntegerTopic("protover").subscribe(0);
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
     *        {@link #clearCameraPose_RobotSpaceOverride} to return to it. Null
     *        publishes nothing. An all-zero pose is the clear value, so the
     *        camera keeps the pose from the web interface.
     */
    public Limelight(String name, Pose3d cameraPoseRobotSpace) {
        this(name);
        if (cameraPoseRobotSpace == null) { return; }
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
     * Creates an interface to the given Limelight and sets its camera pose in
     * robot space. This overrides the camera pose configured in the web
     * interface. Call {@link #clearCameraPose_RobotSpaceOverride} to return to it.
     * An all-zero pose is the clear value, so the camera keeps the pose from the
     * web interface.
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
    public Limelight(String name, double forward, double left, double up,
            double rollDegrees, double pitchDegrees, double yawDegrees) {
        this(name, new Pose3d(forward, left, up, new Rotation3d(
                Units.degreesToRadians(rollDegrees),
                Units.degreesToRadians(pitchDegrees),
                Units.degreesToRadians(yawDegrees))));
    }

    /**
     * Sets the maximum age of the newest frame. Above this age, {@link #getStatus}
     * reports {@link Status#STALE} and {@link #hasTarget} returns false. Raise this value when
     * you use {@link #setThrottle}.
     *
     * @param seconds Stale threshold in seconds. Positive infinity disables stale
     *        detection.
     * @return this, for chaining with the constructor
     */
    public Limelight withStaleFrameThreshold(double seconds) {
        this.staleFrameSeconds =
                clampArg(seconds, 0.01, Double.MAX_VALUE, STALE_FRAME_SECONDS);
        return this;
    }

    /**
     * Configures filtering and fusion standard deviations for every MegaTag1 pose
     * estimate. MegaTag1 computes heading from tag geometry only. It usually
     * needs stricter ambiguity and tag-count gates than MegaTag2. Rejected
     * estimates report {@code isValid() == false}. {@link PoseEstimate#rejectionFlags}
     * lists the failed checks.
     *
     * @param config The MegaTag1 configuration. The configuration is copied.
     *        Later changes to the object have no effect until you attach it
     *        again. Null resets to {@link PoseEstimateConfig#defaultMT1()}
     * @return this, for chaining with the constructor
     */
    public Limelight withPoseEstimateConfig_MT1(PoseEstimateConfig config) {
        this.megaTag1Config = (config != null)
                ? new PoseEstimateConfig(config) : PoseEstimateConfig.defaultMT1();
        return this;
    }

    /**
     * Configures filtering and fusion standard deviations for every MegaTag2
     * pose estimate.
     *
     * @param config The MegaTag2 configuration. The configuration is copied.
     *        Later changes to the object have no effect until you attach it
     *        again. Null resets to {@link PoseEstimateConfig#defaultMT2()}
     * @return this, for chaining with the constructor
     */
    public Limelight withPoseEstimateConfig_MT2(PoseEstimateConfig config) {
        this.megaTag2Config = (config != null)
                ? new PoseEstimateConfig(config) : PoseEstimateConfig.defaultMT2();
        return this;
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
     * {@link #readPoseEstimateQueue} and {@link #readAcceptedPoseEstimates} process.
     * One estimate can increment several rejection-reason totals. The same
     * estimates appear on the shared {@code limelight_telemetry/Field} Field2d
     * table for Glass and Elastic. Camera health publishes as {@code connected},
     * {@code customCalibration}, and {@code status}. Pose displays clear when
     * the camera is unhealthy. Disabling telemetry unpublishes everything and
     * resets the counters.
     *
     * @return this, for chaining with the constructor
     */
    public Limelight withTelemetry(boolean enabled) {
        if (telemetryEnabled && !enabled) { unpublishTelemetry(); }
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
     * @return The camera's NetworkTables name
     */
    public String getName() {
        return name;
    }

    /**
     * @return The msgpack envelope protocol version from the camera. 0 if the
     *         camera has not connected or runs old software
     */
    public int getProtocolVersion() {
        return (int) protocolVersionSubscriber.get();
    }

    // ---- Results ----

    /**
     * Returns the latest results envelope. Decodes the newest MessagePack frame
     * if one arrived since the last call. This method is cheap to call many times
     * per loop. The decoded envelope is cached until a new frame arrives. This
     * method never consumes the frame queue. You can use it together with
     * {@link #readResultsQueue}.
     *
     * @return The latest LimelightResults, never null. Use {@link #getStatus} for
     *         camera health. Use {@link LimelightResults#valid} for target
     *         validity. Each new frame produces a new object. Call this getter
     *         every loop instead of keeping the returned object.
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
            System.err.println("Limelight - " + name + ": camera protocol version " + version
                    + " is newer than this library supports (" + SUPPORTED_PROTOCOL_VERSION
                    + "). Update Limelight.java. Results may be missing or wrong.");
        }
    }

    /**
     * Decodes a results envelope from raw MessagePack bytes. Decode failures are
     * reported in {@link LimelightResults#error}. This method never throws. Use it
     * for unit tests.
     *
     * <p>{@code receiveTimestampSeconds} stays 0. Pose estimates built from the
     * result are rejected as NO_TIMESTAMP. Use {@link #decode(byte[], long)} for
     * pose estimate tests.
     *
     * @param envelope The raw MessagePack results dump
     * @return The decoded results. {@code receiveTimestampSeconds} stays 0
     */
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
     * Decodes a results envelope and stamps it with its NetworkTables receive
     * time. This matches live decoding and timestamp handling. For log replay:
     * record raw frames with {@link #getLatestRawFrame()} or
     * {@link #readRawFrameQueue()}. Then pass the bytes and the timestamp to this
     * method. Estimates built from the result have latency-compensated
     * timestamps.
     *
     * @param envelope The raw MessagePack results dump
     * @param receiveTimestampMicros NetworkTables receive time in microseconds
     *        (local NetworkTables timebase, WPILib alpha-6 or older), for
     *        example {@link TimestampedRaw#timestamp}
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
     * @return The current health of the camera from the point of view of this
     *         consumer. {@link Status#OK} means a decodable frame arrived within the
     *         stale threshold. The threshold is {@link #STALE_FRAME_SECONDS} unless you
     *         override it with {@link #withStaleFrameThreshold}.
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
     * @return True if the camera is reachable. The status is not {@link Status#NO_DATA}
     *         and not {@link Status#STALE}
     */
    public boolean isConnected() {
        Status status = getStatus();
        return status == Status.OK || status == Status.DECODE_ERROR;
    }

    /**
     * Releases this instance's NetworkTables subscriptions and removes its telemetry
     * topics. Call from test teardown. Robot code normally never needs this.
     */
    @Override
    public void close() {
        resultsSubscriber.close();
        protocolVersionSubscriber.close();
        unpublishTelemetry();
        telemetryEnabled = false;
    }

    /**
     * @return True if the camera is connected, sends fresh frames, and has at
     *         least one valid target. Returns false when the newest frame is
     *         older than the stale threshold.
     */
    public boolean hasTarget() {
        LimelightResults results = getLatestResults();
        return computeStatus() == Status.OK && results.valid;
    }

    /**
     * @return Horizontal offset from crosshair to target in degrees. This value
     *         holds the last received value after a disconnect. Check {@link #hasTarget}
     *         every loop.
     */
    public double getTXDegrees() {
        return getLatestResults().txDegrees;
    }

    /**
     * @return Vertical offset from crosshair to target in degrees. This value
     *         holds the last received value after a disconnect. Check {@link #hasTarget}
     *         every loop.
     */
    public double getTYDegrees() {
        return getLatestResults().tyDegrees;
    }

    /**
     * @return Horizontal offset from principal pixel to target in degrees (crosshair-independent)
     */
    public double getTXDegreesNoCrosshair() {
        return getLatestResults().txDegreesNoCrosshair;
    }

    /**
     * @return Vertical offset from principal pixel to target in degrees (crosshair-independent)
     */
    public double getTYDegreesNoCrosshair() {
        return getLatestResults().tyDegreesNoCrosshair;
    }

    /**
     * @return Target area as a percentage of the image (0-100). This value holds
     *         the last received value after a disconnect. Check {@link #hasTarget} every
     *         loop.
     */
    public double getTargetAreaPercent() {
        return getLatestResults().targetAreaPercent;
    }

    /**
     * @return 3D distance from the camera to the primary fiducial target in
     *         meters. 0 if not available
     */
    public double getTargetDistanceMeters() {
        return getLatestResults().targetDistanceMeters;
    }

    /**
     * @return Total number of retro, fiducial, detector, classifier, and barcode targets
     *         in the latest frame
     */
    public int getTargetCount() {
        return getLatestResults().getTargetCount();
    }

    /**
     * @return Active pipeline index (0-9). -1 before the first frame arrives
     */
    public int getCurrentPipelineIndex() {
        return (int) getLatestResults().pipelineIndex;
    }

    /**
     * @return Active pipeline type, for example "pipe_fiducial", "pipe_color",
     *         or "pipe_detector"
     */
    public String getCurrentPipelineType() {
        return getLatestResults().pipelineType;
    }

    /**
     * @return Targeting/pipeline latency in milliseconds
     */
    public double getTargetingLatencyMillis() {
        return getLatestResults().targetingLatencyMillis;
    }

    /**
     * @return Capture latency in milliseconds
     */
    public double getCaptureLatencyMillis() {
        return getLatestResults().captureLatencyMillis;
    }

    /**
     * @return IMU state from the latest frame
     */
    public IMUData getIMUData() {
        return getLatestResults().imu;
    }

    /**
     * @return Hardware/system stats from the latest frame
     */
    public HardwareData getHardwareData() {
        return getLatestResults().hardware;
    }

    /**
     * @return The camera intrinsics that the running pipeline uses. Includes the
     *         camera matrix scaled to the processing resolution, the OpenCV
     *         distortion coefficients, and the FOV
     */
    public CameraIntrinsics getCameraIntrinsics() {
        return getLatestResults().intrinsics;
    }

    /**
     * @return True if the running pipeline uses a user-uploaded camera
     *         calibration instead of a built-in default. See
     *         {@link CameraIntrinsics#customCalibration}. False before the first frame
     *         arrives.
     */
    public boolean isUsingCustomCalibration() {
        return getLatestResults().intrinsics.customCalibration;
    }

    /**
     * @return Data set by a python snapscript via llpython
     */
    public double[] getPythonScriptData() {
        return getLatestResults().pythonOutput;
    }

    // ---- Pose Estimates ----

    /**
     * The robot pose estimate outputs. Names use the form {algorithm}_{origin}.
     *
     * <p>MegaTag1 (MT1) computes the full robot pose from tag geometry only.
     * MegaTag2 (MT2) also uses the robot orientation from
     * {@link #setRobotOrientation} or {@link #setSharedRobotOrientation}.
     * MT2 is usually more stable. MT2 requires you to publish the orientation
     * every loop.
     */
    public enum PoseEstimateType {
        /** MegaTag1, WPILib blue-alliance-corner origin (botpose_wpiblue) */
        MT1_WPIBLUE,
        /** MegaTag1, WPILib red-alliance-corner origin (botpose_wpired) */
        MT1_WPIRED,
        /** MegaTag2, WPILib blue-alliance-corner origin (botpose_orb_wpiblue) */
        MT2_WPIBLUE,
        /** MegaTag2, WPILib red-alliance-corner origin (botpose_orb_wpired) */
        MT2_WPIRED
    }

    private static double[] poseArray(LimelightResults results, PoseEstimateType type) {
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

    private static boolean usesMT2(PoseEstimateType type) {
        return switch (type) {
            case MT1_WPIBLUE, MT1_WPIRED -> false;
            case MT2_WPIBLUE, MT2_WPIRED -> true;
        };
    }

    private static boolean centeredOrigin(PoseEstimateType type) {
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
     * Gets the pose estimate of the given type from the newest frame. This getter
     * can return the same frame many times, also after a disconnect. Use
     * {@link #readAcceptedPoseEstimates} for fusion. Use {@link #getStatus} for current camera
     * health.
     *
     * @param type Which pose estimate to produce
     */
    public PoseEstimate getPoseEstimate(PoseEstimateType type) {
        return getPoseEstimate(getLatestResults(), type);
    }

    // ---- Queue reads ----

    /**
     * Decodes and returns every buffered results envelope received since the last
     * queue read. The queue holds 20 frames. If more frames arrive between reads,
     * the oldest frames are discarded. The newest frame also becomes the result
     * of {@link #getLatestResults()}.
     *
     * <p>Use only one queue reading method per camera. The queue reading methods
     * are this method, {@link #readPoseEstimateQueue},
     * {@link #readAcceptedPoseEstimates}, and {@link #readRawFrameQueue}. To get
     * several values from each frame, read the queue here one time. Then call
     * {@link #getPoseEstimate(LimelightResults, PoseEstimateType)} for each frame.
     * Other getters only read the newest frame. They do not consume the queue.
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
     * Reads every frame received since the last queue read. Returns one pose
     * estimate of the given type for each frame. This lets your pose estimator
     * use every vision update. Check {@link PoseEstimate#isValid} on each estimate
     * before you fuse it.
     *
     * <p>This method consumes the queue. Do not use another queue reading method
     * for this camera.
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
     * Reads every frame received since the last queue read. Returns only the pose
     * estimates that passed validation and filtering. Each queued frame is
     * processed one time. The pose estimator decides if the timestamp is still
     * in range.
     *
     * <pre>
     * for (PoseEstimate estimate : limelightCamera.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE)) {
     *     poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
     * }
     * </pre>
     *
     * <p>Telemetry shows the newest estimate and keeps rejection counters. Use
     * {@link #readPoseEstimateQueue} to inspect every rejected estimate.
     *
     * <p>This method consumes the queue. Do not use another queue reading method
     * for this camera.
     *
     * @param type Which pose estimate to produce for each frame
     * @return The accepted estimates, oldest first. Empty if no new frame
     *         arrived or no estimate passed
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
     * Returns the newest raw results envelope without decoding it. The result has
     * the MessagePack bytes and the NetworkTables receive timestamp in
     * microseconds (local NetworkTables timebase). This method does not consume
     * the frame queue. Use it with {@link #decode(byte[], long)} for
     * logging and replay.
     *
     * @return The newest raw frame. The value is empty if no frame has arrived
     */
    public TimestampedRaw getLatestRawFrame() {
        warnIfProtocolNewer();
        return resultsSubscriber.getAtomic();
    }

    /**
     * Reads every raw envelope received since the last queue read, without
     * decoding. Use this for logging and replay systems. These systems record raw
     * frames and decode them with {@link #decode(byte[], long)}, live
     * or from a log.
     *
     * <p>This method consumes the queue. Do not use another queue reading method
     * for this camera.
     *
     * @return All buffered raw frames, oldest first. Empty if no new frame
     *         arrived
     */
    public TimestampedRaw[] readRawFrameQueue() {
        warnIfProtocolNewer();
        return resultsSubscriber.readQueue();
    }

    /**
     * @return The full 3D robot pose for the given estimate type. The type
     *         selects the origin and the algorithm
     */
    public Pose3d getRobotPose(PoseEstimateType type) {
        return getLatestResults().getRobotPose(type);
    }

    /**
     * @return Camera pose in robot space (meters, degrees) as a Pose3d
     */
    public Pose3d getCameraPose_RobotSpace() {
        return toPose3D(getLatestResults().cameraPoseRobotSpace);
    }

    /**
     * Builds a pose estimate of the given type from one results envelope. Applies
     * the {@link PoseEstimateConfig} for the algorithm of the estimate. Use this with
     * {@link #readResultsQueue} to get several estimate types (for example MT1 and MT2)
     * from one queue read.
     *
     * @param results The envelope to build from
     * @param type Which pose estimate to produce
     */
    public PoseEstimate getPoseEstimate(LimelightResults results, PoseEstimateType type) {
        double[] poseArray = poseArray(results, type);

        PoseEstimate estimate = new PoseEstimate();
        estimate.type = type;
        estimate.frameIndex = results.frameIndex;
        estimate.latencyMillis = results.captureLatencyMillis + results.targetingLatencyMillis;
        estimate.timestampSeconds =
                results.receiveTimestampSeconds - (estimate.latencyMillis / 1000.0);
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
        if (poseArray.length < 6
                || (poseArray[0] == 0 && poseArray[1] == 0 && poseArray[2] == 0
                        && poseArray[3] == 0 && poseArray[4] == 0 && poseArray[5] == 0)) {
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
            poseTelemetry[i] =
                    new PoseTelemetry(telemetryTable, fieldTable, name, estimate.type);
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
            customCalibrationTelemetry =
                    telemetryTable.getBooleanTopic("customCalibration").publish();
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
    private static final class PoseTelemetry {
        private final StructArrayPublisher<Pose2d> accepted;
        private final StructArrayPublisher<Pose2d> rejected;
        private final StringPublisher rejectionReasons;
        private final DoubleArrayPublisher fieldAccepted;
        private final DoubleArrayPublisher fieldRejected;

        PoseTelemetry(NetworkTable telemetryTable, NetworkTable fieldTable,
                String cameraName, PoseEstimateType type) {
            String prefix = type.name() + "/";
            accepted = telemetryTable.getStructArrayTopic(prefix + "accepted", Pose2d.struct)
                    .publish();
            rejected = telemetryTable.getStructArrayTopic(prefix + "rejected", Pose2d.struct)
                    .publish();
            rejectionReasons =
                    telemetryTable.getStringTopic(prefix + "rejectionReasons").publish();
            fieldAccepted =
                    fieldTable.getDoubleArrayTopic(cameraName + "-" + type.name()).publish();
            fieldRejected = fieldTable
                    .getDoubleArrayTopic(cameraName + "-" + type.name() + "-rejected")
                    .publish();
        }

        void publish(PoseEstimate estimate) {
            boolean drawRejected = estimate.fieldedTagCount > 0 && estimate.rejectionFlags != 0
                    && (estimate.rejectionFlags & PoseEstimateConfig.REJECT_MISSING_POSE) == 0
                    && Double.isFinite(estimate.pose.getX())
                    && Double.isFinite(estimate.pose.getY())
                    && Double.isFinite(estimate.pose.getRotation().getRadians());
            accepted.set(estimate.isValid() ? new Pose2d[] {estimate.pose} : EMPTY_POSES);
            rejected.set(drawRejected ? new Pose2d[] {estimate.pose} : EMPTY_POSES);
            rejectionReasons.set(
                    PoseEstimateConfig.describeRejection(estimate.rejectionFlags));
            fieldAccepted.set(estimate.isValid() ? fieldPose(estimate.pose) : EMPTY);
            fieldRejected.set(drawRejected ? fieldPose(estimate.pose) : EMPTY);
        }

        void clear() {
            accepted.set(EMPTY_POSES);
            rejected.set(EMPTY_POSES);
            rejectionReasons.set("");
            fieldAccepted.set(EMPTY);
            fieldRejected.set(EMPTY);
        }

        void unpublish() {
            accepted.close();
            rejected.close();
            rejectionReasons.close();
            fieldAccepted.close();
            fieldRejected.close();
        }
    }

    private static final class PoseCountsTelemetry {
        private final IntegerPublisher processed;
        private final IntegerPublisher accepted;
        private final IntegerPublisher rejected;
        private final IntegerPublisher[] rejectionCounts =
                new IntegerPublisher[REJECTION_FLAGS.length];
        private final long[] rejectionTotals = new long[REJECTION_FLAGS.length];
        private long processedTotal;
        private long acceptedTotal;
        private long rejectedTotal;

        PoseCountsTelemetry(NetworkTable telemetryTable) {
            NetworkTable countsTable = telemetryTable.getSubTable("counts");
            processed = countsTable.getIntegerTopic("processed").publish();
            accepted = countsTable.getIntegerTopic("accepted").publish();
            rejected = countsTable.getIntegerTopic("rejected").publish();
            processed.set(0);
            accepted.set(0);
            rejected.set(0);
            for (int i = 0; i < rejectionCounts.length; i++) {
                rejectionCounts[i] =
                        countsTable.getIntegerTopic(REJECTION_NAMES[i]).publish();
                rejectionCounts[i].set(0);
            }
        }

        void count(PoseEstimate estimate) {
            processed.set(++processedTotal);
            if (estimate.isValid()) {
                accepted.set(++acceptedTotal);
                return;
            }
            rejected.set(++rejectedTotal);
            for (int i = 0; i < REJECTION_FLAGS.length; i++) {
                if ((estimate.rejectionFlags & REJECTION_FLAGS[i]) != 0) {
                    rejectionCounts[i].set(++rejectionTotals[i]);
                }
            }
        }

        void unpublish() {
            processed.close();
            accepted.close();
            rejected.close();
            for (IntegerPublisher rejectionCount : rejectionCounts) {
                rejectionCount.close();
            }
        }
    }

    private static NetworkTable sharedFieldTable() {
        return NetworkTableInstance.getDefault().getTable(TELEMETRY_TABLE + "/Field");
    }

    private static double[] fieldPose(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }

    // ---- Control ----

    /**
     * Switches to the given pipeline.
     * @param pipelineIndex Pipeline index (0-9)
     */
    public void setPipelineIndex(int pipelineIndex) {
        table.getEntry("pipeline").setDouble((int) clampArg(pipelineIndex, 0, 9));
    }


    /**
     * An immutable, validated pipeline configuration override. It holds the
     * contents of a .vpr file. Create it one time during robot initialization.
     * All file IO and size validation happen at that time. Then publish it at
     * any time with {@link #setPipelineConfigurationOverride(PipelineConfiguration)}:
     * {@snippet :
     * // robotInit
     * PipelineConfiguration aiming = Limelight.PipelineConfiguration.fromDeployFolder("aiming");
     * PipelineConfiguration intake = Limelight.PipelineConfiguration.fromDeployFolder("intake");
     * // mid-match
     * camera.setPipelineConfigurationOverride(aiming);
     * }
     *
     * <p>A failed load produces an instance where {@link #isValid()} is false.
     */
    public static final class PipelineConfiguration {
        private final String contents;
        private final LoadStatus status;

        private PipelineConfiguration(String contents, LoadStatus status) {
            this.contents = contents;
            this.status = status;
        }

        /**
         * Wraps .vpr contents. Checks the
         * {@link #MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES} size cap one time.
         *
         * @param contents The .vpr pipeline file contents
         * @return The wrapped configuration. Not valid when null, empty, or too
         *         large
         */
        public static PipelineConfiguration fromString(String contents) {
            if (contents == null || contents.isEmpty()) {
                System.out.println(
                        "Limelight - pipeline configuration rejected: no contents");
                return new PipelineConfiguration(null, LoadStatus.NO_CONTENTS);
            }
            if (contents.getBytes(StandardCharsets.UTF_8).length
                    > MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES) {
                System.out.println("Limelight - pipeline configuration rejected: larger than "
                        + MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES + " bytes");
                return new PipelineConfiguration(null, LoadStatus.TOO_LARGE);
            }
            return new PipelineConfiguration(contents, LoadStatus.OK);
        }

        /**
         * Reads a .vpr pipeline file from the deploy directory of the robot program
         * ({@code src/main/deploy} in the robot project). If the read fails, this method
         * prints a warning and returns a configuration that is not valid.
         *
         * @param deployRelativePath The pipeline file relative to the deploy
         *        directory, for example "aiming" or "pipelines/aiming.vpr". ".vpr" is
         *        added when missing
         * @return The loaded configuration. Check {@link #isValid}
         */
        public static PipelineConfiguration fromDeployFolder(String deployRelativePath) {
            if (deployRelativePath == null || deployRelativePath.isEmpty()) {
                System.out.println("Limelight - no pipeline configuration file name given");
                return new PipelineConfiguration(null, LoadStatus.NO_CONTENTS);
            }
            String contents = readFileForNT(
                    deployPath(deployRelativePath, ".vpr", "pipeline configuration"),
                    "pipeline configuration");
            if (contents == null) {
                return new PipelineConfiguration(null, LoadStatus.READ_FAILED);
            }
            return fromString(contents);
        }

        /** @return True if this holds a usable configuration */
        public boolean isValid() {
            return status == LoadStatus.OK;
        }

        /** @return Why this configuration is or is not usable */
        public LoadStatus getLoadStatus() {
            return status;
        }

        /** @return The configuration's size in bytes, or 0 when invalid */
        public int sizeBytes() {
            return contents == null ? 0 : contents.getBytes(StandardCharsets.UTF_8).length;
        }
    }

    /**
     * Publishes a pipeline configuration override to this camera. Flushes
     * NetworkTables immediately. The camera runs the override while it is
     * enabled with {@link #setUsePipelineConfigurationOverride}. The ten
     * pipelines on the camera do not change. You can switch between them and
     * the override at any time.
     *
     * <p>Create the {@link PipelineConfiguration} during robot initialization.
     * Publishing is only a NetworkTables write. It does not access the disk. You
     * can switch between several prepared configurations during a match:
     * {@snippet :
     * // robotInit
     * PipelineConfiguration aiming = Limelight.PipelineConfiguration.fromDeployFolder("aiming");
     * // whenever
     * camera.setPipelineConfigurationOverride(aiming);
     * camera.setUsePipelineConfigurationOverride(true);
     * }
     *
     * @param config The configuration to publish. The method ignores null or a
     *        configuration that is not valid and prints a warning
     */
    public void setPipelineConfigurationOverride(PipelineConfiguration config) {
        if (config == null || !config.isValid()) {
            System.out.println(
                    "Limelight - ignoring invalid pipeline configuration override");
            return;
        }
        table.getEntry("codepipeline_set").setString(config.contents);
        flushNT();
    }

    /**
     * Clears the published pipeline configuration override. Flushes NetworkTables
     * immediately. If the override was running, the camera returns immediately to
     * the pipeline selected by {@link #setPipelineIndex}. Otherwise, it returns when the
     * override is next disabled.
     */
    public void clearPipelineConfigurationOverride() {
        table.getEntry("codepipeline_set").setString("");
        flushNT();
    }

    /**
     * Enables or disables the pipeline configuration override. While enabled, the
     * camera runs the pipeline published with {@link #setPipelineConfigurationOverride}.
     * While disabled, the camera runs the pipeline selected by {@link #setPipelineIndex}.
     * The published override stays on the camera in both states. You can switch
     * between the two at any time.
     *
     * @param use True to run the override. False to run the indexed pipeline
     */
    public void setUsePipelineConfigurationOverride(boolean use) {
        table.getEntry("codepipeline_enable_set").setInteger(use ? 1 : 0);
    }

    /**
     * Reads back whether the camera runs the pipeline configuration override. The
     * value comes from the latest results frame. It can differ from the value
     * requested with {@link #setUsePipelineConfigurationOverride}. For example, the
     * camera web interface can force the override off.
     *
     * @return True if the camera reports that the override is running
     */
    public boolean isPipelineConfigurationOverrideEnabled() {
        return getLatestResults().pipelineConfigurationOverrideActive;
    }

    /**
     * @return The camera's pipeline configuration override state from the latest
     *         results frame
     */
    public PipelineConfigurationOverrideState getPipelineConfigurationOverrideState() {
        return getLatestResults().pipelineConfigurationOverrideState;
    }

    /**
     * @return True if the camera localizes with the shared field map published
     *         with {@link #setSharedMap}. The value comes from the latest results frame
     */
    public boolean isSharedMapActive() {
        return getLatestResults().sharedMapActive;
    }

    /**
     * @return The camera's shared field map state from the latest results frame
     */
    public SharedMapState getSharedMapState() {
        return getLatestResults().sharedMapState;
    }


    /**
     * An immutable, validated shared field map. It holds the contents of an
     * .fmap file. Create it one time during robot initialization. All file IO
     * and size validation happen at that time. Then publish it at any time with
     * {@link #setSharedMap(FieldMap)}:
     * {@snippet :
     * // robotInit
     * FieldMap fieldMap = Limelight.FieldMap.fromDeployFolder("field");
     * Limelight.setSharedMap(fieldMap);
     * }
     *
     * <p>A failed load produces an instance where {@link #isValid()} is false.
     */
    public static final class FieldMap {
        private final String contents;
        private final LoadStatus status;

        private FieldMap(String contents, LoadStatus status) {
            this.contents = contents;
            this.status = status;
        }

        /**
         * Wraps .fmap contents. Checks the {@link #MAX_SHARED_MAP_BYTES} size cap
         * one time.
         *
         * @param contents The .fmap field map file contents
         * @return The wrapped field map. Not valid when null, empty, or too large
         */
        public static FieldMap fromString(String contents) {
            if (contents == null || contents.isEmpty()) {
                System.out.println("Limelight - field map rejected: no contents");
                return new FieldMap(null, LoadStatus.NO_CONTENTS);
            }
            if (contents.getBytes(StandardCharsets.UTF_8).length > MAX_SHARED_MAP_BYTES) {
                System.out.println("Limelight - field map rejected: larger than "
                        + MAX_SHARED_MAP_BYTES + " bytes");
                return new FieldMap(null, LoadStatus.TOO_LARGE);
            }
            return new FieldMap(contents, LoadStatus.OK);
        }

        /**
         * Reads an .fmap field map file from the deploy directory of the robot program
         * ({@code src/main/deploy} in the robot project). If the read fails, this method
         * prints a warning and returns a field map that is not valid.
         *
         * @param deployRelativePath The field map file relative to the deploy
         *        directory, for example "field" or "maps/field.fmap". ".fmap" is
         *        added when missing
         * @return The loaded field map. Check {@link #isValid}
         */
        public static FieldMap fromDeployFolder(String deployRelativePath) {
            if (deployRelativePath == null || deployRelativePath.isEmpty()) {
                System.out.println("Limelight - no field map file name given");
                return new FieldMap(null, LoadStatus.NO_CONTENTS);
            }
            String contents = readFileForNT(
                    deployPath(deployRelativePath, ".fmap", "field map"),
                    "field map");
            if (contents == null) {
                return new FieldMap(null, LoadStatus.READ_FAILED);
            }
            return fromString(contents);
        }

        /** @return True if this holds a usable field map */
        public boolean isValid() {
            return status == LoadStatus.OK;
        }

        /** @return Why this field map is or is not usable */
        public LoadStatus getLoadStatus() {
            return status;
        }

        /** @return The field map's size in bytes, or 0 when invalid */
        public int sizeBytes() {
            return contents == null ? 0 : contents.getBytes(StandardCharsets.UTF_8).length;
        }
    }

    /**
     * Publishes a shared field map on the "limelightshared" table. Flushes
     * NetworkTables immediately. Publishing is only a NetworkTables write. All
     * file IO and validation happened when the {@link FieldMap} was created.
     * While the shared map is not empty, every Limelight on the network
     * localizes with it instead of its uploaded map.
     *
     * @param fieldMap The field map to publish. The method ignores null or a
     *        field map that is not valid and prints a warning
     */
    public static void setSharedMap(FieldMap fieldMap) {
        if (fieldMap == null || !fieldMap.isValid()) {
            System.out.println("Limelight - ignoring invalid shared field map");
            return;
        }
        NetworkTableInstance.getDefault().getTable("limelightshared")
                .getEntry("map_set")
                .setString(fieldMap.contents);
        flushNT();
    }

    /**
     * Clears the shared field map. Flushes NetworkTables immediately. Every
     * Limelight on the network returns to its own uploaded map.
     */
    public static void clearSharedMap() {
        NetworkTableInstance.getDefault().getTable("limelightshared")
                .getEntry("map_set")
                .setString("");
        flushNT();
    }

    // Loader failures print a single line to stdout so they show up in robot
    // service logs, and never throw.
    private static Path deployPath(String deployRelativePath, String extension,
            String what) {
        String name = deployRelativePath.toLowerCase(java.util.Locale.ROOT).endsWith(extension)
                ? deployRelativePath
                : deployRelativePath + extension;
        try {
            Path relative = Path.of(name);
            if (relative.isAbsolute()) {
                System.out.println("Limelight - " + what
                        + " path must be relative to the deploy directory: " + name);
                return null;
            }
            return Filesystem.getDeployDirectory().toPath().resolve(relative);
        } catch (Exception e) {
            System.out.println(
                    "Limelight - could not resolve the deploy directory: " + e);
            return null;
        }
    }

    private static String readFileForNT(Path path, String what) {
        if (path == null) {
            return null;
        }
        try {
            return Files.readString(path, StandardCharsets.UTF_8);
        } catch (Exception e) {
            System.out.println(
                    "Limelight - could not read " + what + " file " + path + ": " + e);
            return null;
        }
    }

    /**
     * Sets the priority AprilTag ID for tx/ty targeting.
     * @param id Priority tag ID
     */
    public void setPriorityTagIDOverride(int id) {
        table.getEntry("priorityid").setDouble((int) clampArg(id, -1, Integer.MAX_VALUE));
    }

    /** Clears the priority AprilTag ID override. tx/ty targeting returns to the
     *  target selection of the pipeline. */
    public void clearPriorityTagIDOverride() {
        table.getEntry("priorityid").setDouble(-1);
    }

    /**
     * LED behavior modes.
     */
    public enum LEDMode {
        /** LED behavior is controlled by the current pipeline */
        PIPELINE_CONTROL(0),
        /** LEDs forced off */
        FORCE_OFF(1),
        /** LEDs forced to blink */
        FORCE_BLINK(2),
        /** LEDs forced on */
        FORCE_ON(3);

        private final int ntValue;

        LEDMode(int ntValue) {
            this.ntValue = ntValue;
        }
    }

    /**
     * Sets the LED behavior.
     */
    public void setLEDMode(LEDMode mode) {
        if (mode == null) { return; }
        table.getEntry("ledMode").setDouble(mode.ntValue);
    }

    /**
     * Sets the crop window. The crop window in the web interface must be fully
     * open (as large as possible).
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    public void setCropWindowOverride(double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
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
     * @param horizontal Horizontal keystone value (-0.95 to 0.95)
     * @param vertical Vertical keystone value (-0.95 to 0.95)
     */
    public void setKeystoneOverride(double horizontal, double vertical) {
        table.getEntry("keystone_set").setDoubleArray(new double[] {
                clampArg(horizontal, -0.95, 0.95, 0),
                clampArg(vertical, -0.95, 0.95, 0)});
    }

    /** Clears the keystone override. */
    public void clearKeystoneOverride() {
        table.getEntry("keystone_set").setDoubleArray(new double[] {0, 0});
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
    public void setFiducial3DOffsetOverride(double forward, double left, double up) {
        if (!(Double.isFinite(forward) && Double.isFinite(left) && Double.isFinite(up))) { return; }
        table.getEntry("fiducial_offset_set").setDoubleArray(new double[] {forward, left, up});
    }

    /**
     * Moves the 3D targeting point away from the center of the primary in-view
     * fiducial. The offset is in target space (the coordinate system of the
     * target). Same as {@link #setFiducial3DOffsetOverride(double, double, double)}
     * with a Translation3d.
     *
     * @param offset Offset from the target in meters (x = forward, y = left,
     *        z = up). Null is ignored
     */
    public void setFiducial3DOffsetOverride(Translation3d offset) {
        if (offset == null) { return; }
        setFiducial3DOffsetOverride(offset.getX(), offset.getY(), offset.getZ());
    }

    /** Clears the fiducial 3D offset override. The 3D targeting point returns to
     *  the center of the primary in-view fiducial. */
    public void clearFiducial3DOffsetOverride() {
        table.getEntry("fiducial_offset_set").setDoubleArray(new double[] {0, 0, 0});
    }

    /**
     * (ADVANCED) Sets the individual robot orientation of this camera for the
     * MegaTag2 algorithm. Call this every loop. Most robots should use
     * {@link #setSharedRobotOrientation} instead. It updates every camera at one time.
     *
     * <p>Every call makes this camera ignore the shared orientation from
     * {@link #setSharedRobotOrientation}. Use {@link #setUseSharedOrientation} to make it use
     * the shared orientation again.
     *
     * @param yawDegrees Robot yaw in degrees.
     * @param flush True to flush NetworkTables immediately. Pass false when you
     *        update several cameras in one loop. Then call {@link #flushNT} one time.
     */
    public void setRobotOrientation(double yawDegrees, boolean flush) {
        setRobotOrientationInternal(yawDegrees, 0, 0, 0, 0, 0, flush);
    }

    /**
     * (ADVANCED) Sets the full individual robot orientation of this camera for
     * the MegaTag2 algorithm. Every call makes this camera ignore the shared
     * orientation (see {@link #setUseSharedOrientation}).
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
     *        update several cameras in one loop. Then call {@link #flushNT} one time.
     */
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate,
            double roll, double rollRate, boolean flush) {
        setRobotOrientationInternal(yaw, yawRate, pitch, pitchRate, roll, rollRate, flush);
    }

    private void setRobotOrientationInternal(double yaw, double yawRate, double pitch, double pitchRate,
            double roll, double rollRate, boolean flush) {
        if (!(Double.isFinite(yaw) && Double.isFinite(yawRate) && Double.isFinite(pitch)
                && Double.isFinite(pitchRate) && Double.isFinite(roll)
                && Double.isFinite(rollRate))) { return; }
        setUseSharedOrientation(false);
        table.getEntry("robot_orientation_set")
                .setDoubleArray(new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
        if (flush) {
            flushNT();
        }
    }

    /**
     * (ADVANCED) Controls whether this camera reads the shared orientation from
     * {@link #setSharedRobotOrientation}. Every {@link #setRobotOrientation} call makes the
     * camera ignore the shared orientation. Call this method with true to use
     * the shared orientation again. For example, call {@link #setRobotOrientation} one
     * time to seed the camera. Then let the shared orientation drive MegaTag2.
     *
     * @param useShared True to follow the shared orientation. False to use only
     *        the individual orientation of this camera
     */
    public void setUseSharedOrientation(boolean useShared) {
        table.getEntry("robot_orientation_ignoreshared_set").setInteger(useShared ? 0 : 1);
    }

    /**
     * Sets the robot orientation for MegaTag2 on the shared "limelightshared"
     * table. Flushes NetworkTables immediately. Every Limelight on the network
     * reads this table. One call updates all cameras. You do not need to call
     * {@link #setRobotOrientation} for each instance.
     *
     * <p>Each camera follows this shared orientation unless it was opted out. Every
     * {@link #setRobotOrientation} call opts a camera out (see
     * {@link #setUseSharedOrientation}). For example, a turret camera can use its own
     * orientation while every other camera follows the shared value.
     *
     * @param yawDegrees Robot yaw in degrees.
     */
    public static void setSharedRobotOrientation(double yawDegrees) {
        setSharedRobotOrientation(yawDegrees, 0, 0, 0, 0, 0);
    }

    /**
     * Sets the full shared robot orientation for MegaTag2 on the
     * "limelightshared" table. Flushes NetworkTables immediately. Each camera
     * follows this shared orientation unless it was opted out with
     * {@link #setRobotOrientation} or {@link #setUseSharedOrientation}.
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
    public static void setSharedRobotOrientation(double yaw, double yawRate, double pitch,
            double pitchRate, double roll, double rollRate) {
        if (!(Double.isFinite(yaw) && Double.isFinite(yawRate) && Double.isFinite(pitch)
                && Double.isFinite(pitchRate) && Double.isFinite(roll)
                && Double.isFinite(rollRate))) { return; }
        NetworkTableInstance.getDefault().getTable("limelightshared")
                .getEntry("robot_orientation_set")
                .setDoubleArray(new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
        flushNT();
    }

    /**
     * IMU sources for the robot yaw that MegaTag2 uses. The external orientation
     * comes from {@link #setRobotOrientation} or
     * {@link #setSharedRobotOrientation}. Only cameras with a built-in IMU use
     * this.
     */
    public enum IMUMode {
        /** Use the external orientation directly */
        EXTERNAL(0),
        /** Uses the external orientation. Continuously seeds the internal IMU with
         *  it */
        EXTERNAL_SEED_INTERNAL(1),
        /** Use the internal IMU only */
        INTERNAL(2),
        /** Internal IMU with a complementary filter. The filter converges on the
         *  MegaTag1 yaw */
        INTERNAL_MT1_ASSIST(3),
        /** Internal IMU with a complementary filter converging on the external orientation */
        INTERNAL_EXTERNAL_ASSIST(4),
        /** The camera reported a mode this library does not know (never valid to set) */
        UNKNOWN(-1);

        private final int ntValue;

        IMUMode(int ntValue) {
            this.ntValue = ntValue;
        }

        static IMUMode fromNtValue(int value) {
            for (IMUMode mode : values()) {
                if (mode.ntValue == value && mode != UNKNOWN) {
                    return mode;
                }
            }
            return UNKNOWN;
        }
    }

    /**
     * Configures the robot-yaw source for MegaTag2 localization.
     */
    public void setIMUMode(IMUMode mode) {
        if (mode == null || mode == IMUMode.UNKNOWN) { return; }
        table.getEntry("imumode_set").setDouble(mode.ntValue);
    }

    /**
     * Configures the complementary filter alpha for the IMU assist modes (modes 3
     * and 4).
     * @param alpha Default .001. Higher values converge on the assist source
     *        faster.
     */
    public void setIMUAssistAlpha(double alpha) {
        table.getEntry("imuassistalpha_set").setDouble(clampArg(alpha, 0.0001, 1, 0.001));
    }

    /**
     * Configures the throttle value. The Limelight skips {@code throttle} frames
     * between processed frames. Set 100-200 while the robot is disabled to reduce
     * heat. Use {@link #withStaleFrameThreshold} with this so {@link #getStatus()}
     * does not report the camera as stale.
     *
     * @param throttle Default 0. The camera processes one frame, then skips this
     *        many frames.
     */
    public void setThrottle(int throttle) {
        table.getEntry("throttle_set").setDouble(Math.max(0, throttle));
    }

    /**
     * Overrides the valid AprilTag IDs for localization. Tags not in this list are
     * ignored for robot pose estimation. They do not get the "fielded" flag.
     * @param validIDs Valid AprilTag IDs to track
     */
    public void setFiducialIDFiltersOverride(int[] validIDs) {
        if (validIDs == null) { return; }
        double[] validIDsDouble = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) {
            validIDsDouble[i] = Math.max(0, validIDs[i]);
        }
        table.getEntry("fiducial_id_filters_set").setDoubleArray(validIDsDouble);
    }

    /** Clears the AprilTag ID filter override. The camera returns to the ID
     *  filters of the pipeline. */
    public void clearFiducialIDFiltersOverride() {
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[0]);
    }

    /**
     * AprilTag detector downscaling factors. More downscaling improves
     * performance. It can reduce the detection range.
     */
    public enum DownscaleOverride {
        /** Use the downscale configured in the current pipeline */
        PIPELINE_CONTROL(0),
        /** No downscaling */
        X1(1),
        /** 1.5x downscale */
        X1_5(2),
        /** 2x downscale */
        X2(3),
        /** 3x downscale */
        X3(4),
        /** 4x downscale */
        X4(5);

        private final int ntValue;

        DownscaleOverride(int ntValue) {
            this.ntValue = ntValue;
        }
    }

    /**
     * Overrides the AprilTag detector's downscaling factor.
     */
    public void setFiducialDownscalingOverride(DownscaleOverride downscale) {
        if (downscale == null) { return; }
        table.getEntry("fiducial_downscale_set").setDouble(downscale.ntValue);
    }

    /** Clears the AprilTag downscaling override. The camera returns to the
     *  downscale configured in the current pipeline. */
    public void clearFiducialDownscalingOverride() {
        setFiducialDownscalingOverride(DownscaleOverride.PIPELINE_CONTROL);
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
     *        update several cameras in one loop. Then call {@link #flushNT} one time.
     */
    public void setCameraPose_RobotSpaceOverride(double forward, double left, double up,
            double roll, double pitch, double yaw, boolean flush) {
        publishCameraPose(forward, left, up, roll, pitch, yaw);
        if (flush) {
            flushNT();
        }
    }

    /**
     * Sets the camera pose relative to the robot. The camera applies updates
     * live. See {@link #setCameraPose_RobotSpaceOverride(double, double, double, double, double, double, boolean)}.
     *
     * @param cameraPoseRobotSpace The pose of the camera relative to the robot
     *        center (x = forward, y = left, z = up, in meters). Null is ignored.
     *        An all-zero pose is the clear value
     * @param flush True to flush NetworkTables immediately. Pass false when you
     *        update several cameras in one loop. Then call {@link #flushNT} one time.
     */
    public void setCameraPose_RobotSpaceOverride(Pose3d cameraPoseRobotSpace, boolean flush) {
        if (cameraPoseRobotSpace == null) { return; }
        setCameraPose_RobotSpaceOverride(
                cameraPoseRobotSpace.getX(), cameraPoseRobotSpace.getY(),
                cameraPoseRobotSpace.getZ(),
                Math.toDegrees(cameraPoseRobotSpace.getRotation().getX()),
                Math.toDegrees(cameraPoseRobotSpace.getRotation().getY()),
                Math.toDegrees(cameraPoseRobotSpace.getRotation().getZ()),
                flush);
    }

    /**
     * Clears the camera pose override. Flushes NetworkTables immediately. The
     * camera returns to the camera pose configured in its web interface.
     */
    public void clearCameraPose_RobotSpaceOverride() {
        table.getEntry("camerapose_robotspace_set")
                .setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        flushNT();
    }

    private void publishCameraPose(double forward, double left, double up,
            double roll, double pitch, double yaw) {
        if (!(Double.isFinite(forward) && Double.isFinite(left) && Double.isFinite(up)
                && Double.isFinite(roll) && Double.isFinite(pitch)
                && Double.isFinite(yaw))) { return; }
        table.getEntry("camerapose_robotspace_set")
                .setDoubleArray(new double[] {forward, left, up, roll, pitch, yaw});
    }

    /**
     * Sends data to a running python snapscript via llrobot.
     */
    public void setPythonScriptData(double[] outgoingPythonData) {
        if (outgoingPythonData == null) { return; }
        table.getEntry("llrobot").setDoubleArray(outgoingPythonData);
    }

    /**
     * Triggers a snapshot capture. The Limelight firmware rate-limits this.
     */
    public void triggerSnapshot() {
        double current = table.getEntry("snapshot").getDouble(0);
        table.getEntry("snapshot").setDouble(current + 1);
    }

    /**
     * Enables or pauses the rewind buffer recording.
     * @param enabled True to enable recording. False to pause. Default true on
     *        supported platforms.
     */
    public void setRewindEnabled(boolean enabled) {
        table.getEntry("rewind_enable_set").setDouble(enabled ? 1 : 0);
    }

    /**
     * Triggers a rewind capture with the given duration. The maximum duration is
     * 165 seconds. The camera rate-limits this.
     * @param durationSeconds Duration of the rewind capture in seconds (maximum
     *        165)
     */
    public void triggerRewindCapture(double durationSeconds) {
        double[] currentArray = table.getEntry("capture_rewind").getDoubleArray(EMPTY);
        double counter = (currentArray.length > 0) ? currentArray[0] : 0;
        table.getEntry("capture_rewind")
                .setDoubleArray(new double[] {counter + 1, clampArg(durationSeconds, 1, 165)});
    }

    /**
     * Flushes NetworkTables immediately. These methods call it automatically: the
     * pose-setting constructors, {@link #setSharedRobotOrientation},
     * {@link #setPipelineConfigurationOverride},
     * {@link #clearPipelineConfigurationOverride}, {@link #setSharedMap},
     * {@link #clearSharedMap}, {@link #clearCameraPose_RobotSpaceOverride}, and
     * {@link #setRobotOrientation} / {@link #setCameraPose_RobotSpaceOverride}
     * when their flush argument is true. The other setters and clear methods do
     * not flush. Call it yourself after a group of them.
     */
    public static void flushNT() {
        NetworkTableInstance.getDefault().flush();
    }

    private static double clampArg(double value, double min, double max) {
        return clampArg(value, min, max, min);
    }

    private static double clampArg(double value, double min, double max, double nanFallback) {
        if (Double.isNaN(value)) {
            return nanFallback;
        }
        return Math.max(min, Math.min(max, value));
    }

    // ---- Geometry utilities ----

    /**
     * Converts a pose array to a Pose3d. The array has 6 values: [x, y, z, roll,
     * pitch, yaw]. Units are meters and degrees.
     * @return The pose. Returns an empty Pose3d if the array is not valid
     */
    public static Pose3d toPose3D(double[] inData) {
        if (inData == null || inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    /**
     * Converts a pose array to a Pose2d. The array has 6 values: [x, y, z, roll,
     * pitch, yaw]. Units are meters and degrees. Uses only the x, y, and yaw
     * values.
     * @return The pose. Returns an empty Pose2d if the array is not valid
     */
    public static Pose2d toPose2D(double[] inData) {
        if (inData == null || inData.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
                new Translation2d(inData[0], inData[1]),
                new Rotation2d(Units.degreesToRadians(inData[5])));
    }

    // ---- Result classes ----

    /**
     * The complete decoded results for one frame. All fields come from one
     * MessagePack payload.
     */
    public static class LimelightResults {
        /** True if at least one target is valid in this frame */
        public boolean valid;
        /** Legacy timestamp in milliseconds since vision-process boot */
        public double timestampMillis;
        /** Microseconds since vision-process boot */
        public long timestampMicros;
        /** System wall-clock time in microseconds since epoch */
        public long systemTimestampMicros;
        /** NetworkTables server time in microseconds. 0 if the camera is not
         *  connected to a server */
        public long ntTimestampMicros;
        /** Frame counter starting at 0, or -1 before the first frame */
        public long frameIndex = -1;
        /** Targeting/pipeline latency in milliseconds */
        public double targetingLatencyMillis;
        /** Capture latency in milliseconds */
        public double captureLatencyMillis;
        /** Active pipeline index */
        public double pipelineIndex = -1;
        /** Active pipeline type (e.g. "pipe_fiducial") */
        public String pipelineType = "";
        /** Focus metric for focus-assist */
        public double focusMetric;

        /** Camera-reported MegaTag1 standard deviations [x, y, z, roll, pitch,
         *  yaw]. The camera averages these over a long window. Use for telemetry
         *  only. Do not use for fusion */
        public double[] stdDevsMT1 = EMPTY;
        /** Camera-reported MegaTag2 standard deviations [x, y, z, roll, pitch,
         *  yaw]. The camera averages these over a long window. Use for telemetry
         *  only. Do not use for fusion */
        public double[] stdDevsMT2 = EMPTY;

        /** MegaTag1 robot pose in field space [x, y, z, roll, pitch, yaw] (meters, degrees) */
        public double[] robotPoseMT1 = EMPTY;
        /** MegaTag1 robot pose, WPILib red-alliance coordinate system */
        public double[] robotPoseMT1WpiRed = EMPTY;
        /** MegaTag1 robot pose, WPILib blue-alliance coordinate system */
        public double[] robotPoseMT1WpiBlue = EMPTY;
        /** MegaTag2 robot pose in field space */ 
        public double[] robotPoseMT2 = EMPTY;
        /** MegaTag2 robot pose, WPILib red-alliance coordinate system */
        public double[] robotPoseMT2WpiRed = EMPTY;
        /** MegaTag2 robot pose, WPILib blue-alliance coordinate system */
        public double[] robotPoseMT2WpiBlue = EMPTY;

        /** Number of AprilTags contributing to the botpose estimates */
        public int reportedTagCount;
        /** Max distance between contributing tags in meters */
        public double tagSpanMeters;
        /** Average distance to contributing tags in meters */
        public double avgTagDistanceMeters;
        /** Average area of contributing tags (percentage of image) */
        public double avgTagAreaPercent;

        /** Final camera pose in robot space [x, y, z, roll, pitch, yaw] (meters, degrees) */
        public double[] cameraPoseRobotSpace = EMPTY;
        /** Data set by a python snapscript via llpython */
        public double[] pythonOutput = EMPTY;

        /** Horizontal offset from crosshair to primary target in degrees */
        public double txDegrees;
        /** Vertical offset from crosshair to primary target in degrees */
        public double tyDegrees;
        /** Horizontal offset from principal pixel to primary target in degrees */
        public double txDegreesNoCrosshair;
        /** Vertical offset from principal pixel to primary target in degrees */
        public double tyDegreesNoCrosshair;
        /** Primary target area (percentage of image, 0-100) */
        public double targetAreaPercent;
        /** 3D distance from camera to primary target in meters */
        public double targetDistanceMeters;

        public RetroTarget[] retroTargets = new RetroTarget[0];
        public FiducialTarget[] fiducialTargets = new FiducialTarget[0];
        public DetectorTarget[] detectorTargets = new DetectorTarget[0];
        public ClassifierTarget[] classifierTargets = new ClassifierTarget[0];
        /** Classifier results below the confidence threshold. For visualization
         *  and debugging */
        public ClassifierTarget[] classifierTargetsRejected = new ClassifierTarget[0];
        public BarcodeTarget[] barcodeTargets = new BarcodeTarget[0];
        public CounterTarget[] counterTargets = new CounterTarget[0];
        public OCRTarget[] ocrTargets = new OCRTarget[0];

        public IMUData imu = new IMUData();
        public HardwareData hardware = new HardwareData();
        public RewindData rewind = new RewindData();
        public RobotOrientationData robotOrientation = new RobotOrientationData();
        public CameraIntrinsics intrinsics = new CameraIntrinsics();

        /** Image source index */
        public int imageSource;
        /** Hardware type identifier */
        public int hardwareType;
        /** UI refresh counter */
        public int uiRefresh;
        /** True if the camera is ignoring NetworkTables input */
        public boolean ignoreNT;
        /** True if the camera is connected to a NetworkTables server */
        public boolean ntConnected;
        /** True if the camera is running a robot-published pipeline configuration
         *  override */
        public boolean pipelineConfigurationOverrideActive;
        /** Pipeline configuration override state reported by the camera */
        public PipelineConfigurationOverrideState pipelineConfigurationOverrideState =
                PipelineConfigurationOverrideState.OFF;
        /** True if the camera is localizing against the robot-published shared
         *  field map */
        public boolean sharedMapActive;
        /** Shared field map state reported by the camera */
        public SharedMapState sharedMapState = SharedMapState.OFF;

        /** Time spent decoding this envelope on the robot, in milliseconds.
         *  Library-side overhead for logging, not part of the camera's latency. */
        public double parseLatencyMillis;
        /** Local NetworkTables time when this frame arrived, in seconds. 0 if this
         *  object did not come from the network */
        public double receiveTimestampSeconds;
        /** Non-empty if the envelope failed to decode */
        public String error = "";
        private boolean fromLiveSubscriber;

        /**
         * @return Seconds since this frame arrived. If this value keeps growing, the
         *         camera has stopped sending frames. Possible causes: disconnected,
         *         stopped, or renamed.
         */
        public double getAgeSeconds() {
            return NetworkTablesJNI.now() / Limelight.NT_TICKS_PER_SECOND - receiveTimestampSeconds;
        }

        /** @return Total number of retro, fiducial, detector, classifier, and
         *          barcode targets. Counter and OCR results are not included */
        public int getTargetCount() {
            return retroTargets.length + fiducialTargets.length + detectorTargets.length
                    + classifierTargets.length + barcodeTargets.length;
        }

        /** @return MegaTag1 robot pose in field space (field-center origin) */
        public Pose3d getRobotPose_MT1() {
            return toPose3D(robotPoseMT1);
        }

        /** @return MegaTag2 robot pose in field space. The origin is the field center */
        public Pose3d getRobotPose_MT2() {
            return toPose3D(robotPoseMT2);
        }

        /** @return The full 3D robot pose for the given estimate type. The type
         *          selects the origin and the algorithm */
        public Pose3d getRobotPose(PoseEstimateType type) {
            return toPose3D(poseArray(this, type));
        }

        /** @return Camera pose in robot space (meters, degrees) */
        public Pose3d getCameraPose_RobotSpace() {
            return toPose3D(cameraPoseRobotSpace);
        }
    }

    /**
     * Fields common to all image-space targets.
     */
    public abstract static class LimelightTarget {
        LimelightTarget() {}

        /** Horizontal offset from crosshair to target in degrees */
        public double txDegrees;
        /** Vertical offset from crosshair to target in degrees */
        public double tyDegrees;
        /** Horizontal offset from principal pixel to target in degrees */
        public double txDegreesNoCrosshair;
        /** Vertical offset from principal pixel to target in degrees */
        public double tyDegreesNoCrosshair;
        /** Target x position in pixels */
        public double txPixels;
        /** Target y position in pixels */
        public double tyPixels;
        /** Target area (percentage of image, 0-100) */
        public double targetAreaPercent;
        /** Target corners as [x, y] pixel pairs. Empty unless "send corners" is
         *  enabled */
        public double[][] corners = EMPTY_CORNERS;
    }

    /**
     * A color/retroreflective target.
     */
    public static class RetroTarget extends LimelightTarget {
        /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters, degrees) */
        public double[] cameraPoseTargetSpace = EMPTY;
        /** Target pose in camera space */
        public double[] targetPoseCameraSpace = EMPTY;
        /** Target pose in robot space */
        public double[] targetPoseRobotSpace = EMPTY;
        /** Robot pose in target space */
        public double[] robotPoseTargetSpace = EMPTY;
        /** Robot pose in field space */
        public double[] robotPoseFieldSpace = EMPTY;

        /** @return Camera pose in target space as a Pose3d */
        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPoseTargetSpace);
        }

        /** @return Target pose in camera space as a Pose3d */
        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPoseCameraSpace);
        }

        /** @return Target pose in robot space as a Pose3d */
        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPoseRobotSpace);
        }

        /** @return Robot pose in target space as a Pose3d */
        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPoseTargetSpace);
        }

        /** @return Robot pose in field space as a Pose3d */
        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPoseFieldSpace);
        }
    }

    /**
     * An AprilTag/fiducial target.
     */
    public static class FiducialTarget extends LimelightTarget {
        /** Fiducial/AprilTag ID */
        public int fiducialId;
        /** Fiducial family (e.g. "36h11") */
        public String family = "";
        /** Target skew values */
        public double[] skew = EMPTY;
        /** Pose ambiguity. Lower is better. Values above 0.7 are usually not
         *  reliable */
        public double ambiguity;
        /** True if this tag contributed to pose estimation. The tag matched the
         *  field map and the ID filters did not exclude it. */
        public boolean fielded;

        /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters, degrees) */
        public double[] cameraPoseTargetSpace = EMPTY;
        /** Target pose in camera space */
        public double[] targetPoseCameraSpace = EMPTY;
        /** Target pose in robot space */
        public double[] targetPoseRobotSpace = EMPTY;
        /** Robot pose in target space */
        public double[] robotPoseTargetSpace = EMPTY;
        /** Robot pose in field space (MegaTag1, this tag only) */
        public double[] robotPoseFieldSpace = EMPTY;
        /** Robot pose in field space (MegaTag2, this tag only) */
        public double[] robotPoseFieldSpaceMT2 = EMPTY;

        /** @return 3D distance from the camera to this tag in meters */
        public double getDistanceToCamera() {
            return distance3d(targetPoseCameraSpace);
        }

        /** @return 3D distance from the robot center to this tag in meters */
        public double getDistanceToRobot() {
            return distance3d(targetPoseRobotSpace);
        }

        /** @return Camera pose in target space as a Pose3d */
        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPoseTargetSpace);
        }

        /** @return Target pose in camera space as a Pose3d */
        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPoseCameraSpace);
        }

        /** @return Target pose in robot space as a Pose3d */
        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPoseRobotSpace);
        }

        /** @return Robot pose in target space as a Pose3d */
        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPoseTargetSpace);
        }

        /** @return MegaTag1 robot pose in field space from this tag only, as a Pose3d */
        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPoseFieldSpace);
        }

        /** @return MegaTag2 robot pose in field space from this tag only, as a Pose3d */
        public Pose3d getRobotPose_FieldSpace_MT2() {
            return toPose3D(robotPoseFieldSpaceMT2);
        }

        private static double distance3d(double[] pose) {
            if (pose.length < 3) {
                return 0;
            }
            return Math.sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
        }
    }

    /**
     * A neural detector target.
     */
    public static class DetectorTarget extends LimelightTarget {
        /** Class index from the class list */
        public int classId = -1;
        /** Human-readable class name */
        public String className = "";
        /** Detection confidence (0-1) */
        public double confidence;
        /** Tracking ID for object counting pipelines. -1 means no tracking */
        public int trackId = -1;
    }

    /**
     * A neural classifier result.
     */
    public static class ClassifierTarget {
        /** Class index from the class list */
        public int classId = -1;
        /** Human-readable class name */
        public String className = "";
        /** Classification confidence (0-1) */
        public double confidence;
    }

    /**
     * A barcode/QR code target.
     */
    public static class BarcodeTarget extends LimelightTarget {
        /** Barcode family (e.g. "QRCODE") */
        public String family = "";
        /** Decoded barcode data string */
        public String data = "";
    }

    /**
     * A per-class object counter result from the neural counter pipeline.
     */
    public static class CounterTarget {
        public int classId = -1;
        public String className = "";
        public int count;
    }

    /**
     * An OCR (text recognition) result.
     */
    public static class OCRTarget {
        public boolean valid;
        public int regionId = -1;
        /** Recognized text */
        public String text = "";
        public double confidence;
        public double numericValue;
        public boolean hasNumericValue;
        public int digitCount;
        public String unit = "";
        public boolean isCounter;
        public double ratePerMin;
        public int alarmState;
        public String alarmString = "";
        /** Bounding box [x, y, width, height] in pixels */
        public double[] boundingBox = EMPTY;
        public double processingTimeMs;
    }

    /**
     * Live camera intrinsics for the running pipeline. The camera matrix is scaled
     * to the current processing resolution.
     */
    public static class CameraIntrinsics {
        /** True when the camera uses a user-uploaded calibration file. False when
         *  the camera uses a default calibration */
        public boolean customCalibration;
        /** Horizontal field of view in degrees */
        public double hfovDegrees;
        /** Vertical field of view in degrees */
        public double vfovDegrees;
        /** Processing-resolution width in pixels (what the camera matrix is scaled to) */
        public double resolutionWidthPixels;
        /** Processing-resolution height in pixels */
        public double resolutionHeightPixels;
        /** Row-major 3x3 camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1], scaled to
         *  the processing resolution. Empty until the camera publishes */
        public double[] cameraMatrix = EMPTY;
        /** OpenCV distortion coefficients. Empty until the camera publishes */
        public double[] distortionCoefficients = EMPTY;
    }

    /**
     * The state of the camera's internal IMU.
     */
    public static class IMUData {
        /** Orientation quaternion [w, x, y, z] */
        public double[] quaternion = {1, 0, 0, 0};
        /** Final fused robot yaw in degrees, the value MegaTag2 uses (includes yawOffsetDegrees) */
        public double robotYawDegrees;
        /** Yaw offset in degrees currently applied to the internal IMU */
        public double yawOffsetDegrees;
        /** Internal IMU roll in degrees */
        public double rollDegrees;
        /** Internal IMU pitch in degrees */
        public double pitchDegrees;
        /** Raw internal IMU yaw in degrees (before yawOffsetDegrees, see {@link #robotYawDegrees}) */
        public double yawDegrees;
        /** Gyro angular velocity about X in degrees per second */
        public double gyroXDegreesPerSecond;
        /** Gyro angular velocity about Y in degrees per second */
        public double gyroYDegreesPerSecond;
        /** Gyro angular velocity about Z in degrees per second */
        public double gyroZDegreesPerSecond;
        /** Accelerometer X (forward) axis */
        public double accelX;
        /** Accelerometer Y (left) axis */
        public double accelY;
        /** Accelerometer Z (up) axis */
        public double accelZ;
    }

    /**
     * Camera hardware and system stats.
     */
    public static class HardwareData {
        /** CPU temperature in degrees Celsius */
        public double cpuTempCelsius;
        /** CPU usage percentage */
        public double cpuUsagePercent;
        /** RAM usage percentage */
        public double ramUsagePercent;
        /** Free disk space in MB */
        public long diskFreeMB;
        /** Total disk space in MB */
        public long diskTotalMB;
        /** Camera sensor ID */
        public String cameraId = "";
        /** True if an AI accelerator is present */
        public boolean accelPresent;
        /** AI accelerator type */
        public String accelType = "";
        /** AI accelerator temperature in degrees Celsius */
        public double accelTempCelsius;
        /** AI accelerator power draw in watts */
        public double accelPowerWatts;
        /** True if the AI accelerator is throttling */
        public boolean accelThrottling;
    }

    /**
     * Rewind buffer state.
     */
    public static class RewindData {
        public boolean enabled;
        public double storedSeconds;
        public long frameCount;
        public double bufferUsage;
        public boolean flushing;
        /** Main-thread latency penalty in microseconds */
        public int latencyPenaltyMicros;
    }

    /**
     * The robot-orientation state the camera is using for MegaTag2.
     */
    public static class RobotOrientationData {
        /** Active IMU mode ({@link IMUMode#UNKNOWN} before the first frame) */
        public IMUMode imuMode = IMUMode.UNKNOWN;
        /** IMU assist alpha */
        public double alpha;
        /** Last interpolated robot yaw used for botpose, in degrees */
        public double interpolatedYawDegrees;
    }

    /**
     * A robot pose estimate with the metadata needed for pose-estimator fusion.
     */
    public static class PoseEstimate {
        public Pose2d pose = new Pose2d();
        /**
         * Latency-compensated capture timestamp in the local NetworkTables timebase.
         * Pass this value directly to {@code addVisionMeasurement} in WPILib.
         */
        public double timestampSeconds;
        /** Total latency (capture + targeting) in milliseconds */
        public double latencyMillis;
        /** Camera-reported botpose tag count, telemetry only */
        public int reportedTagCount;
        /** Number of fielded tags. Used for validity, filtering, and standard
         *  deviation scaling */
        public int fieldedTagCount;
        /** Max distance between contributing tags in meters */
        public double tagSpanMeters;
        /** Average distance to contributing tags in meters */
        public double avgTagDistanceMeters;
        /** Average area of contributing tags (percentage of image) */
        public double avgTagAreaPercent;
        /**
         * Fusion-ready standard deviations [x meters, y meters, theta radians]. The
         * {@link PoseEstimateConfig} of this camera computes them from tag distance and tag
         * count. Pass this value as the third argument to {@code addVisionMeasurement}.
         */
        public Vector<N3> stdDevs = VecBuilder.fill(
                PoseEstimateConfig.UNTRUSTED,
                PoseEstimateConfig.UNTRUSTED,
                PoseEstimateConfig.UNTRUSTED);
        /**
         * Standard deviations [x, y, z, roll, pitch, yaw] reported by the camera. The
         * camera averages them over a long window. Use them for telemetry only. Do not
         * give them to a pose estimator. Use {@link #stdDevs} instead.
         */
        public double[] reportedStdDevs = EMPTY;
        /** The fiducials visible in this frame */
        public FiducialTarget[] rawFiducials = new FiducialTarget[0];
        /** Which pose estimate output this is (origin + algorithm) */
        public PoseEstimateType type = PoseEstimateType.MT1_WPIBLUE;
        /**
         * The frame index of the source camera. Estimates from the same camera with
         * the same index came from one frame. Do not fuse more than one pose type
         * from one frame. The index can reset when the camera restarts.
         */
        public long frameIndex = -1;
        /**
         * OR of the {@link PoseEstimateConfig} REJECT_* flags for every check this estimate
         * failed. 0 = accepted. Decode with {@link PoseEstimateConfig#describeRejection}.
         * The flags reflect the filter configured for the algorithm of this estimate.
         */
        public int rejectionFlags;

        /**
         * @return The visible fiducials that contributed to pose estimation. These
         *         have {@link FiducialTarget#fielded} set
         */
        public FiducialTarget[] getFieldedFiducials() {
            int count = 0;
            for (FiducialTarget fiducial : rawFiducials) {
                if (fiducial.fielded) {
                    count++;
                }
            }
            FiducialTarget[] out = new FiducialTarget[count];
            int i = 0;
            for (FiducialTarget fiducial : rawFiducials) {
                if (fiducial.fielded) {
                    out[i++] = fiducial;
                }
            }
            return out;
        }

        /** @return True if this estimate was produced by the MegaTag2 algorithm */
        public boolean isMT2() {
            return usesMT2(type);
        }

        /** @return True if at least one fielded tag produced this estimate and no
         *          check rejected it. This does not show current camera health. It
         *          does not show whether the frame was already read. */
        public boolean isValid() {
            return fieldedTagCount > 0 && rejectionFlags == 0;
        }

        @Override
        public String toString() {
            return String.format(
                    "PoseEstimate(%s x=%.2f y=%.2f deg=%.1f fieldedTags=%d reportedTags=%d "
                            + "avgDist=%.2f ts=%.3f%s)",
                    type, pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
                    fieldedTagCount, reportedTagCount, avgTagDistanceMeters, timestampSeconds,
                    rejectionFlags != 0
                            ? " REJECTED:" + PoseEstimateConfig.describeRejection(rejectionFlags)
                            : "");
        }
    }

    /**
     * A reusable configuration object for rejection filters and standard
     * deviation scaling. Start from a factory ({@link #defaultMT1()},
     * {@link #defaultMT2()}, {@link #noFiltering()}) or from a new instance. Chain
     * the with* methods. Attach the result with
     * {@link Limelight#withPoseEstimateConfig_MT1} or
     * {@link Limelight#withPoseEstimateConfig_MT2}.
     *
     * <p>A new instance accepts every structurally valid estimate that has at
     * least one fielded tag. It uses the same standard deviation model as
     * {@link #defaultMT2()}.
     *
     * <pre>
     * new Limelight("limelight")
     *     .withPoseEstimateConfig_MT1(PoseEstimateConfig.defaultMT1()
     *         .withMinTagCount(2)
     *         .withFieldBounds(17.55, 8.05))
     *     .withPoseEstimateConfig_MT2(PoseEstimateConfig.defaultMT2()
     *         .withMaxAvgTagDistance(4.0)
     *         .withFieldBounds(17.55, 8.05));
     * </pre>
     */
    public static class PoseEstimateConfig {
        /** Rejection flag: too few contributing tags */
        public static final int REJECT_TAG_COUNT = 1 << 0;
        /** Rejection flag: single-tag ambiguity above the configured maximum */
        public static final int REJECT_AMBIGUITY = 1 << 1;
        /** Rejection flag: tag distance above a configured maximum */
        public static final int REJECT_TAG_DISTANCE = 1 << 2;
        /** Rejection flag: average tag area below the configured minimum */
        public static final int REJECT_TAG_AREA = 1 << 3;
        /** Rejection flag: pose outside the configured field bounds */
        public static final int REJECT_FIELD_BOUNDS = 1 << 4;

        /** Rejection flag set by the library: the source envelope had a decode
         *  error */
        public static final int REJECT_DECODE_ERROR = 1 << 5;

        /** Rejection flag set by the library: a value needed for fusion was NaN or
         *  infinite */
        public static final int REJECT_NONFINITE = 1 << 6;

        /** Rejection flag set by the library: no usable capture timestamp. For
         *  example, an envelope decoded with {@link Limelight#decode(byte[])} has no receive
         *  time */
        public static final int REJECT_NO_TIMESTAMP = 1 << 7;

        /** Rejection flag set by the library: the pose array was absent or too
         *  short, or it was the all-zero value the camera sends when it has no
         *  estimate */
        public static final int REJECT_MISSING_POSE = 1 << 8;

        /** Rejection flag set by the library: tags contributed but the reported tag
         *  distance was zero or negative */
        public static final int REJECT_BAD_METADATA = 1 << 9;

        /** Rejection flag set by the library: fiducials were visible but none
         *  contributed to pose estimation */
        public static final int REJECT_NO_FIELDED_TAGS = 1 << 11;

        private int minTagCount = 1;
        private double maxSingleTagAmbiguity = 1.0;
        private double maxSingleTagDistance = 0;
        private double maxAvgTagDistance = 0;
        private double minAvgTagArea = 0;
        private double fieldLengthMeters = 0;
        private double fieldWidthMeters = 0;
        private double fieldBoundsMarginMeters = 0.5;

        /** Creates a configuration with the default values. */
        public PoseEstimateConfig() {}

        /**
         * Creates an independent copy of another configuration.
         * @param other The configuration to copy. Null gives the default values
         */
        public PoseEstimateConfig(PoseEstimateConfig other) {
            if (other == null) { return; }
            this.minTagCount = other.minTagCount;
            this.maxSingleTagAmbiguity = other.maxSingleTagAmbiguity;
            this.maxSingleTagDistance = other.maxSingleTagDistance;
            this.maxAvgTagDistance = other.maxAvgTagDistance;
            this.minAvgTagArea = other.minAvgTagArea;
            this.fieldLengthMeters = other.fieldLengthMeters;
            this.fieldWidthMeters = other.fieldWidthMeters;
            this.fieldBoundsMarginMeters = other.fieldBoundsMarginMeters;
            this.xyStdDev = other.xyStdDev;
            this.thetaStdDev = other.thetaStdDev;
            this.distanceExponent = other.distanceExponent;
            this.minScalingDistance = other.minScalingDistance;
            this.maxScalingDistance = other.maxScalingDistance;
            this.tagCountExponent = other.tagCountExponent;
            this.minXYStdDev = other.minXYStdDev;
            this.maxXYStdDev = other.maxXYStdDev;
            this.minThetaStdDev = other.minThetaStdDev;
            this.maxThetaStdDev = other.maxThetaStdDev;
        }

        /**
         * Requires at least this many contributing tags. The count uses the
         * {@link FiducialTarget#fielded} flags. At least one fielded tag is always required.
         * Use {@code withMinTagCount(2)} to ignore single-tag MT1 estimates.
         */
        public PoseEstimateConfig withMinTagCount(int minTagCount) {
            this.minTagCount = Math.max(1, minTagCount);
            return this;
        }

        /**
         * Rejects single-tag estimates when the ambiguity of the fielded tag is above
         * this value (0-1). A value of 1 disables this check.
         */
        public PoseEstimateConfig withMaxSingleTagAmbiguity(double maxSingleTagAmbiguity) {
            this.maxSingleTagAmbiguity = clampArg(maxSingleTagAmbiguity, 0, 1, 1);
            return this;
        }

        /**
         * Rejects single-tag estimates when the tag is farther than this distance in
         * meters. Single-tag estimates lose accuracy with distance faster than
         * multi-tag estimates. Set this value tighter than {@link #withMaxAvgTagDistance}.
         * For example, allow multi-tag estimates to 6 m and single-tag estimates to
         * 2.5 m. Applies only when exactly one tag contributes. 0 disables this
         * check.
         */
        public PoseEstimateConfig withMaxSingleTagDistance(double maxSingleTagDistanceMeters) {
            this.maxSingleTagDistance = clampArg(maxSingleTagDistanceMeters, 0, Double.MAX_VALUE);
            return this;
        }

        /** Rejects estimates when the average tag distance is above this value in
         *  meters. A value of 0 disables this check. */
        public PoseEstimateConfig withMaxAvgTagDistance(double maxAvgTagDistanceMeters) {
            this.maxAvgTagDistance = clampArg(maxAvgTagDistanceMeters, 0, Double.MAX_VALUE);
            return this;
        }

        /** Rejects estimates when the average tag area (percentage of image) is
         *  below this value. A value of 0 disables this check. */
        public PoseEstimateConfig withMinAvgTagArea(double minAvgTagArea) {
            this.minAvgTagArea = clampArg(minAvgTagArea, 0, 100);
            return this;
        }

        /**
         * Rejects estimates outside the field. The bounds depend on the coordinate
         * origin of the estimate. Corner origins (wpiblue, wpired) span [0, length] x
         * [0, width]. Centered origins span +/-length/2 x +/-width/2.
         *
         * <p>A zero length or width disables this check.
         *
         * @param fieldLengthMeters Field length (x extent) in meters
         * @param fieldWidthMeters Field width (y extent) in meters
         */
        public PoseEstimateConfig withFieldBounds(double fieldLengthMeters, double fieldWidthMeters) {
            this.fieldLengthMeters = clampArg(fieldLengthMeters, 0, Double.MAX_VALUE);
            this.fieldWidthMeters = clampArg(fieldWidthMeters, 0, Double.MAX_VALUE);
            return this;
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
        public PoseEstimateConfig withFieldBoundsMargin(double marginMeters) {
            this.fieldBoundsMarginMeters = Double.isNaN(marginMeters)
                    ? 0 : clamp(marginMeters, -Double.MAX_VALUE, Double.MAX_VALUE);
            return this;
        }

        private void applyTo(PoseEstimate estimate) {
            estimate.stdDevs = compute(estimate);
            estimate.rejectionFlags |= evaluate(estimate);
        }

        private int evaluate(PoseEstimate estimate) {
            int flags = 0;
            double maxFieldedAmbiguity = 0;
            for (FiducialTarget fiducial : estimate.rawFiducials) {
                if (fiducial.fielded) {
                    maxFieldedAmbiguity = Math.max(maxFieldedAmbiguity, fiducial.ambiguity);
                }
            }
            int contributingTags = estimate.fieldedTagCount;

            if (contributingTags < minTagCount) {
                flags |= REJECT_TAG_COUNT;
            }
            if (maxSingleTagAmbiguity < 1.0 && contributingTags == 1) {
                if (maxFieldedAmbiguity > maxSingleTagAmbiguity) {
                    flags |= REJECT_AMBIGUITY;
                }
            }
            if (maxSingleTagDistance > 0 && contributingTags == 1
                    && estimate.avgTagDistanceMeters > maxSingleTagDistance) {
                flags |= REJECT_TAG_DISTANCE;
            }
            if (maxAvgTagDistance > 0 && estimate.avgTagDistanceMeters > maxAvgTagDistance) {
                flags |= REJECT_TAG_DISTANCE;
            }
            if (minAvgTagArea > 0 && estimate.avgTagAreaPercent < minAvgTagArea) {
                flags |= REJECT_TAG_AREA;
            }
            if (fieldLengthMeters > 0 && fieldWidthMeters > 0) {
                double x = estimate.pose.getX();
                double y = estimate.pose.getY();
                boolean outOfBounds;
                if (centeredOrigin(estimate.type)) {
                    outOfBounds =
                            Math.abs(x) > fieldLengthMeters / 2 + fieldBoundsMarginMeters
                            || Math.abs(y) > fieldWidthMeters / 2 + fieldBoundsMarginMeters;
                } else {
                    outOfBounds = x < -fieldBoundsMarginMeters
                            || x > fieldLengthMeters + fieldBoundsMarginMeters
                            || y < -fieldBoundsMarginMeters
                            || y > fieldWidthMeters + fieldBoundsMarginMeters;
                }
                if (outOfBounds) {
                    flags |= REJECT_FIELD_BOUNDS;
                }
            }
            return flags;
        }

        /**
         * @return The rejection flags as readable text, for example
         *         "TAG_COUNT|AMBIGUITY". Empty if the estimate was accepted
         */
        public static String describeRejection(int flags) {
            if (flags == 0) {
                return"";
            }
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < REJECTION_FLAGS.length; i++) {
                appendFlag(sb, flags, REJECTION_FLAGS[i], REJECTION_NAMES[i]);
            }
            return sb.toString();
        }

        private static void appendFlag(StringBuilder sb, int flags, int flag, String name) {
            if ((flags & flag) != 0) {
                if (sb.length() > 0) {
                    sb.append('|');
                }
                sb.append(name);
            }
        }

        // ---- Fusion standard deviations ----
        // xy = clamp(xyStdDev * distance^distExp / fieldedTagCount^tagExp, minXY, maxXY)

        /** A standard deviation so large that a pose estimator ignores the
         *  measurement */
        public static final double UNTRUSTED = 9999999;
        private static final double MIN_XY_STD_DEV = 0.0001;
        /** Smallest heading standard deviation this configuration produces, in radians */
        public static final double MIN_THETA_STD_DEV = 0.01;

        /**
         * The MegaTag1 defaults. The library uses these when you do not provide an
         * MT1 configuration. Single-tag estimates must pass the ambiguity gate (0.7)
         * and the distance gate (3 m). MT1 computes the full pose from tag geometry
         * only. Heading is not fused. XY standard deviation = 0.5 * tagDistanceMeters
         * / sqrt(fieldedTagCount), with a minimum of 0.0001 m. These are untuned
         * initial values. Tune them on your robot. Start from this configuration and
         * chain changes: {@code PoseEstimateConfig.defaultMT1().withFieldBounds(...)}.
         */
        public static PoseEstimateConfig defaultMT1() {
            return new PoseEstimateConfig()
                    .withMaxSingleTagAmbiguity(0.7)
                    .withMaxSingleTagDistance(3.0)
                    .withStdDevXY(0.5);
        }

        /**
         * The MegaTag2 defaults. The library uses these when you do not provide an
         * MT2 configuration. There are no acceptance gates. Gyro-fused MT2 is robust,
         * and structural validation still applies. Heading is not fused. XY standard
         * deviation = 0.3 * tagDistanceMeters / sqrt(fieldedTagCount), with a minimum
         * of 0.0001 m. These are untuned initial values. Tune them on your robot.
         * Start from this configuration and chain changes.
         */
        public static PoseEstimateConfig defaultMT2() {
            return new PoseEstimateConfig()
                    .withStdDevXY(0.3);
        }

        /**
         * Accepts every structurally valid pose. Uses a fixed 0.5 m XY standard
         * deviation. Heading is untrusted.
         */
        public static PoseEstimateConfig noFiltering() {
            return noFiltering(0.5, UNTRUSTED);
        }

        /**
         * Accepts every structurally valid pose. Uses the fixed standard deviations
         * that you select.
         *
         * @param xyStdDevMeters Fixed XY standard deviation in meters
         * @param thetaStdDevRadians Fixed heading standard deviation in radians. Pass
         *        {@link #UNTRUSTED} to exclude vision heading from fusion
         */
        public static PoseEstimateConfig noFiltering(double xyStdDevMeters, double thetaStdDevRadians) {
            return new PoseEstimateConfig()
                    .withStdDevXY(xyStdDevMeters)
                    .withStdDevTheta(thetaStdDevRadians)
                    .withStdDevDistanceScaling(0)
                    .withStdDevTagCountDivision(0);
        }

        private double xyStdDev = 0.3;
        private double thetaStdDev = UNTRUSTED;
        private double distanceExponent = 1.0;
        private double minScalingDistance = 0.0;
        private double maxScalingDistance = Double.MAX_VALUE;
        private double tagCountExponent = 0.5;
        private double minXYStdDev = MIN_XY_STD_DEV;
        private double maxXYStdDev = Double.MAX_VALUE;
        private double minThetaStdDev = MIN_THETA_STD_DEV;
        private double maxThetaStdDev = Double.MAX_VALUE;

        /** Sets the base XY standard deviation in meters. When distance scaling is
         *  active, this value is per meter of average tag distance. */
        public PoseEstimateConfig withStdDevXY(double baseMeters) {
            this.xyStdDev = clampArg(
                    baseMeters, MIN_XY_STD_DEV, UNTRUSTED, UNTRUSTED);
            return this;
        }

        /** Sets the base XY standard deviation. Clamps the computed value to
         *  [min, max] meters. */
        public PoseEstimateConfig withStdDevXY(double baseMeters, double minMeters, double maxMeters) {
            withStdDevXY(baseMeters);
            double lo = clampArg(
                    minMeters, MIN_XY_STD_DEV, UNTRUSTED, MIN_XY_STD_DEV);
            double hi = clampArg(
                    maxMeters, MIN_XY_STD_DEV, UNTRUSTED, UNTRUSTED);
            this.minXYStdDev = Math.min(lo, hi);
            this.maxXYStdDev = Math.max(lo, hi);
            return this;
        }

        /** Sets the base heading standard deviation in radians. The default excludes
         *  vision heading from fusion. Lower this value only to fuse vision heading.
         *  The minimum is {@link #MIN_THETA_STD_DEV} radians. */
        public PoseEstimateConfig withStdDevTheta(double baseRadians) {
            this.thetaStdDev = clampArg(baseRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
            return this;
        }

        /** Sets the base heading standard deviation. Clamps the computed value to
         *  [min, max] radians. The minimum is {@link #MIN_THETA_STD_DEV} radians. */
        public PoseEstimateConfig withStdDevTheta(double baseRadians, double minRadians, double maxRadians) {
            withStdDevTheta(baseRadians);
            double lo = clampArg(minRadians, MIN_THETA_STD_DEV, UNTRUSTED, MIN_THETA_STD_DEV);
            double hi = clampArg(maxRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
            this.minThetaStdDev = Math.min(lo, hi);
            this.maxThetaStdDev = Math.max(lo, hi);
            return this;
        }

        /**
         * Scales the standard deviations by avgTagDistance^exponent. 1 = linear
         * (default). 2 = quadratic. 0 = no distance scaling.
         */
        public PoseEstimateConfig withStdDevDistanceScaling(double exponent) {
            this.distanceExponent = clampArg(exponent, 0, 10, 1);
            return this;
        }

        /** Same as {@link #withStdDevDistanceScaling(double)}. Clamps the tag distance to
         *  [min, max] meters before the exponent applies. Tags closer than min scale
         *  as if at min. Tags farther than max scale as if at max. */
        public PoseEstimateConfig withStdDevDistanceScaling(double exponent, double minMeters, double maxMeters) {
            withStdDevDistanceScaling(exponent);
            double lo = clampArg(minMeters, 0, Double.MAX_VALUE, 0);
            double hi = clampArg(maxMeters, 0, Double.MAX_VALUE, Double.MAX_VALUE);
            this.minScalingDistance = Math.min(lo, hi);
            this.maxScalingDistance = Math.max(lo, hi);
            return this;
        }

        /** Divides the standard deviations by fieldedTagCount^exponent. The default
         *  0.5 divides by the square root of the fielded tag count. 0 disables this. */
        public PoseEstimateConfig withStdDevTagCountDivision(double exponent) {
            this.tagCountExponent = clampArg(exponent, 0, 10, 0.5);
            return this;
        }

        private Vector<N3> compute(PoseEstimate estimate) {
            double scale = 1.0;
            if (distanceExponent != 0) {
                if (!Double.isFinite(estimate.avgTagDistanceMeters)
                        || estimate.avgTagDistanceMeters <= 0) {
                    return VecBuilder.fill(UNTRUSTED, UNTRUSTED, UNTRUSTED);
                }
                double distance = clamp(estimate.avgTagDistanceMeters, minScalingDistance, maxScalingDistance);
                scale *= Math.pow(distance, distanceExponent);
            }
            if (tagCountExponent != 0 && estimate.fieldedTagCount > 1) {
                scale /= Math.pow(estimate.fieldedTagCount, tagCountExponent);
            }
            double xy = clamp(xyStdDev * scale, minXYStdDev, maxXYStdDev);
            double theta = clamp(thetaStdDev * scale, minThetaStdDev, maxThetaStdDev);
            return VecBuilder.fill(xy, xy, theta);
        }

        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    // ---- Decoding ----

    private static void decodeResults(MsgPackReader r, LimelightResults out) {
        int n = r.expectMapHeader();
        for (int i = 0; i < n; i++) {
            String key = r.readString();
            switch (key) {
                case "v": out.valid = r.readLong() != 0; break;
                case "ts": out.timestampMillis = r.readDouble(); break;
                case "ts_us": out.timestampMicros = r.readLong(); break;
                case "ts_sys": out.systemTimestampMicros = r.readLong(); break;
                case "ts_nt": out.ntTimestampMicros = r.readLong(); break;
                case "fidx": out.frameIndex = r.readLong(); break;
                case "tl": out.targetingLatencyMillis = r.readDouble(); break;
                case "cl": out.captureLatencyMillis = r.readDouble(); break;
                case "pID": out.pipelineIndex = r.readDouble(); break;
                case "pTYPE": out.pipelineType = r.readString(); break;
                case "focus_metric": out.focusMetric = r.readDouble(); break;
                case "stdev_mt1": out.stdDevsMT1 = r.readDoubleArray(); break;
                case "stdev_mt2": out.stdDevsMT2 = r.readDoubleArray(); break;
                case "botpose": out.robotPoseMT1 = r.readDoubleArray(); break;
                case "botpose_wpired": out.robotPoseMT1WpiRed = r.readDoubleArray(); break;
                case "botpose_wpiblue": out.robotPoseMT1WpiBlue = r.readDoubleArray(); break;
                case "botpose_orb": out.robotPoseMT2 = r.readDoubleArray(); break;
                case "botpose_orb_wpired": out.robotPoseMT2WpiRed = r.readDoubleArray(); break;
                case "botpose_orb_wpiblue": out.robotPoseMT2WpiBlue = r.readDoubleArray(); break;
                case "botpose_tagcount": out.reportedTagCount = (int) r.readLong(); break;
                case "botpose_span": out.tagSpanMeters = r.readDouble(); break;
                case "botpose_avgdist": out.avgTagDistanceMeters = r.readDouble(); break;
                case "botpose_avgarea": out.avgTagAreaPercent = r.readDouble(); break;
                case "t6c_rs": out.cameraPoseRobotSpace = r.readDoubleArray(); break;
                case "PythonOut": out.pythonOutput = r.readDoubleArray(); break;
                case "tx": out.txDegrees = r.readDouble(); break;
                case "ty": out.tyDegrees = r.readDouble(); break;
                case "txnc": out.txDegreesNoCrosshair = r.readDouble(); break;
                case "tync": out.tyDegreesNoCrosshair = r.readDouble(); break;
                case "ta": out.targetAreaPercent = r.readDouble(); break;
                case "tdist": out.targetDistanceMeters = r.readDouble(); break;
                case "Retro": out.retroTargets = decodeRetroTargets(r); break;
                case "Fiducial": out.fiducialTargets = decodeFiducialTargets(r); break;
                case "Detector": out.detectorTargets = decodeDetectorTargets(r); break;
                case "Classifier": out.classifierTargets = decodeClassifierTargets(r); break;
                case "ClassifierRejected": out.classifierTargetsRejected = decodeClassifierTargets(r); break;
                case "Barcode": out.barcodeTargets = decodeBarcodeTargets(r); break;
                case "Counter": out.counterTargets = decodeCounterTargets(r); break;
                case "OCR": out.ocrTargets = decodeOCRTargets(r); break;
                case "imu": decodeIMU(r, out.imu); break;
                case "hw": decodeHardware(r, out.hardware); break;
                case "rewind": decodeRewind(r, out.rewind); break;
                case "botorient": decodeBotOrientation(r, out.robotOrientation); break;
                case "intrinsics": decodeIntrinsics(r, out.intrinsics); break;
                case "imgsrc": out.imageSource = (int) r.readLong(); break;
                case "hwtype": out.hardwareType = (int) r.readLong(); break;
                case "uirefresh": out.uiRefresh = (int) r.readLong(); break;
                case "ignorent": out.ignoreNT = r.readLong() != 0; break;
                case "ntconnected": out.ntConnected = r.readLong() != 0; break;
                case "codepipeline": out.pipelineConfigurationOverrideActive = r.readLong() != 0; break;
                case "codepipelinestate": out.pipelineConfigurationOverrideState = parseOverrideState(r.readString()); break;
                case "codemap": out.sharedMapActive = r.readLong() != 0; break;
                case "codemapstate": out.sharedMapState = parseSharedMapState(r.readString()); break;
                default: r.skipValue(); break;
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
                    case "t6c_ts": t.cameraPoseTargetSpace = r.readDoubleArray(); break;
                    case "t6t_cs": t.targetPoseCameraSpace = r.readDoubleArray(); break;
                    case "t6t_rs": t.targetPoseRobotSpace = r.readDoubleArray(); break;
                    case "t6r_ts": t.robotPoseTargetSpace = r.readDoubleArray(); break;
                    case "t6r_fs": t.robotPoseFieldSpace = r.readDoubleArray(); break;
                    default: r.skipValue(); break;
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
                    case "fID": t.fiducialId = (int) r.readLong(); break;
                    case "fam": t.family = r.readString(); break;
                    case "skew": t.skew = r.readDoubleArray(); break;
                    case "ambig": t.ambiguity = r.readDouble(); break;
                    case "fielded": t.fielded = r.readLong() != 0; break;
                    case "t6c_ts": t.cameraPoseTargetSpace = r.readDoubleArray(); break;
                    case "t6t_cs": t.targetPoseCameraSpace = r.readDoubleArray(); break;
                    case "t6t_rs": t.targetPoseRobotSpace = r.readDoubleArray(); break;
                    case "t6r_ts": t.robotPoseTargetSpace = r.readDoubleArray(); break;
                    case "t6r_fs": t.robotPoseFieldSpace = r.readDoubleArray(); break;
                    case "t6r_fs_orb": t.robotPoseFieldSpaceMT2 = r.readDoubleArray(); break;
                    default: r.skipValue(); break;
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
                    case "classID": t.classId = (int) r.readLong(); break;
                    case "class": t.className = r.readString(); break;
                    case "conf": t.confidence = r.readDouble(); break;
                    case "tID": t.trackId = (int) r.readLong(); break;
                    default: r.skipValue(); break;
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
                    case "classID": t.classId = (int) r.readLong(); break;
                    case "class": t.className = r.readString(); break;
                    case "conf": t.confidence = r.readDouble(); break;
                    default: r.skipValue(); break;
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
                    case "fam": t.family = r.readString(); break;
                    case "data": t.data = r.readString(); break;
                    default: r.skipValue(); break;
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
                    case "classID": t.classId = (int) r.readLong(); break;
                    case "class": t.className = r.readString(); break;
                    case "count": t.count = (int) r.readLong(); break;
                    default: r.skipValue(); break;
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
                    case "valid": t.valid = r.readLong() != 0; break;
                    case "regionId": t.regionId = (int) r.readLong(); break;
                    case "text": t.text = r.readString(); break;
                    case "confidence": t.confidence = r.readDouble(); break;
                    case "numericValue": t.numericValue = r.readDouble(); break;
                    case "hasNumericValue": t.hasNumericValue = r.readLong() != 0; break;
                    case "digitCount": t.digitCount = (int) r.readLong(); break;
                    case "unit": t.unit = r.readString(); break;
                    case "isCounter": t.isCounter = r.readLong() != 0; break;
                    case "ratePerMin": t.ratePerMin = r.readDouble(); break;
                    case "alarmState": t.alarmState = (int) r.readLong(); break;
                    case "alarmString": t.alarmString = r.readString(); break;
                    case "bbox": t.boundingBox = r.readDoubleArray(); break;
                    case "processingTimeMs": t.processingTimeMs = r.readDouble(); break;
                    default: r.skipValue(); break;
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
                case "quat": imu.quaternion = r.readDoubleArray(); break;
                case "yaw_offset": imu.yawOffsetDegrees = r.readDouble(); break;
                // The "yaw" key carries the same fused yaw as data[0]. The array is the single source.
                case "data": {
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
                default: r.skipValue(); break;
            }
        }
    }

    private static void decodeHardware(MsgPackReader r, HardwareData hw) {
        int n = r.readMapHeader();
        for (int i = 0; i < n; i++) {
            String key = r.readString();
            switch (key) {
                case "temp": hw.cpuTempCelsius = r.readDouble(); break;
                case "cpu": hw.cpuUsagePercent = r.readDouble(); break;
                case "ram": hw.ramUsagePercent = r.readDouble(); break;
                case "dfree": hw.diskFreeMB = r.readLong(); break;
                case "dtot": hw.diskTotalMB = r.readLong(); break;
                case "cid": hw.cameraId = r.readString(); break;
                case "hailo": {
                    int m = r.readMapHeader();
                    for (int j = 0; j < m; j++) {
                        String hkey = r.readString();
                        switch (hkey) {
                            case "present": hw.accelPresent = r.readLong() != 0; break;
                            case "type": hw.accelType = r.readString(); break;
                            case "temp": hw.accelTempCelsius = r.readDouble(); break;
                            case "power": hw.accelPowerWatts = r.readDouble(); break;
                            case "throttle": hw.accelThrottling = r.readLong() != 0; break;
                            default: r.skipValue(); break;
                        }
                    }
                    break;
                }
                default: r.skipValue(); break;
            }
        }
    }

    private static void decodeRewind(MsgPackReader r, RewindData rw) {
        int n = r.readMapHeader();
        for (int i = 0; i < n; i++) {
            String key = r.readString();
            switch (key) {
                case "enabled": rw.enabled = r.readLong() != 0; break;
                case "storedSeconds": rw.storedSeconds = r.readDouble(); break;
                case "frameCount": rw.frameCount = r.readLong(); break;
                case "bufferUsage": rw.bufferUsage = r.readDouble(); break;
                case "flushing": rw.flushing = r.readLong() != 0; break;
                case "latpen": rw.latencyPenaltyMicros = (int) r.readLong(); break;
                default: r.skipValue(); break;
            }
        }
    }

    private static void decodeIntrinsics(MsgPackReader r, CameraIntrinsics intrinsics) {
        int n = r.readMapHeader();
        for (int i = 0; i < n; i++) {
            String key = r.readString();
            switch (key) {
                case "customcal": intrinsics.customCalibration = r.readLong() != 0; break;
                case "hfov": intrinsics.hfovDegrees = r.readDouble(); break;
                case "vfov": intrinsics.vfovDegrees = r.readDouble(); break;
                case "resw": intrinsics.resolutionWidthPixels = r.readDouble(); break;
                case "resh": intrinsics.resolutionHeightPixels = r.readDouble(); break;
                case "cameramatrix": intrinsics.cameraMatrix = r.readDoubleArray(); break;
                case "distcoeffs": intrinsics.distortionCoefficients = r.readDoubleArray(); break;
                default: r.skipValue(); break;
            }
        }
    }

    private static void decodeBotOrientation(MsgPackReader r, RobotOrientationData bo) {
        int n = r.readMapHeader();
        for (int i = 0; i < n; i++) {
            String key = r.readString();
            switch (key) {
                case "imumode": bo.imuMode = IMUMode.fromNtValue((int) r.readLong()); break;
                case "alpha": bo.alpha = r.readDouble(); break;
                case "interpbotyaw": bo.interpolatedYawDegrees = r.readDouble(); break;
                default: r.skipValue(); break;
            }
        }
    }

    private static boolean decodeCommonTargetKey(
            LimelightTarget target, String key, MsgPackReader r) {
        switch (key) {
            case "tx": target.txDegrees = r.readDouble(); return true;
            case "ty": target.tyDegrees = r.readDouble(); return true;
            case "tx_nocross": target.txDegreesNoCrosshair = r.readDouble(); return true;
            case "ty_nocross": target.tyDegreesNoCrosshair = r.readDouble(); return true;
            case "txp": target.txPixels = r.readDouble(); return true;
            case "typ": target.tyPixels = r.readDouble(); return true;
            case "ta": target.targetAreaPercent = r.readDouble(); return true;
            case "pts": target.corners = r.readPointArray(); return true;
            default: return false;
        }
    }

    // ---- MessagePack decoder ----

    /**
     * Minimal, allocation-light MessagePack reader covering the full msgpack spec
     * as produced by the camera (nil, bool, all int/uint widths, float32/64,
     * str, bin, array, map, with skippable ext types). Nil is read leniently as
     * 0 / "" / empty so absent optional data never throws.
     */
    private static final class MsgPackReader {
        private static final int MAX_SKIP_DEPTH = 64;

        private final byte[] buf;
        private int pos;
        private int skipDepth;

        MsgPackReader(byte[] buf) {
            this.buf = buf;
        }

        private int u8() {
            return buf[pos++] & 0xFF;
        }

        // Peeks at the next token without consuming it
        boolean nextIsMap() {
            int b = buf[pos] & 0xFF;
            return (b & 0xF0) == 0x80 || b == 0xDE || b == 0xDF;
        }

        private int u16() {
            int v = ((buf[pos] & 0xFF) << 8) | (buf[pos + 1] & 0xFF);
            pos += 2;
            return v;
        }

        private long u32() {
            long v = ((long) (buf[pos] & 0xFF) << 24) | ((buf[pos + 1] & 0xFF) << 16)
                    | ((buf[pos + 2] & 0xFF) << 8) | (buf[pos + 3] & 0xFF);
            pos += 4;
            return v;
        }

        private long u64() {
            long v = 0;
            for (int i = 0; i < 8; i++) {
                v = (v << 8) | (buf[pos + i] & 0xFF);
            }
            pos += 8;
            return v;
        }

        // The envelope root must be a map. Anything else is a decode error.
        int expectMapHeader() {
            int b = u8();
            if ((b & 0xF0) == 0x80) {
                return b & 0x0F;
            }
            switch (b) {
                case 0xDE: return checkedCount(u16(), 2, "map");
                case 0xDF: return checkedCount((int) u32(), 2, "map");
                default: throw badToken("map", b);
            }
        }

        // The typed readers below are lenient about the value type. A known key
        // whose value has an unexpected type is skipped and reads as its default
        // (0, 0.0, "", or empty) so one surprising field never fails the frame.
        int readMapHeader() {
            int b = u8();
            if ((b & 0xF0) == 0x80) {
                return b & 0x0F;
            }
            switch (b) {
                case 0xC0: return 0; // nil -> empty map
                case 0xDE: return checkedCount(u16(), 2, "map");
                case 0xDF: return checkedCount((int) u32(), 2, "map");
                default: skipMismatched(); return 0;
            }
        }

        // Rewinds to the token that did not match and skips the whole value
        private void skipMismatched() {
            pos--;
            skipValue();
        }

        int readArrayHeader() {
            int b = u8();
            if ((b & 0xF0) == 0x90) {
                return b & 0x0F;
            }
            switch (b) {
                case 0xC0: return 0; // nil -> empty array
                case 0xDC: return checkedCount(u16(), 1, "array");
                case 0xDD: return checkedCount((int) u32(), 1, "array");
                default: skipMismatched(); return 0;
            }
        }

        String readString() {
            int b = u8();
            int len;
            if (b >= 0xA0 && b <= 0xBF) {
                len = b & 0x1F;
            } else {
                switch (b) {
                    case 0xC0: return ""; // nil -> empty string
                    case 0xD9: len = u8(); break;
                    case 0xDA: len = u16(); break;
                    case 0xDB: len = (int) u32(); break;
                    default: skipMismatched(); return "";
                }
            }
            checkedCount(len, 1, "string");
            String s = new String(buf, pos, len, StandardCharsets.UTF_8);
            pos += len;
            return s;
        }

        // Rejects corrupt length prefixes before they drive a huge allocation or a
        // silent cursor desync (n items need >= n * minBytesPerItem remaining bytes)
        private int checkedCount(int count, int minBytesPerItem, String what) {
            if (count < 0 || (long) count * minBytesPerItem > buf.length - pos) {
                throw new IllegalStateException(
                        "corrupt " + what + " length " + count + " near byte " + pos);
            }
            return count;
        }

        long readLong() {
            int b = u8();
            if (b <= 0x7F) {
                return b; // positive fixint
            }
            if (b >= 0xE0) {
                return (byte) b; // negative fixint
            }
            switch (b) {
                case 0xC0: return 0; // nil
                case 0xC2: return 0; // false
                case 0xC3: return 1; // true
                case 0xCA: return (long) Float.intBitsToFloat((int) u32());
                case 0xCB: return (long) Double.longBitsToDouble(u64());
                case 0xCC: return u8();
                case 0xCD: return u16();
                case 0xCE: return u32();
                case 0xCF: return u64(); // uint64: values > Long.MAX_VALUE wrap, never produced in practice
                case 0xD0: return (byte) u8();
                case 0xD1: return (short) u16();
                case 0xD2: return (int) u32();
                case 0xD3: return u64();
                default: skipMismatched(); return 0;
            }
        }

        double readDouble() {
            int b = u8();
            if (b <= 0x7F) {
                return b;
            }
            if (b >= 0xE0) {
                return (byte) b;
            }
            switch (b) {
                case 0xC0: return 0.0;
                case 0xC2: return 0.0;
                case 0xC3: return 1.0;
                case 0xCA: return Float.intBitsToFloat((int) u32());
                case 0xCB: return Double.longBitsToDouble(u64());
                case 0xCC: return u8();
                case 0xCD: return u16();
                case 0xCE: return u32();
                case 0xCF: return u64();
                case 0xD0: return (byte) u8();
                case 0xD1: return (short) u16();
                case 0xD2: return (int) u32();
                case 0xD3: return u64();
                default: skipMismatched(); return 0.0;
            }
        }

        double[] readDoubleArray() {
            int n = readArrayHeader();
            if (n == 0) {
                return EMPTY;
            }
            double[] out = new double[n];
            for (int i = 0; i < n; i++) {
                out[i] = readDouble();
            }
            return out;
        }

        double[][] readPointArray() {
            int n = readArrayHeader();
            if (n == 0) {
                return EMPTY_CORNERS;
            }
            double[][] out = new double[n][];
            for (int i = 0; i < n; i++) {
                out[i] = readDoubleArray();
            }
            return out;
        }

        void skipValue() {
            // A corrupt envelope of nested container headers would recurse once per
            // byte and StackOverflowError past decode()'s never-throws catch
            if (++skipDepth > MAX_SKIP_DEPTH) {
                throw new IllegalStateException("msgpack nested deeper than " + MAX_SKIP_DEPTH);
            }
            try {
                skipValueInner();
            } finally {
                skipDepth--;
            }
        }

        private void skipValueInner() {
            int b = u8();
            if (b <= 0x7F || b >= 0xE0) {
                return; // fixint
            }
            if (b >= 0xA0 && b <= 0xBF) {
                skipBytes(b & 0x1F); // fixstr
                return;
            }
            if (b >= 0x90 && b <= 0x9F) {
                skipValues(b & 0x0F); // fixarray
                return;
            }
            if (b >= 0x80 && b <= 0x8F) {
                skipValues(2 * (b & 0x0F)); // fixmap
                return;
            }
            switch (b) {
                case 0xC0: case 0xC2: case 0xC3: return;
                case 0xCC: case 0xD0: skipBytes(1); return;
                case 0xCD: case 0xD1: skipBytes(2); return;
                case 0xCE: case 0xD2: case 0xCA: skipBytes(4); return;
                case 0xCF: case 0xD3: case 0xCB: skipBytes(8); return;
                case 0xD9: case 0xC4: skipBytes(u8()); return;
                case 0xDA: case 0xC5: skipBytes(u16()); return;
                case 0xDB: case 0xC6: skipBytes((int) u32()); return;
                case 0xD4: skipBytes(2); return;  // fixext1
                case 0xD5: skipBytes(3); return;  // fixext2
                case 0xD6: skipBytes(5); return;  // fixext4
                case 0xD7: skipBytes(9); return;  // fixext8
                case 0xD8: skipBytes(17); return; // fixext16
                case 0xC7: skipBytes(1 + u8()); return;
                case 0xC8: skipBytes(1 + u16()); return;
                case 0xC9: {
                    long length = u32();
                    if (length > Integer.MAX_VALUE - 1L) {
                        throw new IllegalStateException(
                                "corrupt ext32 length " + length + " near byte " + pos);
                    }
                    skipBytes(1 + (int) length);
                    return;
                }
                case 0xDC: skipValues(checkedCount(u16(), 1, "skipped array")); return;
                case 0xDD: skipValues(checkedCount((int) u32(), 1, "skipped array")); return;
                case 0xDE: skipValues(2 * checkedCount(u16(), 2, "skipped map")); return;
                case 0xDF: skipValues(2 * checkedCount((int) u32(), 2, "skipped map")); return;
                default: throw badToken("value", b);
            }
        }

        private void skipBytes(int count) {
            if (count < 0 || count > buf.length - pos) {
                throw new IllegalStateException("truncated msgpack value near byte " + pos);
            }
            pos += count;
        }

        private void skipValues(int count) {
            for (int i = 0; i < count; i++) {
                skipValue();
            }
        }

        private IllegalStateException badToken(String expected, int token) {
            return new IllegalStateException(
                    "expected " + expected + ", got 0x" + Integer.toHexString(token) + " at byte " + (pos - 1));
        }
    }

}
