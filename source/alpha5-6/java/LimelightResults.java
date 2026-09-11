// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.NetworkTablesJNI;

/** The complete decoded results for one frame. All fields come from one MessagePack payload. */
public class LimelightResults {
  /** True if at least one target is valid in this frame. */
  public boolean valid;

  /** Legacy timestamp in milliseconds since vision-process boot. */
  public double timestampMillis;

  /** Microseconds since vision-process boot. */
  public long timestampMicros;

  /** System wall-clock time in microseconds since epoch. */
  public long systemTimestampMicros;

  /** NetworkTables server time in microseconds. 0 if the camera is not connected to a server */
  public long ntTimestampMicros;

  /** Frame counter starting at 0, or -1 before the first frame. */
  public long frameIndex = -1;

  /** Targeting/pipeline latency in milliseconds. */
  public double targetingLatencyMillis;

  /** Capture latency in milliseconds. */
  public double captureLatencyMillis;

  /** Active pipeline index. */
  public double pipelineIndex = -1;

  /** Active pipeline type (e.g. "pipe_fiducial") */
  public String pipelineType = "";

  /** Focus metric for focus-assist. */
  public double focusMetric;

  /**
   * Camera-reported MegaTag1 standard deviations [x, y, z, roll, pitch, yaw]. The camera averages
   * these over a long window. Use for telemetry only. Do not use for fusion
   */
  public double[] stdDevsMT1 = new double[Limelight.POSE_ARRAY_LENGTH];

  /**
   * Camera-reported MegaTag2 standard deviations [x, y, z, roll, pitch, yaw]. The camera averages
   * these over a long window. Use for telemetry only. Do not use for fusion
   */
  public double[] stdDevsMT2 = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag1 robot pose in field space [x, y, z, roll, pitch, yaw] (meters, degrees). */
  public double[] robotPoseMT1 = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag1 robot pose, WPILib red-alliance coordinate system. */
  public double[] robotPoseMT1WpiRed = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag1 robot pose, WPILib blue-alliance coordinate system. */
  public double[] robotPoseMT1WpiBlue = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag2 robot pose in field space. */
  public double[] robotPoseMT2 = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag2 robot pose, WPILib red-alliance coordinate system. */
  public double[] robotPoseMT2WpiRed = new double[Limelight.POSE_ARRAY_LENGTH];

  /** MegaTag2 robot pose, WPILib blue-alliance coordinate system. */
  public double[] robotPoseMT2WpiBlue = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Number of AprilTags contributing to the botpose estimates. */
  public int reportedTagCount;

  /** Max distance between contributing tags in meters. */
  public double tagSpanMeters;

  /** Average distance to contributing tags in meters. */
  public double avgTagDistanceMeters;

  /** Average area of contributing tags (percentage of image). */
  public double avgTagAreaPercent;

  /** Final camera pose in robot space [x, y, z, roll, pitch, yaw] (meters, degrees). */
  public double[] cameraPoseRobotSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Data set by a python snapscript via llpython. */
  public double[] pythonOutput = Limelight.EMPTY;

  /** Horizontal offset from crosshair to primary target in degrees. */
  public double txDegrees;

  /** Vertical offset from crosshair to primary target in degrees. */
  public double tyDegrees;

  /** Horizontal offset from principal pixel to primary target in degrees. */
  public double txDegreesNoCrosshair;

  /** Vertical offset from principal pixel to primary target in degrees. */
  public double tyDegreesNoCrosshair;

  /** Primary target area (percentage of image, 0-100). */
  public double targetAreaPercent;

  /** 3D distance from camera to primary target in meters. */
  public double targetDistanceMeters;

  public RetroTarget[] retroTargets = new RetroTarget[0];
  public FiducialTarget[] fiducialTargets = new FiducialTarget[0];
  public DetectorTarget[] detectorTargets = new DetectorTarget[0];
  public ClassifierTarget[] classifierTargets = new ClassifierTarget[0];

  /** Classifier results below the confidence threshold. For visualization and debugging */
  public ClassifierTarget[] classifierTargetsRejected = new ClassifierTarget[0];

  public BarcodeTarget[] barcodeTargets = new BarcodeTarget[0];
  public CounterTarget[] counterTargets = new CounterTarget[0];
  public OCRTarget[] ocrTargets = new OCRTarget[0];

  public IMUData imu = new IMUData();
  public HardwareData hardware = new HardwareData();
  public RewindData rewind = new RewindData();
  public RobotOrientationData robotOrientation = new RobotOrientationData();
  public CameraIntrinsics intrinsics = new CameraIntrinsics();

  /** Image source index. */
  public int imageSource;

  /** Hardware type identifier. */
  public int hardwareType;

  /** UI refresh counter. */
  public int uiRefresh;

  /** True if the camera is ignoring NetworkTables input. */
  public boolean ignoreNT;

  /** True if the camera is connected to a NetworkTables server. */
  public boolean ntConnected;

  /** True if the camera is running a robot-published pipeline configuration override. */
  public boolean pipelineConfigurationOverrideActive;

  /** Pipeline configuration override state reported by the camera. */
  public PipelineConfigurationOverrideState pipelineConfigurationOverrideState =
      PipelineConfigurationOverrideState.OFF;

  /** True if the camera is localizing against the robot-published shared field map. */
  public boolean sharedMapActive;

  /** Shared field map state reported by the camera. */
  public SharedMapState sharedMapState = SharedMapState.OFF;

  /**
   * Time spent decoding this envelope in robot code, in milliseconds. It happens in this library,
   * not in the camera, so it is not part of the camera's latency. Useful for logging.
   */
  public double parseLatencyMillis;

  /**
   * Local NetworkTables time when this frame arrived, in seconds. 0 if this object did not come
   * from the network
   */
  public double receiveTimestampSeconds;

  /** Non-empty if the envelope failed to decode. */
  public String error = "";

  boolean fromLiveSubscriber;

  /**
   * Returns seconds since this frame arrived.
   *
   * @return Seconds since this frame arrived. If this value keeps growing, the camera has stopped
   *     sending frames. Possible causes: disconnected, stopped, or renamed.
   */
  public double getAgeSeconds() {
    return NetworkTablesJNI.now() / Limelight.NT_TICKS_PER_SECOND - receiveTimestampSeconds;
  }

  /**
   * Returns total number of retro, fiducial, detector, classifier, and barcode targets.
   *
   * @return Total number of retro, fiducial, detector, classifier, and barcode targets. Counter and
   *     OCR results are not included
   */
  public int getTargetCount() {
    return retroTargets.length
        + fiducialTargets.length
        + detectorTargets.length
        + classifierTargets.length
        + barcodeTargets.length;
  }

  /**
   * Returns megaTag1 robot pose in field space (field-center origin).
   *
   * @return MegaTag1 robot pose in field space (field-center origin)
   */
  public Pose3d getRobotPose_MT1() {
    return Limelight.toPose3D(robotPoseMT1);
  }

  /**
   * Returns megaTag2 robot pose in field space.
   *
   * @return MegaTag2 robot pose in field space. The origin is the field center
   */
  public Pose3d getRobotPose_MT2() {
    return Limelight.toPose3D(robotPoseMT2);
  }

  /**
   * Returns the full 3D robot pose for the given estimate type.
   *
   * @return The full 3D robot pose for the given estimate type. The type selects the origin and the
   *     algorithm
   */
  public Pose3d getRobotPose(PoseEstimateType type) {
    return Limelight.toPose3D(Limelight.poseArray(this, type));
  }

  /**
   * Returns camera pose in robot space (meters, degrees).
   *
   * @return Camera pose in robot space (meters, degrees)
   */
  public Pose3d getCameraPose_RobotSpace() {
    return Limelight.toPose3D(cameraPoseRobotSpace);
  }
}
