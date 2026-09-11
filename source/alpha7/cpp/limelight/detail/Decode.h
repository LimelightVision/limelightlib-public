// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string_view>
#include <utility>
#include <vector>

#include "limelight/BarcodeTarget.h"
#include "limelight/CameraIntrinsics.h"
#include "limelight/ClassifierTarget.h"
#include "limelight/CounterTarget.h"
#include "limelight/DetectorTarget.h"
#include "limelight/FiducialTarget.h"
#include "limelight/HardwareData.h"
#include "limelight/IMUData.h"
#include "limelight/IMUMode.h"
#include "limelight/LimelightResults.h"
#include "limelight/LimelightTarget.h"
#include "limelight/OCRTarget.h"
#include "limelight/PipelineConfigurationOverrideState.h"
#include "limelight/RetroTarget.h"
#include "limelight/RewindData.h"
#include "limelight/RobotOrientationData.h"
#include "limelight/SharedMapState.h"
#include "limelight/detail/MsgPackReader.h"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/power.hpp"
#include "wpi/units/temperature.hpp"
#include "wpi/units/time.hpp"
namespace limelight {

namespace detail {

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

inline bool DecodeCommonTargetKey(LimelightTarget& target, std::string_view key,
                                  MsgPackReader& r) {
  if (key == "tx") {
    target.tx = wpi::units::degree_t{r.ReadDouble()};
  } else if (key == "ty") {
    target.ty = wpi::units::degree_t{r.ReadDouble()};
  } else if (key == "tx_nocross") {
    target.txNoCrosshair = wpi::units::degree_t{r.ReadDouble()};
  } else if (key == "ty_nocross") {
    target.tyNoCrosshair = wpi::units::degree_t{r.ReadDouble()};
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
        t.cameraPoseTargetSpace = r.ReadPoseArray();
      } else if (key == "t6t_cs") {
        t.targetPoseCameraSpace = r.ReadPoseArray();
      } else if (key == "t6t_rs") {
        t.targetPoseRobotSpace = r.ReadPoseArray();
      } else if (key == "t6r_ts") {
        t.robotPoseTargetSpace = r.ReadPoseArray();
      } else if (key == "t6r_fs") {
        t.robotPoseFieldSpace = r.ReadPoseArray();
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
        t.cameraPoseTargetSpace = r.ReadPoseArray();
      } else if (key == "t6t_cs") {
        t.targetPoseCameraSpace = r.ReadPoseArray();
      } else if (key == "t6t_rs") {
        t.targetPoseRobotSpace = r.ReadPoseArray();
      } else if (key == "t6r_ts") {
        t.robotPoseTargetSpace = r.ReadPoseArray();
      } else if (key == "t6r_fs") {
        t.robotPoseFieldSpace = r.ReadPoseArray();
      } else if (key == "t6r_fs_orb") {
        t.robotPoseFieldSpaceMT2 = r.ReadPoseArray();
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
      imu.yawOffset = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "data") {
      // The "yaw" key carries the same fused yaw as data[0]. The array is the
      // single source.
      std::vector<double> d = r.ReadDoubleArray();
      if (d.size() >= 10) {
        imu.robotYaw = wpi::units::degree_t{d[0]};
        imu.roll = wpi::units::degree_t{d[1]};
        imu.pitch = wpi::units::degree_t{d[2]};
        imu.yaw = wpi::units::degree_t{d[3]};
        imu.gyroX = wpi::units::degrees_per_second_t{d[4]};
        imu.gyroY = wpi::units::degrees_per_second_t{d[5]};
        imu.gyroZ = wpi::units::degrees_per_second_t{d[6]};
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
      hw.cpuTemp = wpi::units::celsius_t{r.ReadDouble()};
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
          hw.accelTemp = wpi::units::celsius_t{r.ReadDouble()};
        } else if (hkey == "power") {
          hw.accelPower = wpi::units::watt_t{r.ReadDouble()};
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
      rw.stored = wpi::units::second_t{r.ReadDouble()};
    } else if (key == "frameCount") {
      rw.frameCount = r.ReadLong();
    } else if (key == "bufferUsage") {
      rw.bufferUsage = r.ReadDouble();
    } else if (key == "flushing") {
      rw.flushing = r.ReadLong() != 0;
    } else if (key == "latpen") {
      rw.latencyPenalty = wpi::units::microsecond_t{
          static_cast<double>(static_cast<int>(r.ReadLong()))};
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
      intrinsics.hfov = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "vfov") {
      intrinsics.vfov = wpi::units::degree_t{r.ReadDouble()};
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
      bo.interpolatedYaw = wpi::units::degree_t{r.ReadDouble()};
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
      out.timestamp = wpi::units::millisecond_t{r.ReadDouble()};
    } else if (key == "ts_us") {
      out.timestampMicros = r.ReadLong();
    } else if (key == "ts_sys") {
      out.systemTimestampMicros = r.ReadLong();
    } else if (key == "ts_nt") {
      out.ntTimestampMicros = r.ReadLong();
    } else if (key == "fidx") {
      out.frameIndex = r.ReadLong();
    } else if (key == "tl") {
      out.targetingLatency = wpi::units::millisecond_t{r.ReadDouble()};
    } else if (key == "cl") {
      out.captureLatency = wpi::units::millisecond_t{r.ReadDouble()};
    } else if (key == "pID") {
      out.pipelineIndex = r.ReadDouble();
    } else if (key == "pTYPE") {
      out.pipelineType = r.ReadString();
    } else if (key == "focus_metric") {
      out.focusMetric = r.ReadDouble();
    } else if (key == "stdev_mt1") {
      out.stdDevsMT1 = r.ReadPoseArray();
    } else if (key == "stdev_mt2") {
      out.stdDevsMT2 = r.ReadPoseArray();
    } else if (key == "botpose") {
      out.robotPoseMT1 = r.ReadPoseArray();
    } else if (key == "botpose_wpired") {
      out.robotPoseMT1WpiRed = r.ReadPoseArray();
    } else if (key == "botpose_wpiblue") {
      out.robotPoseMT1WpiBlue = r.ReadPoseArray();
    } else if (key == "botpose_orb") {
      out.robotPoseMT2 = r.ReadPoseArray();
    } else if (key == "botpose_orb_wpired") {
      out.robotPoseMT2WpiRed = r.ReadPoseArray();
    } else if (key == "botpose_orb_wpiblue") {
      out.robotPoseMT2WpiBlue = r.ReadPoseArray();
    } else if (key == "botpose_tagcount") {
      out.reportedTagCount = static_cast<int>(r.ReadLong());
    } else if (key == "botpose_span") {
      out.tagSpan = wpi::units::meter_t{r.ReadDouble()};
    } else if (key == "botpose_avgdist") {
      out.avgTagDistance = wpi::units::meter_t{r.ReadDouble()};
    } else if (key == "botpose_avgarea") {
      out.avgTagAreaPercent = r.ReadDouble();
    } else if (key == "t6c_rs") {
      out.cameraPoseRobotSpace = r.ReadPoseArray();
    } else if (key == "PythonOut") {
      out.pythonOutput = r.ReadDoubleArray();
    } else if (key == "tx") {
      out.tx = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "ty") {
      out.ty = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "txnc") {
      out.txNoCrosshair = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "tync") {
      out.tyNoCrosshair = wpi::units::degree_t{r.ReadDouble()};
    } else if (key == "ta") {
      out.targetAreaPercent = r.ReadDouble();
    } else if (key == "tdist") {
      out.targetDistance = wpi::units::meter_t{r.ReadDouble()};
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

}  // namespace limelight
