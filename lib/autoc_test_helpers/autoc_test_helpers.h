#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <ArduinoEigenDense.h>

#include <GP/autoc/aircraft_state.h>
#include <gp_program.h>

struct ReplaySample {
  int pathIndex = 0;
  double simTimeMsec = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d bodyVelocity = Eigen::Vector3d::Zero();
  double relVel = 0.0;
  double expectedRoll = 0.0;
  double expectedPitch = 0.0;
  double expectedThrottle = 0.0;
};

struct ReplayDataset {
  std::vector<Path> paths;
  std::vector<ReplaySample> samples;
};

struct SampleDiff {
  std::size_t sampleIndex = 0;
  double expectedRoll = 0.0;
  double actualRoll = 0.0;
  double expectedPitch = 0.0;
  double actualPitch = 0.0;
  double expectedThrottle = 0.0;
  double actualThrottle = 0.0;
};

struct ReplayStats {
  std::size_t sampleCount = 0;
  double rollRmse = 0.0;
  double pitchRmse = 0.0;
  double throttleRmse = 0.0;
  double rollMaxError = 0.0;
  double pitchMaxError = 0.0;
  double throttleMaxError = 0.0;
};

ReplayDataset loadReplayDataset(const std::string& filePath);

ReplayStats evaluateReplayDataset(const ReplayDataset& dataset,
                                  double failureTolerance,
                                  std::vector<SampleDiff>& failures,
                                  std::size_t maxFailures = 10);

bool evaluateReplaySample(const ReplayDataset& dataset,
                          std::size_t sampleIndex,
                          SampleDiff& outDiff);
