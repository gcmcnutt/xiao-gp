#include "autoc_test_helpers.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

class ReplayPathProvider : public PathProvider {
public:
  ReplayPathProvider(const std::vector<Path>& paths, int currentIndex)
    : paths_(paths), currentIndex_(currentIndex) {}

  const Path& getPath(int index) const override {
    if (paths_.empty()) {
      throw std::out_of_range("ReplayPathProvider: no paths loaded");
    }
    if (index < 0) {
      index = 0;
    }
    if (index >= static_cast<int>(paths_.size())) {
      index = static_cast<int>(paths_.size()) - 1;
    }
    return paths_[static_cast<std::size_t>(index)];
  }

  int getCurrentIndex() const override { return currentIndex_; }
  int getPathSize() const override { return static_cast<int>(paths_.size()); }

private:
  const std::vector<Path>& paths_;
  int currentIndex_;
};

double safeGetDouble(const std::vector<std::string>& tokens, std::size_t index) {
  if (index >= tokens.size()) {
    throw std::runtime_error("Replay dataset parse error: not enough columns");
  }
  return std::stod(tokens[index]);
}

} // namespace

ReplayDataset loadReplayDataset(const std::string& filePath) {
  ReplayDataset dataset;

  std::ifstream infile(filePath);
  if (!infile.is_open()) {
    throw std::runtime_error("Unable to open replay dataset: " + filePath);
  }

  std::string line;
  bool headerSkipped = false;

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    }

    if (!headerSkipped) {
      headerSkipped = true;
      continue;
    }

    for (char& ch : line) {
      if (ch == ':') {
        ch = ' ';
      }
    }

    std::stringstream ss(line);
    std::vector<std::string> tokens;
    std::string token;

    while (ss >> token) {
      tokens.push_back(token);
    }

    if (tokens.size() < 33) {
      throw std::runtime_error("Unexpected column count while parsing replay dataset");
    }

    ReplaySample sample;
    sample.pathIndex = static_cast<int>(std::stoi(tokens[3]));
    sample.simTimeMsec = safeGetDouble(tokens, 2);
    double totalDistance = safeGetDouble(tokens, 4);

    Eigen::Vector3d pathStart(
      safeGetDouble(tokens, 5),
      safeGetDouble(tokens, 6),
      safeGetDouble(tokens, 7));

    if (sample.pathIndex >= static_cast<int>(dataset.paths.size())) {
      dataset.paths.resize(static_cast<std::size_t>(sample.pathIndex) + 1);
    }

    Path& pathEntry = dataset.paths[static_cast<std::size_t>(sample.pathIndex)];
    pathEntry.start = pathStart;
    pathEntry.orientation = Eigen::Vector3d::UnitX();
    pathEntry.distanceFromStart = totalDistance;
    pathEntry.radiansFromStart = 0.0;
    pathEntry.simTimeMsec = sample.simTimeMsec;

    sample.position = Eigen::Vector3d(
      safeGetDouble(tokens, 8),
      safeGetDouble(tokens, 9),
      safeGetDouble(tokens, 10));

    sample.relVel = safeGetDouble(tokens, 14);
    sample.expectedRoll = safeGetDouble(tokens, 15);
    sample.expectedPitch = safeGetDouble(tokens, 16);
    sample.expectedThrottle = safeGetDouble(tokens, 17);

    sample.orientation = Eigen::Quaterniond(
      safeGetDouble(tokens, 21),
      safeGetDouble(tokens, 22),
      safeGetDouble(tokens, 23),
      safeGetDouble(tokens, 24));

    if (sample.orientation.norm() == 0.0) {
      sample.orientation = Eigen::Quaterniond::Identity();
    } else {
      sample.orientation.normalize();
    }

    sample.bodyVelocity = Eigen::Vector3d(
      safeGetDouble(tokens, 25),
      safeGetDouble(tokens, 26),
      safeGetDouble(tokens, 27));

    dataset.samples.push_back(sample);
  }

  return dataset;
}

ReplayStats evaluateReplayDataset(const ReplayDataset& dataset,
                                  double failureTolerance,
                                  std::vector<SampleDiff>& failures,
                                  std::size_t maxFailures) {
  ReplayStats stats;

  if (dataset.samples.empty() || dataset.paths.empty()) {
    return stats;
  }

  double rollSqError = 0.0;
  double pitchSqError = 0.0;
  double throttleSqError = 0.0;

  double rollMax = 0.0;
  double pitchMax = 0.0;
  double throttleMax = 0.0;

  std::size_t count = 0;

  for (std::size_t i = 0; i < dataset.samples.size(); ++i) {
    SampleDiff diff;
    if (!evaluateReplaySample(dataset, i, diff)) {
      continue;
    }

    double rollError = diff.actualRoll - diff.expectedRoll;
    double pitchError = diff.actualPitch - diff.expectedPitch;
    double throttleError = diff.actualThrottle - diff.expectedThrottle;

    rollSqError += rollError * rollError;
    pitchSqError += pitchError * pitchError;
    throttleSqError += throttleError * throttleError;

    rollMax = std::max(rollMax, std::abs(rollError));
    pitchMax = std::max(pitchMax, std::abs(pitchError));
    throttleMax = std::max(throttleMax, std::abs(throttleError));

    if (failureTolerance >= 0.0 &&
        (std::abs(rollError) > failureTolerance ||
         std::abs(pitchError) > failureTolerance ||
         std::abs(throttleError) > failureTolerance)) {
      if (failures.size() < maxFailures) {
        failures.push_back(diff);
      }
    }

    ++count;
  }

  if (count == 0) {
    return stats;
  }

  stats.sampleCount = count;
  stats.rollRmse = std::sqrt(rollSqError / static_cast<double>(count));
  stats.pitchRmse = std::sqrt(pitchSqError / static_cast<double>(count));
  stats.throttleRmse = std::sqrt(throttleSqError / static_cast<double>(count));
  stats.rollMaxError = rollMax;
  stats.pitchMaxError = pitchMax;
  stats.throttleMaxError = throttleMax;

  return stats;
}

bool evaluateReplaySample(const ReplayDataset& dataset,
                          std::size_t sampleIndex,
                          SampleDiff& outDiff) {
  if (sampleIndex >= dataset.samples.size()) {
    return false;
  }

  const ReplaySample& sample = dataset.samples[sampleIndex];
  if (sample.pathIndex < 0 ||
      sample.pathIndex >= static_cast<int>(dataset.paths.size())) {
    return false;
  }

  AircraftState aircraftState;
  aircraftState.setThisPathIndex(sample.pathIndex);
  aircraftState.setRelVel(sample.relVel);
  aircraftState.setOrientation(sample.orientation);
  aircraftState.setPosition(sample.position);
  aircraftState.setSimTimeMsec(static_cast<unsigned long>(sample.simTimeMsec));

  Eigen::Vector3d worldVelocity = sample.orientation * sample.bodyVelocity;
  aircraftState.setVelocity(worldVelocity);

  aircraftState.setRollCommand(0.0);
  aircraftState.setPitchCommand(0.0);
  aircraftState.setThrottleCommand(0.0);

  ReplayPathProvider pathProvider(dataset.paths, sample.pathIndex);
  generatedGPProgram(pathProvider, aircraftState, 0.0);

  outDiff.sampleIndex = sampleIndex;
  outDiff.expectedRoll = sample.expectedRoll;
  outDiff.actualRoll = aircraftState.getRollCommand();
  outDiff.expectedPitch = sample.expectedPitch;
  outDiff.actualPitch = aircraftState.getPitchCommand();
  outDiff.expectedThrottle = sample.expectedThrottle;
  outDiff.actualThrottle = aircraftState.getThrottleCommand();

  return true;
}
