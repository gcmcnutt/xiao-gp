#include <unity.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

#include "autoc_test_helpers.h"

namespace {

std::string resolveDatasetPath() {
  const char* envPath = std::getenv("AUTOC_DATA_PATH");
  if (envPath && *envPath) {
    return std::string(envPath);
  }
  return "../GP/autoc/data.dat";
}

void printStats(const ReplayStats& stats) {
  std::printf("Samples: %zu\n", stats.sampleCount);
  std::printf("RMSE (roll/pitch/throttle): %.6f %.6f %.6f\n",
              stats.rollRmse, stats.pitchRmse, stats.throttleRmse);
  std::printf("Max error (roll/pitch/throttle): %.6f %.6f %.6f\n",
              stats.rollMaxError, stats.pitchMaxError, stats.throttleMaxError);
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_gp_program_matches_reference_trace() {
  ReplayDataset dataset;
  try {
    dataset = loadReplayDataset(resolveDatasetPath());
  } catch (const std::exception& ex) {
    TEST_IGNORE_MESSAGE(ex.what());
    return;
  }

  TEST_ASSERT_FALSE_MESSAGE(dataset.samples.empty(), "Replay dataset contains no samples");
  TEST_ASSERT_FALSE_MESSAGE(dataset.paths.empty(), "Replay dataset contains no path data");

  std::vector<SampleDiff> unusedFailures;
  ReplayStats stats = evaluateReplayDataset(dataset, -1.0, unusedFailures, 0);

  printStats(stats);

  std::vector<SampleDiff> allSamples;
  allSamples.reserve(dataset.samples.size());
  for (std::size_t i = 0; i < dataset.samples.size(); ++i) {
    SampleDiff diff;
    if (evaluateReplaySample(dataset, i, diff)) {
      allSamples.push_back(diff);
    }
  }

  std::printf("All sample outputs (expected -> actual):\n");
  for (const auto& diff : allSamples) {
    double rollError = diff.actualRoll - diff.expectedRoll;
    double pitchError = diff.actualPitch - diff.expectedPitch;
    double throttleError = diff.actualThrottle - diff.expectedThrottle;
    std::printf("  sample %3zu | roll %+0.4f -> %+0.4f (err=%+0.4f) | pitch %+0.4f -> %+0.4f (err=%+0.4f) | throttle %+0.4f -> %+0.4f (err=%+0.4f)\n",
                diff.sampleIndex,
                diff.expectedRoll, diff.actualRoll, rollError,
                diff.expectedPitch, diff.actualPitch, pitchError,
                diff.expectedThrottle, diff.actualThrottle, throttleError);
  }

  const double relativeThreshold = 0.10; // 10%
  const double absoluteFloor = 0.02;     // fallback when expected is near zero
  std::vector<SampleDiff> toleranceFailures;

  for (const auto& diff : allSamples) {
    double rollError = std::abs(diff.actualRoll - diff.expectedRoll);
    double pitchError = std::abs(diff.actualPitch - diff.expectedPitch);
    double throttleError = std::abs(diff.actualThrottle - diff.expectedThrottle);

    double rollLimit = std::max(absoluteFloor, std::abs(diff.expectedRoll) * relativeThreshold);
    double pitchLimit = std::max(absoluteFloor, std::abs(diff.expectedPitch) * relativeThreshold);
    double throttleLimit = std::max(absoluteFloor, std::abs(diff.expectedThrottle) * relativeThreshold);

    if (rollError > rollLimit || pitchError > pitchLimit || throttleError > throttleLimit) {
      toleranceFailures.push_back(diff);
    }
  }

  TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(0, stats.sampleCount, "No samples evaluated");
  if (!toleranceFailures.empty()) {
    std::printf("Samples beyond tolerance (>|10%%| or >%0.2f absolute):\n", absoluteFloor);
    for (const auto& diff : toleranceFailures) {
      double rollError = diff.actualRoll - diff.expectedRoll;
      double pitchError = diff.actualPitch - diff.expectedPitch;
      double throttleError = diff.actualThrottle - diff.expectedThrottle;
      std::printf("  sample %3zu | roll err=%+0.4f | pitch err=%+0.4f | throttle err=%+0.4f\n",
                  diff.sampleIndex,
                  rollError, pitchError, throttleError);
    }
  }
  TEST_ASSERT_TRUE_MESSAGE(toleranceFailures.empty(), "GP output deviates from reference beyond relaxed tolerance");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_gp_program_matches_reference_trace);
  return UNITY_END();
}
