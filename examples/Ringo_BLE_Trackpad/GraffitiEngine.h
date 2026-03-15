#ifndef RINGO_GRAFFITI_ENGINE_H
#define RINGO_GRAFFITI_ENGINE_H

#include <Arduino.h>

#include "GraffitiPrototypeModel.h"

class GraffitiEngine {
 public:
  enum class Backend : uint8_t {
    None = 0,
    Prototype = 1,
  };

  enum class Condition : uint8_t {
    Letters = 0,
    Punct = 1,
    Numeric = 2,
  };

  struct Point {
    float x;
    float y;
  };

  struct Result {
    char symbol;
    const char *label;
    float confidence;
    float rawScore;
    float rawDistance;
    Backend backend;
    bool accepted;
    uint8_t tokenCount;
  };

  static constexpr float kDefaultAcceptConfidence = 0.016f;

  bool classify(const Point *points, uint16_t count, Result &out);
  void setCondition(Condition condition);
  Condition condition() const;
  const char *conditionName() const;
  void setAcceptConfidence(float threshold);
  float acceptConfidence() const;
  void setLegacyPointFallbackEnabled(bool enabled);
  bool legacyPointFallbackEnabled() const;
  uint32_t trieAcceptCount() const;
  uint32_t pointAcceptCount() const;
  uint8_t lastTokenCount() const;
  const char *lastTokenSequence() const;
  void resetStats();

 private:
  static constexpr uint16_t kResampledPointCount = GraffitiPrototypeModelData::kStrokeSampleCount;
  static constexpr uint16_t kStrokeFeatureDim = GraffitiPrototypeModelData::kStrokeFeatureDim;
  static constexpr uint16_t kInputFeatureDim = GraffitiPrototypeModelData::kInputFeatureDim;

  static bool resampleStroke(const Point *input, uint16_t count, Point *out, uint16_t outCount);
  static void normalizeResampledPoints(Point *points, uint16_t count);
  bool buildFeatureVector(const Point *points, uint16_t count, float *out) const;

  Condition condition_ = Condition::Letters;
  float acceptConfidence_ = kDefaultAcceptConfidence;
  uint32_t acceptCount_ = 0;
  uint32_t rejectCount_ = 0;
  char lastSequence_[16] = "MODEL";
};

#endif  // RINGO_GRAFFITI_ENGINE_H
