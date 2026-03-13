#ifndef RINGO_GRAFFITI_PROTOTYPE_MODEL_H
#define RINGO_GRAFFITI_PROTOTYPE_MODEL_H

#include <Arduino.h>

namespace GraffitiPrototypeModelData {

constexpr uint8_t kConditionCount = 3;
constexpr uint16_t kStrokeSampleCount = 32;
constexpr uint16_t kStrokeFeatureDim = kStrokeSampleCount * 4;
constexpr uint16_t kInputFeatureDim = kStrokeFeatureDim + kConditionCount;

struct PrototypeEntry {
  char symbol;
  uint8_t conditionIndex;
  const char *label;
  const float *vector;
};

extern const float kConditionWeight;
extern const float kMeans[kStrokeFeatureDim];
extern const float kStds[kStrokeFeatureDim];
extern const uint16_t kPrototypeCount;
extern const PrototypeEntry kPrototypes[];

}  // namespace GraffitiPrototypeModelData

#endif  // RINGO_GRAFFITI_PROTOTYPE_MODEL_H
