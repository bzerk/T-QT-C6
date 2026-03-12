#ifndef RINGO_GRAFFITI_ENGINE_H
#define RINGO_GRAFFITI_ENGINE_H

#include <Arduino.h>

#include "GraffitiRecognizer.h"

class GraffitiEngine {
 public:
  enum class Backend : uint8_t {
    None = 0,
    Trie = 1,
    LegacyPoint = 2,
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

  static constexpr float kAcceptConfidence = GraffitiRecognizer::kAcceptScore;

  bool classify(const Point *points, uint16_t count, Result &out);
  void setLegacyPointFallbackEnabled(bool enabled);
  bool legacyPointFallbackEnabled() const;
  uint32_t trieAcceptCount() const;
  uint32_t pointAcceptCount() const;
  uint8_t lastTokenCount() const;
  const char *lastTokenSequence() const;
  void resetStats();

 private:
  GraffitiRecognizer recognizer_;
};

#endif  // RINGO_GRAFFITI_ENGINE_H
