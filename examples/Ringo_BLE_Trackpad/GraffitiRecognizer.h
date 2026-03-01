#ifndef RINGO_GRAFFITI_RECOGNIZER_H
#define RINGO_GRAFFITI_RECOGNIZER_H

#include <Arduino.h>

class GraffitiRecognizer {
 public:
  struct Point {
    float x;
    float y;
  };

  struct Result {
    char symbol;
    const char *name;
    float score;
    float distance;
  };

  static constexpr uint8_t kSamplePoints = 32;
  static constexpr float kAcceptScore = 0.58f;

  bool recognize(const Point *rawPoints, uint16_t rawCount, Result &out);

 private:
  static constexpr uint8_t kMaxDirectionTokens = 24;
  static constexpr float kVectorAcceptScore = 0.62f;
  static constexpr uint16_t kMaxDirectionTrieNodes = 512;

  static bool recognizeByDirectionTrie(const Point *rawPoints, uint16_t rawCount, Result &out);
  static void buildDirectionTrie();
  static void searchDirectionTrieNode(uint16_t nodeIndex, const uint8_t *inputTokens, uint8_t inputCount,
                                      const float *prevRow, float &bestCost, float &secondCost, int &bestEntry);
  static float directionSubstitutionCost(uint8_t inputToken, uint8_t templateToken);
  static uint8_t quantizeDirection(float dx, float dy);
  static uint8_t extractDirectionTokens(const Point *input, uint16_t count, uint8_t *tokens, uint8_t maxTokens);

  static bool prepareStroke(const Point *input, uint16_t count, Point *out, uint16_t outCount);
  static bool resample(const Point *input, uint16_t inCount, Point *out, uint16_t outCount);
  static void normalize(Point *points, uint16_t count);
  static float pathDistance(const Point *a, const Point *b, uint16_t count);
  static float distance(const Point &a, const Point &b);
};

#endif  // RINGO_GRAFFITI_RECOGNIZER_H
