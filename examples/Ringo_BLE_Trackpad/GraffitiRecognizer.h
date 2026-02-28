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
  static bool prepareStroke(const Point *input, uint16_t count, Point *out, uint16_t outCount);
  static bool resample(const Point *input, uint16_t inCount, Point *out, uint16_t outCount);
  static void normalize(Point *points, uint16_t count);
  static float pathDistance(const Point *a, const Point *b, uint16_t count);
  static float distance(const Point &a, const Point &b);
};

#endif  // RINGO_GRAFFITI_RECOGNIZER_H
