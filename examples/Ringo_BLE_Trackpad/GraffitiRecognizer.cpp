#include "GraffitiRecognizer.h"

#include <algorithm>
#include <float.h>
#include <math.h>

namespace {
struct TemplateDefinition {
  char symbol;
  const char *name;
  const GraffitiRecognizer::Point *anchors;
  uint8_t anchorCount;
};

// Canonical Graffiti-1 "A" is a caret-like single stroke (^), not a printed "a".
constexpr GraffitiRecognizer::Point kGlyphA[] = {
    {0.20f, 0.86f}, {0.50f, 0.10f}, {0.82f, 0.86f},
};

constexpr GraffitiRecognizer::Point kGlyphI[] = {
    {0.50f, 0.12f}, {0.50f, 0.88f},
};

constexpr GraffitiRecognizer::Point kGlyphL[] = {
    {0.34f, 0.12f}, {0.34f, 0.88f}, {0.82f, 0.88f},
};

constexpr GraffitiRecognizer::Point kGlyphN[] = {
    {0.18f, 0.88f}, {0.18f, 0.15f}, {0.80f, 0.88f}, {0.80f, 0.15f},
};

constexpr GraffitiRecognizer::Point kGlyphO[] = {
    {0.52f, 0.10f}, {0.76f, 0.16f}, {0.90f, 0.40f}, {0.86f, 0.70f}, {0.62f, 0.90f},
    {0.30f, 0.86f}, {0.11f, 0.62f}, {0.14f, 0.30f}, {0.38f, 0.12f}, {0.52f, 0.10f},
};

constexpr GraffitiRecognizer::Point kGlyphU[] = {
    {0.20f, 0.16f}, {0.20f, 0.74f}, {0.50f, 0.90f}, {0.80f, 0.74f}, {0.80f, 0.16f},
};

constexpr GraffitiRecognizer::Point kGlyphV[] = {
    {0.20f, 0.15f}, {0.50f, 0.90f}, {0.80f, 0.15f},
};

constexpr GraffitiRecognizer::Point kGlyphZ[] = {
    {0.14f, 0.18f}, {0.88f, 0.18f}, {0.22f, 0.86f}, {0.88f, 0.86f},
};

constexpr GraffitiRecognizer::Point kGlyphBackspace[] = {
    {0.88f, 0.50f}, {0.12f, 0.50f},
};

constexpr GraffitiRecognizer::Point kGlyphSpace[] = {
    {0.12f, 0.50f}, {0.88f, 0.50f},
};

constexpr TemplateDefinition kTemplates[] = {
    {'a', "A", kGlyphA, sizeof(kGlyphA) / sizeof(kGlyphA[0])},
    {'i', "I", kGlyphI, sizeof(kGlyphI) / sizeof(kGlyphI[0])},
    {'l', "L", kGlyphL, sizeof(kGlyphL) / sizeof(kGlyphL[0])},
    {'n', "N", kGlyphN, sizeof(kGlyphN) / sizeof(kGlyphN[0])},
    {'o', "O", kGlyphO, sizeof(kGlyphO) / sizeof(kGlyphO[0])},
    {'u', "U", kGlyphU, sizeof(kGlyphU) / sizeof(kGlyphU[0])},
    {'v', "V", kGlyphV, sizeof(kGlyphV) / sizeof(kGlyphV[0])},
    {'z', "Z", kGlyphZ, sizeof(kGlyphZ) / sizeof(kGlyphZ[0])},
    {'\b', "BKSP", kGlyphBackspace, sizeof(kGlyphBackspace) / sizeof(kGlyphBackspace[0])},
    {' ', "SPACE", kGlyphSpace, sizeof(kGlyphSpace) / sizeof(kGlyphSpace[0])},
};

constexpr uint8_t kTemplateCount = sizeof(kTemplates) / sizeof(kTemplates[0]);
constexpr float kDistanceRejectCutoff = 0.70f;
}  // namespace

bool GraffitiRecognizer::recognize(const Point *rawPoints, uint16_t rawCount, Result &out) {
  if (rawPoints == nullptr || rawCount < 4) {
    return false;
  }

  static bool templatesReady = false;
  static Point templatePoints[kTemplateCount][kSamplePoints];

  if (!templatesReady) {
    for (uint8_t i = 0; i < kTemplateCount; ++i) {
      if (!prepareStroke(kTemplates[i].anchors, kTemplates[i].anchorCount, templatePoints[i], kSamplePoints)) {
        return false;
      }
    }
    templatesReady = true;
  }

  Point normalized[kSamplePoints];
  if (!prepareStroke(rawPoints, rawCount, normalized, kSamplePoints)) {
    return false;
  }

  float bestDistance = FLT_MAX;
  float secondBest = FLT_MAX;
  int bestIndex = -1;

  for (uint8_t i = 0; i < kTemplateCount; ++i) {
    const float d = pathDistance(normalized, templatePoints[i], kSamplePoints);
    if (d < bestDistance) {
      secondBest = bestDistance;
      bestDistance = d;
      bestIndex = i;
    } else if (d < secondBest) {
      secondBest = d;
    }
  }

  if (bestIndex < 0) {
    return false;
  }

  float score = 1.0f - std::min(1.0f, (bestDistance / kDistanceRejectCutoff));
  const float margin = secondBest - bestDistance;
  if (margin < 0.04f) {
    score = std::max(0.0f, score - 0.12f);
  }

  out.symbol = kTemplates[bestIndex].symbol;
  out.name = kTemplates[bestIndex].name;
  out.score = score;
  out.distance = bestDistance;
  return true;
}

bool GraffitiRecognizer::prepareStroke(const Point *input, uint16_t count, Point *out, uint16_t outCount) {
  if (!resample(input, count, out, outCount)) {
    return false;
  }

  normalize(out, outCount);
  return true;
}

bool GraffitiRecognizer::resample(const Point *input, uint16_t inCount, Point *out, uint16_t outCount) {
  if (input == nullptr || out == nullptr || inCount < 2 || outCount < 2) {
    return false;
  }

  float totalLength = 0.0f;
  for (uint16_t i = 1; i < inCount; ++i) {
    totalLength += distance(input[i - 1], input[i]);
  }

  if (totalLength < 0.001f) {
    return false;
  }

  const float step = totalLength / static_cast<float>(outCount - 1);
  out[0] = input[0];

  uint16_t outIndex = 1;
  float carried = 0.0f;
  Point start = input[0];

  for (uint16_t i = 1; i < inCount && outIndex < outCount; ++i) {
    Point end = input[i];
    float seg = distance(start, end);
    if (seg < 0.0001f) {
      start = end;
      continue;
    }

    while ((carried + seg) >= step && outIndex < outCount) {
      const float t = (step - carried) / seg;
      Point q;
      q.x = start.x + t * (end.x - start.x);
      q.y = start.y + t * (end.y - start.y);
      out[outIndex++] = q;

      start = q;
      seg = distance(start, end);
      carried = 0.0f;
      if (seg < 0.0001f) {
        break;
      }
    }

    carried += seg;
    start = end;
  }

  while (outIndex < outCount) {
    out[outIndex] = input[inCount - 1];
    ++outIndex;
  }

  return true;
}

void GraffitiRecognizer::normalize(Point *points, uint16_t count) {
  if (points == nullptr || count == 0) {
    return;
  }

  float cx = 0.0f;
  float cy = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    cx += points[i].x;
    cy += points[i].y;
  }
  cx /= static_cast<float>(count);
  cy /= static_cast<float>(count);

  const float angle = -atan2f(points[0].y - cy, points[0].x - cx);
  const float c = cosf(angle);
  const float s = sinf(angle);

  for (uint16_t i = 0; i < count; ++i) {
    const float tx = points[i].x - cx;
    const float ty = points[i].y - cy;
    points[i].x = (tx * c) - (ty * s);
    points[i].y = (tx * s) + (ty * c);
  }

  float minX = FLT_MAX;
  float minY = FLT_MAX;
  float maxX = -FLT_MAX;
  float maxY = -FLT_MAX;

  for (uint16_t i = 0; i < count; ++i) {
    minX = std::min(minX, points[i].x);
    minY = std::min(minY, points[i].y);
    maxX = std::max(maxX, points[i].x);
    maxY = std::max(maxY, points[i].y);
  }

  const float width = maxX - minX;
  const float height = maxY - minY;
  const float scale = std::max(width, height);
  if (scale < 0.0001f) {
    return;
  }

  for (uint16_t i = 0; i < count; ++i) {
    points[i].x = (points[i].x - minX) / scale;
    points[i].y = (points[i].y - minY) / scale;
  }

  float ncx = 0.0f;
  float ncy = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    ncx += points[i].x;
    ncy += points[i].y;
  }
  ncx /= static_cast<float>(count);
  ncy /= static_cast<float>(count);

  for (uint16_t i = 0; i < count; ++i) {
    points[i].x -= ncx;
    points[i].y -= ncy;
  }
}

float GraffitiRecognizer::pathDistance(const Point *a, const Point *b, uint16_t count) {
  if (a == nullptr || b == nullptr || count == 0) {
    return FLT_MAX;
  }

  float d = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    d += distance(a[i], b[i]);
  }
  return d / static_cast<float>(count);
}

float GraffitiRecognizer::distance(const Point &a, const Point &b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return sqrtf((dx * dx) + (dy * dy));
}
