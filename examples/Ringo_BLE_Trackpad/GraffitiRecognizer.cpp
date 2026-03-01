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

struct DirectionTemplateDefinition {
  char symbol;
  const char *name;
  const uint8_t *tokens;
  uint8_t tokenCount;
};

enum DirectionToken : uint8_t {
  kTokR = 0,
  kTokUR = 1,
  kTokU = 2,
  kTokUL = 3,
  kTokL = 4,
  kTokDL = 5,
  kTokD = 6,
  kTokDR = 7,
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

constexpr uint8_t kDirA[] = {kTokUR, kTokDR};
constexpr uint8_t kDirI[] = {kTokD};
constexpr uint8_t kDirL[] = {kTokD, kTokR};
constexpr uint8_t kDirN[] = {kTokU, kTokDR, kTokU};
constexpr uint8_t kDirOCardinal[] = {kTokR, kTokD, kTokL, kTokU};
constexpr uint8_t kDirODiagonal[] = {kTokDR, kTokDL, kTokUL, kTokUR};
constexpr uint8_t kDirU[] = {kTokD, kTokDR, kTokUR, kTokU};
constexpr uint8_t kDirV[] = {kTokDR, kTokUR};
constexpr uint8_t kDirZ[] = {kTokR, kTokDL, kTokR};
constexpr uint8_t kDirBackspace[] = {kTokL};
constexpr uint8_t kDirSpace[] = {kTokR};

constexpr DirectionTemplateDefinition kDirectionTemplates[] = {
    {'a', "A", kDirA, sizeof(kDirA) / sizeof(kDirA[0])},
    {'i', "I", kDirI, sizeof(kDirI) / sizeof(kDirI[0])},
    {'l', "L", kDirL, sizeof(kDirL) / sizeof(kDirL[0])},
    {'n', "N", kDirN, sizeof(kDirN) / sizeof(kDirN[0])},
    {'o', "O", kDirOCardinal, sizeof(kDirOCardinal) / sizeof(kDirOCardinal[0])},
    {'o', "O", kDirODiagonal, sizeof(kDirODiagonal) / sizeof(kDirODiagonal[0])},
    {'u', "U", kDirU, sizeof(kDirU) / sizeof(kDirU[0])},
    {'v', "V", kDirV, sizeof(kDirV) / sizeof(kDirV[0])},
    {'z', "Z", kDirZ, sizeof(kDirZ) / sizeof(kDirZ[0])},
    {'\b', "BKSP", kDirBackspace, sizeof(kDirBackspace) / sizeof(kDirBackspace[0])},
    {' ', "SPACE", kDirSpace, sizeof(kDirSpace) / sizeof(kDirSpace[0])},
};

constexpr uint8_t kTemplateCount = sizeof(kTemplates) / sizeof(kTemplates[0]);
constexpr uint8_t kDirectionTemplateCount = sizeof(kDirectionTemplates) / sizeof(kDirectionTemplates[0]);
constexpr float kDistanceRejectCutoff = 0.70f;
constexpr float kDirectionRejectCutoff = 1.18f;
}  // namespace

bool GraffitiRecognizer::recognize(const Point *rawPoints, uint16_t rawCount, Result &out) {
  if (rawPoints == nullptr || rawCount < 4) {
    return false;
  }

  Result directionOut = {};
  const bool directionOk = recognizeByDirectionSequence(rawPoints, rawCount, directionOut);

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
    if (directionOk) {
      out = directionOut;
      return true;
    }
    return false;
  }

  Result pointOut = {};
  pointOut.symbol = kTemplates[bestIndex].symbol;
  pointOut.name = kTemplates[bestIndex].name;
  pointOut.score = 1.0f - std::min(1.0f, (bestDistance / kDistanceRejectCutoff));
  pointOut.distance = bestDistance;
  const float margin = secondBest - bestDistance;
  if (margin < 0.04f) {
    pointOut.score = std::max(0.0f, pointOut.score - 0.12f);
  }

  if (directionOk && directionOut.score >= kVectorAcceptScore && directionOut.score >= pointOut.score) {
    out = directionOut;
  } else {
    out = pointOut;
  }
  return true;
}

bool GraffitiRecognizer::recognizeByDirectionSequence(const Point *rawPoints, uint16_t rawCount, Result &out) {
  uint8_t tokens[kMaxDirectionTokens] = {0};
  const uint8_t tokenCount = extractDirectionTokens(rawPoints, rawCount, tokens, kMaxDirectionTokens);
  if (tokenCount == 0) {
    return false;
  }

  float bestDistance = FLT_MAX;
  float secondBest = FLT_MAX;
  int bestIndex = -1;

  for (uint8_t i = 0; i < kDirectionTemplateCount; ++i) {
    const DirectionTemplateDefinition &templ = kDirectionTemplates[i];
    if (templ.tokenCount == 0 || templ.tokenCount > kMaxDirectionTokens) {
      continue;
    }

    const float d = directionSequenceDistance(tokens, tokenCount, templ.tokens, templ.tokenCount);
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

  float score = 1.0f - std::min(1.0f, bestDistance / kDirectionRejectCutoff);
  const float margin = secondBest - bestDistance;
  if (margin < 0.20f) {
    score = std::max(0.0f, score - 0.10f);
  }

  out.symbol = kDirectionTemplates[bestIndex].symbol;
  out.name = kDirectionTemplates[bestIndex].name;
  out.score = score;
  out.distance = bestDistance;
  return true;
}

uint8_t GraffitiRecognizer::quantizeDirection(float dx, float dy) {
  if (!isfinite(dx) || !isfinite(dy)) {
    return 0;
  }
  if ((dx * dx) + (dy * dy) < 1e-6f) {
    return 0;
  }

  // Device coordinates are +Y downward, so negate Y for directional bins.
  constexpr float kPi = 3.14159265358979323846f;
  const float angle = atan2f(-dy, dx);
  const float scaled = angle / (kPi / 4.0f);
  int bin = static_cast<int>(lroundf(scaled));
  bin %= 8;
  if (bin < 0) {
    bin += 8;
  }
  return static_cast<uint8_t>(bin);
}

uint8_t GraffitiRecognizer::extractDirectionTokens(const Point *input, uint16_t count, uint8_t *tokens,
                                                   uint8_t maxTokens) {
  if (input == nullptr || tokens == nullptr || count < 2 || maxTokens == 0) {
    return 0;
  }

  float minX = input[0].x;
  float minY = input[0].y;
  float maxX = input[0].x;
  float maxY = input[0].y;
  for (uint16_t i = 1; i < count; ++i) {
    minX = std::min(minX, input[i].x);
    minY = std::min(minY, input[i].y);
    maxX = std::max(maxX, input[i].x);
    maxY = std::max(maxY, input[i].y);
  }

  const float width = maxX - minX;
  const float height = maxY - minY;
  const float diag = sqrtf((width * width) + (height * height));
  const float minSegLen = std::max(4.0f, diag * 0.10f);
  const float minSegLenSq = minSegLen * minSegLen;

  uint8_t outCount = 0;
  Point pivot = input[0];
  for (uint16_t i = 1; i < count; ++i) {
    const float dx = input[i].x - pivot.x;
    const float dy = input[i].y - pivot.y;
    const float d2 = (dx * dx) + (dy * dy);
    if (d2 < minSegLenSq) {
      continue;
    }

    const uint8_t token = quantizeDirection(dx, dy);
    if (outCount == 0 || tokens[outCount - 1] != token) {
      if (outCount >= maxTokens) {
        break;
      }
      tokens[outCount++] = token;
    }
    pivot = input[i];
  }

  if (outCount == 0) {
    const float dx = input[count - 1].x - input[0].x;
    const float dy = input[count - 1].y - input[0].y;
    if ((dx * dx) + (dy * dy) >= 1.0f) {
      tokens[outCount++] = quantizeDirection(dx, dy);
    }
  }

  return outCount;
}

float GraffitiRecognizer::directionSequenceDistance(const uint8_t *a, uint8_t aCount, const uint8_t *b,
                                                    uint8_t bCount) {
  if (a == nullptr || b == nullptr || aCount == 0 || bCount == 0) {
    return FLT_MAX;
  }

  float dp[kMaxDirectionTokens + 1][kMaxDirectionTokens + 1];
  for (uint8_t i = 0; i <= aCount; ++i) {
    dp[i][0] = static_cast<float>(i);
  }
  for (uint8_t j = 0; j <= bCount; ++j) {
    dp[0][j] = static_cast<float>(j);
  }

  for (uint8_t i = 1; i <= aCount; ++i) {
    for (uint8_t j = 1; j <= bCount; ++j) {
      int step = abs(static_cast<int>(a[i - 1]) - static_cast<int>(b[j - 1]));
      step = std::min(step, 8 - step);
      const float substCost = static_cast<float>(step) * 0.55f;

      const float delCost = dp[i - 1][j] + 1.0f;
      const float insCost = dp[i][j - 1] + 1.0f;
      const float subCost = dp[i - 1][j - 1] + substCost;
      dp[i][j] = std::min(subCost, std::min(delCost, insCost));
    }
  }

  const float denom = static_cast<float>(std::max<uint8_t>(aCount, bCount));
  if (denom <= 0.0f) {
    return FLT_MAX;
  }
  return dp[aCount][bCount] / denom;
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
