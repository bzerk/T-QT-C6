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

struct DirectionGlyphDefinition {
  char symbol;
  const char *name;
  const uint8_t *tokens;
  uint8_t tokenCount;
};

struct DirectionTrieNode {
  int16_t child[8];
  int16_t terminalGlyph;
  uint8_t depth;
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
constexpr uint8_t kDirB[] = {kTokD, kTokUR, kTokDR, kTokDL};
constexpr uint8_t kDirC[] = {kTokUR, kTokUL, kTokDL, kTokDR};
constexpr uint8_t kDirD[] = {kTokD, kTokUR, kTokDR, kTokDL, kTokU};
constexpr uint8_t kDirE[] = {kTokR, kTokDL, kTokR};
constexpr uint8_t kDirF[] = {kTokR, kTokD};
constexpr uint8_t kDirG[] = {kTokUR, kTokUL, kTokDL, kTokDR, kTokR};
constexpr uint8_t kDirH[] = {kTokD, kTokU, kTokD};
constexpr uint8_t kDirI[] = {kTokD};
constexpr uint8_t kDirJ[] = {kTokD, kTokDL};
constexpr uint8_t kDirK[] = {kTokD, kTokUR, kTokDL, kTokUR};
constexpr uint8_t kDirL[] = {kTokD, kTokR};
constexpr uint8_t kDirM[] = {kTokD, kTokU, kTokD, kTokU, kTokD};
constexpr uint8_t kDirN[] = {kTokU, kTokDR, kTokU};
constexpr uint8_t kDirO[] = {kTokDR, kTokDL, kTokUL, kTokUR};
constexpr uint8_t kDirP[] = {kTokD, kTokUR, kTokDR, kTokD};
constexpr uint8_t kDirQ[] = {kTokDR, kTokDL, kTokUL, kTokUR, kTokD};
constexpr uint8_t kDirR[] = {kTokD, kTokUR, kTokDR};
constexpr uint8_t kDirS[] = {kTokR, kTokDL, kTokL, kTokDR, kTokR};
constexpr uint8_t kDirT[] = {kTokD, kTokR};
constexpr uint8_t kDirU[] = {kTokD, kTokDR, kTokUR, kTokU};
constexpr uint8_t kDirV[] = {kTokDR, kTokUR};
constexpr uint8_t kDirW[] = {kTokDR, kTokUR, kTokDR, kTokUR};
constexpr uint8_t kDirX[] = {kTokDR, kTokUL, kTokDR};
constexpr uint8_t kDirY[] = {kTokD, kTokDR, kTokUR, kTokD};
constexpr uint8_t kDirZ[] = {kTokR, kTokDL, kTokR};

constexpr uint8_t kDir0[] = {kTokDR, kTokDL, kTokUL, kTokUR};
constexpr uint8_t kDir1[] = {kTokD};
constexpr uint8_t kDir2[] = {kTokR, kTokDL, kTokR};
constexpr uint8_t kDir3[] = {kTokR, kTokDL, kTokR, kTokDL, kTokR};
constexpr uint8_t kDir4[] = {kTokD, kTokR, kTokU};
constexpr uint8_t kDir5[] = {kTokR, kTokD, kTokL, kTokR};
constexpr uint8_t kDir6[] = {kTokDL, kTokD, kTokR, kTokU, kTokL};
constexpr uint8_t kDir7[] = {kTokR, kTokDL};
constexpr uint8_t kDir8[] = {kTokDR, kTokUR, kTokDL, kTokUR, kTokDR};
constexpr uint8_t kDir9[] = {kTokDR, kTokDL, kTokUL, kTokUR, kTokD};

constexpr uint8_t kDirBackspace[] = {kTokL};
constexpr uint8_t kDirSpace[] = {kTokR};
constexpr uint8_t kDirReturn[] = {kTokDL};

constexpr DirectionGlyphDefinition kDirectionGlyphs[] = {
    {'a', "A", kDirA, sizeof(kDirA) / sizeof(kDirA[0])},
    {'b', "B", kDirB, sizeof(kDirB) / sizeof(kDirB[0])},
    {'c', "C", kDirC, sizeof(kDirC) / sizeof(kDirC[0])},
    {'d', "D", kDirD, sizeof(kDirD) / sizeof(kDirD[0])},
    {'e', "E", kDirE, sizeof(kDirE) / sizeof(kDirE[0])},
    {'f', "F", kDirF, sizeof(kDirF) / sizeof(kDirF[0])},
    {'g', "G", kDirG, sizeof(kDirG) / sizeof(kDirG[0])},
    {'h', "H", kDirH, sizeof(kDirH) / sizeof(kDirH[0])},
    {'i', "I", kDirI, sizeof(kDirI) / sizeof(kDirI[0])},
    {'j', "J", kDirJ, sizeof(kDirJ) / sizeof(kDirJ[0])},
    {'k', "K", kDirK, sizeof(kDirK) / sizeof(kDirK[0])},
    {'l', "L", kDirL, sizeof(kDirL) / sizeof(kDirL[0])},
    {'m', "M", kDirM, sizeof(kDirM) / sizeof(kDirM[0])},
    {'n', "N", kDirN, sizeof(kDirN) / sizeof(kDirN[0])},
    {'o', "O", kDirO, sizeof(kDirO) / sizeof(kDirO[0])},
    {'p', "P", kDirP, sizeof(kDirP) / sizeof(kDirP[0])},
    {'q', "Q", kDirQ, sizeof(kDirQ) / sizeof(kDirQ[0])},
    {'r', "R", kDirR, sizeof(kDirR) / sizeof(kDirR[0])},
    {'s', "S", kDirS, sizeof(kDirS) / sizeof(kDirS[0])},
    {'t', "T", kDirT, sizeof(kDirT) / sizeof(kDirT[0])},
    {'u', "U", kDirU, sizeof(kDirU) / sizeof(kDirU[0])},
    {'v', "V", kDirV, sizeof(kDirV) / sizeof(kDirV[0])},
    {'w', "W", kDirW, sizeof(kDirW) / sizeof(kDirW[0])},
    {'x', "X", kDirX, sizeof(kDirX) / sizeof(kDirX[0])},
    {'y', "Y", kDirY, sizeof(kDirY) / sizeof(kDirY[0])},
    {'z', "Z", kDirZ, sizeof(kDirZ) / sizeof(kDirZ[0])},
    {'0', "0", kDir0, sizeof(kDir0) / sizeof(kDir0[0])},
    {'1', "1", kDir1, sizeof(kDir1) / sizeof(kDir1[0])},
    {'2', "2", kDir2, sizeof(kDir2) / sizeof(kDir2[0])},
    {'3', "3", kDir3, sizeof(kDir3) / sizeof(kDir3[0])},
    {'4', "4", kDir4, sizeof(kDir4) / sizeof(kDir4[0])},
    {'5', "5", kDir5, sizeof(kDir5) / sizeof(kDir5[0])},
    {'6', "6", kDir6, sizeof(kDir6) / sizeof(kDir6[0])},
    {'7', "7", kDir7, sizeof(kDir7) / sizeof(kDir7[0])},
    {'8', "8", kDir8, sizeof(kDir8) / sizeof(kDir8[0])},
    {'9', "9", kDir9, sizeof(kDir9) / sizeof(kDir9[0])},
    {' ', "SPACE", kDirSpace, sizeof(kDirSpace) / sizeof(kDirSpace[0])},
    {'\n', "RET", kDirReturn, sizeof(kDirReturn) / sizeof(kDirReturn[0])},
    {'\b', "BKSP", kDirBackspace, sizeof(kDirBackspace) / sizeof(kDirBackspace[0])},
};

constexpr uint8_t kTemplateCount = sizeof(kTemplates) / sizeof(kTemplates[0]);
constexpr uint8_t kDirectionGlyphCount = sizeof(kDirectionGlyphs) / sizeof(kDirectionGlyphs[0]);
constexpr bool kEnableLegacyPointFallback = false;
constexpr float kDistanceRejectCutoff = 0.70f;
constexpr float kDirectionRejectCutoff = 1.28f;
constexpr float kDirectionBeamMargin = 0.42f;
constexpr float kDirectionHardPrune = 2.40f;
constexpr uint16_t kDirectionTrieMaxNodes = 512;
constexpr int16_t kNoTrieNode = -1;

DirectionTrieNode g_directionTrie[kDirectionTrieMaxNodes];
uint16_t g_directionTrieNodeCount = 0;
bool g_directionTrieReady = false;
}  // namespace

bool GraffitiRecognizer::recognize(const Point *rawPoints, uint16_t rawCount, Result &out) {
  if (rawPoints == nullptr || rawCount < 4) {
    return false;
  }

  Result directionOut = {};
  const bool directionOk = recognizeByDirectionTrie(rawPoints, rawCount, directionOut);
  if (!kEnableLegacyPointFallback) {
    if (directionOk) {
      out = directionOut;
      return true;
    }
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

void GraffitiRecognizer::buildDirectionTrie() {
  if (g_directionTrieReady) {
    return;
  }

  g_directionTrieNodeCount = 1;
  for (uint8_t t = 0; t < 8; ++t) {
    g_directionTrie[0].child[t] = kNoTrieNode;
  }
  g_directionTrie[0].terminalGlyph = -1;
  g_directionTrie[0].depth = 0;

  for (uint8_t glyphIndex = 0; glyphIndex < kDirectionGlyphCount; ++glyphIndex) {
    const DirectionGlyphDefinition &glyph = kDirectionGlyphs[glyphIndex];
    if (glyph.tokenCount == 0 || glyph.tokenCount > kMaxDirectionTokens) {
      continue;
    }

    uint16_t node = 0;
    bool dropped = false;
    for (uint8_t i = 0; i < glyph.tokenCount; ++i) {
      const uint8_t tok = glyph.tokens[i] & 0x07;
      int16_t child = g_directionTrie[node].child[tok];
      if (child == kNoTrieNode) {
        if (g_directionTrieNodeCount >= kDirectionTrieMaxNodes) {
          dropped = true;
          break;
        }
        child = static_cast<int16_t>(g_directionTrieNodeCount++);
        for (uint8_t t = 0; t < 8; ++t) {
          g_directionTrie[child].child[t] = kNoTrieNode;
        }
        g_directionTrie[child].terminalGlyph = -1;
        g_directionTrie[child].depth = static_cast<uint8_t>(g_directionTrie[node].depth + 1);
        g_directionTrie[node].child[tok] = child;
      }
      node = static_cast<uint16_t>(child);
    }

    if (dropped) {
      continue;
    }
    if (g_directionTrie[node].terminalGlyph < 0) {
      g_directionTrie[node].terminalGlyph = glyphIndex;
    }
  }

  g_directionTrieReady = true;
}

float GraffitiRecognizer::directionSubstitutionCost(uint8_t inputToken, uint8_t templateToken) {
  int step = abs(static_cast<int>(inputToken & 0x07) - static_cast<int>(templateToken & 0x07));
  step = std::min(step, 8 - step);
  return static_cast<float>(step) * 0.52f;
}

void GraffitiRecognizer::searchDirectionTrieNode(uint16_t nodeIndex, const uint8_t *inputTokens,
                                                 uint8_t inputCount, const float *prevRow,
                                                 float &bestCost, float &secondCost, int &bestEntry) {
  if (nodeIndex >= g_directionTrieNodeCount || prevRow == nullptr || inputTokens == nullptr) {
    return;
  }

  const DirectionTrieNode &node = g_directionTrie[nodeIndex];
  for (uint8_t token = 0; token < 8; ++token) {
    const int16_t childIndex = node.child[token];
    if (childIndex == kNoTrieNode) {
      continue;
    }

    float nextRow[kMaxDirectionTokens + 1];
    nextRow[0] = prevRow[0] + 1.0f;
    float rowMin = nextRow[0];

    for (uint8_t j = 1; j <= inputCount; ++j) {
      const float subCost = prevRow[j - 1] + directionSubstitutionCost(inputTokens[j - 1], token);
      const float insCost = nextRow[j - 1] + 1.0f;
      const float delCost = prevRow[j] + 1.0f;
      nextRow[j] = std::min(subCost, std::min(insCost, delCost));
      rowMin = std::min(rowMin, nextRow[j]);
    }

    const DirectionTrieNode &child = g_directionTrie[childIndex];
    if (child.terminalGlyph >= 0) {
      const float candidateCost =
          nextRow[inputCount] / static_cast<float>(std::max<uint8_t>(inputCount, child.depth));
      if (candidateCost < bestCost) {
        secondCost = bestCost;
        bestCost = candidateCost;
        bestEntry = child.terminalGlyph;
      } else if (candidateCost < secondCost) {
        secondCost = candidateCost;
      }
    }

    const float activePrune = std::min(kDirectionHardPrune, bestCost + kDirectionBeamMargin);
    if (rowMin <= activePrune && child.depth < kMaxDirectionTokens) {
      searchDirectionTrieNode(static_cast<uint16_t>(childIndex), inputTokens, inputCount, nextRow,
                              bestCost, secondCost, bestEntry);
    }
  }
}

bool GraffitiRecognizer::recognizeByDirectionTrie(const Point *rawPoints, uint16_t rawCount, Result &out) {
  buildDirectionTrie();

  uint8_t tokens[kMaxDirectionTokens] = {0};
  const uint8_t tokenCount = extractDirectionTokens(rawPoints, rawCount, tokens, kMaxDirectionTokens);
  if (tokenCount == 0) {
    return false;
  }

  float baseRow[kMaxDirectionTokens + 1];
  for (uint8_t i = 0; i <= tokenCount; ++i) {
    baseRow[i] = static_cast<float>(i);
  }

  float bestCost = FLT_MAX;
  float secondCost = FLT_MAX;
  int bestEntry = -1;
  searchDirectionTrieNode(0, tokens, tokenCount, baseRow, bestCost, secondCost, bestEntry);
  if (bestEntry < 0) {
    return false;
  }

  float score = 1.0f - std::min(1.0f, bestCost / kDirectionRejectCutoff);
  const float margin = secondCost - bestCost;
  if (margin < 0.18f) {
    score = std::max(0.0f, score - 0.12f);
  }

  out.symbol = kDirectionGlyphs[bestEntry].symbol;
  out.name = kDirectionGlyphs[bestEntry].name;
  out.score = score;
  out.distance = bestCost;
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
