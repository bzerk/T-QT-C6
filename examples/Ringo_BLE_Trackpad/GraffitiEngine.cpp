#include "GraffitiEngine.h"

namespace {
constexpr uint16_t kMaxEnginePoints = 192;
constexpr float kMinPointDistanceSq = 1.0f;

bool isSharedLettersControl(const GraffitiPrototypeModelData::PrototypeEntry &prototype,
                            GraffitiEngine::Condition activeCondition) {
  const bool isBksp = (strcmp(prototype.label, "BKSP") == 0);
  const bool isSpace = (strcmp(prototype.label, "SPACE") == 0);
  if (activeCondition == GraffitiEngine::Condition::Numeric) {
    return isBksp || isSpace;
  }
  if (activeCondition == GraffitiEngine::Condition::Punct) {
    return isBksp;
  }
  return false;
}

float squaredDistance(const float *lhs, const float *rhs, uint16_t count) {
  float total = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    const float delta = lhs[i] - rhs[i];
    total += delta * delta;
  }
  return total;
}
}  // namespace

void GraffitiEngine::setCondition(Condition condition) {
  condition_ = condition;
}

GraffitiEngine::Condition GraffitiEngine::condition() const {
  return condition_;
}

const char *GraffitiEngine::conditionName() const {
  switch (condition_) {
    case Condition::Letters:
      return "letters";
    case Condition::Punct:
      return "punct";
    case Condition::Numeric:
      return "numeric";
    default:
      return "letters";
  }
}

void GraffitiEngine::setAcceptConfidence(float threshold) {
  acceptConfidence_ = threshold;
}

float GraffitiEngine::acceptConfidence() const {
  return acceptConfidence_;
}

bool GraffitiEngine::resampleStroke(const Point *input, uint16_t count, Point *out, uint16_t outCount) {
  if (input == nullptr || out == nullptr || count == 0 || outCount == 0) {
    return false;
  }

  Point deduped[kMaxEnginePoints];
  uint16_t dedupCount = 0;
  for (uint16_t i = 0; i < count && dedupCount < kMaxEnginePoints; ++i) {
    if (dedupCount == 0) {
      deduped[dedupCount++] = input[i];
      continue;
    }
    const float dx = input[i].x - deduped[dedupCount - 1].x;
    const float dy = input[i].y - deduped[dedupCount - 1].y;
    if (((dx * dx) + (dy * dy)) <= kMinPointDistanceSq) {
      continue;
    }
    deduped[dedupCount++] = input[i];
  }

  if (dedupCount == 0) {
    return false;
  }
  if (dedupCount == 1) {
    for (uint16_t i = 0; i < outCount; ++i) {
      out[i] = deduped[0];
    }
    return true;
  }

  float cumulative[kMaxEnginePoints];
  cumulative[0] = 0.0f;
  for (uint16_t i = 1; i < dedupCount; ++i) {
    const float dx = deduped[i].x - deduped[i - 1].x;
    const float dy = deduped[i].y - deduped[i - 1].y;
    cumulative[i] = cumulative[i - 1] + sqrtf((dx * dx) + (dy * dy));
  }

  const float totalLength = cumulative[dedupCount - 1];
  if (totalLength <= 1e-6f) {
    for (uint16_t i = 0; i < outCount; ++i) {
      out[i] = deduped[0];
    }
    return true;
  }

  uint16_t segIndex = 1;
  const uint16_t denom = (outCount > 1) ? static_cast<uint16_t>(outCount - 1) : 1;
  for (uint16_t i = 0; i < outCount; ++i) {
    const float targetDistance = totalLength * static_cast<float>(i) / static_cast<float>(denom);
    while (segIndex < (dedupCount - 1) && cumulative[segIndex] < targetDistance) {
      ++segIndex;
    }

    const float prevDist = cumulative[segIndex - 1];
    const float nextDist = cumulative[segIndex];
    const Point &prevPoint = deduped[segIndex - 1];
    const Point &nextPoint = deduped[segIndex];
    if (nextDist <= prevDist) {
      out[i] = prevPoint;
      continue;
    }

    const float mix = (targetDistance - prevDist) / (nextDist - prevDist);
    out[i].x = prevPoint.x + ((nextPoint.x - prevPoint.x) * mix);
    out[i].y = prevPoint.y + ((nextPoint.y - prevPoint.y) * mix);
  }
  return true;
}

void GraffitiEngine::normalizeResampledPoints(Point *points, uint16_t count) {
  if (points == nullptr || count == 0) {
    return;
  }

  float minX = points[0].x;
  float maxX = points[0].x;
  float minY = points[0].y;
  float maxY = points[0].y;
  for (uint16_t i = 1; i < count; ++i) {
    minX = min(minX, points[i].x);
    maxX = max(maxX, points[i].x);
    minY = min(minY, points[i].y);
    maxY = max(maxY, points[i].y);
  }

  const float centerX = (minX + maxX) * 0.5f;
  const float centerY = (minY + maxY) * 0.5f;
  const float scale = max(max(maxX - minX, maxY - minY), 1.0f);
  for (uint16_t i = 0; i < count; ++i) {
    points[i].x = ((points[i].x - centerX) * 2.0f) / scale;
    points[i].y = ((points[i].y - centerY) * 2.0f) / scale;
  }
}

bool GraffitiEngine::buildFeatureVector(const Point *points, uint16_t count, float *out) const {
  if (points == nullptr || out == nullptr || count == 0 || count > kMaxEnginePoints) {
    return false;
  }

  Point normalized[kResampledPointCount];
  if (!resampleStroke(points, count, normalized, kResampledPointCount)) {
    return false;
  }
  normalizeResampledPoints(normalized, kResampledPointCount);

  uint16_t featureIndex = 0;
  float prevX = 0.0f;
  float prevY = 0.0f;
  for (uint16_t i = 0; i < kResampledPointCount; ++i) {
    const float curX = normalized[i].x;
    const float curY = normalized[i].y;
    const float dx = (i == 0) ? 0.0f : (curX - prevX);
    const float dy = (i == 0) ? 0.0f : (curY - prevY);

    const float rawValues[4] = {curX, curY, dx, dy};
    for (uint8_t j = 0; j < 4; ++j) {
      const float mean = GraffitiPrototypeModelData::kMeans[featureIndex];
      const float stdv = GraffitiPrototypeModelData::kStds[featureIndex];
      out[featureIndex] = (rawValues[j] - mean) / stdv;
      ++featureIndex;
    }
    prevX = curX;
    prevY = curY;
  }

  for (uint8_t i = 0; i < GraffitiPrototypeModelData::kConditionCount; ++i) {
    out[featureIndex++] = (static_cast<uint8_t>(condition_) == i)
                              ? GraffitiPrototypeModelData::kConditionWeight
                              : 0.0f;
  }
  return true;
}

bool GraffitiEngine::classify(const Point *points, uint16_t count, Result &out) {
  if (points == nullptr || count == 0 || count > kMaxEnginePoints ||
      GraffitiPrototypeModelData::kPrototypeCount == 0) {
    return false;
  }

  float featureVector[kInputFeatureDim];
  if (!buildFeatureVector(points, count, featureVector)) {
    return false;
  }
  float lettersFeatureVector[kInputFeatureDim];
  bool haveLettersFeatureVector = false;
  if (condition_ == Condition::Numeric || condition_ == Condition::Punct) {
    const Condition saved = condition_;
    condition_ = Condition::Letters;
    haveLettersFeatureVector = buildFeatureVector(points, count, lettersFeatureVector);
    condition_ = saved;
  }

  const GraffitiPrototypeModelData::PrototypeEntry *bestPrototype = nullptr;
  float bestDistance = INFINITY;
  float secondDistance = INFINITY;
  for (uint16_t i = 0; i < GraffitiPrototypeModelData::kPrototypeCount; ++i) {
    const auto &prototype = GraffitiPrototypeModelData::kPrototypes[i];
    const bool conditionMatch = (prototype.conditionIndex == static_cast<uint8_t>(condition_));
    const bool sharedLettersControl =
        (condition_ != Condition::Letters) &&
        (prototype.conditionIndex == static_cast<uint8_t>(Condition::Letters)) &&
        isSharedLettersControl(prototype, condition_) &&
        haveLettersFeatureVector;
    if (!conditionMatch && !sharedLettersControl) {
      continue;
    }
    const float *candidateVector = sharedLettersControl ? lettersFeatureVector : featureVector;
    const float distance = squaredDistance(candidateVector, prototype.vector, kInputFeatureDim);
    if (distance < bestDistance) {
      secondDistance = bestDistance;
      bestDistance = distance;
      bestPrototype = &prototype;
    } else if (distance < secondDistance) {
      secondDistance = distance;
    }
  }

  if (bestPrototype == nullptr || !isfinite(bestDistance)) {
    return false;
  }

  const float confidence = 1.0f / (1.0f + bestDistance);
  const float margin = isfinite(secondDistance) ? (secondDistance - bestDistance) : bestDistance;
  out.symbol = bestPrototype->symbol;
  out.label = bestPrototype->label;
  out.confidence = confidence;
  out.rawScore = margin;
  out.rawDistance = bestDistance;
  out.backend = Backend::Prototype;
  out.accepted = (confidence >= acceptConfidence_);
  out.tokenCount = 0;

  strlcpy(lastSequence_, conditionName(), sizeof(lastSequence_));
  if (out.accepted) {
    ++acceptCount_;
  } else {
    ++rejectCount_;
  }
  return true;
}

void GraffitiEngine::setLegacyPointFallbackEnabled(bool enabled) {
  (void)enabled;
}

bool GraffitiEngine::legacyPointFallbackEnabled() const {
  return false;
}

uint32_t GraffitiEngine::trieAcceptCount() const {
  return acceptCount_;
}

uint32_t GraffitiEngine::pointAcceptCount() const {
  return rejectCount_;
}

uint8_t GraffitiEngine::lastTokenCount() const {
  return 0;
}

const char *GraffitiEngine::lastTokenSequence() const {
  return lastSequence_;
}

void GraffitiEngine::resetStats() {
  acceptCount_ = 0;
  rejectCount_ = 0;
  strlcpy(lastSequence_, "MODEL", sizeof(lastSequence_));
}
