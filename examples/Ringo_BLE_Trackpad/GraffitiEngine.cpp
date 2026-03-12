#include "GraffitiEngine.h"

namespace {
constexpr uint16_t kMaxEnginePoints = 192;

GraffitiEngine::Backend mapBackend(GraffitiRecognizer::Engine engine) {
  switch (engine) {
    case GraffitiRecognizer::Engine::Trie:
      return GraffitiEngine::Backend::Trie;
    case GraffitiRecognizer::Engine::LegacyPoint:
      return GraffitiEngine::Backend::LegacyPoint;
    default:
      return GraffitiEngine::Backend::None;
  }
}
}  // namespace

bool GraffitiEngine::classify(const Point *points, uint16_t count, Result &out) {
  if (points == nullptr || count == 0 || count > kMaxEnginePoints) {
    return false;
  }

  GraffitiRecognizer::Point scratch[kMaxEnginePoints];
  for (uint16_t i = 0; i < count; ++i) {
    scratch[i].x = points[i].x;
    scratch[i].y = points[i].y;
  }

  GraffitiRecognizer::Result raw = {};
  if (!recognizer_.recognize(scratch, count, raw)) {
    return false;
  }

  out.symbol = raw.symbol;
  out.label = raw.name;
  out.confidence = raw.score;
  out.rawScore = raw.score;
  out.rawDistance = raw.distance;
  out.backend = mapBackend(raw.engine);
  out.accepted = (raw.score >= kAcceptConfidence);
  out.tokenCount = raw.tokenCount;
  return true;
}

void GraffitiEngine::setLegacyPointFallbackEnabled(bool enabled) {
  recognizer_.setLegacyPointFallbackEnabled(enabled);
}

bool GraffitiEngine::legacyPointFallbackEnabled() const {
  return recognizer_.legacyPointFallbackEnabled();
}

uint32_t GraffitiEngine::trieAcceptCount() const {
  return recognizer_.trieAcceptCount();
}

uint32_t GraffitiEngine::pointAcceptCount() const {
  return recognizer_.pointAcceptCount();
}

uint8_t GraffitiEngine::lastTokenCount() const {
  return recognizer_.lastTokenCount();
}

const char *GraffitiEngine::lastTokenSequence() const {
  return recognizer_.lastTokenSequence();
}

void GraffitiEngine::resetStats() {
  recognizer_.resetStats();
}
