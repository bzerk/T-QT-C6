# Graffiti Recognizer Research Notes

This note tracks primary references for moving from point-cloud matching to
vector/delta and sequence-first recognition.

## Core papers and systems

- Rubine, 1991, "Specifying Gestures by Example" (SIGGRAPH): online gesture
  recognition from a compact feature vector + statistical classifier.
  DOI: https://doi.org/10.1145/122718.122753
- Sakoe & Chiba, 1978, "Dynamic programming algorithm optimization for spoken
  word recognition" (DTW): canonical alignment method for variable-speed
  sequences.
  DOI: https://doi.org/10.1109/TASSP.1978.1163055
- Wobbrock et al., 2007, "$1 Unistroke Recognizer" (UIST): low-overhead
  template recognizer that performs resample/rotate/scale/translate.
  Project: https://depts.washington.edu/acelab/proj/dollar/index.html
  PDF: https://depts.washington.edu/acelab/proj/dollar/dollar.pdf
- Li, 2010, "Protractor: A Fast and Accurate Gesture Recognizer" (CHI):
  angular/vector-space optimization of $1-style matching.
  PDF: https://userinterfaces.aalto.fi/protractor/protractor.pdf
  ACM: https://dl.acm.org/doi/10.1145/1753326.1753654
- Anthony & Wobbrock, 2014, "$N-Protractor" (GI): multistroke extension using
  Protractor-style speedups.
  PDF: https://faculty.washington.edu/wobbrock/pubs/gi-14.01.pdf
- Vatavu et al., 2018, "$Q" (MobileHCI): point-cloud style recognizer tuned for
  low-resource/mobile constraints.
  PDF: https://faculty.washington.edu/wobbrock/pubs/mobilehci-18.pdf

## Relevance to Ringo

- Current firmware matcher is closest to `$1` (point resample + normalize +
  template distance).
- Vector/delta sequence matching is a better fit for Graffiti's deliberate,
  differentiable stroke directions.
- Tree/trie decoding can be layered on top of quantized directional tokens for:
  - early pruning
  - partial-stroke prediction
  - lower compute than full point-template pass

## Suggested architecture path

1. Direction-token front-end (8-way quantized deltas + run-length compression).
2. Template trie / beam decode over token sequences (with insertion/substitution
   penalties similar to edit distance).
3. Optional fallback matcher:
   - keep point-template path for ambiguous cases
   - or move to Protractor/$Q-style backend once full symbol set is onboarded
