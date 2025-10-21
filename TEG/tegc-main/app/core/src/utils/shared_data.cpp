#include "shared_data.hpp"

CsvData csvData;

void removeSamplesByAge() {}
void removeSamplesUniformly(size_t newMaxSize) {
  for (auto &[_, values] : csvData) {
    if (values.size() < newMaxSize) {
      continue;
    }
    size_t n = values.size() - newMaxSize;
    size_t oneEvery = values.size() / n;
    for (int i = n - 1; i >= 0; i--) {
      values.erase(values.begin() + oneEvery * i);
    }
  }
}
void removeSamplesByRange(size_t start, size_t end) {
  if (start > end) {
    return;
  }
  for (auto &[_, values] : csvData) {
    values.erase(values.begin() + start, values.begin() + end);
  }
}
void keepFirstNSamples(size_t samplesToKeep) {
  for (auto &[_, values] : csvData) {
    if (samplesToKeep > values.size()) {
      continue;
    }
    values.erase(values.begin(),
                 values.begin() + (values.size() - samplesToKeep));
  }
}
