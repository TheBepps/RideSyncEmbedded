#pragma once

#include <map>
#include <string>
#include <vector>

typedef std::map<std::string, std::vector<double>> CsvData;

extern CsvData csvData;

void removeSamplesByAge();
void removeSamplesUniformly(size_t newMaxSize);
void removeSamplesByRange(size_t start, size_t end);
void keepFirstNSamples(size_t samplesToKeep);
