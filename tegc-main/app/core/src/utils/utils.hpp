#pragma once

#include <cstdint>
#include <string>

double getTimestampSeconds();
uint64_t getTimestampMicroseconds();

bool cmd_compare(const char *command, const std::string &line);
