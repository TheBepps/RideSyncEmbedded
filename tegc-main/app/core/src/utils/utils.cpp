#include "utils.hpp"

#include <sys/time.h>

#include <cstring>
#include <ctime>
#include <string>

double getTimestampSeconds() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

uint64_t getTimestampMicroseconds() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000000 + tv.tv_usec;
}

bool cmd_compare(const char *command, const std::string &line) {
  return strncmp(command, line.c_str(), strlen(command)) == 0;
}
