#pragma once

#include <string>
#if __has_include(<experimental/source_location>)
#include <experimental/source_location>
namespace std {
using namespace experimental;
}
#else
#include <source_location>
#endif

namespace err {
enum class error : int {
  ok = 0,
  // - serial (ser)
  ser_empty_address,
  ser_opening_fd,
  ser_getaddress,
  ser_setaddress,

  // number of errors
  size
};

static_assert((int)err::error::ok == 0);
std::string errorStr(error err);

class Error {
 private:
  error e;
  std::source_location loc;

 public:
  Error() { e = error::ok; };
  Error(error e_, std::source_location loc_ = std::source_location::current());
  Error(const Error &err);

  error get() const;
  std::string str();

  constexpr err::Error &operator=(const err::Error &rhs);
  operator bool() { return e != error::ok; };
  operator int() { return (int)e; };
  bool operator==(const err::error &rhs) const { return e == rhs; }
};

}  // namespace err
