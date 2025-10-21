#include "error.hpp"

#include <string>

namespace err {

std::string errorStr(error err) {
  switch (err) {
    case error::ok:
      return "ok";
    case error::ser_empty_address:
      return "ser_empty_address";
    case error::ser_opening_fd:
      return "ser_opening_fd";
    case error::ser_getaddress:
      return "ser_getaddress";
    case error::ser_setaddress:
      return "ser_setaddress";
    case error::size:
    default:
      return "unknown_error";
  }
}

Error::Error(error e_, std::source_location loc_) {
  e = e_;
  loc = loc_;
}
Error::Error(const Error &err) {
  e = err.e;
  loc = err.loc;
};
constexpr err::Error &Error::operator=(const err::Error &rhs) {
  e = rhs.e;
  loc = rhs.loc;
  return *this;
}

error Error::get() const { return e; }
std::string Error::str() {
  return std::string("error: ") + loc.file_name() + " " + loc.function_name() +
         ":(" + std::to_string(loc.line()) + ", " +
         std::to_string(loc.column()) + "): " + errorStr(e);
};

}  // namespace err
