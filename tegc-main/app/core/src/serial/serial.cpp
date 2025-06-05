#include "serial.hpp"

#include <fcntl.h>
#include <unistd.h>

#include "errors/error.hpp"

err::Error SerialPort::Open(const std::string &address, speed_t speed) {
  if (address.empty()) return err::error::ser_empty_address;

  m_open = false;
  m_fd = open(address.c_str(), O_RDWR);
  // Handle in case of error
  if (m_fd == -1) {
    return err::error::ser_opening_fd;
  }

  struct termios tty;

  // Read in existing settings and handle errors
  if (tcgetattr(m_fd, &tty) != 0) {
    return err::error::ser_getaddress;
  }

  // Setting baud rate
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag &= ~PARENB;         // disable parity bit
  tty.c_cflag &= ~CSTOPB;         // clear stop field
  tty.c_cflag |= CS8;             // 8 data bits per byte
  tty.c_cflag &= ~CRTSCTS;        // disable TRS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ and ignore control lines,
                                  // setting CLOCAL allows us to read data
  // local modes
  tty.c_lflag &= ~ICANON;  // disable canonical mode, in canonical mode input
                           // data is received line by line, usually undesired
                           // when dealing with serial ports
  tty.c_lflag &=
      ~ECHO;  // if this bit (ECHO) is set, sent characters will be echoed back.
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &=
      ~ISIG;  // when the ISIG bit is set, INTR,QUIT and SUSP characters are
              // interpreted. we don't want this with a serial port
  // the c_iflag member of the termios struct contains low-level settings for
  // input processing. the c_iflag member is an int
  tty.c_iflag &=
      ~(IXON | IXOFF |
        IXANY);  // clearing IXON,IXOFF,IXANY disable software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // clearing all of this bits disable any special
                            // handling of received bytes, i want raw data
  // output modes (c_oflag). the c_oflag member of the termios struct contain
  // low level settings for output processing, we want to disable any special
  // handling of output chars/bytes
  tty.c_oflag &= ~OPOST;  // prevent special interpretation of output bytes
  tty.c_oflag &=
      ~ONLCR;  // prevent conversion of newline to carriage return/line feed
  // setting VTIME VMIN
  tty.c_cc[VTIME] = 10;  // read() will block until either any amount of data is
                         // received or the timeout ocurs
  tty.c_cc[VMIN] = 0;

  // After changing settings we need to save the tty termios struct, also error
  // checking
  if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
    return err::error::ser_setaddress;
  }

  m_open = true;

  return err::error::ok;
}

void SerialPort::Close() {
  if (!m_open) return;
  close(m_fd);
  m_open = false;
}

bool SerialPort::IsOpen() const { return m_open; }

int SerialPort::ReadBytes(uint8_t *bytes, size_t nBytes) {
  if (!m_open) {
    return -1000;
  }
  return read(m_fd, bytes, nBytes);
}

void SerialPort::Send(const std::string &data) {
  if (!m_open) {
    return;
  }
  write(m_fd, data.c_str(), data.size());
}
