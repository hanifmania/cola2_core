/*
 * ISerialPort.h
 *
 *  Created on: May 10, 2010
 *      Author: Enric Galceran
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_ISERIALPORT_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_ISERIALPORT_H_

#include <stdexcept>
#include <string>
#include <vector>

namespace cola2 {
namespace io {

/*
 * The allowed set of baud rates.
 */
enum EBaudRate
{
  BAUD_50,
  BAUD_75,
  BAUD_110,
  BAUD_134,
  BAUD_150,
  BAUD_200,
  BAUD_300,
  BAUD_600,
  BAUD_1200,
  BAUD_1800,
  BAUD_2400,
  BAUD_4800,
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200,
  BAUD_230400,
  //
  // Note: B460800 is defined on Linux but not on Mac OS
  // X. What about other operating systems ?
  //
#ifdef __linux__
  BAUD_460800,
#endif
  BAUD_INVALID,
  BAUD_DEFAULT = BAUD_57600
};

/*
 * The allowed character sizes.
 */
enum ECharacterSize
{
  CHAR_SIZE_5,  // < 5 bit characters.
  CHAR_SIZE_6,  // < 6 bit characters.
  CHAR_SIZE_7,  // < 7 bit characters.
  CHAR_SIZE_8,  // < 8 bit characters.
  CHAR_SIZE_INVALID,
  CHAR_SIZE_DEFAULT = CHAR_SIZE_8
};

/*
 * The allowed numbers of stop bits.
 */
enum EStopBits
{
  STOP_BITS_1,   // 1 stop bit.
  STOP_BITS_2,   // 2 stop bits.
  STOP_BITS_INVALID,
  STOP_BITS_DEFAULT = STOP_BITS_1
};

/*
 * The allowed parity checking modes.
 */
enum EParity
{
  PARITY_EVEN,     // < Even parity.
  PARITY_ODD,      // < Odd parity.
  PARITY_NONE,     // < No parity i.e. parity checking disabled.
  PARITY_INVALID,
  PARITY_DEFAULT = PARITY_NONE
};

/*
 * The allowed flow control modes.
 */
enum EFlowControl
{
  FLOW_CONTROL_HARD,
  FLOW_CONTROL_SOFT,
  FLOW_CONTROL_NONE,
  FLOW_CONTROL_INVALID,
  FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
};

/*
 * ISerialPort
 * Interface for a serial port device, supporting most common operations.
 */
class ISerialPort
{
 public:
  typedef std::vector< unsigned char > DataBuffer;

  /*
   * Exceptions
   */
  class NotOpen : public std::logic_error
  {
   public:
    NotOpen() : logic_error("Serial port is not open") { }
    NotOpen(const std::string& whatArg) : logic_error(whatArg) { }
  };

  class OpenFailed : public std::runtime_error
  {
   public:
    OpenFailed() : runtime_error("Serial port failed to open") { }
    OpenFailed(const std::string& whatArg) : runtime_error(whatArg) { }
  };

  class AlreadyOpen : public std::logic_error
  {
   public:
    AlreadyOpen() : logic_error("Serial port was already open") { }
    AlreadyOpen(const std::string& whatArg) : logic_error(whatArg) { }
  };

  class UnsupportedBaudRate : public std::runtime_error
  {
   public:
    UnsupportedBaudRate() : runtime_error("Unsupported baud rate") { }
    UnsupportedBaudRate(const std::string& whatArg) : runtime_error(whatArg) { }
  };

  class ReadTimeout : public std::runtime_error
  {
   public:
    ReadTimeout() : runtime_error("Serial port read timeout") { }
    ReadTimeout(const std::string& whatArg) : runtime_error(whatArg) { }
  };
  /*
   * Helper functions.
   */
  static EBaudRate baudRateFromInteger(const unsigned int baudRate)
  throw(std::invalid_argument);

  static ECharacterSize charSizeFromInteger(const unsigned int charSize)
  throw(std::invalid_argument);

  static EStopBits numOfStopBitsFromInteger(const unsigned int stopBits)
  throw(std::invalid_argument);

  static EParity parityFromString(const std::string& parity)
  throw(std::invalid_argument);

  static EFlowControl flowControlFromString(const std::string& flowControl)
  throw(std::invalid_argument);

  /*
   * Open a serial port.
   */
  virtual void open(const std::string& serialPortPath)
  throw(AlreadyOpen, OpenFailed, UnsupportedBaudRate, std::invalid_argument) = 0;

  /*
   * Getters for serial port configuration.
   */
  virtual EBaudRate getBaudRate() const
  throw(NotOpen, std::runtime_error) = 0;

  virtual ECharacterSize getCharSize() const
  throw(NotOpen, std::runtime_error) = 0;

  virtual EStopBits getNumOfStopBits() const
  throw(NotOpen, std::runtime_error) = 0;

  virtual EParity getParity() const
  throw(NotOpen, std::runtime_error) = 0;

  virtual EFlowControl getFlowControl() const
  throw(NotOpen, std::runtime_error) = 0;

  /*
   * Serial port configuration settings.
   */
  virtual void setBaudRate(const EBaudRate baudRate)
  throw(UnsupportedBaudRate, NotOpen, std::invalid_argument, std::runtime_error) = 0;

  virtual void setCharSize(const ECharacterSize charSize)
  throw(NotOpen, std::invalid_argument, std::runtime_error) = 0;

  virtual void setNumOfStopBits(const EStopBits stopBits)
  throw(NotOpen, std::invalid_argument, std::runtime_error) = 0;

  virtual void setParity(const EParity parity) throw(NotOpen, std::invalid_argument, std::runtime_error) = 0;

  virtual void setFlowControl(const EFlowControl flowControl)
  throw(NotOpen, std::invalid_argument, std::runtime_error) = 0;

  virtual bool isOpen() const = 0;

  /**
   * Read a single byte from the serial port. If no data is
   * available in the specified number of milliseconds(msTimeout),
   * then this method will throw ReadTimeout exception. If msTimeout
   * is 0, then this method will block till data is available.
   */
  virtual unsigned char readByte(const unsigned int msTimeout = 0)
  throw(NotOpen, ReadTimeout, std::runtime_error) = 0;

  /**
    * Read the specified number of bytes from the serial port. The
    * method will timeout if no data is received in the specified
    * number of milliseconds(msTimeout). If msTimeout is 0, then
    * this method will block till all requested bytes are
    * received. If numOfBytes is zero, then this method will keep
    * reading data till no more data is available at the serial
    * port. In all cases, all read data is available in dataBuffer on
    * return from this method.
    */
  virtual void read(DataBuffer& dataBuffer, const unsigned int numOfBytes = 0, const unsigned int msTimeout = 0)
  throw(NotOpen, ReadTimeout, std::runtime_error) = 0;

  /**
    * Read a line of characters from the serial port.
    */
  virtual std::string readLine(const unsigned int msTimeout = 0, const char lineTerminator = '\n')
  throw(NotOpen, ReadTimeout, std::runtime_error) = 0;

  /**
    * Send a single byte to the serial port.
    *
    * @throw NotOpen Thrown if this method is called while the serial
    * port is not open.
    */
  virtual void writeByte(const unsigned char dataByte)
  throw(NotOpen, std::runtime_error) = 0;

  /**
    * Write the data from the specified vector to the serial port.
    */
  virtual void write(const DataBuffer& dataBuffer)
  throw(NotOpen, std::runtime_error) = 0;

  /**
    * Write a string to the serial port.
    */
  virtual void write(const std::string& dataString)
  throw(NotOpen, std::runtime_error) = 0;

  /**
    * Send a break sequence to the serial port.
    */
  virtual void sendBreak()
  throw(NotOpen, std::runtime_error) = 0;

  /*
   * Close the serial port.
   */
  virtual void close()
  throw(NotOpen) = 0;
};

}  // namespace io
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_ISERIALPORT_H_

