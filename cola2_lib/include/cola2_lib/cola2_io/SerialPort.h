
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*
 * SerialPort.h
 *
 *  Created on: Jun 23, 2010
 *      Author: Enric Galceran
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_SERIALPORT_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_SERIALPORT_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>

#include <string>

#include "./ISerialPort.h"

namespace cola2 {
namespace io {

class SerialPort : public ISerialPort, private boost::noncopyable
{
 public:
  SerialPort();

  void open(const std::string& serialPortPath)
  throw(AlreadyOpen, OpenFailed, UnsupportedBaudRate, std::invalid_argument);

  EBaudRate getBaudRate() const
  throw(NotOpen, std::runtime_error);

  ECharacterSize getCharSize() const
  throw(NotOpen, std::runtime_error);

  EStopBits getNumOfStopBits() const
  throw(NotOpen, std::runtime_error);

  EParity getParity() const
  throw(NotOpen, std::runtime_error);

  EFlowControl getFlowControl() const
  throw(NotOpen, std::runtime_error);

  /*
   * Serial port configuration settings.
   */
  void setBaudRate(const EBaudRate baudRate)
  throw(UnsupportedBaudRate, NotOpen, std::invalid_argument, std::runtime_error);

  void setCharSize(const ECharacterSize charSize)
  throw(NotOpen, std::invalid_argument, std::runtime_error);

  void setNumOfStopBits(const EStopBits stopBits)
  throw(NotOpen, std::invalid_argument, std::runtime_error);

  void setParity(const EParity parity)
  throw(NotOpen, std::invalid_argument, std::runtime_error);

  void setFlowControl(const EFlowControl flowControl)
  throw(NotOpen, std::invalid_argument, std::runtime_error);

  bool isOpen() const;

  unsigned char readByte(const unsigned int msTimeout = 0)
  throw(NotOpen, ReadTimeout, std::runtime_error);

  void read(DataBuffer &dataBuffer, const unsigned int numOfBytes = 0, const unsigned int msTimeout = 0)
  throw(NotOpen, ReadTimeout, std::runtime_error);

  std::string readLine(const unsigned int msTimeout = 0, const char lineTerminator = '\n')
  throw(NotOpen, ReadTimeout, std::runtime_error);

  void writeByte(const unsigned char dataByte)
  throw(NotOpen, std::runtime_error);

  void write(const DataBuffer& dataBuffer)
  throw(NotOpen, std::runtime_error);

  void write(const std::string& dataString)
  throw(NotOpen, std::runtime_error);

  void write(const DataBuffer& dataBuffer, std::string node_name, double critic_time)
  throw(NotOpen, std::runtime_error);

  void sendBreak()
  throw(NotOpen, std::runtime_error);

  void close()
  throw(NotOpen);

 private:
  boost::asio::io_service io_;
  mutable boost::asio::serial_port serialPort_;

  void timedReadHandler(boost::optional< boost::system::error_code >* a, const boost::system::error_code& b);
};

}  // namespace io
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_SERIALPORT_H_
