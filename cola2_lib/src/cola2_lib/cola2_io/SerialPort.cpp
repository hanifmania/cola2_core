
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

#include "cola2_lib/cola2_io/SerialPort.h"

namespace cola2 {
namespace io {

SerialPort::SerialPort():
  io_(),
  serialPort_(io_)
{
}

void SerialPort::open(const std::string& serialPortPath)
throw(AlreadyOpen, OpenFailed, UnsupportedBaudRate, std::invalid_argument)
{
  if (serialPort_.is_open())
    throw AlreadyOpen();

  boost::system::error_code ec;
  serialPort_.open(serialPortPath, ec);

  if (ec)
    throw OpenFailed(ec.message());
}

EBaudRate SerialPort::getBaudRate() const
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  boost::asio::serial_port::baud_rate br;
  serialPort_.get_option(br, ec);

  if (ec)
    throw std::runtime_error("Error getting baud rate");

  switch (br.value())
  {
  case 50 :
    return BAUD_50;

  case 75 :
    return BAUD_75;

  case 110 :
    return BAUD_110;

  case 134 :
    return BAUD_134;

  case 150 :
    return BAUD_150;

  case 200 :
    return BAUD_200;

  case 300 :
    return BAUD_300;

  case 600 :
    return BAUD_600;

  case 1200 :
    return BAUD_1200;

  case 1800 :
    return BAUD_1800;

  case 2400 :
    return BAUD_2400;

  case 4800 :
    return BAUD_4800;

  case 9600 :
    return BAUD_9600;

  case 19200 :
    return BAUD_19200;

  case 38400 :
    return BAUD_38400;

  case 57600 :
    return BAUD_57600;

  case 115200 :
    return BAUD_115200;

  case 230400 :
    return BAUD_230400;

#ifdef __linux__
  case 460800 :
    return BAUD_460800;
#endif

  default :
    return BAUD_INVALID;
  }
}

ECharacterSize SerialPort::getCharSize() const
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  boost::asio::serial_port::character_size cs;
  serialPort_.get_option(cs, ec);

  if (ec)
    throw std::runtime_error("Error getting character size");

  switch (cs.value())
  {
  case 5 :
    return CHAR_SIZE_5;

  case 6 :
    return CHAR_SIZE_6;

  case 7 :
    return CHAR_SIZE_7;

  case 8 :
    return CHAR_SIZE_8;

  default :
    return CHAR_SIZE_INVALID;
  }
}

EStopBits SerialPort::getNumOfStopBits() const
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  boost::asio::serial_port::stop_bits sb;
  serialPort_.get_option(sb, ec);

  if (ec)
    throw std::runtime_error("Error getting num. of stop bits");

  switch (sb.value()) {
  case boost::asio::serial_port_base::stop_bits::one :
    return STOP_BITS_1;

  case boost::asio::serial_port_base::stop_bits::two :
    return STOP_BITS_2;

  default:
    return STOP_BITS_INVALID;  // TODO: Add 'stop_bits::one_point_five' option
  }
}

EParity SerialPort::getParity() const
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  boost::asio::serial_port::parity p;
  serialPort_.get_option(p, ec);

  if (ec)
    throw std::runtime_error("Error getting parity");

  switch (p.value()) {
  case boost::asio::serial_port_base::parity::even :
    return PARITY_EVEN;

  case boost::asio::serial_port_base::parity::odd :
    return PARITY_ODD;

  case boost::asio::serial_port_base::parity::none :
    return PARITY_NONE;

  default :
    return PARITY_INVALID;
  }
}

EFlowControl SerialPort::getFlowControl() const
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  boost::asio::serial_port::flow_control fc;
  serialPort_.get_option(fc, ec);

  if (ec)
    throw std::runtime_error("Error getting flow control");

  switch (fc.value()) {
  case boost::asio::serial_port_base::flow_control::hardware :
    return FLOW_CONTROL_HARD;

  case boost::asio::serial_port_base::flow_control::software :
    return FLOW_CONTROL_SOFT;

  case boost::asio::serial_port_base::flow_control::none :
    return FLOW_CONTROL_NONE;

  default :
    return FLOW_CONTROL_INVALID;
  }
}

/*
 * Serial port configuration settings.
 */
void SerialPort::setBaudRate(const EBaudRate baudRate)
throw(UnsupportedBaudRate, NotOpen, std::invalid_argument, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;

  switch (baudRate)
  {
  case BAUD_50 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(50), ec);
    break;

  case BAUD_75 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(75), ec);
    break;

  case BAUD_110 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(110), ec);
    break;
  case BAUD_134 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(134), ec);
    break;
  case BAUD_150 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(150), ec);
    break;
  case BAUD_200 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(200), ec);
    break;
  case BAUD_300 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(300), ec);
    break;
  case BAUD_600 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(600), ec);
    break;
  case BAUD_1200 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(1200), ec);
    break;
  case BAUD_1800 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(1800), ec);
    break;
  case BAUD_2400 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(2400), ec);
    break;
  case BAUD_4800 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(4800), ec);
    break;
  case BAUD_9600 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(9600), ec);
    break;
  case BAUD_19200 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(19200), ec);
    break;
  case BAUD_38400 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(38400), ec);
    break;
  case BAUD_57600 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(57600), ec);
    break;
  case BAUD_115200 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(115200), ec);
    break;
  case BAUD_230400 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(230400), ec);
    break;
#ifdef __linux__
  case BAUD_460800 :
    serialPort_.set_option(boost::asio::serial_port_base::baud_rate(460800), ec);
    break;
#endif
  default :
    throw std::invalid_argument("Invalid baud rate");
    break;
  }

  if (ec)
    throw std::runtime_error("Error setting baud rate");
}

void SerialPort::setCharSize(const ECharacterSize charSize)
throw(NotOpen, std::invalid_argument, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;

  switch (charSize)
  {
  case CHAR_SIZE_5 :
    serialPort_.set_option(boost::asio::serial_port_base::character_size(5), ec);
    break;
  case CHAR_SIZE_6 :
    serialPort_.set_option(boost::asio::serial_port_base::character_size(6), ec);
    break;
  case CHAR_SIZE_7 :
    serialPort_.set_option(boost::asio::serial_port_base::character_size(7), ec);
    break;
  case CHAR_SIZE_8 :
    serialPort_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    break;
  default :
    throw std::invalid_argument("Invalid character size");
    break;
  }

  if (ec)
    throw std::runtime_error("Error setting character size");
}

void SerialPort::setNumOfStopBits(const EStopBits stopBits)
throw(NotOpen, std::invalid_argument, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;

  switch (stopBits)
  {
  case STOP_BITS_1:
    serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
    break;
  case STOP_BITS_2:
    serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two), ec);
    break;
  default:
    serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
    break;
  }
  if (ec)
    throw std::runtime_error("Error setting num. of stop bits");
}

void SerialPort::setParity(const EParity parity)
throw(NotOpen, std::invalid_argument, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;

  switch (parity) {
  case PARITY_EVEN :
    serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even), ec);
    break;

  case PARITY_ODD :
    serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd), ec);
    break;

  case PARITY_NONE :
    serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
    break;

  default :
    throw std::invalid_argument("Invalid parity type");
    break;
  }

  if (ec)
    throw std::runtime_error("Error setting parity");
}

void SerialPort::setFlowControl(const EFlowControl flowControl)
throw(NotOpen, std::invalid_argument, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;

  switch (flowControl)
  {
  case FLOW_CONTROL_HARD :
    serialPort_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware), ec);
    break;

  case FLOW_CONTROL_SOFT :
    serialPort_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software), ec);
    break;

  case FLOW_CONTROL_NONE :
    serialPort_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
    break;

  default :
    throw std::invalid_argument("Invalid flow control type");
    break;
  }

  if (ec)
    throw std::runtime_error("Error setting flow control type");
}

bool SerialPort::isOpen() const
{
  return serialPort_.is_open();
}

void SerialPort::read(DataBuffer &dataBuffer, const unsigned int numOfBytes, const unsigned int msTimeout)
throw(NotOpen, ReadTimeout, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  dataBuffer.resize(0);
  dataBuffer.reserve(numOfBytes);

  for (size_t i = 0; i < numOfBytes; ++i)
    dataBuffer.push_back(readByte(msTimeout));
}

std::string SerialPort::readLine(const unsigned int msTimeout, const char lineTerminator)
throw(NotOpen, ReadTimeout, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  std::string result;
  char nextChar = 0;
  do
  {
    nextChar = this->readByte(msTimeout);
    result += nextChar;
  } while (nextChar != lineTerminator);
  return result;
}

void SerialPort::writeByte(const unsigned char dataByte)
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  size_t n = boost::asio::write(serialPort_, boost::asio::buffer(&dataByte, 1), boost::asio::transfer_all(), ec);

  if (ec)
    throw std::runtime_error("Could not write to serial port: " + ec.message());

  if (n != 1)
    throw std::runtime_error("Unexpected number of bytes sent");
}

void SerialPort::write(const DataBuffer& dataBuffer)
throw(NotOpen, std::runtime_error)
{
  BOOST_FOREACH(DataBuffer::value_type byte, dataBuffer)
  {
    writeByte(byte);
  }
}

void SerialPort::write(const std::string& dataString)
throw(NotOpen, std::runtime_error)
{
  BOOST_FOREACH(std::string::value_type c, dataString)
  {
    writeByte(c);
  }
}

void SerialPort::write(const DataBuffer& dataBuffer, std::string node_name, double critic_time)
throw(NotOpen, std::runtime_error)
{
  // Timer
  boost::posix_time::ptime lastSetpointCallback;

  if (isOpen())
  {
    boost::posix_time::ptime previousTime = boost::posix_time::microsec_clock::universal_time();
    lastSetpointCallback = boost::posix_time::microsec_clock::universal_time();

    BOOST_FOREACH(DataBuffer::value_type byte, dataBuffer)
    {
      writeByte(byte);
    }

    double dt = 0.001 * boost::posix_time::time_period(previousTime, boost::posix_time::microsec_clock::universal_time()).length().total_microseconds();

    // Compute time
    if (dt > critic_time)
    {
      std::cout << node_name << ": slow serial port. Total elapsed time: " << " ms" << std::endl;
    }
  }
  else
  {
    std::cerr << node_name << ": serial port not open when sending!" << std::endl;
  }
}

void SerialPort::sendBreak()
throw(NotOpen, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();
  boost::system::error_code ec;
  serialPort_.send_break(ec);
  if (ec)
    throw std::runtime_error(ec.message());
}

void SerialPort::close()
throw(NotOpen)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  boost::system::error_code ec;
  serialPort_.close(ec);
}

void SerialPort::timedReadHandler(boost::optional< boost::system::error_code >* a, const boost::system::error_code& b)
{
  a->reset(b);
}

}  // namespace io
}  // namespace cola2

unsigned char cola2::io::SerialPort::readByte(const unsigned int msTimeout)
throw(cola2::io::ISerialPort::NotOpen, cola2::io::ISerialPort::ReadTimeout, std::runtime_error)
{
  if (!serialPort_.is_open())
    throw NotOpen();

  unsigned char byte;

  if (msTimeout == 0)
  {
    // Perform synchronous read
    boost::system::error_code ec;
    size_t numOfBytesRead = boost::asio::read(serialPort_, boost::asio::buffer(&byte, 1), boost::asio::transfer_all(), ec);

    if (ec)
      throw std::runtime_error("Could not read synchronously from serial port: " + ec.message());

    if (numOfBytesRead != 1)
      throw std::runtime_error("Unexpected number of bytes read");
  }
  else
  {
    // Perform asynchronous timed read
    boost::optional< boost::system::error_code > timerResult;
    boost::asio::deadline_timer timer(serialPort_.get_io_service());
    timer.expires_from_now(boost::posix_time::milliseconds(msTimeout));
    timer.async_wait(boost::bind(&SerialPort::timedReadHandler, this, &timerResult, _1));

    boost::optional< boost::system::error_code > readResult;
    boost::asio::async_read(serialPort_,
                            boost::asio::buffer(&byte, 1),
                            boost::bind(&SerialPort::timedReadHandler, this, &readResult, _1));

    serialPort_.get_io_service().reset();

    boost::system::error_code ec;
    bool timeoutOccurred = false;

    while (serialPort_.get_io_service().run_one(ec))
    {
      if (readResult)
      {
        timer.cancel(ec);
      }
      else if (timerResult)
      {
        serialPort_.cancel(ec);
        timeoutOccurred = true;
      }
    }

    if (timeoutOccurred)
      throw ReadTimeout();

    if (*readResult)
      throw std::runtime_error("Could not perform timed read from serial port (1): " + readResult->message());

    if (ec)
      throw std::runtime_error("Could not perform timed read from serial port (2): " + ec.message());
  }

  return byte;
}
