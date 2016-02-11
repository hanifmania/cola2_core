/*
 * ISerialPort.h
 *
 *  Created on: May 10, 2010
 *      Author: Enric Galceran
 */

#include "cola2_lib/cola2_io/ISerialPort.h"

namespace cola2 {
namespace io {

/*
 * Helper functions.
 */
EBaudRate ISerialPort::baudRateFromInteger(const unsigned int baudRate)
throw(std::invalid_argument)
{
  switch (baudRate)
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
  default:
    throw std::invalid_argument("Invalid baud rate string");
  }
}

ECharacterSize ISerialPort::charSizeFromInteger(const unsigned int charSize)
throw(std::invalid_argument)
{
  switch (charSize)
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
    throw std::invalid_argument("Invalid char size string");
  }
}

EStopBits ISerialPort::numOfStopBitsFromInteger(const unsigned int stopBits)
throw(std::invalid_argument)
{
  switch (stopBits)
  {
  case 1 :
    return STOP_BITS_1;
  case 2 :
    return STOP_BITS_2;
  default :
    throw std::invalid_argument("Invalid num. of stop bits string");
  }
}

EParity ISerialPort::parityFromString(const std::string& parity)
throw(std::invalid_argument)
{
  if (parity == "EVEN")
    return PARITY_EVEN;
  else if (parity == "ODD")
    return PARITY_ODD;
  else if (parity == "NONE")
    return PARITY_NONE;
  else
    throw std::invalid_argument("Invalid parity string");
}

EFlowControl ISerialPort::flowControlFromString(const std::string& flowControl)
throw(std::invalid_argument)
{
  if (flowControl == "HARD")
    return FLOW_CONTROL_HARD;
  else if (flowControl == "SOFT")
    return FLOW_CONTROL_SOFT;
  else if (flowControl == "NONE")
    return FLOW_CONTROL_NONE;
  else
    throw std::invalid_argument("Invalid flow control string");
}

}  // namespace io
}  // namespace cola2
