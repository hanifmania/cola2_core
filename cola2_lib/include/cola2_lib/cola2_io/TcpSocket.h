#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_TCPSOCKET_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_TCPSOCKET_H_

#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace cola2 {
namespace io {

typedef std::vector< unsigned char > DataBuffer;

class TcpSocket
{
 private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;

 public:
  TcpSocket();

  void connect(const std::string& IP, const unsigned short port);

  unsigned char readByte(const unsigned int msTimeout) throw(std::runtime_error);
  void read(DataBuffer& dataBuffer, const unsigned int numOfBytes, const unsigned int msTimeout) throw(std::runtime_error);
  void readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout) throw(std::runtime_error);
  std::string readLine(const unsigned int msTimeout, const unsigned char lineTerminator) throw(std::runtime_error);

  void writeByte(const unsigned char dataByte) throw(std::runtime_error);
  void write(const DataBuffer& dataBuffer) throw(std::runtime_error);
  void write(const std::string& dataString) throw(std::runtime_error);

  void close();
  void timedReadHandler(boost::optional< boost::system::error_code >* a, const boost::system::error_code& b);
};

}  // namespace io
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_TCPSOCKET_H_
