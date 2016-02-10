#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_UDPSOCKET_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_UDPSOCKET_H_

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <string>

namespace cola2 {
namespace io {
namespace udp_socket {

class Client
{
 private:
  std::string _ip;
  std::string _port;
  bool _enabled_port;

 public:
  Client();

  Client& setIp(const std::string& ip);
  Client& setPort(const std::string& port);
  std::string read();
};

class Server
{
 private:
  boost::mutex _message_mutex;
  std::string _port;
  bool _enabled_port;
  std::string _message;
  boost::shared_ptr<boost::thread> thread_ptr;

 public:
  Server();

  Server& setPort(const std::string& port);
  Server& launch();
  Server& run();
  Server& join();
  Server& update(const std::string& new_message);
};

}  // namespace udp_socket
}  // namespace io
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_UDPSOCKET_H_
