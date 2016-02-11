#include "cola2_lib/cola2_io/UdpSocket.h"

namespace cola2 {
namespace io {
namespace udp_socket {

Client::Client()
{
  _enabled_port = false;
  _ip = "127.0.0.1";  // Default ip -> localhost
}

Client& Client::setIp(const std::string& ip)
{
  _ip = ip;
  return *this;
}


Client& Client::setPort(const std::string& port)
{
  _port = port;
  _enabled_port = true;
  return *this;
}


std::string Client::read()
{
  if (_enabled_port)
  {
    try
    {
      boost::asio::io_service io_service;

      boost::asio::ip::udp::resolver resolver(io_service);
      boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), _ip, _port);
      boost::asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);

      boost::asio::ip::udp::socket socket(io_service);
      socket.open(boost::asio::ip::udp::v4());

      boost::array<char, 1> send_buf  = { 0 };
      socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

      boost::array<char, 1000> recv_buf;  // TODO: Fixed buffer size of 1000 char. It works with std::vector<char> but it is not safe
      boost::asio::ip::udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

      std::string data(recv_buf.begin(), recv_buf.begin()+len);  // TODO: If the server is not ready, this line will never be reached (the client waits forever)
      return data;
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }

    return "";
  }
  else
  {
    std::cerr << "UdpSocket client not enabled yet" << std::endl;
    return "";
  }
}

Server::Server()
{
  _message = "";
  _enabled_port = false;
}

Server& Server::setPort(const std::string& port)
{
  _port = port;
  _enabled_port = true;
  return *this;
}

Server& Server::launch()
{
  if (_enabled_port)
  {
    thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Server::run, this));
  }
  else
  {
    std::cerr << "UdpSocket server not enabled yet" << std::endl;
  }
  return *this;
}

Server& Server::run()
{
  try
  {
    boost::asio::io_service io_service;

    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), boost::lexical_cast<unsigned int>(_port)));

    for (;;)
    {
      boost::array<char, 1> recv_buf;
      boost::asio::ip::udp::endpoint remote_endpoint;
      boost::system::error_code error;
      socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

      if (error && error != boost::asio::error::message_size)
      {
        throw boost::system::system_error(error);
      }
      boost::system::error_code ignored_error;
      {
        boost::mutex::scoped_lock serial_lock(_message_mutex);
        socket.send_to(boost::asio::buffer(_message), remote_endpoint, 0, ignored_error);
      }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return *this;
}

Server& Server::join()
{
  (*thread_ptr.get()).join();
  return *this;
}

Server& Server::update(const std::string& new_message)
{
  boost::mutex::scoped_lock serial_lock(_message_mutex);
  _message = new_message;
  return *this;
}

}  // namespace udp_socket
}  // namespace io
}  // namespace cola2
