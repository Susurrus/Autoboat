
#ifndef __UDP_SERVER_HPP__
#define __UDP_SERVER_HPP__

#include <boost/asio.hpp>

#include "serial_server.hpp"

class udp_server
{
public:
  udp_server(boost::asio::io_service& io_service);

  void start_receive();

  void handle_udp_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/);

  void handle_serial_send(const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/);

  // Initialized internal variables
  boost::asio::ip::udp::socket socket_;
  boost::asio::serial_port port;

  // Intermediate internal variables
  boost::asio::ip::udp::endpoint remote_endpoint_;
  boost::array<char, 128> recv_buffer_;
};

#endif // __UDP_SERVER_HPP__