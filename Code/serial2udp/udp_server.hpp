
#ifndef __UDP_SERVER_HPP__
#define __UDP_SERVER_HPP__

#include <boost/asio.hpp>

#include "serial_server.hpp"

class udp_server
{
public:
  udp_server(boost::asio::io_service& io_service);

  void start_udp_receive();

  void start_serial_receive();

  void handle_udp_receive(const boost::system::error_code& error,
      std::size_t s/*bytes_transferred*/);

  void handle_serial_receive(const boost::system::error_code& error,
      std::size_t s/*bytes_transferred*/);

  void handle_serial_send(const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/);

  // Initialized internal variables
  boost::asio::ip::udp::socket socket_;
  boost::asio::serial_port port;

  // Intermediate internal variables
  boost::asio::ip::udp::endpoint remote_endpoint;
  boost::array<char, 128> udp_receive_buffer;
  boost::array<char, 1> serial_receive_buffer;
};

#endif // __UDP_SERVER_HPP__