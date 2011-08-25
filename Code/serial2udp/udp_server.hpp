
#ifndef __UDP_SERVER_HPP__
#define __UDP_SERVER_HPP__

#include <boost/asio.hpp>

class udp_server
{
public:
  udp_server(boost::asio::io_service& io_service);

private:
  void start_receive();

  void handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/);

  void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/);

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  boost::array<char, 1> recv_buffer_;
};

#endif // __UDP_SERVER_HPP__