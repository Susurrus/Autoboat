
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include "udp_server.hpp"

using boost::asio::ip::udp;

udp_server::udp_server(boost::asio::io_service& io_service) : socket_(io_service, udp::endpoint(udp::v4(), 1300)), port(boost::asio::serial_port(io_service, "COM1"))
{
	// Configure the options for the serial port
	port.set_option(boost::asio::serial_port::baud_rate(115200));
    port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    port.set_option(boost::asio::serial_port::character_size(8));

	start_receive();
}

void udp_server::start_receive()
{
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), remote_endpoint_,
		boost::bind(&udp_server::handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_receive(const boost::system::error_code& error,
      std::size_t s/*bytes_transferred*/)
{
	if (!error || error == boost::asio::error::message_size)
	{
		/*socket_.async_send_to(boost::asio::buffer("hey"), remote_endpoint_,
			boost::bind(&udp_server::handle_send, this, "Hey",
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));*/
		boost::asio::write(port, boost::asio::buffer(recv_buffer_, s));
		//unsigned char commands[4] = {0, 1, 2, 3};
		//boost::asio::write(port, boost::asio::buffer(commands, 4));
		start_receive();
	}
}

void udp_server::handle_send(boost::shared_ptr<std::string> /*message*/,
    const boost::system::error_code& /*error*/,
    std::size_t /*bytes_transferred*/)
{
}