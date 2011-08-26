
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "udp_server.hpp"

using boost::asio::ip::udp;

udp_server::udp_server(boost::asio::io_service& io_service) : socket(io_service, udp::endpoint(boost::asio::ip::address_v4::loopback(), 15000)), port(boost::asio::serial_port(io_service, "COM1"))
{
	// Configure the options for the serial port. I was having problems with the default options for one
	// or more of these options, so I just declare them all to what I want.
	port.set_option(boost::asio::serial_port::baud_rate(115200));
    port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    port.set_option(boost::asio::serial_port::character_size(8));

	start_udp_receive();
	start_serial_receive();
}

void udp_server::start_udp_receive()
{
	socket.async_receive_from(
		boost::asio::buffer(udp_receive_buffer), udp::endpoint(boost::asio::ip::address_v4::loopback(), 16000),
		boost::bind(&udp_server::handle_udp_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void udp_server::start_serial_receive()
{

	boost::asio::async_read(port, boost::asio::buffer(serial_receive_buffer, 29), boost::bind(&udp_server::handle_serial_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_udp_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size) {
		// Asynchronouosly write all the UDP data to the serial port.
		boost::asio::async_write(port, boost::asio::buffer(udp_receive_buffer, bytes_transferred), boost::bind(&udp_server::handle_serial_send, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
		
		// Resume the waiting for UDP datagrams
		start_udp_receive();
	} else {
		std::cerr << error.message() << std::endl;
	}
}

void udp_server::handle_udp_send(const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
	if (error) {
		std::cerr << error.message() << std::endl;
	}
}

void udp_server::handle_serial_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size) {
		//std::cout << "\"" << serial_receive_buffer.at(0) << "\"" << std::endl;

		socket.async_send_to(boost::asio::buffer(serial_receive_buffer, bytes_transferred), udp::endpoint(boost::asio::ip::address_v4::loopback(), 16000),
          boost::bind(&udp_server::handle_udp_send, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));

		// Resume waiting for more serial data
		start_serial_receive();
	} else {
		std::cerr << error.message() << std::endl;
	}
}

void udp_server::handle_serial_send(const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
	if (error) {
		std::cerr << error.message() << std::endl;
	}
}