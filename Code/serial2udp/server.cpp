//
// timer.cpp
// ~~~~~~~~~
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

// Required by WIN32 compilation to specify the version of Windows being targeted. In this case Windows XP.
#define _WIN32_WINNT 0x0501

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Initialized internal variables
boost::asio::ip::udp::socket *udpSocket;
boost::asio::serial_port *serialPort;

// Intermediate internal variables
boost::asio::ip::udp::endpoint remote_endpoint;
boost::array<unsigned char, 46> udp_receive_buffer;
boost::array<unsigned char, 29> serial_receive_buffer;

// Function prototypes
void start_udp_receive();
void start_serial_receive();
void handle_udp_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_udp_send(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_send(const boost::system::error_code &error, size_t bytes_transferred);
void print_error(const std::string message);

int main()
{
	try {
		// Start a Boost IO service.
		boost::asio::io_service io_service;

		udpSocket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::loopback(), 15000));
	
		serialPort = new boost::asio::serial_port(io_service, "COM1");

		// Configure the options for the serial port. I was having problems with the default options for one
		// or more of these options, so I just declare them all to what I want.
		serialPort->set_option(boost::asio::serial_port::baud_rate(115200));
		serialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
		serialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serialPort->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
		serialPort->set_option(boost::asio::serial_port::character_size(8));

		start_udp_receive();
		start_serial_receive();

		// Then start everything running
		io_service.run();
	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}

	return 0;
}

void start_udp_receive()
{
	udpSocket->async_receive_from(
		boost::asio::buffer(udp_receive_buffer), 
		boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::loopback(), 16000),
		&handle_udp_receive
	);
}

void start_serial_receive()
{

	boost::asio::async_read(
		*serialPort,
		boost::asio::buffer(serial_receive_buffer, 29),
		&handle_serial_receive
	);
}

void handle_udp_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size) {
		// Asynchronouosly write all the UDP data to the serial port.
		boost::asio::async_write(
			*serialPort,
			boost::asio::buffer(udp_receive_buffer, bytes_transferred),
			&handle_serial_send
		);
		
		// Resume the waiting for UDP datagrams
		start_udp_receive();
	} else {
		print_error(error.message());
	}
}

void handle_udp_send(const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}

void handle_serial_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size) {
		//std::cout << "\"" << serial_receive_buffer.at(0) << "\"" << std::endl;

		udpSocket->async_send_to(
			boost::asio::buffer(serial_receive_buffer, bytes_transferred),
			boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::loopback(), 16000),
			&handle_udp_send
		);

		// Resume waiting for more serial data
		start_serial_receive();
	} else {
		print_error(error.message());
	}
}

void handle_serial_send(const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}

void print_error(const std::string message)
{
	std::ostringstream msg;
	const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
	std::cout << boost::posix_time::to_simple_string(now) << " - " << message << std::endl;
}