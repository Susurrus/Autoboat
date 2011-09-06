// Required by WIN32 compilation to specify the version of Windows being targeted. In this case Windows XP.
#define _WIN32_WINNT 0x0501

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>

// Set up some shorthand for namespaces
namespace po = boost::program_options;

// Default to the standard namespace
using namespace std;

// Initialized internal variables
boost::asio::ip::udp::socket *udpSocket;
boost::asio::serial_port *serialPort;

// Intermediate internal variables
boost::asio::ip::udp::endpoint remote_endpoint;
boost::array<unsigned char, 128> udp_receive_buffer;
boost::array<unsigned char, 128> serial_receive_buffer;

// Function prototypes
void start_udp_receive();
void start_serial_receive();
void handle_udp_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_udp_send(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_send(const boost::system::error_code &error, size_t bytes_transferred);
void print_error(const string message);

// Keep track of all of the command-line options
int opt_baud_rate;
string opt_port;
int opt_remote_socket;
string opt_remote_addr;
int opt_local_socket;
int opt_udp_packet_size;
int opt_serial_packet_size;

int main(int ac, char* av[])
{
	try {

		// Declare the supported options.
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "produce help message")
			("port", po::value<string>(&opt_port)->default_value("COM1"), "serial port to use")
			("baud_rate", po::value<int>(&opt_baud_rate)->default_value(115200), "set the serial port baud rate")
			("serial_packet_size", po::value<int>(&opt_serial_packet_size)->default_value(34), "size of the serial packets to be expected")
			("local_socket", po::value<int>(&opt_local_socket)->default_value(15000), "local UDP socket to use")
			("remote_socket", po::value<int>(&opt_remote_socket)->default_value(16000), "remote UDP socket to use")
			("remote_addr", po::value<string>(&opt_remote_addr)->default_value("127.0.0.1"), "remote IP address")
			("udp_packet_size", po::value<int>(&opt_udp_packet_size)->default_value(46), "size of a UDP datagram in bytes")
		;

		// Now parse the user-specified options
		po::variables_map vm;
		po::store(po::parse_command_line(ac, av, desc), vm);
		po::notify(vm);

		// If the help menu was specified, spit out the help text and quit.
		if (vm.count("help")) {
			cout << desc << "\n";
			return 1;
		}

		// Set up the endpoints. These are the local and remote server definitions (IP + socket)
		remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::from_string(opt_remote_addr), opt_remote_socket);

		// Start a Boost IO service and attach a new UDP socket and serial port to it.
		boost::asio::io_service io_service;

		udpSocket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::loopback(), opt_local_socket));
	
		serialPort = new boost::asio::serial_port(io_service, opt_port);

		// Configure the options for the serial port. I was having problems with the default options for one
		// or more of these options, so I just declare them all to what I want.
		serialPort->set_option(boost::asio::serial_port::baud_rate(opt_baud_rate));
		serialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
		serialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serialPort->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
		serialPort->set_option(boost::asio::serial_port::character_size(8));

		// Set up event handlers for receiving data on both ends
		start_udp_receive();
		start_serial_receive();

		// Then start everything running
		io_service.run();
	} catch (exception& e) {
		cerr << e.what() << endl;
	}

	return 0;
}

void start_udp_receive()
{
	udpSocket->async_receive_from(
		boost::asio::buffer(udp_receive_buffer, opt_udp_packet_size), 
		remote_endpoint,
		&handle_udp_receive
	);
}

void start_serial_receive()
{
	boost::asio::async_read(
		*serialPort,
		boost::asio::buffer(serial_receive_buffer, opt_serial_packet_size),
		&handle_serial_receive
	);
}

void handle_udp_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
	// Once a UDP datagram is received, send it out. If the reception was unsuccessful,
	// print an error message.
	if (!error || error == boost::asio::error::message_size) {
		boost::asio::async_write(
			*serialPort,
			boost::asio::buffer(udp_receive_buffer, bytes_transferred),
			&handle_serial_send
		);
	} else if (error != boost::asio::error::connection_refused) {
		print_error(error.message());
	}
		
	// Resume the waiting for UDP datagrams
	start_udp_receive();
}

void handle_udp_send(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}

void handle_serial_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
	// If there isn't an error or the error is just one about message size
	if (!error || error == boost::asio::error::message_size) {

		udpSocket->async_send_to(
			boost::asio::buffer(serial_receive_buffer, bytes_transferred),
			remote_endpoint,
			&handle_udp_send
		);
	} else if (error != boost::asio::error::connection_refused) {
		print_error(error.message());
	}

	// Resume waiting for more serial data
	start_serial_receive();
}

void handle_serial_send(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}

void print_error(const string message)
{
	ostringstream msg;
	const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
	cerr << boost::posix_time::to_simple_string(now) << " - " << message << endl;
}