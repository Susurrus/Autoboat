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
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include "udp_server.hpp"
#include "serial_server.hpp"

int main()
{
  try
  {
	// Start a Boost IO service.
    boost::asio::io_service io_service;

	// Create and attach a UDP server/client
	udp_server udpServ(io_service);

	// Then start everything running
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
