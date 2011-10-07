= serial2udp =

== Description ==

This project converts a serial port into a UDP port in software. It is used for the Autoboat project to avoid issues with the serial port on some platforms.

== Usage ==

This program is run on the command line and all CLI options are enumerated when the program is run without options. Specifying -h or --help at the command line will also list all options.
Be sure to specify a packet size for both the serial and UDP lines. This size will specify how many bytes the program waits for before triggering and interrupt on the serial side. On the UDP
side it must be larger than the largest UDP datagram size or the program may not work properly.

== Building ==

=== Setting up Boost ===
- Download the Boost library from boost.org
- Extract the library to a specified location
  - A subdirectory in serial2udp would work fine.
- Compile the boost library following step 5 in the "Getting Started with Boost in Windows" page.
  - Open a terminal in the Boost root directory
  - Execute "bootstrap"
  - Execute ".\b2 variant=release link=static threading=multi runtime-link=static"
  
=== Building the program ===
- Create a new MSVC++ 2010 project
- Modify the project to add the root Boost folder to the header path
  - Under Properties->C/C++->General->Additional Include Directories
- Modify the project to use the static non-debug version of the Boost library
  - Under Properties->C/C++->Code Generation->Runtime Library
- Modify the project to add the Boost libraries for linking
  - Under Properties->C/C++->Linker->General->Additional Library Directories
- ON WINDOWS: Set the version of Windows being compiled for to Windows XP
  - Add "-D_WIN32_WINNT=0x0501" to Properties->C/C++->Command Line->Additional Options