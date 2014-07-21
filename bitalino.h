/**
 * \file
 * \copyright  Copyright 2014 PLUX - Wireless Biosignals, S.A.
 * \author     Filipe Silva
 * \version    1.1
 * \date       July 2014
 * 
 * \section LICENSE
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 */


 /**
  \mainpage
  
  The %BITalino C++ API (available at http://github.com/BITalinoWorld/cpp-api) is a cross-platform library which enables C++ applications to communicate
  with a %BITalino device through a simple interface.
  The API is composed of a header file (bitalino.h)
  and an implementation file ([bitalino.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.cpp)).
  A sample test application in C++ ([test.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/test.cpp)) is also provided.
  
  There are three ways to connect to a %BITalino device:
  - direct Bluetooth connection using the device Bluetooth MAC address (Windows and Linux);
  - indirect Bluetooth connection using a virtual serial port (all platforms);
  - wired UART connection using a serial port (all platforms).
  
  The API exposes a single class (BITalino). Each instance of this class represents a connection
  to a %BITalino device. The connection is established in the constructor and released in the destructor,
  thus following the RAII paradigm. An application can create several instances (to distinct devices).
  The library is thread-safe between distinct instances. Each instance can be a local variable
  (as in the sample application) or it can be allocated on the heap (using new and delete operators).
  
  \section sampleapp About the sample application
  
  The sample application ([test.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/test.cpp)) creates an instance to a %BITalino device.
  Then it starts acquiring all channels on the device at 1000 Hz and enters a loop while dumping
  one frame out of 100 and toggling the device green LED. Pressing the Enter key exits the loop,
  destroys the instance and closes the application.
  
  One of the provided constructor calls must be used to connect to the device.
  The string passed to the constructor can be a Bluetooth MAC address (you must change the one provided)
  or a serial port. The serial port string format depends on the platform.
  
  In order to have a more compact and readable code, the sample test code uses C++11 vector initializer lists.
  This new C++ feature is supported only in Visual Studio 2013 (on Windows), GCC 4.4 or later and Clang 3.1
  or later. If you are using an older compiler, use the commented alternative code for the `start()` and
  `trigger()` methods calls.
  
  \section windows Compiling on Windows
  
  The API was tested in Windows XP (32-bit) and Windows 7 (32-bit and 64-bit).
  
  To compile the library and the sample application:
  - create a C++ Empty Project in Visual Studio;
  - copy [bitalino.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.cpp),
  [bitalino.h](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.h) and
  [test.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/test.cpp) to the project directory;
  - add bitalino.cpp and test.cpp files to the project at the “Source Files” folder;
  - edit test.cpp as described \ref sampleapp "above";
  - add a reference to `ws2_32.lib` in Project Properties → Configuration Properties → Linker → Input → Additional Dependencies;
  - build the solution and run the application.
  
  \section linux Compiling on Linux

  The API was tested in Ubuntu (32-bit and 64-bit) and Raspberry Pi (Raspbian).
  
  To compile the library and the sample application:
  - `make` and `g++` must be installed;
  - packages `bluez`, `libbluetooth3` and `libbluetooth-dev` must be installed if you want to
    compile the library with Bluetooth functionality (to search for Bluetooth devices and
    to make direct Bluetooth connections);
  - copy [bitalino.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.cpp),
  [bitalino.h](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.h),
  [test.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/test.cpp) and
  [Makefile](http://github.com/BITalinoWorld/cpp-api/tree/master/Makefile) into a new directory;
  - if you want to compile the library without Bluetooth functionality, disable the line in
    Makefile where `LINUX_BT` is defined;
  - if your compiler doesn't support vector initializer lists, remove the flag `-std=c++0x`
    from the test.cpp compiling rule in Makefile;
  - edit test.cpp as described \ref sampleapp "above";
  - enter command `make` in the command line to build the library and the application;
  - enter command `./test` in the command line to run the application.
  
  \section macosx Compiling on Mac OS X
  
  The API was tested in Mac OS X 10.6 and 10.9.
  
  On Mac OS X, the %BITalino API Bluetooth functionality is not available, so it is only possible
  to connect to a %BITalino device through a serial port for indirect Bluetooth connections or for wired UART connections.
  
  To compile the library and the sample application:
  - copy [bitalino.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.cpp),
  [bitalino.h](http://github.com/BITalinoWorld/cpp-api/tree/master/bitalino.h),
  [test.cpp](http://github.com/BITalinoWorld/cpp-api/tree/master/test.cpp) and
  [Makefile](http://github.com/BITalinoWorld/cpp-api/tree/master/Makefile) into a new directory;
  - if your compiler doesn't support vector initializer lists, remove the flag `-std=c++0x` from
    the test.cpp compiling rule in Makefile;
  - edit test.cpp as described \ref sampleapp "above";
  - enter command `make` in the command line to build the library and the application;
  - enter command `./test` in the command line to run the application.  
  */

#ifndef _BITALINOHEADER_
#define _BITALINOHEADER_

#include <string>
#include <vector>

#ifdef _WIN32 // 32-bit or 64-bit Windows

#include <winsock2.h>

#endif

/**
 * The %BITalino device class.
 */
class BITalino
{
public:
// Type definitions

   typedef std::vector<bool>  Vbool;
   typedef std::vector<int>   Vint;

   struct DevInfo
   {
      std::string macAddr; ///< MAC address of a Bluetooth device
      std::string name;    ///< name of a Bluetooth device
   };
   typedef std::vector<DevInfo> VDevInfo;

   struct Frame
   {
      char  seq;        ///< Frame sequence number (0...15)
      bool  digital[4]; ///< Array of digital inputs states (false or true)
      short analog[6];  ///< Array of analog inputs values (0...1023 or 0...63)
   };
   typedef std::vector<Frame> VFrame;

   class Exception
   {
   public:
      enum Code
      {
         INVALID_ADDRESS = 1,       ///< The specified address is invalid
         BT_ADAPTER_NOT_FOUND,      ///< No Bluetooth adapter was found
         DEVICE_NOT_FOUND,          ///< The device could not be found
         CONTACTING_DEVICE,         ///< The computer lost communication with the device
         PORT_COULD_NOT_BE_OPENED,  ///< The communication port does not exist or it is already being used
         PORT_INITIALIZATION,       ///< The communication port could not be initialized
         DEVICE_NOT_IDLE,           ///< The device is not idle
         DEVICE_NOT_IN_ACQUISITION, ///< The device is not in acquisition mode
         INVALID_PARAMETER,         ///< Invalid parameter
      } code;

      Exception(Code c) : code(c) {}
      const char* getDescription(void);   ///< Returns an exception description string
   };

// Static methods

   /**
    * Searches for Bluetooth devices in range.
    * \return a list of found devices
    * \exception Exception (Exception::PORT_INITIALIZATION)
    * \exception Exception (Exception::BT_ADAPTER_NOT_FOUND)
    */
   static VDevInfo find(void);

// Instance methods

   /**
    * Connects to a %BITalino device.
    * \param[in] address The device Bluetooth MAC address ("xx:xx:xx:xx:xx:xx")
    * or a serial port ("COMx" on Windows or "/dev/..." on Linux or Mac OS X)
    * \exception Exception (Exception::PORT_COULD_NOT_BE_OPENED)
    * \exception Exception (Exception::PORT_INITIALIZATION)
    * \exception Exception (Exception::INVALID_ADDRESS)
    * \exception Exception (Exception::BT_ADAPTER_NOT_FOUND) - Windows only
    * \exception Exception (Exception::DEVICE_NOT_FOUND) - Windows only
    */
   BITalino(const char *address);
   
   /**
    * Disconnects from a %BITalino device. If an aquisition is running, it is stopped. 
    */
   ~BITalino();

   /**
    * Returns the device firmware version string.
    * \remarks This method cannot be called during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IDLE)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */
   std::string version(void);
   
   /**
    * Starts a signal acquisition from the device.
    * \param[in] samplingRate Sampling rate in Hz. Accepted values are 1, 10, 100 or 1000 Hz. Default value is 1000 Hz.
    * \param[in] channels Set of channels to acquire. Accepted channels are 0, 1, 2, 3, 4, and 5.
    * If this set is empty or if it is not given, all 6 analog channels will be acquired.
    * \param[in] simulated If true, start in simulated mode. Otherwise start in live mode. Default is to start in live mode.
    * \remarks This method cannot be called during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IDLE)
    * \exception Exception (Exception::INVALID_PARAMETER)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */
   void start(int samplingRate = 1000, const Vint &channels = Vint(), bool simulated = false);
   
   /**
    * Stops a signal acquisition.
    * \remarks This method must be called only during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */
   void stop(void);
   
   /**
    * Reads acquisition frames from the device.
    * This method does not return until all requested frames are received from the device.
    * \param[out] frames Vector of frames to be filled. If the vector is empty, it is resized to 100 frames.
    * \remarks This method must be called only during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */   
   void read(VFrame &frames);
   
   /**
    * Sets the battery voltage threshold for the low-battery LED.
    * \param[in] value Battery voltage threshold. Default value is 0.
    * Value | Voltage Threshold
    * ----- | -----------------
    *     0 |   3.4 V
    *  ...  |   ...
    *    63 |   3.8 V
    * \remarks This method cannot be called during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IDLE)
    * \exception Exception (Exception::INVALID_PARAMETER)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */
   void battery(int value = 0);
   
   /**
    * Assigns the digital outputs states.
    * \param[in] digitalOutput Vector of booleans to assign to digital outputs, starting at output 0.
    * This vector cannot contain more than 4 elements.
    * If the vector contains less than 4 elements, the remaining outputs are set to low state.
    * If this parameter is not given, all digital outputs are set to low state.
    * \remarks This method must be called only during an acquisition.
    * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
    * \exception Exception (Exception::INVALID_PARAMETER)
    * \exception Exception (Exception::CONTACTING_DEVICE)
    */
   void trigger(const Vbool &digitalOutput = Vbool());


private:
   void send(char cmd);
   void recv(void *data, int nbyttoread);
   void close(void);

   bool started;
   char nChannels;
#ifdef _WIN32
   SOCKET	s;
   timeval  readtimeout;
   HANDLE   hCom;
#else // Linux or Mac OS
   int      s, comm;
#endif
};

#endif // _BITALINOHEADER_
