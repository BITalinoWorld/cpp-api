/**
 * \copyright  Copyright 2014-2015 PLUX - Wireless Biosignals, S.A.
 * \author     Filipe Silva
 * \version    1.1a
 * \date       March 2015
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

#ifdef _WIN32 // 32-bit or 64-bit Windows

#define HASBLUETOOTH

#include <winsock2.h>
#include <ws2bth.h>

#else // Linux or Mac OS

#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef HASBLUETOOTH  // Linux only

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <stdlib.h>

#endif // HASBLUETOOTH

void Sleep(int millisecs)
{
   usleep(millisecs*1000);
}

#endif // Linux or Mac OS


#include "bitalino.h"

// Public methods

BITalino::VDevInfo BITalino::find(void)
{
   VDevInfo devs;
   DevInfo  devInfo;

#ifdef _WIN32
   char     addrStr[40];
	WSADATA  m_data;

   if (WSAStartup(0x202, &m_data) != 0)	throw Exception(Exception::PORT_INITIALIZATION);

  WSAQUERYSETA querySet;
  ZeroMemory(&querySet, sizeof querySet);
  querySet.dwSize = sizeof(querySet);
  querySet.dwNameSpace = NS_BTH;
  
  HANDLE hLookup;
  DWORD flags = LUP_CONTAINERS | LUP_RETURN_ADDR | LUP_RETURN_NAME | LUP_FLUSHCACHE;
  bool tryempty = true;
  bool again;

  do
  {
	  again = false;
     if (WSALookupServiceBeginA(&querySet, flags, &hLookup) != 0)
     {
        WSACleanup();
        throw Exception(Exception::BT_ADAPTER_NOT_FOUND);
     }
  
	  while (1)
     {
        BYTE buffer[1500];
        DWORD bufferLength = sizeof(buffer);
        WSAQUERYSETA *pResults = (WSAQUERYSETA*)&buffer;
        if (WSALookupServiceNextA(hLookup, flags, &bufferLength, pResults) != 0)	break;
        if (pResults->lpszServiceInstanceName[0] == 0 && tryempty)
        {  // empty name : may happen on the first inquiry after the device was connected
           tryempty = false;   // redo the inquiry a second time only (there may be a device with a real empty name)
           again = true;
			  break;
        }

        DWORD strSiz = sizeof addrStr;
        if (WSAAddressToStringA(pResults->lpcsaBuffer->RemoteAddr.lpSockaddr, pResults->lpcsaBuffer->RemoteAddr.iSockaddrLength,
                                NULL, addrStr, &strSiz) == 0)
        {
           addrStr[strlen(addrStr)-1] = 0;   // remove trailing ')'
           devInfo.macAddr = addrStr+1;   // remove leading '('
           devInfo.name = pResults->lpszServiceInstanceName;
           devs.push_back(devInfo);
	     }
	  }

	  WSALookupServiceEnd(hLookup);
  } while (again);

  WSACleanup();

#else // Linux or Mac OS

#ifdef HASBLUETOOTH
    
    #define MAX_DEVS 255

    int dev_id = hci_get_route(NULL);
    int sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0)
      throw Exception(Exception::PORT_INITIALIZATION);

    inquiry_info ii[MAX_DEVS];
    inquiry_info *pii = ii;

    int num_rsp = hci_inquiry(dev_id, 8, MAX_DEVS, NULL, &pii, IREQ_CACHE_FLUSH);
    if(num_rsp < 0)
    {
      ::close(sock);
      throw Exception(Exception::PORT_INITIALIZATION);
    }

    for (int i = 0; i < num_rsp; i++)
    {
        char addr[19], name[248];

        ba2str(&ii[i].bdaddr, addr);
        if (hci_read_remote_name(sock, &ii[i].bdaddr, sizeof name, name, 0) >= 0)
        {
           devInfo.macAddr = addr;
           devInfo.name = name;
           devs.push_back(devInfo);        
        }
    }

    ::close(sock);
    if (pii != ii)   free(pii);
   
#else
   
   throw Exception(Exception::BT_ADAPTER_NOT_FOUND);
   
#endif // HASBLUETOOTH
   
#endif // Linux or Mac OS

    return devs;
}

BITalino::BITalino(const char *address) : started(false)
{
#ifdef _WIN32
   if (_memicmp(address, "COM", 3) == 0)
   {
      s = INVALID_SOCKET;

	   char xport[40] = "\\\\.\\";   // preppend "\\.\"

	   strcat_s(xport, 40, address);

	   hCom = CreateFileA(xport,  // comm port name
					   GENERIC_READ | GENERIC_WRITE,
					   0,      // comm devices must be opened w/exclusive-access 
					   NULL,   // no security attributes 
					   OPEN_EXISTING, // comm devices must use OPEN_EXISTING 
					   0,      // not overlapped I/O 
					   NULL);  // hTemplate must be NULL for comm devices 

      if (hCom == INVALID_HANDLE_VALUE)
         throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);

      DCB dcb;
      if (!GetCommState(hCom, &dcb))
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }
      dcb.BaudRate = CBR_115200;
      dcb.fBinary = TRUE;
      dcb.fParity = FALSE;
      dcb.fOutxCtsFlow = FALSE;
      dcb.fOutxDsrFlow = FALSE;
      dcb.fDtrControl = DTR_CONTROL_DISABLE;
      dcb.fDsrSensitivity = FALSE;
      dcb.fOutX = FALSE;
      dcb.fInX = FALSE;
      dcb.fNull = FALSE;
      dcb.fRtsControl = RTS_CONTROL_DISABLE;
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
      if (!SetCommState(hCom, &dcb))
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }

	   COMMTIMEOUTS ct;
	   ct.ReadIntervalTimeout         = 0;
	   ct.ReadTotalTimeoutConstant    = 5000; // 5 s
	   ct.ReadTotalTimeoutMultiplier  = 0;
	   ct.WriteTotalTimeoutConstant   = 5000; // 5 s
	   ct.WriteTotalTimeoutMultiplier = 0;

	   if (!SetCommTimeouts(hCom, &ct)) 
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }
   }
   else // address is a Bluetooth MAC address
   {
      hCom = INVALID_HANDLE_VALUE;

      WSADATA m_data;
      if (WSAStartup(0x202, &m_data) != 0)
         throw Exception(Exception::PORT_INITIALIZATION);

      SOCKADDR_BTH so_bt;
      int siz = sizeof so_bt;
      if (WSAStringToAddressA((LPSTR)address, AF_BTH, NULL, (sockaddr*)&so_bt, &siz) != 0)
      {
         WSACleanup();
         throw Exception(Exception::INVALID_ADDRESS);
      }
      so_bt.port = 1;

      s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
      if (s == INVALID_SOCKET)
      {
         WSACleanup();
         throw Exception(Exception::PORT_INITIALIZATION);
      }

      if (connect(s, (const sockaddr*)&so_bt, sizeof so_bt) != 0)
      {
         int err = WSAGetLastError();
         close();

         switch(err)
         {
         case WSAENETDOWN:
            throw Exception(Exception::BT_ADAPTER_NOT_FOUND);

         case WSAETIMEDOUT:
            throw Exception(Exception::DEVICE_NOT_FOUND);

         default:
            throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
         }
      }

      readtimeout.tv_sec = 5;
      readtimeout.tv_usec = 0;
   }

#else // Linux or Mac OS

   if (memcmp(address, "/dev/", 5) == 0)
   {
      s = -1;
   
      comm = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
      if (comm < 0)
		   throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
      
      if (fcntl(comm, F_SETFL, 0) == -1)  // remove the O_NDELAY flag
      {
         close();
		   throw Exception(Exception::PORT_INITIALIZATION);
      }
   
      termios term;
      if (tcgetattr(comm, &term) != 0)
      {
         close();
		   throw Exception(Exception::PORT_INITIALIZATION);
      }
   
      cfmakeraw(&term);
      term.c_oflag &= ~(OPOST);
   
      term.c_cc[VMIN] = 1;
      term.c_cc[VTIME] = 1;
   
      term.c_iflag &= ~(INPCK | PARMRK | ISTRIP | IGNCR | ICRNL | INLCR | IXON | IXOFF | IMAXBEL); // no flow control
      term.c_iflag |= (IGNPAR | IGNBRK);
   
      term.c_cflag &= ~(CRTSCTS | PARENB | CSTOPB | CSIZE); // no parity, 1 stop bit
      term.c_cflag |= (CLOCAL | CREAD | CS8);    // raw mode, 8 bits
   
      term.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOPRT | ECHOK | ECHOKE | ECHONL | ECHOCTL | ISIG | IEXTEN | TOSTOP);  // raw mode
   
      if (cfsetspeed(&term, B115200) != 0)
      {
         close();
		   throw Exception(Exception::PORT_INITIALIZATION);
      }
   
      if (tcsetattr(comm, TCSANOW, &term) != 0)
      {
         close();
		   throw Exception(Exception::PORT_INITIALIZATION);
      }   
   }
   else // address is a Bluetooth MAC address
#ifdef HASBLUETOOTH
   {
      comm = -1;

      sockaddr_rc so_bt;
      so_bt.rc_family = AF_BLUETOOTH;
      if (str2ba(address, &so_bt.rc_bdaddr) < 0)
         throw Exception(Exception::INVALID_ADDRESS);
         
      so_bt.rc_channel = 1;

      s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
      if (s < 0)
         throw Exception(Exception::PORT_INITIALIZATION);

      if (connect(s, (const sockaddr*)&so_bt, sizeof so_bt) != 0)
      {
         close();
         throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
      }
   }
#else
      throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
#endif // HASBLUETOOTH

#endif // Linux or Mac OS
}

BITalino::~BITalino(void)
{
   try
   {
      if (started) stop();
   }
   catch (Exception) {} // if stop() fails, close anyway

   close();
}

std::string BITalino::version(void)
{
   if (started)   throw Exception(Exception::DEVICE_NOT_IDLE);
   
   const char *header = "BITalino";
   
   const int headerLen = strlen(header);

   // send "get version string" command to device
   send(7);
   
   std::string str;
   while(1)
   {
      char chr;
      recv(&chr, sizeof chr);
      const int len = str.size();
      if (len >= headerLen)
      {
         if (chr == '\n')  return str;
         str.push_back(chr);
      }
      else
         if (chr == header[len])
            str.push_back(chr);
         else
         {
            str.clear();   // discard all data before version header
            if (chr == header[0])   str.push_back(chr);
         }
   }
}

void BITalino::start(int samplingRate, const Vint &channels, bool simulated)
{
   if (started)   throw Exception(Exception::DEVICE_NOT_IDLE);

   char cmd;
   switch (samplingRate)
   {
   case 1:
      cmd = 0;
      break;
   case 10:
      cmd = 1;
      break;
   case 100:
      cmd = 2;
      break;
   case 1000:
      cmd = 3;
      break;
   default:
      throw Exception(Exception::INVALID_PARAMETER);
   }

   char chMask;
   if (channels.empty())
   {
      chMask = 0x3F;    // all 6 analog channels
      nChannels = 6;
   }
   else
   {
      chMask = 0;
      nChannels = 0;
      for(Vint::const_iterator it = channels.begin(); it != channels.end(); it++)
      {
         int ch = *it;
         if (ch < 0 || ch > 5)   throw Exception(Exception::INVALID_PARAMETER);
         const char mask = 1 << ch;
         if (chMask & mask)   throw Exception(Exception::INVALID_PARAMETER);
         chMask |= mask;
         nChannels++;
      }
   }

   // send "set samplerate" command to device
   send((cmd << 6) | 0x03);

   // send "start" command to device
   send((chMask << 2) | (simulated ? 0x02 : 0x01));

   started = true;
}

void BITalino::stop(void)
{
   if (!started)   throw Exception(Exception::DEVICE_NOT_IN_ACQUISITION);

   // send "stop" command to device
   send(0);

   started = false;

   version();  // to flush pending frames in input buffer
}

void BITalino::read(VFrame &frames)
{
   if (!started)   throw Exception(Exception::DEVICE_NOT_IN_ACQUISITION);

   unsigned char buffer[8]; // frame maximum size is 8 bytes

   if (frames.empty())   frames.resize(100);

   char nBytes = nChannels + 2;
   if (nChannels >= 3 && nChannels <= 5)  nBytes++;

   for(VFrame::iterator it = frames.begin(); it != frames.end(); it++)
   {
      recv(buffer, nBytes);

      // check CRC
      unsigned char crc = buffer[nBytes-1] & 0x0F;
      buffer[nBytes-1] &= 0xF0;  // clear CRC bits in frame
      unsigned char x = 0;
      for(char i = 0; i < nBytes; i++)
         for(signed char bit = 7; bit >= 0; bit--)
         {
            x <<= 1;
            if (x & 0x10)  x ^= 0x03;
            x ^= ((buffer[i] >> bit) & 0x01);
         }

      if (crc != (x & 0x0F))  throw Exception(Exception::CONTACTING_DEVICE);

      Frame &f = *it;
      f.seq = buffer[nBytes-1] >> 4;
      for(char i = 0; i < 4; i++)
         f.digital[i] = ((buffer[nBytes-2] & (0x80 >> i)) != 0);

      f.analog[0] = (short(buffer[nBytes-2] & 0x0F) << 6) | (buffer[nBytes-3] >> 2);
      if (nChannels > 1)
         f.analog[1] = (short(buffer[nBytes-3] & 0x03) << 8) | buffer[nBytes-4];
      if (nChannels > 2)
         f.analog[2] = (short(buffer[nBytes-5]) << 2) | (buffer[nBytes-6] >> 6);
      if (nChannels > 3)
         f.analog[3] = (short(buffer[nBytes-6] & 0x3F) << 4) | (buffer[nBytes-7] >> 4);
      if (nChannels > 4)
         f.analog[4] = ((buffer[nBytes-7] & 0x0F) << 2) | (buffer[nBytes-8] >> 6);
      if (nChannels > 5)
         f.analog[5] = buffer[nBytes-8] & 0x3F;
   }
}

void BITalino::battery(int value)
{
   if (started)   throw Exception(Exception::DEVICE_NOT_IDLE);

   if (value < 0 || value > 63)   throw Exception(Exception::INVALID_PARAMETER);

   // send "set battery" command to device
   send(value << 2);

}

void BITalino::trigger(const Vbool &digitalOutput)
{
   if (!started)   throw Exception(Exception::DEVICE_NOT_IN_ACQUISITION);

   const int len = digitalOutput.size();

   if (len > 4)   throw Exception(Exception::INVALID_PARAMETER);

   char cmd = 0;
   for (int i = 0; i < len; i++)
      if (digitalOutput[i])
         cmd |= 1 << i;

   // send "set digital output" command to device
   send((cmd << 2) | 0x03);
}

const char* BITalino::Exception::getDescription(void)
{
	switch (code)
   {
		case INVALID_ADDRESS:
			return "The specified address is invalid.";

		case BT_ADAPTER_NOT_FOUND:
			return "No Bluetooth adapter was found.";

		case DEVICE_NOT_FOUND:
			return "The device could not be found.";

		case CONTACTING_DEVICE:
			return "The computer lost communication with the device.";

		case PORT_COULD_NOT_BE_OPENED:
			return "The communication port does not exist or it is already being used.";

		case PORT_INITIALIZATION:
			return "The communication port could not be initialized.";

		case DEVICE_NOT_IDLE:
			return "The device is not idle.";
			
		case DEVICE_NOT_IN_ACQUISITION:
	        return "The device is not in acquisition mode.";
		
		case INVALID_PARAMETER:
			return "Invalid parameter.";

		default:
			return "Unknown error.";
	}
}

// Private methods

void BITalino::send(char cmd)
{
   Sleep(150);

#ifdef _WIN32
   if (s == INVALID_SOCKET)
   {
      DWORD nbytwritten = 0;
	   if (!WriteFile(hCom, &cmd, sizeof cmd, &nbytwritten, NULL))
 		   throw Exception(Exception::CONTACTING_DEVICE);

      if (nbytwritten != sizeof cmd)
 		   throw Exception(Exception::CONTACTING_DEVICE);
 		
      return;
   }
   
#else // Linux or Mac OS

   if (s == -1)
   {
      if (write(comm, &cmd, sizeof cmd) != sizeof cmd)
         throw Exception(Exception::CONTACTING_DEVICE);
      
      return;
   }
#endif

#ifdef HASBLUETOOTH
   if (::send(s, &cmd, sizeof cmd, 0) != sizeof cmd)
      throw Exception(Exception::CONTACTING_DEVICE);
#endif
}

void BITalino::recv(void *data, int nbyttoread)
{
#ifdef _WIN32
   if (s == INVALID_SOCKET)
   {
      for(int n = 0; n < nbyttoread;)
      {
         DWORD nbytread = 0;
	      if (!ReadFile(hCom, (char *) data+n, nbyttoread-n, &nbytread, NULL))
 		      throw Exception(Exception::CONTACTING_DEVICE);

         if (nbytread == 0)
 		      throw Exception(Exception::CONTACTING_DEVICE);

         n += nbytread;
      }

	   return;
   }
#endif

   fd_set   readfds;
   FD_ZERO(&readfds);
#ifdef _WIN32
   FD_SET(s, &readfds);
#else // Linux or Mac OS
   FD_SET((s == -1) ? comm : s, &readfds);

   timeval  readtimeout;
   readtimeout.tv_sec = 5;
   readtimeout.tv_usec = 0;
#endif

   for(int n = 0; n < nbyttoread;)
   {
      int state = select(FD_SETSIZE, &readfds, NULL, NULL, &readtimeout);
      if(state <= 0)	 throw Exception(Exception::CONTACTING_DEVICE);
      int ret =
#ifndef _WIN32 // Linux or Mac OS
         (s == -1) ? ::read(comm, (char *) data+n, nbyttoread-n) :
#endif
#ifdef HASBLUETOOTH
                     ::recv(s, (char *) data+n, nbyttoread-n, 0);
#else
                     0;
#endif
      if(ret <= 0)   throw Exception(Exception::CONTACTING_DEVICE);
      n += ret;
   }
}

void BITalino::close(void)
{
#ifdef _WIN32
   if (s == INVALID_SOCKET)
      CloseHandle(hCom);
   else
   {
      closesocket(s);
      WSACleanup();
   }
   
#else // Linux or Mac OS

   ::close((s == -1) ? comm : s);
#endif
}

