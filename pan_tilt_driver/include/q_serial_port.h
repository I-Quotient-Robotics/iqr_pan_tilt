/*******************************************************************************
** MIT License
**
** Copyright(c) 2019 QuartzYan https://github.com/QuartzYan
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files(the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions :
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*******************************************************************************/

#ifndef QSERIALPORT_H
#define QSERIALPORT_H

#include <iostream>

#if defined(_WIN32)
#include <windows.h>
#else
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <linux/serial.h>
#endif

class QSerialPort
{
#if defined(_WIN32)
  typedef void* Handle;
#else
  typedef int Handle;
  typedef uint8_t byte;
#endif

public:

  enum BaudRate 
  {
    Baud1200 = 1200,
    Baud2400 = 2400,
    Baud4800 = 4800,
    Baud9600 = 9600,
    Baud19200 = 19200,
    Baud38400 = 38400,
    Baud57600 = 57600,
    Baud115200 = 115200,
    Baud256000 = 256000,
    UnknownBaud = -1
  };

  enum DataBits 
  {
    Data5 = 5,
    Data6 = 6,
    Data7 = 7,
    Data8 = 8,
    UnknownDataBits = -1
  };

  enum Parity 
  {
    NoParity = 0,
    EvenParity = 2,
    OddParity = 3,
    SpaceParity = 4,
    MarkParity = 5,
    UnknownParity = -1
  };

  enum StopBits 
  {
    OneStop = 1,
    OneAndHalfStop = 3,
    TwoStop = 2,
    UnknownStopBits = -1
  };

  enum FlowControl 
  {
    NoFlowControl,
    HardwareControl,
    SoftwareControl,
    UnknownFlowControl = -1
  };

  enum SerialPortError 
  {
    NoError,
    DeviceNotFoundError,
    PermissionError,
    OpenError,
    ParityError,
    FramingError,
    BreakConditionError,
    WriteError,
    ReadError,
    ResourceError,
    UnsupportedOperationError,
    UnknownError,
    TimeoutError,
    NotOpenError
  };

  enum OpenMode
  {
    ReadOnly = 0x00001,
    WriteOnly = 0x00002,
    ReadAndWrite = ReadOnly | WriteOnly
  };

  QSerialPort(const std::string &portName);
  QSerialPort(const std::string &portName, BaudRate baudRate, DataBits dataBits, StopBits stopBits, Parity parity, FlowControl flowControl);
  ~QSerialPort();

  bool open(OpenMode mode);
  void close();

  bool setBaudRate(QSerialPort::BaudRate baudRate);
  bool setDataBits(QSerialPort::DataBits dataBits);
  bool setParity(QSerialPort::Parity parity);
  bool setStopBits(QSerialPort::StopBits stopBits);
  bool setFlowControl(QSerialPort::FlowControl flowControl);

  bool isOpen() const { return _openFlage; };
  SerialPortError getError() const { return _error; };

  int64_t read(byte *data, uint64_t length);
  int64_t write(const byte *data, uint64_t maxSize);
  

protected:
  bool initialize(OpenMode mode);
  inline void setError(const SerialPortError &errorInfo);
  SerialPortError getSystemError(int systemErrorCode = -1) const;

private:
  std::string _portName;
  QSerialPort::BaudRate _baudRate = QSerialPort::Baud9600;
  QSerialPort::DataBits _dataBits = QSerialPort::Data8;
  QSerialPort::Parity _parity = QSerialPort::NoParity;
  QSerialPort::StopBits _stopBits = QSerialPort::OneStop;
  QSerialPort::FlowControl _flowControl = QSerialPort::NoFlowControl;

  bool _openFlage = false;// TODO:
  SerialPortError _error = SerialPortError::NoError;

#if defined(_WIN32)
  HANDLE _handle = INVALID_HANDLE_VALUE;
  DCB _restoredDcb;
  COMMTIMEOUTS _currentCommTimeouts;
  COMMTIMEOUTS _restoredCommTimeouts;

  bool setDcb(DCB *dcb);
  bool getDcb(DCB *dcb);
#else
  Handle _handle = -1;
  struct termios _restoredTermios;

  bool setTermios(const termios *tio);
  bool getTermios(termios *tio);
#endif

};

#endif //QSERIALPORT_H