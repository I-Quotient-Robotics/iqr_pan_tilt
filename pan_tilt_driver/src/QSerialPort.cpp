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

#include "QSerialPort.h"

#if defined(_WIN32)

static inline void set_common_props(DCB *dcb)
{
  dcb->fBinary = TRUE;
  dcb->fAbortOnError = FALSE;
  dcb->fNull = FALSE;
  dcb->fErrorChar = FALSE;

  if (dcb->fDtrControl == DTR_CONTROL_HANDSHAKE)
    dcb->fDtrControl = DTR_CONTROL_DISABLE;

  if (dcb->fRtsControl != RTS_CONTROL_HANDSHAKE)
    dcb->fRtsControl = RTS_CONTROL_DISABLE;
}

static inline void set_baudrate(DCB *dcb, uint32_t baudrate)
{
  dcb->BaudRate = baudrate;
}

static inline void set_databits(DCB *dcb, QSerialPort::DataBits databits)
{
  dcb->ByteSize = databits;
}

static inline void set_parity(DCB *dcb, QSerialPort::Parity parity)
{
  dcb->fParity = TRUE;
  switch (parity)
  {
  case QSerialPort::NoParity:
    dcb->Parity = NOPARITY;
    dcb->fParity = FALSE;
    break;
  case QSerialPort::OddParity:
    dcb->Parity = ODDPARITY;
    break;
  case QSerialPort::EvenParity:
    dcb->Parity = EVENPARITY;
    break;
  case QSerialPort::MarkParity:
    dcb->Parity = MARKPARITY;
    break;
  case QSerialPort::SpaceParity:
    dcb->Parity = SPACEPARITY;
    break;
  default:
    dcb->Parity = NOPARITY;
    dcb->fParity = FALSE;
    break;
  }
}

static inline void set_stopbits(DCB *dcb, QSerialPort::StopBits stopbits)
{
  switch (stopbits)
  {
  case QSerialPort::OneStop:
    dcb->StopBits = ONESTOPBIT;
    break;
  case QSerialPort::OneAndHalfStop:
    dcb->StopBits = ONE5STOPBITS;
    break;
  case QSerialPort::TwoStop:
    dcb->StopBits = TWOSTOPBITS;
    break;
  default:
    dcb->StopBits = ONESTOPBIT;
    break;
  }
}

static inline void set_flowcontrol(DCB *dcb, QSerialPort::FlowControl flowcontrol)
{
  dcb->fInX = FALSE;
  dcb->fOutX = FALSE;
  dcb->fOutxCtsFlow = FALSE;
  if (dcb->fRtsControl == RTS_CONTROL_HANDSHAKE)
    dcb->fRtsControl = RTS_CONTROL_DISABLE;
  switch (flowcontrol)
  {
  case QSerialPort::NoFlowControl:
    break;
  case QSerialPort::SoftwareControl:
    dcb->fInX = TRUE;
    dcb->fOutX = TRUE;
    break;
  case QSerialPort::HardwareControl:
    dcb->fOutxCtsFlow = TRUE;
    dcb->fRtsControl = RTS_CONTROL_HANDSHAKE;
    break;
  default:
    break;
  }
}
#else
static inline void set_common_props(termios *tio, QSerialPort::OpenMode m)
{
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation
  tio->c_iflag &= ~(IGNBRK | BRKINT | ICRNL |INLCR);

  // enable rx and tx
  tio->c_cflag |= (CLOCAL | CREAD);

  // disable 0x0d -> 0x0a
  tio->c_iflag &= ~ (INLCR | ICRNL | IGNCR);
  tio->c_oflag &= ~(ONLCR | OCRNL);

  tio->c_lflag &= ~ (ICANON | ECHO | ECHOE | ISIG);

  // raw input mode
  tio->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  // raw output mode
  tio->c_oflag &= ~OPOST;

  tio->c_cc[VTIME] = 0;
  tio->c_cc[VMIN] = 0;
}

static inline void set_baudrate(termios *tio, QSerialPort::BaudRate baudrate)
{
  switch (baudrate)
  {
  case QSerialPort::Baud1200:
    cfsetispeed(tio, B1200);
    cfsetospeed(tio, B1200);
    break;
  case QSerialPort::Baud2400:
    cfsetispeed(tio, B2400);
    cfsetospeed(tio, B2400);
    break;
  case QSerialPort::Baud4800:
    cfsetispeed(tio, B4800);
    cfsetospeed(tio, B4800);
    break;
  case QSerialPort::Baud9600:
    cfsetispeed(tio, B9600);
    cfsetospeed(tio, B9600);
    break;
  case QSerialPort::Baud19200:
    cfsetispeed(tio, B19200);
    cfsetospeed(tio, B19200);
    break;
  case QSerialPort::Baud38400:
    cfsetispeed(tio, B38400);
    cfsetospeed(tio, B38400);
    break;
  case QSerialPort::Baud57600:
    cfsetispeed(tio, B57600);
    cfsetospeed(tio, B57600);
    break;
  case QSerialPort::Baud115200:
    cfsetispeed(tio, B115200);
    cfsetospeed(tio, B115200);
    break;
  default:
    cfsetispeed(tio, B9600);
    cfsetospeed(tio, B9600);
    break;
  }
}

static inline void set_databits(termios *tio, QSerialPort::DataBits databits)
{
  tio->c_cflag &= ~CSIZE;
  switch (databits)
  {
  case QSerialPort::Data5:
    tio->c_cflag |= CS5;
    break;
  case QSerialPort::Data6:
    tio->c_cflag |= CS6;
    break;
  case QSerialPort::Data7:
    tio->c_cflag |= CS7;
    break;
  case QSerialPort::Data8:
    tio->c_cflag |= CS8;
    break;
  default:
    tio->c_cflag |= CS8;
    break;
  }
}

static inline void set_parity(termios *tio, QSerialPort::Parity parity)
{
  tio->c_iflag &= ~(PARMRK | INPCK);
  tio->c_iflag |= IGNPAR;

  switch (parity)
  {
#ifdef CMSPAR
    // Here Installation parity only for GNU/Linux where the macro CMSPAR.
  case QSerialPort::SpaceParity:
    tio->c_cflag &= ~PARODD;
    tio->c_cflag |= PARENB | CMSPAR;
    break;
  case QSerialPort::MarkParity:
    tio->c_cflag |= PARENB | CMSPAR | PARODD;
    break;
#endif
  case QSerialPort::NoParity:
    tio->c_cflag &= ~PARENB;
    break;
  case QSerialPort::EvenParity:
    tio->c_cflag &= ~PARODD;
    tio->c_cflag |= PARENB;
    break;
  case QSerialPort::OddParity:
    tio->c_cflag |= PARENB | PARODD;
    break;
  default:
    tio->c_cflag |= PARENB;
    tio->c_iflag |= PARMRK | INPCK;
    tio->c_iflag &= ~IGNPAR;
    break;
  }
}

static inline void set_stopbits(termios *tio, QSerialPort::StopBits stopbits)
{
  switch (stopbits)
  {
  case QSerialPort::OneStop:
    tio->c_cflag &= ~CSTOPB;
    break;
  case QSerialPort::TwoStop:
    tio->c_cflag |= CSTOPB;
    break;
  default:
    tio->c_cflag &= ~CSTOPB;
    break;
  }
}

static inline void set_flowcontrol(termios *tio, QSerialPort::FlowControl flowcontrol)
{
  switch (flowcontrol)
  {
  case QSerialPort::NoFlowControl:
    tio->c_cflag &= ~CRTSCTS;
    tio->c_iflag &= ~(IXON | IXOFF | IXANY);
    break;
  case QSerialPort::HardwareControl:
    tio->c_cflag |= CRTSCTS;
    tio->c_iflag &= ~(IXON | IXOFF | IXANY);
    break;
  case QSerialPort::SoftwareControl:
    tio->c_cflag &= ~CRTSCTS;
    tio->c_iflag |= IXON | IXOFF | IXANY;
    break;
  default:
    tio->c_cflag &= ~CRTSCTS;
    tio->c_iflag &= ~(IXON | IXOFF | IXANY);
    break;
  }
}
#endif

QSerialPort::QSerialPort(const std::string &portName)
{
  _portName = portName;
  _openFlage = false;
}

QSerialPort::QSerialPort(const std::string &portName, BaudRate baudRate, DataBits dataBits, StopBits stopBits, Parity parity, FlowControl flowControl)
{
  _portName = portName;
  _baudRate = baudRate;
  _dataBits = dataBits;
  _stopBits = stopBits;
  _parity = parity;
  _flowControl = flowControl;
  _openFlage = false;
}

QSerialPort::~QSerialPort()
{
  close();
}

bool QSerialPort::open(OpenMode mode)
{
#if defined(_WIN32)
  DWORD desiredAccess = 0;

  if (mode & QSerialPort::ReadOnly)
    desiredAccess |= GENERIC_READ;
  if (mode & QSerialPort::WriteOnly)
    desiredAccess |= GENERIC_WRITE;

  _handle = ::CreateFile(&_portName[0], desiredAccess, 0, nullptr, OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL, nullptr);

  //SetupComm(_handle, 4096, 4096);

  if (_handle == INVALID_HANDLE_VALUE)
  {
    setError(getSystemError());
    return false;
  }

  if (initialize(mode))
  {
    _openFlage = true;
    return true;
  }

  ::CloseHandle(_handle);
  return false;
#else
  int flags = O_NOCTTY | O_NONBLOCK;

  switch (mode & QSerialPort::ReadAndWrite)
  {
  case QSerialPort::WriteOnly:
    flags |= O_WRONLY;
    break;
  case QSerialPort::ReadAndWrite:
    flags |= O_RDWR;
    break;
  default:
    flags |= O_RDONLY;
    break;
  }

  _handle = ::open(&_portName[0], flags);//O_RDWR | O_NOCTTY | O_NDELAY);

  if (_handle == -1)
  {
    setError(getSystemError());
    return false;
  }

  if (!initialize(mode))
  {
    ::close(_handle);
    return false;
  }
  _openFlage = true;
  return true;
#endif
}

bool QSerialPort::initialize(OpenMode mode)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;

  _restoredDcb = dcb;

  set_common_props(&dcb);
  set_baudrate(&dcb, _baudRate);
  set_databits(&dcb, _dataBits);
  set_parity(&dcb, _parity);
  set_stopbits(&dcb, _stopBits);
  set_flowcontrol(&dcb, _flowControl);

  if (!setDcb(&dcb))
    return false;
  if (!::GetCommTimeouts(_handle, &_restoredCommTimeouts))
  {
    setError(getSystemError());
    return false;
  }
  ::ZeroMemory(&_currentCommTimeouts, sizeof(_currentCommTimeouts));
  _currentCommTimeouts.ReadIntervalTimeout = MAXDWORD;
  if (!::SetCommTimeouts(_handle, &_currentCommTimeouts))
  {
    setError(getSystemError());
    return false;
  }
  const DWORD eventMask = (mode & QSerialPort::ReadOnly) ? EV_RXCHAR : 0;
  if (!::SetCommMask(_handle, eventMask))
  {
    setError(getSystemError());
    return false;
  }
  return true;
#else
  termios tio;
  if (!getTermios(&tio))
  {
    return false;
  }
  _restoredTermios = tio;

	::ioctl(_handle, TCIFLUSH);

  set_common_props(&tio, mode);
  set_baudrate(&tio, _baudRate);
  set_databits(&tio, _dataBits);
  set_parity(&tio, _parity);
  set_stopbits(&tio, _stopBits);
  set_flowcontrol(&tio, _flowControl);

  if (!setTermios(&tio))
  {
    return false;
  }

  return true;
#endif
}

void QSerialPort::close()
{
  _openFlage = false;
#if defined(_WIN32)
  ::CancelIo(_handle);

  //if (0) {
  //::SetCommState(handle, &restoredDcb);
  //::SetCommTimeouts(handle, &restoredCommTimeouts);
  //}

  ::CloseHandle(_handle);
  _handle = INVALID_HANDLE_VALUE;
#else

  ::close(_handle);
#endif
}

int64_t QSerialPort::read(byte *data, uint64_t length)
{
#if defined(_WIN32)
  DWORD bytesRead;
  COMSTAT status;
  DWORD errors;
  uint64_t toRead = 0;

  ClearCommError(_handle, &errors, &status);

  if (status.cbInQue > 0)
  {
    if (status.cbInQue > length)
    {
      toRead = length;
    }
    else
      toRead = status.cbInQue;
  }
  else
    return 0;

  memset(data, 0, length);

  if (!::ReadFile(_handle, (void *)data, toRead, &bytesRead, NULL))
  {
    setError(getSystemError());
    return 0;
  }
  else
    return bytesRead;
#else
  //TODO: fix here
  int len_, fs_sel;
  fd_set fs_read;

  struct timeval time;

  FD_ZERO(&fs_read);
  FD_SET(_handle, &fs_read);

  time.tv_sec = 0;
  time.tv_usec = 100000;
  fs_sel = select(_handle + 1, &fs_read, NULL, NULL, &time);
  if (fs_sel)
  {
    len_ = ::read(_handle, data, length);    
    return len_;
  }
  else
  {
    return -1;
  }
#endif
}

int64_t QSerialPort::write(const byte *data, uint64_t maxSize)
{
#if defined(_WIN32)
  DWORD bytesSend;

  if (!WriteFile(_handle, (void *)data, maxSize, &bytesSend, NULL))
  {
    setError(getSystemError());
    return 0;
  }
  else
    return bytesSend;
#else
  //TODO: fix here
  int _len = ::write(_handle, data, maxSize);

  if (_len == maxSize)
  {
    return _len;
  }
  else
  {
    tcflush(_handle, TCOFLUSH);
    return -1;
  }
#endif
}

inline void QSerialPort::setError(const SerialPortError &errorInfo)
{
  _error = errorInfo;
}

QSerialPort::SerialPortError QSerialPort::getSystemError(int systemErrorCode) const
{
#if defined(_WIN32)
  if (systemErrorCode == -1)
    systemErrorCode = ::GetLastError();

  QSerialPort::SerialPortError error;

  switch (systemErrorCode)
  {
  case ERROR_SUCCESS:
    error = QSerialPort::NoError;
    break;
  case ERROR_IO_PENDING:
    error = QSerialPort::NoError;
    break;
  case ERROR_MORE_DATA:
    error = QSerialPort::NoError;
    break;
  case ERROR_FILE_NOT_FOUND:
    error = QSerialPort::DeviceNotFoundError;
    break;
  case ERROR_PATH_NOT_FOUND:
    error = QSerialPort::DeviceNotFoundError;
    break;
  case ERROR_INVALID_NAME:
    error = QSerialPort::DeviceNotFoundError;
    break;
  case ERROR_ACCESS_DENIED:
    error = QSerialPort::PermissionError;
    break;
  case ERROR_INVALID_HANDLE:
    error = QSerialPort::ResourceError;
    break;
  case ERROR_INVALID_PARAMETER:
    error = QSerialPort::UnsupportedOperationError;
    break;
  case ERROR_BAD_COMMAND:
    error = QSerialPort::ResourceError;
    break;
  case ERROR_DEVICE_REMOVED:
    error = QSerialPort::ResourceError;
    break;
  case ERROR_OPERATION_ABORTED:
    error = QSerialPort::ResourceError;
    break;
  case WAIT_TIMEOUT:
    error = QSerialPort::TimeoutError;
    break;
  default:
    error = QSerialPort::UnknownError;
    break;
  }
  return error;
#else
  if (systemErrorCode == -1)
    systemErrorCode = errno;

  QSerialPort::SerialPortError error;

  switch (systemErrorCode)
  {
  case ENODEV:
    error = QSerialPort::DeviceNotFoundError;
    break;
#ifdef ENOENT
  case ENOENT:
    error = QSerialPort::DeviceNotFoundError;
    break;
#endif
  case EACCES:
    error = QSerialPort::PermissionError;
    break;
  case EBUSY:
    error = QSerialPort::PermissionError;
    break;
  case EAGAIN:
    error = QSerialPort::ResourceError;
    break;
  case EIO:
    error = QSerialPort::ResourceError;
    break;
  case EBADF:
    error = QSerialPort::ResourceError;
    break;
#ifdef Q_OS_OSX
  case ENXIO:
    error = QSerialPort::ResourceError;
    break;
#endif
#ifdef EINVAL
  case EINVAL:
    error = QSerialPort::UnsupportedOperationError;
    break;
#endif
#ifdef ENOIOCTLCMD
  case ENOIOCTLCMD:
    error = QSerialPort::UnsupportedOperationError;
    break;
#endif
#ifdef ENOTTY
  case ENOTTY:
    error = QSerialPort::UnsupportedOperationError;
    break;
#endif
#ifdef EPERM
  case EPERM:
    error = QSerialPort::PermissionError;
    break;
#endif
  default:
    error = QSerialPort::UnknownError;
    break;
  }
  return error;
#endif
}

#if defined(_WIN32)
bool QSerialPort::setDcb(DCB *dcb)
{
  if (!::SetCommState(_handle, dcb))
  {
    setError(getSystemError());
    return false;
  }
  return true;
}

bool QSerialPort::getDcb(DCB *dcb)
{
  ::ZeroMemory(dcb, sizeof(DCB));
  dcb->DCBlength = sizeof(DCB);

  if (!::GetCommState(_handle, dcb))
  {
    setError(getSystemError());
    return false;
  }
  return true;
}
#else
bool QSerialPort::setTermios(const termios *tio)
{
  if (::tcsetattr(_handle, TCSANOW, tio) == -1)
  {
    setError(getSystemError());
    return false;
  }
  return true;
}

bool QSerialPort::getTermios(termios *tio)
{
  memset(tio, 0, sizeof(termios));
  if (::tcgetattr(_handle, tio) == -1)
  {
    setError(getSystemError());
    return false;
  }
  return true;
}
#endif

bool QSerialPort::setBaudRate(QSerialPort::BaudRate baudRate)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;
  set_baudrate(&dcb, baudRate);
  return setDcb(&dcb);
#else
  termios tio;
  if (!getTermios(&tio))
    return false;
  set_baudrate(&tio, baudRate);
  return setTermios(&tio);
#endif
}

bool QSerialPort::setDataBits(QSerialPort::DataBits dataBits)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;
  set_databits(&dcb, dataBits);
  return setDcb(&dcb);
#else
  termios tio;
  if (!getTermios(&tio))
    return false;
  set_databits(&tio, dataBits);
  return setTermios(&tio);
#endif
}

bool QSerialPort::setParity(QSerialPort::Parity parity)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;
  set_parity(&dcb, parity);
  return setDcb(&dcb);
#else
  termios tio;
  if (!getTermios(&tio))
    return false;
  set_parity(&tio, parity);
  return setTermios(&tio);
#endif
}

bool QSerialPort::setStopBits(QSerialPort::StopBits stopBits)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;
  set_stopbits(&dcb, stopBits);
  return setDcb(&dcb);
#else
  termios tio;
  if (!getTermios(&tio))
    return false;
  set_stopbits(&tio, stopBits);
  return setTermios(&tio);
#endif
}

bool QSerialPort::setFlowControl(QSerialPort::FlowControl flowControl)
{
#if defined(_WIN32)
  DCB dcb;
  if (!getDcb(&dcb))
    return false;
  set_flowcontrol(&dcb, flowControl);
  return setDcb(&dcb);
#else
  termios tio;
  if (!getTermios(&tio))
    return false;
  set_flowcontrol(&tio, flowControl);
  return setTermios(&tio);
#endif
}