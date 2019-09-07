/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#include "PanTiltDriver.h"

#define SLEEP_TIME_MS 10

static inline void delay()
{
#if defined(_WIN32)
  Sleep(SLEEP_TIME_MS);
#else
  usleep(SLEEP_TIME_MS * 1000);
#endif
}

IQR::PanTiltDriver::PanTiltDriver(const std::string &portName)
    : _yaw(0.0), _pitch(0.0), _readFlage(false)
{
  _com = new QSerialPort(portName,
                         QSerialPort::Baud115200,
                         QSerialPort::Data8,
                         QSerialPort::OneStop,
                         QSerialPort::NoParity,
                         QSerialPort::NoFlowControl);
  _com->open(QSerialPort::ReadAndWrite);
  if (_com->getError())
  {
    std::cout << "open " << portName << " serial port error code:" << _com->getError() << std::endl;
  }
  _frame = new QSerialFrame(0x01);

  start();
  sendGetVersionMsg();
}

IQR::PanTiltDriver::~PanTiltDriver()
{
  stop();
  delete _com;
  delete _frame;
}

void IQR::PanTiltDriver::goZero()
{
  if (_com->isOpen())
  {
    byte msg[4] = {
        0,
    };
    msg[0] = 0x02;
    std::lock_guard<std::mutex> lck(_mtx);
    _frame->BuildFrame(msg, 4);
    _com->write(_frame->SenData, _frame->SenData_Len);
  }
  else
  {
  }
}

void IQR::PanTiltDriver::holdPanTilt()
{
  if (_com->isOpen())
  {
    byte msg[4] = {
        0,
    };
    msg[0] = 0x04;
    std::lock_guard<std::mutex> lck(_mtx);
    _frame->BuildFrame(msg, 4);
    _com->write(_frame->SenData, _frame->SenData_Len);
  }
  else
  {
  }
}

void IQR::PanTiltDriver::releasePanTilt()
{
  if (_com->isOpen())
  {
    byte msg[4] = {
        0,
    };
    msg[0] = 0x03;
    std::lock_guard<std::mutex> lck(_mtx);
    _frame->BuildFrame(msg, 4);
    _com->write(_frame->SenData, _frame->SenData_Len);
  }
  else
  {
  }
}

void IQR::PanTiltDriver::getPose(float &yaw, float &pitch)
{
  std::lock_guard<std::mutex> lck(_mtx);
  yaw = _yaw;
  pitch = _pitch;
}

void IQR::PanTiltDriver::setPose(const float &yaw, const float &pitch)
{
  setPose(yaw, pitch, 10);
}

void IQR::PanTiltDriver::setPose(const float &yaw, const float &pitch, const uint16_t &speed)
{
  if (_com->isOpen())
  {
    if (abs(yaw) > 60 || abs(pitch) > 60 || speed > 30)
      return;

    byte msg[12] = {
        0,
    };
    msg[0] = 0x01;
    QSerialFrame::float32ToByte(yaw, msg + 2);
    QSerialFrame::float32ToByte(pitch, msg + 6);
    QSerialFrame::int16ToByte(speed, msg + 10);
    std::lock_guard<std::mutex> lck(_mtx);
    _frame->BuildFrame(msg, 12);
    _com->write(_frame->SenData, _frame->SenData_Len);
  }
  else
  {
  }
}

std::string IQR::PanTiltDriver::getPanTiltVersion()
{
  if (_com->isOpen())
  {
    if(!_version.str().empty())
        return _version.str();
      else
      {
        sendGetVersionMsg();
        return _version.str();
      }     
  }
  else
  {
    return NULL;
  }
}

void IQR::PanTiltDriver::run()
{
   _readFlage = true;
  byte buf[16] = {
      0,
  };
  int32_t date_len = 0;
  while ( _readFlage)
  {
    _mtx.lock();
    date_len = _com->read(buf, 16);
    //std::cout << date_len << "\n";
    for (int32_t i = 0; i < date_len; i++)
    {
      //std::cout << i << "\n";
      if (_frame->ReadFrame(buf[i]))
      {
        switch (_frame->RcvData[0])
        {
        case 0x01:
          break;
        case 0x05:
          float nowYaw, nowPitch;
          nowYaw = QSerialFrame::byteToFloat32(_frame->RcvData + 2);
          nowPitch = QSerialFrame::byteToFloat32(_frame->RcvData + 6);
          _yaw = nowYaw;
          _pitch = nowPitch;
          break;
        case 0x06:
          _version << "v" << int(_frame->RcvData[2]) << "." << int(_frame->RcvData[3]);
          break;
        default:
          break;
        }
      }
    }
    _mtx.unlock();
    delay();
  }
}

inline void IQR::PanTiltDriver::stop()
{
  std::lock_guard<std::mutex> lck(_mtx);
   _readFlage = false;
  //join();
}

inline void IQR::PanTiltDriver::sendGetVersionMsg()
{
  byte msg[4] = {
      0,
  };
  msg[0] = 0x06;
  _frame->BuildFrame(msg, 4);
  _com->write(_frame->SenData, _frame->SenData_Len);

#if defined(_WIN32)
  Sleep(2*SLEEP_TIME_MS);
#else
  usleep(2*SLEEP_TIME_MS * 1000);
#endif
}
