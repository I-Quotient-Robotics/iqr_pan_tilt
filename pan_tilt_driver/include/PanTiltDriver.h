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
#ifndef PANTILTDRIVER_H
#define PANTILTDRIVER_H

#include <iostream>
#include <sstream>
#include <mutex>
#include "QSerialFrame.h"
#include "QSerialPort.h"
#include "QThread.h"

namespace IQR
{
class PanTiltDriver : public QThread
{

public:
  PanTiltDriver(const std::string &portName);
  ~PanTiltDriver();

  void goZero();
  void holdPanTilt();
  void releasePanTilt();
  void getPose(float &yaw, float &pitch);
  std::string getPanTiltVersion();
  void setPose(const float &yaw, const float &pitch);
  void setPose(const float &yaw, const float &pitch, const uint16_t &speed);

protected:
  void run();
  inline void stop();
  inline void sendGetVersionMsg();
private:
  bool _readFlage;
  std::mutex _mtx;
  volatile float _yaw;
  volatile float _pitch;
  std::stringstream _version;
  QSerialPort *_com;
  QSerialFrame *_frame;
};
} // namespace IQR

#endif //PANTILTDRIVER_H