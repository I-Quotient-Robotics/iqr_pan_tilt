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
#ifndef QSERIALFRAME_H
#define QSERIALFRAME_H

#define MAX_RcvBUFF_LEN 1024
#define MAX_SenBUFF_LEN 1024

#include <iostream>

typedef uint8_t byte;

class QSerialFrame 
{
public:
  QSerialFrame(const byte id);
  ~QSerialFrame();

  byte RcvData[MAX_RcvBUFF_LEN];
  uint32_t RcvData_Len;
  byte SenData[MAX_SenBUFF_LEN];
  uint32_t SenData_Len;

  bool ReadFrame(const byte buf);
  uint32_t  BuildFrame(byte* buf, uint32_t length);

  static int32_t bytesToInt32(byte* bytes);
  static void  int32ToByte(int32_t i, byte *bytes);

  static int16_t bytesToInt16(byte* bytes);
  static void  int16ToByte(int16_t i, byte *bytes);

  static float byteToFloat32(byte *bytes);
  static void float32ToByte(float f, byte *bytes);

private:
  bool CheckDataSum(byte *data, const uint32_t data_L, const byte checksum);
  byte AddDataSum(byte *data, const uint32_t data_L);
  bool DelDataZero(byte *data, const uint32_t data_L);

private:
  const byte id_;
  const byte frame_head_1_;
  const byte frame_head_2_;
};

#endif //QSERIALFRAME_H