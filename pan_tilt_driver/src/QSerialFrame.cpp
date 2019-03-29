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
#include "QSerialFrame.h"
#include <string.h>


QSerialFrame::QSerialFrame(const byte id)
  :id_(id), frame_head_1_(0xFA), frame_head_2_(0xAA)
{
  RcvData_Len = 0;
  SenData_Len = 0;
}

QSerialFrame::~QSerialFrame()
{

}

bool QSerialFrame::ReadFrame(const byte buf)
{
  static byte rcvBuf_[MAX_RcvBUFF_LEN] = {0,};
  static uint32_t frameLength_ = 0;
  static uint32_t rcvIndex_ = 0;

  rcvBuf_[rcvIndex_] = buf;
  rcvIndex_++;
  if(rcvIndex_ < 5)
  {
    if(rcvIndex_ == 1 && rcvBuf_[0] == frame_head_1_)
    {
      return false;
    }
    else if(rcvIndex_ == 2 && rcvBuf_[1] == frame_head_2_)//head
    {
      return false;
    }
    else if(rcvIndex_ == 3 && rcvBuf_[2] == id_)//id
    {
      return false;
    }
    else if(rcvIndex_ == 4 && rcvBuf_[3] > 0)//len
    {
      frameLength_ = buf;
      return false;
    }
    else
    {
      rcvIndex_ = 0;
      frameLength_ = 0;
      return false;
    }
  }
  else
  {
    if(rcvIndex_ < frameLength_)
    {
      RcvData[rcvIndex_ - 5] = buf;
      return false;
    }
    else
    {
      if(DelDataZero(rcvBuf_, frameLength_))
      {
        rcvIndex_ = 0;
        frameLength_ = 0;
        return true;
      }
      else
      {
        rcvIndex_ = 0;
        frameLength_ = 0;
        return false;
      }
    }
  }
  return false;
}

uint32_t QSerialFrame::BuildFrame(byte *buf, uint32_t length)
{
  byte b0 = 0;
  byte b1 = 0;

  SenData[0] = frame_head_1_;
  SenData[1] = frame_head_2_;
  SenData[2] = id_;

  uint32_t j = 4;
  for (uint32_t i = 0; i < length; i++)
  {
    b0 = buf[i];
    if (b0 == frame_head_2_ && b1 == frame_head_1_)
    {
      SenData[j] = b0;
      j++;
      SenData[j] = 0x00;
      j++;
    }
    else
    {
      SenData[j] = b0;
      j++;
    }
    b1 = b0;
  }
  SenData[3] = j + 1;

  SenData[j] = AddDataSum(SenData, j);

  SenData_Len = j + 1;
  return SenData_Len;
}

bool QSerialFrame::CheckDataSum(byte *data, const uint32_t data_L, const byte checksum)
{
  byte sum = 0x00;
  for (uint32_t i = 0; i < data_L; i++)
  {
    sum += data[i];
  }
  if (sum == checksum)
  {
    return true;
  }
  else
  {
    return false;
  }
}

byte QSerialFrame::AddDataSum(byte *data, const uint32_t data_L)
{
  byte sum = 0x00;
  for (uint32_t i = 0; i < data_L; i++)
  {
    sum += data[i];
  }
  return sum;
}

bool QSerialFrame::DelDataZero(byte *data, const uint32_t data_L)
{
  if (CheckDataSum(data, data_L-1, data[data_L - 1]) == false)
  {
    return false;
  }
  byte b0 = 0x00;
  byte b1 = 0x00;
  byte b2 = 0x00;

  uint32_t j = 0;
  for (uint32_t i = 4; i < data_L - 1; i++)
  {
    b0 = data[i];
    if (b0 == 0x00 && b1 == frame_head_2_ && b2 == frame_head_1_)
    {
      ;
    }
    else
    {
      RcvData[j] = b0;
      j++;
    }
    b2 = b1;
    b1 = b0;
  }
  RcvData_Len = j;

  return true;
}

int32_t QSerialFrame::bytesToInt32(byte *bytes)
{
  int32_t addr = bytes[3];
  addr |= (bytes[2] << 8);
  addr |= (bytes[1] << 16);
  addr |= (bytes[0] << 24);
  return addr;
}

void QSerialFrame::int32ToByte(int32_t i, byte *bytes)
{
  bytes[3] = byte(0xff & i);
  bytes[2] = byte((0xff00 & i) >> 8);
  bytes[1] = byte((0xff0000 & i) >> 16);
  bytes[0] = byte((0xff000000 & i) >> 24);
  return;
}

int16_t QSerialFrame::bytesToInt16(byte * bytes)
{
  int16_t addr = bytes[1];
  addr |= (bytes[0] << 8);
  return addr;
}

void QSerialFrame::int16ToByte(int16_t i, byte * bytes)
{
  bytes[1] = byte(0xff & i);
  bytes[0] = byte((0xff00 & i) >> 8);
  return;
}

float QSerialFrame::byteToFloat32(byte *bytes)
{
  union {
    float a;
    byte bytes[4];
  } thing;
  memcpy(thing.bytes, bytes, 4);
  return thing.a;
}

void QSerialFrame::float32ToByte(float f, byte *bytes)
{
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = f;
  memcpy(bytes, thing.bytes, 4);
}
