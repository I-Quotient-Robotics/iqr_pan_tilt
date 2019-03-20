#include <iostream>
#if defined(_WIN32)
#include <windows.h>
#else
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#endif

#include "PanTiltDriver.h"
#include "QSerialPort.h"

using namespace std;

int main(int argv, char *argc[])
{
  float yaw, pitch;

  IQR::PanTiltDriver pt("/dev/ttyACM0");
  sleep(2);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.goZero();
  sleep(2);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.setPose(0.0, -30.0, 5);
  sleep(5);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.releasePanTilt();
  sleep(2);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.holdPanTilt();
  sleep(2);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.setPose(30.0, 30.0, 5);
  sleep(5);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.goZero();
  sleep(2);
  pt.getPose(yaw, pitch);
  cout << yaw << "  " << pitch << endl;

  pt.releasePanTilt();
  sleep(2);
  pt.holdPanTilt();

  cout << "hhhhhhhhh" << endl;
  cout << pt.getPanTiltVersion() << endl;

  return 0;
}