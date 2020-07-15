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
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "pan_tilt_msg/PanTiltCmd.h"
#include "PanTiltDriver.h"

using namespace std;

class PanTiltDriverNode
{
public:
  PanTiltDriverNode();
  ~PanTiltDriverNode();
  void JointStatePublish();

protected:
  void callBack(const pan_tilt_msg::PanTiltCmd &msg);

private:
  IQR::PanTiltDriver *_pt;
  std::string _portName;
  std::string _yawJointName;
  std::string _pitchJointName;
  sensor_msgs::JointState _js;
  ros::NodeHandle _nh;
  ros::Subscriber _cmdSub;
  ros::Publisher _jointPub;
};

PanTiltDriverNode::PanTiltDriverNode()
    : _nh("~")
{
  //init params
  _nh.param<std::string>("port_name", _portName, "/dev/ttyUSB0");
  _nh.param<std::string>("yaw_joint_name", _yawJointName, "iqr_pan_tilt_yaw_joint");
  _nh.param<std::string>("pitch_joint_name", _pitchJointName, "iqr_pan_tilt_pitch_joint");

  _js.name.resize(2);
  _js.position.resize(2);
  _js.velocity.resize(2);
  _js.effort.resize(2);
  _js.name[0] = _yawJointName;
  _js.name[1] = _pitchJointName;

  //set subscriber
  _jointPub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 50);
  //set publisher
  _cmdSub = _nh.subscribe("pan_tilt_cmd", 50, &PanTiltDriverNode::callBack, this);

  cout << "pan-tilt port name:" << _portName << endl;
  _pt = new IQR::PanTiltDriver(_portName);
  sleep(2);
}

PanTiltDriverNode::~PanTiltDriverNode()
{
  delete _pt;
}

void PanTiltDriverNode::JointStatePublish()
{
  float yawDeg = 0.0;
  float pitchDeg = 0.0;
  _pt->getPose(yawDeg, pitchDeg);

  double yawRad = 0.01745329 * yawDeg;
  double pitchRad = 0.01745329 * pitchDeg;

  _js.header.stamp = ros::Time::now();
  _js.position[0] = yawRad;
  _js.position[1] = pitchRad;
  _jointPub.publish(_js);
}

void PanTiltDriverNode::callBack(const pan_tilt_msg::PanTiltCmd &msg)
{
  // ROS_INFO("[%f,%f,%i]", msg.yaw, msg.pitch, msg.speed);
  _pt->setPose(msg.yaw, msg.pitch, msg.speed);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "PanTiltDriverNode");

  PanTiltDriverNode nd;
  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();
    nd.JointStatePublish();
    r.sleep();
  }
  return 0;
}
