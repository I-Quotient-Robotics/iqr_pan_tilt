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


#ifndef QTHREAD_H
#define QTHREAD_H
#include <iostream>
#include <thread>
#include <functional>

class QThread
{
public:
  enum THREAD_STATUS
  {
    THREAD_STATUS_NEW = 0,
    THREAD_STATUS_RUNNING = 1,
    THREAD_STATUS_EXIT = -1
  };

  QThread();
  virtual ~QThread();

  void start();
  void join();
  void detach();

  std::thread::id getPid() const;
  THREAD_STATUS getThreadStatus() const { return threadStatus; };


protected:
  virtual void run() = 0;

private:
  THREAD_STATUS threadStatus;
  std::thread *td;
};

#endif //QTHREAD_H
