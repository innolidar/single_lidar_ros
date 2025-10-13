/*********************************************************************************************************************
Copyright (c) 2023 innolight
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For innolight LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the innolight, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <chrono>
#include <ctime>
#include <condition_variable>
#ifdef _WIN32

typedef unsigned long long uint64_t;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

typedef int int32_t;
typedef short int16_t;

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>

inline void inno_msleep(uint64_t ms)
{
  uint32_t mlsec = (uint32_t)(ms/1000);
  if (mlsec < 1) mlsec =1;
  Sleep(mlsec);
}

class inno_mutex
{
public:
  inno_mutex()
  {
    InitializeCriticalSection(&cs);
  }

  void lock()
  {
	  EnterCriticalSection(&cs);
  }

  void unlock()
  {
	  LeaveCriticalSection(&cs);
  }

private:
  CRITICAL_SECTION cs;
};

typedef void (*Cb_Thread_Proc)(void*);

void inno_thread_proc (void* obj);

class inno_thread
{
public:

  explicit inno_thread (Cb_Thread_Proc cb, void* obj)
    : exited_(false)
  {
    cb_ = cb;
    obj_ = obj;
    handle_ = (HANDLE)_beginthread(inno_thread_proc, 0, this);
  }

  static void inno_thread_proc (void* obj)
  {
    inno_thread* thr = (inno_thread*) obj;
    thr->proc();
  }

  void proc(void)
  {
    cb_(obj_);
    exited_ = true;
    _endthread();
  }

  void join()
  {
    while(!exited_)
    {
      Sleep(1);
    }
  }

private:
  HANDLE handle_;
  Cb_Thread_Proc cb_;
  void* obj_;
  bool exited_;
};


#else //__linux__

#include <arpa/inet.h>

#include <cstdint>
#include <mutex>
#include <thread>

inline void inno_msleep(uint64_t ms)
{
  std::this_thread::sleep_for(std::chrono::microseconds(ms));
}

typedef std::mutex inno_mutex;
typedef std::thread inno_thread;

#endif

//
// define M_PI
// 
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif

#include <math.h>

#define DEGREE_TO_RADIAN(deg)  ((deg) * M_PI / 180)
#define RADIAN_TO_DEGREE(deg)  ((deg) * 180 / M_PI)

inline int inno_round(double value)
{
  return (int)(value + 0.5f);
}

inline int16_t INNO_SWAP_INT16(int16_t value)
{
  uint8_t* v = (uint8_t*)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}


