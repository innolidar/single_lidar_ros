/*********************************************************************************************************************
Copyright (c) 2023 InnoLight
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For InnoLight LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the InnoLight, nor Suteng Innovation Technology, nor the names of other contributors may be used
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

#include <lidar_driver/common/lidar_common.hpp>

#include <queue>

namespace lidar
{
template <typename T>
class SyncQueue
{
public:
  inline size_t Push(const T& value)
  {
    m_mtx.lock();
    m_queue.push(value);
    m_size = m_queue.size();
    m_mtx.unlock();

    return m_size;
  }
  inline size_t GetSize() const
  {
    return m_size;
  }

  inline T Pop()
  {
    T value = NULL;

    m_mtx.lock();
    if (!m_queue.empty())
    {
      value = m_queue.front();
      m_queue.pop();
    }
    m_mtx.unlock();

    return value;
  }

  inline T PopWait(uint32_t msec = 1)
  {
    T value = NULL;
    bool empty = true;

    m_mtx.lock();
    if (!m_queue.empty())
    {
      value = m_queue.front();
      m_queue.pop();
      empty = false;
    }
    m_mtx.unlock();

    if (empty)
    {
	    inno_msleep(msec);
    }

    return value;
  }

  inline void Clear()
  {
    std::queue<T> empty;

    m_mtx.lock();
    std::swap(empty, m_queue);
    m_mtx.unlock();
  }

private:
  size_t        m_size;
  std::mutex    m_mtx;
  std::queue<T> m_queue;
};
}  // namespace lidar
