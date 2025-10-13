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

namespace innolight
{
namespace lidar
{

class AzimuthSection
{
public:
  AzimuthSection(int32_t start, int32_t end)
  {
    m_full_round = (start == 0) && (end == 36000);

    m_start = start % 36000;
    m_end = end % 36000;
    m_cross_zero = (m_start > m_end);
  }

  bool in(int32_t angle)
  {
    if (m_full_round)
      return true;

    if (m_cross_zero)
    {
      return (angle >= m_start) || (angle < m_end);
    }
    else
    {
      return (angle >= m_start) && (angle < m_end);
    }
  }

#ifndef UNIT_TEST
private:
#endif
  bool    m_full_round;
  bool    m_cross_zero;
  int32_t m_start;
  int32_t m_end;
};

class DistanceSection
{
public:
  DistanceSection (float min, float max, float usr_min, float usr_max)
    : m_min(min), m_max(max)
  {
    if (usr_min < 0) usr_min = 0;
    if (usr_max < 0) usr_max = 0;

    if ((usr_min != 0) || (usr_max != 0))
    {
      m_min = usr_min;
      m_max = usr_max;
    }
  }

  bool in(float distance)
  {
    return ((m_min <= distance) && (distance <= m_max));
  }

#ifndef UNIT_TEST
private:
#endif

  float m_min;
  float m_max;
};

}  // namespace lidar
}  // namespace InnoLight
