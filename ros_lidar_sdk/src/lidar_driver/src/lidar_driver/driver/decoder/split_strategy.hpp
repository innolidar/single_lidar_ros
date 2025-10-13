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


namespace lidar
{

class SplitStrategy
{
public:
  virtual void reset(void) = 0;
  virtual bool newUnit(int32_t angle) = 0;
  virtual ~SplitStrategy(){};
};

class SplitStrategyByAngle : public SplitStrategy
{
public:

  virtual bool newUnit(int32_t angle)
  {
    if (angle < prev_angle_)
    {
      prev_angle_ -= 36000;
    }

    bool v = ((prev_angle_ < split_angle_) && (split_angle_ <= angle));
#if 0
    if (v) 
    {
      std::cout << prev_angle_ << "\t" << angle << std::endl;
    }
#endif
    prev_angle_ = angle;
    return v;
  }

  virtual void reset(void)
  {
    prev_angle_ = split_angle_;
  }

  virtual ~SplitStrategyByAngle(){};

  SplitStrategyByAngle (int32_t split_angle = 0)
   : split_angle_(split_angle), prev_angle_(split_angle)
  {
  }

#ifndef UNIT_TEST
private:
#endif
    const int32_t split_angle_;
    int32_t prev_angle_;
};

class SplitStrategyBySeq : public SplitStrategy
{
public:

  virtual bool newUnit(int32_t angle)
  {
    uint16_t seq = (uint16_t)angle;

    bool split = false;

    if (seq < safe_seq_min_) // rewind
    {
      prev_seq_ = seq;
      split = true;
    }
    else if (seq < prev_seq_)
    {
      // do nothing.
    }
    else if (seq <= safe_seq_max_)
    {
      prev_seq_ = seq;
    }
    else
    {
      if (prev_seq_ == 0) 
        prev_seq_ = seq;

      //do nothing.
    }

    setSafeRange();
    return split;
  }

  virtual void reset(void)
  {
    prev_seq_ = 0;
    setSafeRange();
  }

  virtual ~SplitStrategyBySeq(){}

  SplitStrategyBySeq()
    : prev_seq_(0)
  {
    setSafeRange();
  }

#ifndef UNIT_TEST
private:
#endif

  void setSafeRange()
  {
    const static uint16_t RANGE = 10;
  
    safe_seq_min_ = (prev_seq_ > RANGE) ? (prev_seq_ - RANGE) : 0;
    safe_seq_max_ = prev_seq_ + RANGE;
  }

  uint16_t prev_seq_;
  uint16_t safe_seq_min_;
  uint16_t safe_seq_max_;
};

}  // namespace lidar
