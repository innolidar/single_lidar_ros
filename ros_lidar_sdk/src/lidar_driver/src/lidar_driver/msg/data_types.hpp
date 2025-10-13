/*********************************************************************************************************************
Copyright (c) 2020 InnoLight
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

#include <vector>
#include <string>

namespace lidar{

#define LIDAR_ERROR_STRING "LIDAR_ERROR_STRING"

enum LIDAR_ERROR_E
{
    LIDAR_NOT_ERROR_STATE = 0,     ///<:没有故障
    MOTOR_STOPS_RUNNING = 1,       ///<:电机停转
};

struct Laser_msg{
    void Clear()
    {
        frame_id="";
        timestamp=0;
        angle_min=0;
        angle_max=0;
        angle_increment=0;
        time_increment=0;
        scan_time=0;
        range_min=0;
        range_max=0;
        ranges.clear();
        intensities.clear();
    }
    void PushData(const float &intensity,const float &distnace)
    {
        ranges.push_back(distnace);
        intensities.push_back(intensity);
    }
    std::string frame_id{"base_laser"};
    uint32_t timestamp{0};               ///<:雷达时间戳(毫秒)
    float angle_min{0.f};                ///<:测量起始角度(弧度制)
    float angle_max{0.f};                ///<:测量终止角度(弧度制)
    float angle_increment{0.f};          ///<:角度分辨率(弧度制)
    float time_increment{0.f};           ///<:时间分辨率(单位 ms)
    float scan_time{0.f};                ///<:扫描周期(单位 ms)
    float range_min{0.f};                ///<:最小有效测量范围(单位 m)
    float range_max{0.f};                ///<:最大有效测量范围(单位 m)
    std::vector<float> ranges{};         ///<:测距点云( 单位 m)
    std::vector<float> intensities{};    ///<:强度信息
};
}


