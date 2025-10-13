#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sys/sysinfo.h>
namespace tool
{
    time_t GetSysTime() {
    std::ifstream stat_file("/proc/stat");
    std::string line;
    while (std::getline(stat_file, line)) {
        //std::cout<<line<<std::endl;
        if (line.find("btime") == 0) {
            return std::stol(line.substr(6));
        }
    }
    return 0;
    }
}
