#pragma once

#include <vector>
#include <string>
#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

namespace lidar
{
    std::vector<std::string> SplitString(const std::string &src,const char &flag)
    {
        std::vector<std::string> list;
        std::string temp;
        std::stringstream stream(src);
        while(std::getline(stream,temp,flag))
        {
            list.push_back(temp);
        }
        return list;
    }

    float CalculateVariance(const std::vector<float>& list) 
    {
        size_t size=list.size();
        if (size<=0) return 0.0f;   
        // 计算平均值
    
        float sum = 0.0f;
        for(auto &item:list)
        {
            sum += item;
        }
        float mean = sum / size;
        // 计算方差
        float variance = 0.0f;
        for(auto &item:list)
        {
            float diff = item - mean;
            variance += diff * diff;
            sum += item;
        }
        return variance / size;
    }

    void SearchFiles(const std::string& dir_name,std::vector<std::string>& files) 
    {
        fs::path dir(dir_name);
        for (fs::directory_iterator it(dir); it != fs::directory_iterator(); ++it) 
        {
            if (fs::is_directory(it->status())) 
            {
                SearchFiles(it->path().string(),files);
            } 
            else 
            {
                std::cout << "文件: " << it->path() << std::endl;
                files.push_back(it->path().string());
            }
        }
    }

    double GetSystemTime()
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        std::chrono::nanoseconds nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
        return double(seconds.count()*1.0+(nanoseconds.count()-seconds.count()*1e9)*1e-009);
    }
}