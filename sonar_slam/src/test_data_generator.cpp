// test_data_generator.cpp
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>

// 生成一个模拟的游泳池扫描数据文件
void generateTestSonarFile(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Cannot create file: " << filename << std::endl;
        return;
    }
    
    // 1. 创建214字节文件头
    std::vector<char> header(214, 0);
    file.write(header.data(), 214);
    
    // 2. 生成360个角度的扫描数据（每2度一个）
    for (int angle = 0; angle < 360; angle += 2) {
        // 消息头 (4字节)
        char btitle[] = {'B', 'R', 'M', 'S'};
        file.write(btitle, 4);
        
        // 时间戳 (28字节)
        char timestamp[28] = {0};
        file.write(timestamp, 28);
        
        // 消息结构
        uint16_t msg_start = 0x4252; // "BR"
        uint16_t msglen = 14 + 1200; // 头部14字节 + 1200个样本
        uint16_t msgid = 2300;       // 消息ID
        uint8_t msgsrc = 0x01;
        uint8_t msgdst = 0x02;
        
        file.write(reinterpret_cast<const char*>(&msg_start), 2);
        file.write(reinterpret_cast<const char*>(&msglen), 2);
        file.write(reinterpret_cast<const char*>(&msgid), 2);
        file.write(reinterpret_cast<const char*>(&msgsrc), 1);
        file.write(reinterpret_cast<const char*>(&msgdst), 1);
        
        // 消息数据 (msg2300格式)
        uint8_t mode = 0x01;
        uint8_t gain_setting = 0x02;
        uint16_t angle_raw = static_cast<uint16_t>(angle * 400.0 / 360.0); // 转换为0-400范围
        uint16_t transmit_duration = 200;    // 200us
        uint16_t sample_period = 100;        // 100 * 25ns = 2.5us
        uint16_t transmit_frequency = 750;   // 750kHz
        uint16_t number_of_samples = 1200;
        uint16_t data_length = 1200;
        
        file.write(reinterpret_cast<const char*>(&mode), 1);
        file.write(reinterpret_cast<const char*>(&gain_setting), 1);
        file.write(reinterpret_cast<const char*>(&angle_raw), 2);
        file.write(reinterpret_cast<const char*>(&transmit_duration), 2);
        file.write(reinterpret_cast<const char*>(&sample_period), 2);
        file.write(reinterpret_cast<const char*>(&transmit_frequency), 2);
        file.write(reinterpret_cast<const char*>(&number_of_samples), 2);
        file.write(reinterpret_cast<const char*>(&data_length), 2);
        
        // 生成强度数据（模拟10x6米矩形游泳池）
        float pool_width = 10.0f;
        float pool_height = 6.0f;
        float angle_rad = angle * M_PI / 180.0f;
        
        // 计算到游泳池边界的距离
        float min_range = std::numeric_limits<float>::max();
        
        // 检查四条边
        // 左边界：x = -pool_width/2
        if (fabs(cosf(angle_rad)) > 1e-6) {
            float t = (-pool_width/2) / cosf(angle_rad);
            if (t > 0) {
                float y = t * sinf(angle_rad);
                if (fabs(y) <= pool_height/2) {
                    min_range = std::min(min_range, t);
                }
            }
        }
        
        // 右边界：x = pool_width/2
        if (fabs(cosf(angle_rad)) > 1e-6) {
            float t = (pool_width/2) / cosf(angle_rad);
            if (t > 0) {
                float y = t * sinf(angle_rad);
                if (fabs(y) <= pool_height/2) {
                    min_range = std::min(min_range, t);
                }
            }
        }
        
        // 下边界：y = -pool_height/2
        if (fabs(sinf(angle_rad)) > 1e-6) {
            float t = (-pool_height/2) / sinf(angle_rad);
            if (t > 0) {
                float x = t * cosf(angle_rad);
                if (fabs(x) <= pool_width/2) {
                    min_range = std::min(min_range, t);
                }
            }
        }
        
        // 上边界：y = pool_height/2
        if (fabs(sinf(angle_rad)) > 1e-6) {
            float t = (pool_height/2) / sinf(angle_rad);
            if (t > 0) {
                float x = t * cosf(angle_rad);
                if (fabs(x) <= pool_width/2) {
                    min_range = std::min(min_range, t);
                }
            }
        }
        
        // 生成1200个强度样本
        for (int i = 0; i < 1200; i++) {
            float distance = i * 0.01f; // 每个样本0.01米间隔
            uint8_t intensity = 0;
            
            if (distance >= min_range - 0.1f && distance <= min_range + 0.1f) {
                // 墙壁反射（强信号）
                intensity = 200 + rand() % 56; // 200-255
            } else if (fabs(distance - min_range) < 0.5f) {
                // 近壁区域（中等信号）
                intensity = 100 + rand() % 100; // 100-200
            } else if (i % 50 == 0) {
                // 水中随机噪声（弱信号）
                intensity = 20 + rand() % 30; // 20-50
            }
            
            file.write(reinterpret_cast<const char*>(&intensity), 1);
        }
        
        // 校验和（简化）
        uint16_t checksum = 0;
        file.write(reinterpret_cast<const char*>(&checksum), 2);
    }
    
    file.close();
    std::cout << "Generated test file: " << filename << std::endl;
    std::cout << "File contains 180 scan frames (0-360 degrees, 2-degree steps)" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_data_generator");
    
    if (argc < 2) {
        std::cout << "Usage: rosrun sonar_slam test_data_generator /path/to/output.bin" << std::endl;
        return 1;
    }
    
    std::string filename = argv[1];
    generateTestSonarFile(filename);
    
    return 0;
}