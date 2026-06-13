// sonar_slam/src/data_analyzer.cpp
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>

void analyzeSonarData(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return;
    }
    
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    ROS_INFO("File size: %lu bytes", file_size);
    
    // 跳过214字节头部
    file.seekg(214, std::ios::beg);
    
    int msg2300_count = 0;
    
    while (file.tellg() < file_size && msg2300_count < 10) {
        char btitle[4];
        if (!file.read(btitle, 4)) break;
        
        char mesg_time[28];
        if (!file.read(mesg_time, 28)) break;
        
        char msg_struct[8];
        if (!file.read(msg_struct, 8)) break;
        
        // 解析消息头
        auto read_le16 = [](const char* d) {
            return (static_cast<uint16_t>(static_cast<uint8_t>(d[0]))) |
                   (static_cast<uint16_t>(static_cast<uint8_t>(d[1])) << 8);
        };
        
        uint16_t msglen = read_le16(&msg_struct[2]);
        uint16_t msgid = read_le16(&msg_struct[4]);
        
        if (msgid != 2300) {
            if (msglen > 0) file.seekg(msglen + 2, std::ios::cur);
            continue;
        }
        
        msg2300_count++;
        
        std::vector<char> msgdata(msglen);
        if (!file.read(msgdata.data(), msglen)) break;
        
        file.seekg(2, std::ios::cur); // 校验和
        
        ROS_INFO("\n=== Message 2300 #%d ===", msg2300_count);
        ROS_INFO("msglen: %u bytes", msglen);
        
        // 详细分析数据结构
        for (size_t section_start = 0; section_start < msgdata.size(); ) {
            // 假设每个section有14字节头部
            if (section_start + 14 > msgdata.size()) break;
            
            ROS_INFO("\n--- Section at offset 0x%04lX ---", section_start);
            
            // 显示section头部
            std::stringstream header_ss;
            for (int i = 0; i < 14 && section_start + i < msgdata.size(); i++) {
                header_ss << std::hex << std::setw(2) << std::setfill('0') 
                         << (int)(unsigned char)msgdata[section_start + i] << " ";
            }
            ROS_INFO("Header: %s", header_ss.str().c_str());
            
            // 解析可能的角度值
            for (int offset = 0; offset <= 12; offset += 2) {
                if (section_start + offset + 2 <= msgdata.size()) {
                    uint16_t val = read_le16(&msgdata[section_start + offset]);
                    ROS_INFO("  Offset +%d: 0x%04X (%5u) - possible interpretations:", offset, val, val);
                    
                    if (val <= 399) {
                        ROS_INFO("        Angle: %.1f° (0-399 scale)", val * 360.0f / 399.0f);
                    }
                    if (val <= 1000) {
                        ROS_INFO("        Sample period: %.3f us", val * 25.0f / 1000.0f);
                    }
                    if (val >= 500 && val <= 1000) {
                        ROS_INFO("        Frequency: %u kHz", val);
                    }
                    if (val >= 200 && val <= 1200) {
                        ROS_INFO("        Samples: %u", val);
                    }
                }
            }
            
            // 查找强度数据
            size_t data_start = section_start + 14;
            if (data_start < msgdata.size()) {
                ROS_INFO("  Data starts at offset 0x%04lX", data_start);
                
                // 显示前20个强度值
                std::stringstream data_ss;
                data_ss << "  First 20 intensity values: ";
                for (int i = 0; i < 20 && data_start + i < msgdata.size(); i++) {
                    uint8_t val = static_cast<uint8_t>(msgdata[data_start + i]);
                    data_ss << std::setw(3) << (int)val << " ";
                }
                ROS_INFO("%s", data_ss.str().c_str());
            }
            
            // 移动到下一个可能的section
            section_start += 14;
            
            // 如果有数据长度字段，使用它
            if (section_start >= 14 && msgdata.size() >= section_start - 12) {
                uint16_t data_len = read_le16(&msgdata[section_start - 12]);
                if (data_len > 0) {
                    section_start += data_len;
                }
            }
        }
        
        if (msg2300_count >= 3) {
            ROS_INFO("\n=== Showing only first 3 messages for analysis ===");
            break;
        }
    }
    
    file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_analyzer");
    
    if (argc < 2) {
        ROS_ERROR("Usage: rosrun sonar_slam data_analyzer /path/to/data.bin");
        return 1;
    }
    
    std::string filename = argv[1];
    analyzeSonarData(filename);
    
    return 0;
}