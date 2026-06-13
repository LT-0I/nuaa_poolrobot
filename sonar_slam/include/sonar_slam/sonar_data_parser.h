// sonar_slam/include/sonar_slam/sonar_data_parser.h
#ifndef SONAR_DATA_PARSER_H
#define SONAR_DATA_PARSER_H

#include <vector>
#include <string>
#include <cstdint>

namespace sonar_slam {

struct SonarFrame {
    uint16_t msg_id;          // 消息ID
    uint32_t timestamp;       // 时间戳
    uint16_t angle;           // 角度（0-359度）
    std::vector<uint8_t> intensities;  // 强度数据
    std::vector<float> ranges;         // 距离数据（需要计算）
};

class SonarDataParser {
public:
    SonarDataParser();
    ~SonarDataParser();
    
    // 解析.bin文件
    bool parseBinaryFile(const std::string& filename);
    
    // 获取解析后的声纳帧
    const std::vector<SonarFrame>& getFrames() const { return frames_; }
    
    // 获取msgid=2300的声纳帧
    std::vector<SonarFrame> getSonarFrames() const;
    
    // 清空数据
    void clear();
    
private:
    std::vector<SonarFrame> frames_;
    
    // 解析单个消息
    bool parseMessage(const uint8_t* data, size_t size);
    
    // 解析2300消息（声纳数据）
    bool parseMsg2300(const uint8_t* data, size_t size, SonarFrame& frame);
    
    // 从强度值计算距离（需要根据声纳参数计算）
    float intensityToRange(uint8_t intensity, uint16_t sample_index);
};

} // namespace sonar_slam

#endif // SONAR_DATA_PARSER_H