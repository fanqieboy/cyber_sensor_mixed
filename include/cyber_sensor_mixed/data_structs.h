#pragma once

#include <cstdint>
#include <vector>

namespace CyberSensor {

#pragma pack(push, 1)

struct ParaInfoHeader {
    uint16_t key;
    uint16_t length;
};

struct CmdHeader {
    uint8_t sof = 0xaa;
    uint8_t version = 0;
    uint16_t length;        
    uint32_t seq_num = 0;       
    uint16_t cmd_id;        
    uint8_t cmd_type = 0;       
    uint8_t sender_type = 0;    
    uint8_t resv[6] = {0};        
    uint16_t crc16 = 0;        
    uint32_t crc32 = 0;         
};

struct BroadcastAck {
    uint8_t ret_code;
    uint8_t dev_type;
    uint8_t sn[16];
    uint8_t lidar_ip[4];
    uint16_t cmd_port;
};

struct HostReq {
    uint16_t key_num = 2;
    uint16_t rsvd = 0;

    ParaInfoHeader pcl = {0x0006, 8};
    uint8_t pcl_host[8] = {0};

    ParaInfoHeader imu = {0x0007, 8};
    uint8_t imu_host[8] = {0};
};

struct CmdWriteAck {
    uint8_t ret_code;
    uint16_t error_key;
};

struct DataHeader {
    uint8_t version;            
    uint16_t length;            
    uint16_t time_interval;     
    uint16_t dot_num;           
    uint16_t udp_cnt;           
    uint8_t frame_cnt;          
    uint8_t data_type;          
    uint8_t time_type;          
    uint8_t reserved[12];       
    uint32_t crc32;             
    uint64_t timestamp;         
};

struct DataType0 {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};

struct DataType1 {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t intensity;
    uint8_t tag;
};

#pragma pack(pop)

struct PointsData {
    uint64_t timestamp;
    std::vector<DataType1> points;
};

struct ImuData {
    uint64_t timestamp;
    DataType0 imu;
};

}