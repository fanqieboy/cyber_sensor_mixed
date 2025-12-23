#ifndef UDPSOCKET_HPP
#define UDPSOCKET_HPP

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "cyber_sensor_mixed/data_structs.h"

namespace CyberSensor {

class UdpSocket {
public:
    using PclCallback = std::function<void(const PointsData, const PointsData)>;
    using ImuCallback = std::function<void(const ImuData)>;

    UdpSocket(const std::string& local_ip,
              PclCallback pclCallback,
              ImuCallback imuCallback);
    ~UdpSocket();

    void startReceiving();

private:
    std::string local_ip_;
    
    std::atomic<int> sync_en_{0}; 

    std::vector<std::thread> receiver_threads_;

    PclCallback callback_pcl_;
    ImuCallback callback_imu_;

    bool setupSocket(int port, int& sockfd);

    bool discoverLidar(const std::string local_ip, std::string& lidar_ip, uint16_t& cmd_port);
    bool configLidar(const std::string local_ip, const std::string lidar_ip, const uint16_t cmd_port);

    void receiveInfo();
    void receivePcl();
    void receiveImu();

    uint64_t getCurrentTime();
};

}

#endif // UDPSOCKET_HPP
