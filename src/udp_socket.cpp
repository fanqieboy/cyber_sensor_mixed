#include "cyber_sensor_fixed/udp_socket.h"
#include "cyber_sensor_fixed/data_parser.h"
#include "cyber_sensor_fixed/data_structs.h"
#include "cyber_sensor_fixed/crc.h"

#include <cstring>
#include <chrono>

namespace CyberSensor {

constexpr int BUF_SIZE = 65536;

UdpSocket::UdpSocket(const std::string& local_ip,
                     PclCallback pclCallback,
                     ImuCallback imuCallback)
    : local_ip_(local_ip)
    , callback_pcl_(std::move(pclCallback))
    , callback_imu_(std::move(imuCallback))
{
    std::cout << "UdpSocket initialized with local IP: " << local_ip_ << std::endl;

    std::string lidar_ip;
    uint16_t cmd_port;

    // configLidar(local_ip_, lidar_ip, cmd_port);
    if (discoverLidar(local_ip_, lidar_ip, cmd_port)) {
        configLidar(local_ip_, lidar_ip, cmd_port);
    }
    else {
        std::cout << "can't find lidar" << std::endl;
    }
}

UdpSocket::~UdpSocket()
{
    for (auto& t : receiver_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}

bool UdpSocket::setupSocket(int port, int& sockfd)
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error: Failed to create socket for port " << port << std::endl;
        return false;
    }

    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Error: Set reuse failed for port " << port << std::endl;
        close(sockfd);
        return false;
    }

    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY); 
    local_addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Error: Bind failed for port " << port << std::endl;
        close(sockfd);
        return false;
    }

    std::cout << "UDP Receiver started on port " << port << std::endl;
    return true;
}

bool UdpSocket::discoverLidar(const std::string local_ip, std::string& lidar_ip, uint16_t& cmd_port)
{
    int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0) {
        return false;
    }

    sockaddr_in send_addr{};
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());
    send_addr.sin_port = htons(56000);
    if (bind(send_sock, (sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
        close(send_sock);
        return false;
    }

    int enable = 1;
    setsockopt(send_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    setsockopt(send_sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    setsockopt(send_sock, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(enable));

    int recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sock < 0) {
        return false;
    }

    setsockopt(recv_sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    setsockopt(recv_sock, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(enable));

    sockaddr_in recv_addr{};
    memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = INADDR_ANY;
    recv_addr.sin_port = htons(56000);
    if (bind(recv_sock, (sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
        close(send_sock);
        close(recv_sock);
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    setsockopt(recv_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in broadcast_addr{};
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_addr.s_addr = INADDR_BROADCAST;
    broadcast_addr.sin_port = htons(56000);

    CmdHeader req_header;
    req_header.cmd_id = 0;
    req_header.cmd_type = 0;
    req_header.length = sizeof(CmdHeader);
    req_header.crc16 = crc16_maker((const uint8_t *)&req_header, sizeof(req_header) - 6);
    
    if (sendto(send_sock, (const char*)&req_header, sizeof(req_header), 0, (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr)) < 0) {
        close(send_sock);
        close(recv_sock);
        return false;
    }

    char buffer[1024];
    struct sockaddr_in remote_addr;
    socklen_t addr_len = sizeof(remote_addr);
    while (true) {
        int n = recvfrom(recv_sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&remote_addr, &addr_len);

        if (n < 0) {
            break;
        }

        if (n == sizeof(CmdHeader) + sizeof(BroadcastAck)) {
            CmdHeader ack_header;
            BroadcastAck ack;
            memcpy(&ack_header, buffer, sizeof(ack_header));
            memcpy(&ack, buffer + sizeof(ack_header), sizeof(ack));

            if (ack_header.sof == 0xaa && ack_header.cmd_id == 0 && ack_header.cmd_type == 0x01) {
                cmd_port = ack.cmd_port;
                lidar_ip = std::to_string(ack.lidar_ip[0]) + "." + 
                           std::to_string(ack.lidar_ip[1]) + "." + 
                           std::to_string(ack.lidar_ip[2]) + "." + 
                           std::to_string(ack.lidar_ip[3]);

                std::cout << "lidar ip: " << lidar_ip << " cmd port: " << cmd_port << std::endl;

                close(send_sock);
                close(recv_sock);
                return true;
            }
        }
    }

    close(send_sock);
    close(recv_sock);
    return false;
}

bool UdpSocket::configLidar(const std::string local_ip, const std::string lidar_ip, const uint16_t cmd_port)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        return false;
    }

    sockaddr_in local_addr{};
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());
    local_addr.sin_port = htons(56000);
    bind(sockfd, (sockaddr *)&local_addr, sizeof(local_addr));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in lidar_addr{};
    lidar_addr.sin_family = AF_INET;
    lidar_addr.sin_addr.s_addr = inet_addr(lidar_ip.c_str());;
    lidar_addr.sin_port = htons(cmd_port);

    char buf[1024];
    CmdHeader cmd_header;
    cmd_header.seq_num = 10;
    cmd_header.cmd_id = 0x0100;
    cmd_header.cmd_type = 0x00;
    cmd_header.sender_type = 0;
    
    uint16_t pcl_port = 56301, imu_port = 56401;
    HostReq host_req;
    if (inet_pton(AF_INET, local_ip.c_str(), host_req.pcl_host) != 1) {
        return false;
    }

    memcpy(&host_req.pcl_host[4], &pcl_port, sizeof(pcl_port));

    if (inet_pton(AF_INET, local_ip.c_str(), host_req.imu_host) != 1) {
        close(sockfd);
        return false;
    }

    memcpy(&host_req.imu_host[4], &imu_port, sizeof(imu_port));

    cmd_header.length = sizeof(cmd_header) + sizeof(host_req);
    cmd_header.crc16 = crc16_maker((const uint8_t *)&cmd_header, sizeof(cmd_header) - 6);
    cmd_header.crc32 = crc32_maker((const uint8_t *)&host_req, sizeof(host_req));

    memcpy(buf, &cmd_header, sizeof(cmd_header));
    memcpy(buf + sizeof(cmd_header), &host_req, sizeof(host_req));
    if (sendto(sockfd, buf, sizeof(cmd_header) + sizeof(host_req), 0, (struct sockaddr*)&lidar_addr, sizeof(lidar_addr)) < 0) {
        close(sockfd);
        return false;
    }

    sockaddr_in remote_addr{};
    socklen_t remote_len = sizeof(remote_addr);

    ssize_t bytes = recvfrom(
        sockfd, buf, sizeof(buf), 0, 
        (struct sockaddr*)&remote_addr, &remote_len
    );

    if (bytes == -1) {
        close(sockfd);
        return false;
    }

    if (bytes == sizeof(CmdHeader) + sizeof(CmdWriteAck)) {
        CmdWriteAck ack;
        memcpy(&ack, buf + sizeof(CmdHeader), sizeof(ack));

        if (ack.ret_code == 0) {
            std::cout << "host config ok" << std::endl;
            close(sockfd);
            return true;
        }
    }

    close(sockfd);
    return false;
}

void UdpSocket::receiveInfo()
{
    int sockfd;
    if (!setupSocket(56201, sockfd)) return;

    struct sockaddr_in remote_addr;
    socklen_t len = sizeof(remote_addr);
    std::vector<uint8_t> buffer(BUF_SIZE);
    
    while (true) {
        int n = recvfrom(sockfd, buffer.data(), BUF_SIZE, MSG_WAITALL, 
                         (struct sockaddr *)&remote_addr, &len);

        if (n < 0) {
            std::cerr << "Error: recvfrom failed on port 56201. Thread exiting." << std::endl;
            break; 
        }
        
        if (!buffer.empty()) {
            if (findTimeSyncType(buffer.data(), n)) {
                sync_en_.store(1);
            }
            else {
                sync_en_.store(0);
            }
        }
    }

    close(sockfd);
}

void UdpSocket::receivePcl()
{
    int sockfd;
    if (!setupSocket(56301, sockfd)) return;

    struct sockaddr_in remote_addr;
    socklen_t len = sizeof(remote_addr);
    std::vector<uint8_t> buffer(BUF_SIZE);

    PointsData line0_data, line1_data;
    
    while (true) {
        int n = recvfrom(sockfd, buffer.data(), BUF_SIZE, MSG_WAITALL, 
                         (struct sockaddr *)&remote_addr, &len);

        if (n < 0) {
            std::cerr << "Error: recvfrom failed on port 56301. Thread exiting." << std::endl;
            break; 
        }

        if ((uint16_t)n < sizeof(DataHeader)) {
            std::cout << "Pcl data header length error" << std::endl;
            break;
        }

        const DataHeader *header = reinterpret_cast<const DataHeader*>(buffer.data());

        if (header->length != sizeof(DataHeader) + header->dot_num * sizeof(DataType1)) {
            std::cout << "Pcl data header length error" << std::endl;
            break;
        }

        const uint8_t *data_start = buffer.data() + sizeof(DataHeader);
        const DataType1 *data_ptr = reinterpret_cast<const DataType1*>(data_start);

        for (uint16_t i = 0; i < header->dot_num; ++i) {
            const DataType1& point = data_ptr[i];

            uint8_t tag = point.tag & 0xc0;
            uint8_t new_frame = point.tag & 0x20;

            if (tag == 0) {
                if (line0_data.points.size() == 0) {
                    if (sync_en_.load() == 1) {
                        line0_data.timestamp = header->timestamp;
                    }
                    else {
                        line0_data.timestamp = getCurrentTime();
                    }
                }

                line0_data.points.push_back(point);
            }
            else {
                if (line1_data.points.size() == 0) {
                    if (sync_en_.load() == 1) {
                        line1_data.timestamp = header->timestamp;
                    }
                    else {
                        line1_data.timestamp = getCurrentTime();
                    }
                }

                line1_data.points.push_back(point);
            }

            if (new_frame) {
                if (callback_pcl_) {
                    callback_pcl_(line0_data, line1_data);
                    line0_data.points.clear();
                    line1_data.points.clear();
                }
            }
        }
    }

    close(sockfd);
}

void UdpSocket::receiveImu()
{
    int sockfd;
    if (!setupSocket(56401, sockfd)) return;

    struct sockaddr_in remote_addr;
    socklen_t len = sizeof(remote_addr);
    std::vector<uint8_t> buffer(BUF_SIZE);
    
    while (true) {
        int n = recvfrom(sockfd, buffer.data(), BUF_SIZE, MSG_WAITALL, 
                         (struct sockaddr *)&remote_addr, &len);

        if (n < 0) {
            std::cerr << "Error: recvfrom failed on port 56401. Thread exiting." << std::endl;
            break; 
        }

        const DataHeader *header = reinterpret_cast<const DataHeader*>(buffer.data());

        if (header->length != sizeof(DataHeader) + sizeof(DataType0)) {
            std::cout << "Imu data header length error" << std::endl;
            break;
        }

        const uint8_t *data_start = buffer.data() + sizeof(DataHeader);
        const DataType0 *data_ptr = reinterpret_cast<const DataType0*>(data_start);
        
        ImuData imu;
        imu.imu.gyro_x = data_ptr->gyro_x;
        imu.imu.gyro_y = data_ptr->gyro_y;
        imu.imu.gyro_z = data_ptr->gyro_z;
        imu.imu.acc_x = data_ptr->acc_x;
        imu.imu.acc_y = data_ptr->acc_y;
        imu.imu.acc_z = data_ptr->acc_z;

        if (sync_en_.load() == 1) {
            imu.timestamp = header->timestamp;
        } else {
            imu.timestamp = getCurrentTime();
        }

        if (callback_imu_) {
            callback_imu_(imu);
        }
    }
    
    close(sockfd);
}

// 启动所有接收线程
void UdpSocket::startReceiving() {
    receiver_threads_.emplace_back(&UdpSocket::receiveInfo, this);

    receiver_threads_.emplace_back(&UdpSocket::receivePcl, this);

    receiver_threads_.emplace_back(&UdpSocket::receiveImu, this);
}

uint64_t UdpSocket::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();

    auto duration = now.time_since_epoch();

    uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

    return timestamp_ns;
}

}
