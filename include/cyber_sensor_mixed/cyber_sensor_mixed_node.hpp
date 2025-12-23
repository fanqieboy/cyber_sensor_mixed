#ifndef CYBER_SENSOR_MIXED_NODE_HPP
#define CYBER_SENSOR_MIXED_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "cyber_sensor_mixed/udp_socket.h"
#include "cyber_sensor_mixed/data_structs.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace CyberSensor{

class CyberSensorMixedNode : public rclcpp::Node
{
public:
    CyberSensorMixedNode(const std::string& node_name);

private:
    std::unique_ptr<UdpSocket> udp_socket_; 

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pcl_2d_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_3d_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    void pclDataHandler(const PointsData line0, const PointsData line1);
    void imuDataHandler(const ImuData imu);

    std::string frame_id_;
    std::string local_ip_;
};

}

#endif // CYBER_SENSOR_MIXED_NODE_HPP
