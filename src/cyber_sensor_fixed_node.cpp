#include "cyber_sensor_fixed/cyber_sensor_fixed_node.hpp"

#include <cmath>

namespace CyberSensor {

CyberSensorFixedNode::CyberSensorFixedNode(const std::string& node_name)
    : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "Starting Lidar Driver Node...");

    this->declare_parameter<std::string>("local_ip", "192.168.1.5");
    this->declare_parameter<std::string>("frame_id", "cyber_sensor_fixed");

    this->get_parameter("local_ip", local_ip_);
    this->get_parameter("frame_id", frame_id_);

    std::cout << "Local ip: " << local_ip_ << std::endl;
    std::cout << "Frame id: " << frame_id_ << std::endl;

    pcl_2d_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("~/point_cloud_2d", 10);
    pcl_3d_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud_3d", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu_data", 10);

    udp_socket_ = std::make_unique<UdpSocket>(
        local_ip_,
        std::bind(&CyberSensorFixedNode::pclDataHandler, this, 
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CyberSensorFixedNode::imuDataHandler, this, 
                  std::placeholders::_1)
    );

    udp_socket_->startReceiving();

    RCLCPP_INFO(this->get_logger(), "Lidar Driver Node ready.");
}

void CyberSensorFixedNode::pclDataHandler(const PointsData line0, const PointsData line1)
{
    /*----------2d----------*/
    auto pcl_2d_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    pcl_2d_msg->header.frame_id = frame_id_;
    // pcl_2d_msg->header.stamp = this->now();
    pcl_2d_msg->header.stamp.sec = static_cast<int32_t>(line1.timestamp / 1000000000ULL);
    pcl_2d_msg->header.stamp.nanosec = static_cast<uint32_t>(line1.timestamp % 1000000000ULL);

    pcl_2d_msg->angle_min = -M_PI;
    pcl_2d_msg->angle_max = M_PI;
    pcl_2d_msg->angle_increment = M_PI / (180 * line1.points.size() / 360);
    pcl_2d_msg->time_increment = 1.0 / (15 * line1.points.size());
    pcl_2d_msg->scan_time = 1.0 / 15;
    pcl_2d_msg->range_min = 0.05;
    pcl_2d_msg->range_max = 60.0;

    for (uint16_t i = 0; i < line1.points.size(); ++i) {
        float x = line1.points[i].x / 1000.0f;
        float y = line1.points[i].y / 1000.0f;
        float range = std::sqrt(x * x + y * y);
        pcl_2d_msg->ranges.push_back(range);
        pcl_2d_msg->intensities.push_back(line1.points[i].intensity);
    }

    /*----------3d----------*/
    auto pcl_3d_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl_3d_msg->header.frame_id = frame_id_;
    // pcl_3d_msg->header.stamp = this->now();
    pcl_3d_msg->header.stamp.sec = static_cast<int32_t>(line0.timestamp / 1000000000ULL);
    pcl_3d_msg->header.stamp.nanosec = static_cast<uint32_t>(line0.timestamp % 1000000000ULL);

    pcl_3d_msg->height = 1;
    pcl_3d_msg->width = line0.points.size();
    pcl_3d_msg->is_bigendian = false;
    pcl_3d_msg->is_dense = false;

    pcl_3d_msg->fields.clear();
    pcl_3d_msg->fields.resize(7);

    uint32_t offset = 0;
    pcl_3d_msg->fields[0].name = "x";
    pcl_3d_msg->fields[0].offset = offset;
    pcl_3d_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl_3d_msg->fields[0].count = 1;
    offset += sizeof(float);

    pcl_3d_msg->fields[1].name = "y";
    pcl_3d_msg->fields[1].offset = offset;
    pcl_3d_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl_3d_msg->fields[1].count = 1;
    offset += sizeof(float);

    pcl_3d_msg->fields[2].name = "z";
    pcl_3d_msg->fields[2].offset = offset;
    pcl_3d_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl_3d_msg->fields[2].count = 1;
    offset += sizeof(float);

    pcl_3d_msg->fields[3].name = "intensity";
    pcl_3d_msg->fields[3].offset = offset;
    pcl_3d_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl_3d_msg->fields[3].count = 1;
    offset += sizeof(float);

    pcl_3d_msg->fields[4].name = "tag";
    pcl_3d_msg->fields[4].offset = offset;
    pcl_3d_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    pcl_3d_msg->fields[4].count = 1;
    offset += sizeof(uint8_t);

    pcl_3d_msg->fields[5].name = "line";
    pcl_3d_msg->fields[5].offset = offset;
    pcl_3d_msg->fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
    pcl_3d_msg->fields[5].count = 1;
    offset += sizeof(uint8_t);

    pcl_3d_msg->fields[6].name = "timestamp";
    pcl_3d_msg->fields[6].offset = offset;
    pcl_3d_msg->fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64;
    pcl_3d_msg->fields[6].count = 1;
    offset += sizeof(double);

    pcl_3d_msg->point_step = offset;
    pcl_3d_msg->row_step = pcl_3d_msg->point_step * pcl_3d_msg->width;
    pcl_3d_msg->data.resize(pcl_3d_msg->row_step);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pcl_3d_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pcl_3d_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl_3d_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(*pcl_3d_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(*pcl_3d_msg, "tag"); 
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(*pcl_3d_msg, "line");
    sensor_msgs::PointCloud2Iterator<double> iter_ts(*pcl_3d_msg, "timestamp");

    double timestamp = (double)line0.timestamp / 1000000000ULL;// + (double)(line0.timestamp % 1000000000ULL);
    
    for (const auto& point : line0.points) {
        *iter_x = (float)point.x / 1000.0f;
        *iter_y = (float)point.y / 1000.0f;
        *iter_z = (float)point.z / 1000.0f;
        *iter_i = (float)point.intensity;
        *iter_tag = 0;
        *iter_line = 0;
        *iter_ts = timestamp;
        
        timestamp += 0.000005;
        ++iter_x; ++iter_y; ++iter_z; ++iter_i; ++iter_tag; ++iter_line; ++iter_ts;
    }

    pcl_2d_publisher_->publish(std::move(pcl_2d_msg));
    pcl_3d_publisher_->publish(std::move(pcl_3d_msg));

    // std::cout << line0.points.size() << " "<< line1.points.size() << std::endl;
}

void CyberSensorFixedNode::imuDataHandler(const ImuData imu)
{
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    imu_msg->header.stamp.sec = static_cast<int32_t>(imu.timestamp / 1000000000ULL);
    imu_msg->header.stamp.nanosec = static_cast<uint32_t>(imu.timestamp % 1000000000ULL);

    imu_msg->header.frame_id = frame_id_;

    imu_msg->angular_velocity.x = imu.imu.gyro_x;
    imu_msg->angular_velocity.y = imu.imu.gyro_y;
    imu_msg->angular_velocity.z = imu.imu.gyro_z;

    imu_msg->linear_acceleration.x = imu.imu.acc_x;
    imu_msg->linear_acceleration.y = imu.imu.acc_y;
    imu_msg->linear_acceleration.z = imu.imu.acc_z;

    imu_msg->orientation.w = 1.0;
    imu_msg->orientation.x = 0.0;
    imu_msg->orientation.y = 0.0;
    imu_msg->orientation.z = 0.0;

    imu_msg->orientation_covariance[0] = -1.0;

    imu_publisher_->publish(std::move(imu_msg));
}

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto driver_node = std::make_shared<CyberSensor::CyberSensorFixedNode>("cyber_sensor_fixed_node");

    rclcpp::spin(driver_node);

    rclcpp::shutdown();
    return 0;
}