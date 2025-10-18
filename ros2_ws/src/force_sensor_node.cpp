/**
 * @file force_sensor_node.cpp
 * @brief ROS2 node for publishing 6-axis force sensor data via EtherCAT
 * 
 * This node integrates SOEM EtherCAT library with ROS2 to publish
 * force sensor data as custom ROS2 messages.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <memory>
#include <string>

// SOEM headers
extern "C" {
#include "soem/soem.h"
}

// Custom message
#include "ros2_force_sensor/msg/force_sensor_data.hpp"

class ForceSensorNode : public rclcpp::Node
{
public:
    ForceSensorNode() : Node("force_sensor_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("interface", "eth0");
        this->declare_parameter<int>("cycle_time_us", 8000);
        this->declare_parameter<double>("fx_offset", -16.35);
        this->declare_parameter<double>("fy_offset", -6.70);
        this->declare_parameter<double>("fz_offset", 12.35);
        
        // 获取参数
        interface_ = this->get_parameter("interface").as_string();
        cycle_time_us_ = this->get_parameter("cycle_time_us").as_int();
        fx_offset_ = this->get_parameter("fx_offset").as_double();
        fy_offset_ = this->get_parameter("fy_offset").as_double();
        fz_offset_ = this->get_parameter("fz_offset").as_double();
        
        // 计算频率
        frequency_hz_ = 1000000.0 / cycle_time_us_;
        
        RCLCPP_INFO(this->get_logger(), "Starting Force Sensor Node");
        RCLCPP_INFO(this->get_logger(), "Interface: %s", interface_.c_str());
        RCLCPP_INFO(this->get_logger(), "Cycle time: %d us (%.1f Hz)", cycle_time_us_, frequency_hz_);
        
        // 初始化变量
        ethercat_initialized_ = false;
        
        // 创建发布器
        publisher_ = this->create_publisher<ros2_force_sensor::msg::ForceSensorData>(
            "force_sensor_data", 10);
        
        // 总是创建定时器，但在发布函数中检查EtherCAT状态
        auto cycle_time = std::chrono::microseconds(cycle_time_us_);
        timer_ = this->create_wall_timer(
            cycle_time, std::bind(&ForceSensorNode::publish_sensor_data, this));
        
        // 初始化EtherCAT
        if (init_ethercat()) {
            ethercat_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Force sensor node started successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT");
            RCLCPP_ERROR(this->get_logger(), "Please check:");
            RCLCPP_ERROR(this->get_logger(), "1. Run with sudo: sudo -E ros2 launch ...");
            RCLCPP_ERROR(this->get_logger(), "2. EtherCAT device is connected to %s", interface_.c_str());
            RCLCPP_ERROR(this->get_logger(), "3. Network interface %s exists and is up", interface_.c_str());
            RCLCPP_ERROR(this->get_logger(), "Node will continue running but no data will be published");
        }
    }
    
    ~ForceSensorNode()
    {
        cleanup_ethercat();
    }

private:
    // EtherCAT 相关变量
    ecx_contextt context_;
    uint8_t map_[4096];
    std::string interface_;
    int cycle_time_us_;
    double frequency_hz_;
    double fx_offset_, fy_offset_, fz_offset_;
    bool ethercat_initialized_;
    
    // ROS2 相关
    rclcpp::Publisher<ros2_force_sensor::msg::ForceSensorData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool init_ethercat()
    {
        // 初始化上下文
        memset(&context_, 0, sizeof(context_));
        
        RCLCPP_INFO(this->get_logger(), "Initializing SOEM on '%s'...", interface_.c_str());
        if (!ecx_init(&context_, interface_.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "No socket connection on %s", interface_.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "SOEM initialization done");
        
        RCLCPP_INFO(this->get_logger(), "Finding autoconfig slaves...");
        if (ecx_config_init(&context_) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No slaves found");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "%d slaves found", context_.slavecount);
        
        RCLCPP_INFO(this->get_logger(), "Sequential mapping of I/O...");
        ecx_config_map_group(&context_, map_, 0);
        ec_groupt *grp = context_.grouplist;
        RCLCPP_INFO(this->get_logger(), "Mapped %dO+%dI bytes from %d segments", 
                   grp->Obytes, grp->Ibytes, grp->nsegments);
        
        RCLCPP_INFO(this->get_logger(), "Configuring distributed clock...");
        ecx_configdc(&context_);
        RCLCPP_INFO(this->get_logger(), "DC configuration done");
        
        RCLCPP_INFO(this->get_logger(), "Waiting for all slaves in safe operational...");
        ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        RCLCPP_INFO(this->get_logger(), "Slaves in safe operational");
        
        // 发送一个往返包
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRET);
        RCLCPP_INFO(this->get_logger(), "Initial roundtrip sent");
        
        RCLCPP_INFO(this->get_logger(), "Setting operational state...");
        ec_slavet *slave = context_.slavelist;
        slave->state = EC_STATE_OPERATIONAL;
        ecx_writestate(&context_, 0);
        
        // 等待进入操作状态
        for (int i = 0; i < 10; ++i) {
            ecx_send_processdata(&context_);
            ecx_receive_processdata(&context_, EC_TIMEOUTRET);
            ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
            if (slave->state == EC_STATE_OPERATIONAL) {
                RCLCPP_INFO(this->get_logger(), "All slaves are now operational");
                return true;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Failed to reach operational state");
        return false;
    }
    
    void cleanup_ethercat()
    {
        if (!ethercat_initialized_) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Shutting down EtherCAT...");
        
        // 设置为INIT状态
        ec_slavet *slave = context_.slavelist;
        slave->state = EC_STATE_INIT;
        ecx_writestate(&context_, 0);
        
        // 关闭连接
        ecx_close(&context_);
        ethercat_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "EtherCAT shutdown complete");
    }
    
    void publish_sensor_data()
    {
        if (!ethercat_initialized_) {
            // EtherCAT未初始化，尝试重新初始化
            static int retry_count = 0;
            if (++retry_count % 1000 == 0) {  // 每隔几秒尝试一次
                RCLCPP_WARN(this->get_logger(), "EtherCAT not initialized, attempting to reconnect...");
                if (init_ethercat()) {
                    ethercat_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "EtherCAT reconnected successfully!");
                    retry_count = 0;
                }
            }
            return;
        }
        
        // 执行EtherCAT通信
        auto start_time = std::chrono::high_resolution_clock::now();
        ecx_send_processdata(&context_);
        int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRET);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // 计算往返时间
        auto roundtrip_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();
        
        ec_groupt *grp = context_.grouplist;
        int expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;
        
        // 检查通信质量
        if (wkc < expected_wkc) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "WKC mismatch: got %d, expected %d", wkc, expected_wkc);
            return;
        }
        
        // 解析传感器数据
        if (grp->Ibytes >= 26) {  // 确保有足够的输入数据
            auto msg = ros2_force_sensor::msg::ForceSensorData();
            
            // 设置消息头
            msg.header.stamp = this->now();
            msg.header.frame_id = "force_sensor";
            
            // 解析数据编号 (UINT16, 小端)
            msg.data_no = (grp->inputs[1] << 8) | grp->inputs[0];
            
            // 解析力数据 (REAL32, 小端)
            uint32_t fx_raw = (grp->inputs[5] << 24) | (grp->inputs[4] << 16) | 
                             (grp->inputs[3] << 8) | grp->inputs[2];
            uint32_t fy_raw = (grp->inputs[9] << 24) | (grp->inputs[8] << 16) | 
                             (grp->inputs[7] << 8) | grp->inputs[6];
            uint32_t fz_raw = (grp->inputs[13] << 24) | (grp->inputs[12] << 16) | 
                             (grp->inputs[11] << 8) | grp->inputs[10];
            
            float fx, fy, fz;
            memcpy(&fx, &fx_raw, sizeof(fx));
            memcpy(&fy, &fy_raw, sizeof(fy));
            memcpy(&fz, &fz_raw, sizeof(fz));
            
            msg.force.x = fx + fx_offset_;
            msg.force.y = fy + fy_offset_;
            msg.force.z = fz + fz_offset_;
            
            // 解析力矩数据 (REAL32, 小端)
            uint32_t mx_raw = (grp->inputs[17] << 24) | (grp->inputs[16] << 16) | 
                             (grp->inputs[15] << 8) | grp->inputs[14];
            uint32_t my_raw = (grp->inputs[21] << 24) | (grp->inputs[20] << 16) | 
                             (grp->inputs[19] << 8) | grp->inputs[18];
            uint32_t mz_raw = (grp->inputs[25] << 24) | (grp->inputs[24] << 16) | 
                             (grp->inputs[23] << 8) | grp->inputs[22];
            
            float mx, my, mz;
            memcpy(&mx, &mx_raw, sizeof(mx));
            memcpy(&my, &my_raw, sizeof(my));
            memcpy(&mz, &mz_raw, sizeof(mz));
            
            msg.torque.x = mx;
            msg.torque.y = my;
            msg.torque.z = mz;
            
            // 设置通信信息
            msg.roundtrip_time_us = static_cast<uint32_t>(roundtrip_time);
            msg.wkc = static_cast<uint32_t>(wkc);
            msg.communication_ok = (wkc >= expected_wkc);
            
            // 发布消息
            publisher_->publish(msg);
            
            // 定期打印调试信息
            static int counter = 0;
            if (++counter % 100 == 0) {  // 每100次打印一次
                RCLCPP_INFO(this->get_logger(), 
                           "DataNo=%d | F=[%.3f, %.3f, %.3f] N | T=[%.3f, %.3f, %.3f] Nm | RT=%ld us",
                           msg.data_no, msg.force.x, msg.force.y, msg.force.z,
                           msg.torque.x, msg.torque.y, msg.torque.z, roundtrip_time);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ForceSensorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("force_sensor_node"), 
                     "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}