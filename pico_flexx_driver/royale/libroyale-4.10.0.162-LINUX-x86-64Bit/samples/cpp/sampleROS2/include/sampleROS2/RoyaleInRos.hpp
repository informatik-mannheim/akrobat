/****************************************************************************\
 * Copyright (C) 2021 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <royale.hpp>

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>

namespace royale_in_ros2
{
    class RoyaleInRos :
        public rclcpp::Node, public royale::IDepthDataListener, public royale::IExposureListener
    {
    public:
        RoyaleInRos();
        ~RoyaleInRos();

        void onInit();

        // Starting and stopping the camera
        void start();
        void stop();

    private:
        // Called by Royale for every new frame
        void onNewData (const royale::DepthData *data) override;

        // Will be called when the newly calculated exposure time deviates from currently set exposure time of the current UseCase
        void onNewExposure (const uint32_t newExposureTime) override;

        // Create cameraInfo, return true if the setting is successful, otherwise false
        bool setCameraInfo();

        // Calculate the FPS
        void fpsUpdate();

        // The required parameters for UI's initialization
        void initMsgUpdate();

        // Callback a bool value to determine whether the UI is initialized
        void callbackIsInit (const std_msgs::msg::Bool::SharedPtr msg);

        // Callback the changed settings from UI and set the changed settings
        void callbackUseCase (const std_msgs::msg::String::SharedPtr msg);
        void callbackExpoTime (const std_msgs::msg::UInt32::SharedPtr msg);
        void callbackExpoMode (const std_msgs::msg::Bool::SharedPtr msg);
        void callbackMinFiler (const std_msgs::msg::Float32::SharedPtr msg);
        void callbackMaxFiler (const std_msgs::msg::Float32::SharedPtr msg);
        void callbackDivisor (const std_msgs::msg::UInt16::SharedPtr msg);

        sensor_msgs::msg::CameraInfo m_cameraInfo;
        std_msgs::msg::String        m_msgInitPanel;
        std_msgs::msg::String        m_msgExpoTimeParam;
        std_msgs::msg::UInt32        m_msgExpoTimeValue;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  m_pubCameraInfo;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  m_pubCloud;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  m_pubDepth;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  m_pubGray;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  m_pubInit;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  m_pubExpoTimeParam;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr  m_pubExpoTimeValue;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  m_pubFps;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subIsInit;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subUseCase;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_subExpoTime;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subExpoMode;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subMinFilter;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subMaxFilter;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr m_subDivisor;

        std::unique_ptr<royale::ICameraDevice> m_cameraDevice;
        royale::Pair<uint32_t, uint32_t>       m_limits;

        std::string     m_useCases;
        std::thread     m_fpsProcess;
        uint64_t        m_frames;
        uint32_t        m_exposureTime;
        uint16_t        m_grayDivisor;
        float           m_minFilter;
        float           m_maxFilter;
        bool            m_initPanel;
        bool            m_autoExposure;
    };
}
