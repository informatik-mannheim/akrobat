/****************************************************************************\
 * Copyright (C) 2021 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <QLineEdit>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>

#include <thread>

#define MIN_FILTER 0
#define MAX_FILTER 750
#define MIN_DIVISOR 1
#define MAX_DIVISOR 2000

namespace royale_in_ros2
{
    class RoyaleControl: public rviz_common::Panel
    {
        Q_OBJECT

    public:
        RoyaleControl (QWidget *parent = 0);
        ~RoyaleControl ();

    private Q_SLOTS:
        void setUseCase (const QString &currentMode);
        void setExposureTime (int value);
        void setExposureMode (bool isAutomatic);
        void setMinFilter (int value);
        void setMaxFilter (int value);
        void setDivisor (int value);

        // The precise value can be entered directly via the text editor.
        void preciseExposureTimeSetting();
        void preciseMinFilterSetting();
        void preciseMaxFilterSetting();
        void preciseDivisorSetting();

    private:
        // Callback the camera initial settings after the UI is started
        void callbackInit (const std_msgs::msg::String::SharedPtr msg);

        // Callback the current parameters of exposure time
        // when the UI is started or user case is switched
        void callbackExpoTimeParam (const std_msgs::msg::String::SharedPtr msg);
        void callbackExpoTimeValue (const std_msgs::msg::UInt32::SharedPtr msg);

        // Callback the fps and display it
        void callbackFps (const std_msgs::msg::String::SharedPtr msg);

        void spin();

        QComboBox  *m_comboBoxUseCases;
        QLabel     *m_labelExpoTime;
        QSlider    *m_sliderExpoTime;
        QLineEdit  *m_lineEditExpoTime;
        QCheckBox  *m_checkBoxAutoExpo;
        QLabel     *m_labelEditFps;
        QSlider    *m_sliderDivisor;
        QLineEdit  *m_lineEditDivisor;
        QSlider    *m_sliderMinFilter;
        QLineEdit  *m_lineEditMinFilter;
        QSlider    *m_sliderMaxFilter;
        QLineEdit  *m_lineEditMaxFilter;

        rclcpp::Node::SharedPtr m_nh;
        rclcpp::executors::SingleThreadedExecutor m_exec;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  m_pubIsInit;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  m_pubUseCase;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr  m_pubExpoTime;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  m_pubExpoMode;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr  m_pubMinFilter;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr  m_pubMaxFilter;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr  m_pubDivisor;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subInit;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subExpoTimeParam;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_subExpoTimeValue;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subFps;

        std::thread m_thread;

        bool  m_isInit;
        bool  m_isAutomatic;
        int   m_exposureTime;
        int   m_minETSlider;
        int   m_maxETSlider;

    };
}
