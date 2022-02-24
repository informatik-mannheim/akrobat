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
#include <RoyaleInRos.hpp>

int main (int argc, char **argv)
{
    rclcpp::init (argc, argv);

    rclcpp::spin (std::make_shared<royale_in_ros2::RoyaleInRos>());
    rclcpp::shutdown();

    return 0;
}
