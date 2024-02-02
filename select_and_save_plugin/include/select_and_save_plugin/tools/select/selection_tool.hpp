/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SELECT_AND_SAVE_PLUGIN__TOOLS__SELECT__SELECTION_TOOL_HPP_
#define SELECT_AND_SAVE_PLUGIN__TOOLS__SELECT__SELECTION_TOOL_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_default_plugins/tools/move/move_tool.hpp"

#include <yaml-cpp/yaml.h>
#include <rosbag2_interfaces/srv/play_next.hpp>
#include "realtime_tools/realtime_box.h"
#include "select_and_save_plugin/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace Ogre {
class Viewport;
}

namespace select_and_save_plugin {
namespace tools {

// class rviz_default_plugins::tools::MoveTool;

class SELECT_AND_SAVE_PLUGIN_PUBLIC SelectionTool : public rviz_common::Tool {
   public:
    struct Point{
      Ogre::Vector3 xyz;
      float intensity;
      uint16_t ring;
    };

    struct DataToSave{
      Ogre::Vector3 mean_point;
      Ogre::Vector3 bbox;
      std::list<Point> points;
    };

    SelectionTool();
    virtual ~SelectionTool();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);
    virtual int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel);

    virtual void update(float wall_dt, float ros_dt);

   private:
    rviz_default_plugins::tools::MoveTool* move_tool_;
    int processSelectedArea();
    void save_data_to_yaml();
    bool selecting_;
    int sel_start_x_;
    int sel_start_y_;

    rviz_common::interaction::M_Picked highlight_;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client;
    realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::PointCloud2>> receiver_pc2_msg_ptr;

    std::unique_ptr<DataToSave> data;

    rclcpp::executors::MultiThreadedExecutor executor_;
    std::unique_ptr<std::thread> executor_thread_;
    bool moving_;
};

}  // namespace tools
}  // namespace select_and_save_plugin

#endif  // SELECT_AND_SAVE_PLUGIN__TOOLS__SELECT__SELECTION_TOOL_HPP_
