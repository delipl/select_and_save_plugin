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

#include "select_and_save_plugin/tools/select/selection_tool.hpp"

#include <OgreCamera.h>
#include <OgreMaterialManager.h>
#include <OgreMovableObject.h>
#include <OgreRay.h>
#include <OgreRectangle2D.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include "rviz_default_plugins/tools/move/move_tool.hpp"

#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace select_and_save_plugin {
namespace tools {

SelectionTool::SelectionTool()
    : Tool(),
      move_tool_(new rviz_default_plugins::tools::MoveTool()),
      selecting_(false),
      sel_start_x_(0),
      sel_start_y_(0),
      moving_(false) {
    shortcut_key_ = 's';
    access_all_keys_ = true;
}

SelectionTool::~SelectionTool() { delete move_tool_; }

void SelectionTool::onInitialize() {
    move_tool_->initialize(context_);
    node = std::make_shared<rclcpp::Node>("select_and_save_plugin");
    pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "velodyne_points", 10, [this](const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) -> void {
            RCLCPP_INFO_STREAM(node->get_logger(), "Lidar callback");
            std::shared_ptr<sensor_msgs::msg::PointCloud2> copied_message(msg);
            receiver_pc2_msg_ptr.set(copied_message);
        });
    play_next_client = node->create_client<rosbag2_interfaces::srv::PlayNext>("/rosbag2_player/play_next");

    executor_.add_node(node);
    executor_thread_ =
        std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));
}

void SelectionTool::activate() {
    setStatus("Click and drag to select objects on the screen.");
    context_->getSelectionManager()->setTextureSize(512);
    selecting_ = false;
    moving_ = false;
    //  context_->getSelectionManager()->enableInteraction(true);
}

void SelectionTool::deactivate() { context_->getSelectionManager()->removeHighlight(); }

void SelectionTool::update(float wall_dt, float ros_dt) {
    (void)wall_dt;
    (void)ros_dt;
    auto sel_manager = context_->getSelectionManager();

    if (!selecting_) {
        sel_manager->removeHighlight();
    }
}

int SelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    auto sel_manager = context_->getSelectionManager();

    int flags = 0;

    if (event.alt()) {
        moving_ = true;
        selecting_ = false;
    } else {
        moving_ = false;

        if (event.leftDown()) {
            selecting_ = true;

            sel_start_x_ = event.x;
            sel_start_y_ = event.y;
        }
    }

    if (selecting_) {
        sel_manager->highlight(event.panel->getRenderWindow(), sel_start_x_, sel_start_y_, event.x, event.y);

        if (event.leftUp()) {
            rviz_common::interaction::SelectionManager::SelectType type =
                rviz_common::interaction::SelectionManager::Replace;

            rviz_common::interaction::M_Picked selection;

            if (event.shift()) {
                type = rviz_common::interaction::SelectionManager::Add;
            } else if (event.control()) {
                type = rviz_common::interaction::SelectionManager::Remove;
            }

            sel_manager->select(event.panel->getRenderWindow(), sel_start_x_, sel_start_y_, event.x, event.y, type);

            processSelectedArea();
            selecting_ = false;
        }

        flags |= Render;
    } else if (moving_) {
        sel_manager->removeHighlight();

        flags = move_tool_->processMouseEvent(event);

        if (event.type == QEvent::MouseButtonRelease) {
            moving_ = false;
        }
    } else {
        sel_manager->highlight(event.panel->getRenderWindow(), event.x, event.y, event.x, event.y);
    }

    return flags;
}

int SelectionTool::processSelectedArea() {
    RCLCPP_INFO_STREAM(node->get_logger(), "Getting selected points...");
    auto selection_manager = context_->getSelectionManager();
    rviz_common::interaction::M_Picked selection = selection_manager->getSelection();
    rviz_common::properties::PropertyTreeModel* model = selection_manager->getPropertyModel();
    data = std::make_unique<DataToSave>();
    int i = 0;

    Ogre::Vector3 mean_point;
    Ogre::Vector3 max_point(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                            -std::numeric_limits<float>::max());
    Ogre::Vector3 min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
    while (model->hasIndex(i, 0)) {
        QModelIndex child_index = model->index(i, 0);

        auto child = model->getProp(child_index);
        auto subchild = (rviz_common::properties::VectorProperty*)child->childAt(0);

        float intensity = 0.0;
        uint16_t ring = 0;
        for (int j = 1; j < child->numChildren(); j++) {
            auto grandchild = child->childAt(j);
            QString nameOfChild = grandchild->getName();
            QString nameOfIntensity("intensity");
            QString nameOfRing("ring");

            if (nameOfChild.contains(nameOfIntensity)) {
                auto floatchild = (rviz_common::properties::FloatProperty*)grandchild;
                intensity = floatchild->getValue().toFloat();
            }

            if (nameOfChild.contains(nameOfRing)) {
                auto floatchild = (rviz_common::properties::IntProperty*)grandchild;
                ring = floatchild->getValue().toInt();
            }
        }

        Point point;
        point.xyz = subchild->getVector();
        point.intensity = intensity;
        point.ring = ring;
        data->points.push_back(point);
        mean_point += point.xyz;
        max_point.x = std::max(max_point.x, point.xyz.x);
        max_point.y = std::max(max_point.y, point.xyz.z);
        max_point.z = std::max(max_point.z, point.xyz.z);

        min_point.x = std::min(min_point.x, point.xyz.x);
        min_point.y = std::min(min_point.y, point.xyz.z);
        min_point.z = std::min(min_point.z, point.xyz.z);

        RCLCPP_INFO_STREAM(node->get_logger(),
                           "Point: " << point.xyz << " Intensity: " << intensity << " Ring: " << ring);

        i++;
    }
    if (i == 0) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Did not select any points");
        data.reset();
        return -1;
    }
    data->mean_point = mean_point / i;
    data->bbox = max_point - min_point;
    RCLCPP_INFO_STREAM(node->get_logger(), "Number of points in the selected area: " << i);

    // selected_points_.width = i;
    // selected_points_.header.stamp = ros::Time::now();
    return 0;
}

int SelectionTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) {
    (void)panel;
    auto sel_manager = context_->getSelectionManager();

    if (event->key() == Qt::Key_F) {
        sel_manager->focusOnSelection();
    }

    else if (event->key() == Qt::Key_P) {
        save_data_to_yaml();
    } else if (event->key() == Qt::Key_N) {
        RCLCPP_INFO(node->get_logger(), "Playing next ros2bag message");
        auto request = std::make_shared<rosbag2_interfaces::srv::PlayNext::Request>();
        auto result = play_next_client->async_send_request(request);
    }

    return Render;
}

void SelectionTool::save_data_to_yaml() {
    auto start = std::chrono::high_resolution_clock::now();
    YAML::Node yaml_node;
    YAML::Node frame_node;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2;
    receiver_pc2_msg_ptr.get(pc2);
    if (pc2 != nullptr) {
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "timestamp: " << pc2->header.stamp.sec * 1.0 + pc2->header.stamp.nanosec * 10e-9 << "s");
    }

    frame_node["timestamp"]["sec"] = pc2->header.stamp.sec;
    frame_node["timestamp"]["nanosec"] = pc2->header.stamp.nanosec;
    frame_node["frame_id"] = context_->getFixedFrame().toStdString();
    frame_node["box_size"]["x"] = data->bbox.x;
    frame_node["box_size"]["y"] = data->bbox.y;
    frame_node["box_size"]["z"] = data->bbox.z;

    frame_node["mean_point"]["x"] = data->mean_point.x;
    frame_node["mean_point"]["y"] = data->mean_point.y;
    frame_node["mean_point"]["z"] = data->mean_point.z;

    YAML::Node points_node;
    for(const auto &point : data->points){
      YAML::Node point_node;
      point_node["x"] = point.xyz.x;
      point_node["y"] = point.xyz.y;
      point_node["z"] = point.xyz.z;
      point_node["intensity"] = point.intensity;
      point_node["ring"] = point.ring;
      frame_node["points"].push_back(point_node);
    }

    yaml_node.push_back(frame_node);
    std::ofstream file("rviz_selected_supports.yaml", std::ios::app);
    if (file.is_open()) {
        file << yaml_node << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        RCLCPP_INFO_STREAM(node->get_logger(), "Data saved to file. Saving took: " << microseconds / 10e3 << "ms");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Cannot save data!");
    }
    data.reset();
}

}  // namespace tools
}  // namespace select_and_save_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(select_and_save_plugin::tools::SelectionTool, rviz_common::Tool)
