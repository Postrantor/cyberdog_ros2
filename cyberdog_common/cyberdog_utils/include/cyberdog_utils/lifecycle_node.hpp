// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_UTILS__LIFECYCLE_NODE_HPP_
#define CYBERDOG_UTILS__LIFECYCLE_NODE_HPP_

#include <map>
#include <set>
#include <string>

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"
#include "cyberdog_utils/Enums.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

#define ANSI_COLOR_RESET "\x1b[0m"
#define ANSI_COLOR_BLUE "\x1b[34m"

namespace cyberdog_utils {

/*
  上述代码是一个继承自 `rclcpp_lifecycle::LifecycleNode` 的生命周期节点类
  `LifecycleNode`，其中包含了一些生命周期相关的函数和变量：

  - `add_activation`：添加激活器。
  - `remove_activation`：移除激活器。
  - `remove_activation_pub`：移除激活器发布者。
  - `clear_activation`：清空激活器。
  - `get_activators`：获取激活器集合。
  - `get_activations`：获取激活集合。
  - `get_activators_state`：获取激活器状态映射表。
  - `auto_check`：当节点需要时自动检查。

  此外，还有一些回调函数：

  - `on_configure_internal`：配置回调函数。
  - `on_cleanup_internal`：清理回调函数。
  - `on_shutdown_internal`：关闭回调函数。
  - `on_activate_internal`：激活回调函数。
  - `on_deactivate_internal`：停用回调函数。
  - `on_error_internal`：错误回调函数。

  以及一些其他的函数和变量：

  - `states_pub_`：状态发布者。
  - `activations_pub_`：激活发布者。
  - `activations_sub_`：激活订阅者。
  - `states_sub_`：状态订阅者。
  - `timer_`：定时器。
  - `activators_`：激活器集合。
  - `activations_`：激活集合。
  - `activators_state_`：激活器状态映射表。
  - `governed`：是否受控制。

  其中，函数和变量的具体作用在代码注释中有详细说明。
*/

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using lifecycle_msgs::msg::State;

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief 创建一个指定名称的生命周期节点。
   * @param[in] node_name 节点名称。
   * @param[in] options 控制节点创建的附加选项。
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNode(
      const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief 基于节点名称和 rclcpp::Context 创建节点。
   * @param[in] node_name 节点名称。
   * @param[in] namespace_ 节点命名空间。
   * @param[in] options 控制节点创建的附加选项。
   */
  RCLCPP_LIFECYCLE_PUBLIC
  LifecycleNode(
      const std::string& node_name,
      const std::string& namespace_,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief 添加激活器。
   * @param[in] node_name 激活器名称。
   */
  void add_activation(const std::string& node_name);

  /**
   * @brief 移除激活器。
   * @param[in] node_name 激活器名称。
   */
  void remove_activation(const std::string& node_name);

  /**
   * @brief 移除激活器发布者。
   * @param[in] node_name 激活器名称。
   */
  void remove_activation_pub(const std::string& node_name);

  /**
   * @brief 清空激活器。
   */
  void clear_activation();

  /**
   * @brief 获取激活器集合。
   * @return const std::set<std::string>& 激活器集合。
   */
  const std::set<std::string>& get_activators() const { return activators_; }

  /**
   * @brief 获取激活集合。
   * @return const std::set<std::string>& 激活集合。
   */
  const std::set<std::string>& get_activations() const { return activations_; }

  /**
   * @brief 获取激活器状态映射表。
   * @return const std::map<std::string, uint8_t>& 激活器状态映射表。
   */
  const std::map<std::string, uint8_t>& get_activators_state() const { return activators_state_; }

  /**
   * @brief 当节点需要时自动检查。
   * @param[in] check_type：CHECK_TO_START 激活，CHECK_TO_PAUSE 暂停，CHECK_TO_SHUTDOWN 关闭
   * @return bool：检查结果。
   */
  bool auto_check(uint8_t check_type);

private:
  CallbackReturn on_configure_internal(const rclcpp_lifecycle::State& previous_state);
  CallbackReturn on_cleanup_internal(const rclcpp_lifecycle::State& previous_state);
  CallbackReturn on_shutdown_internal(const rclcpp_lifecycle::State& previous_state);
  CallbackReturn on_activate_internal(const rclcpp_lifecycle::State& previous_state);
  CallbackReturn on_deactivate_internal(const rclcpp_lifecycle::State& previous_state);
  CallbackReturn on_error_internal(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecyclePublisher<cascade_lifecycle_msgs::msg::State>::SharedPtr states_pub_;
  rclcpp_lifecycle::LifecyclePublisher<cascade_lifecycle_msgs::msg::Activation>::SharedPtr
      activations_pub_;

  rclcpp::Subscription<cascade_lifecycle_msgs::msg::Activation>::SharedPtr activations_sub_;
  rclcpp::Subscription<cascade_lifecycle_msgs::msg::State>::SharedPtr states_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::set<std::string> activators_;                 // 激活器集合
  std::set<std::string> activations_;                // 激活集合
  std::map<std::string, uint8_t> activators_state_;  // 激活器状态映射表
  bool governed;

  /**
   * @brief 激活回调函数。
   * @param[in] msg 激活消息。
   */
  void activations_callback(const cascade_lifecycle_msgs::msg::Activation::SharedPtr msg);

  /**
   * @brief 状态回调函数。
   * @param[in] msg 状态消息。
   */
  void states_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg);

  /**
   * @brief 更新状态。
   * @param[in] state 状态。
   */
  void update_state(const uint8_t state = lifecycle_msgs::msg::Transition::TRANSITION_CREATE);

  /**
   * @brief 定时器回调函数。
   */
  void timer_callback();

  /**
   * @brief 输出信息。
   * @param[in] msg 信息。
   */
  void message(const std::string& msg) {
    RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
  }
};

}  // namespace cyberdog_utils

#endif  // CYBERDOG_UTILS__LIFECYCLE_NODE_HPP_
