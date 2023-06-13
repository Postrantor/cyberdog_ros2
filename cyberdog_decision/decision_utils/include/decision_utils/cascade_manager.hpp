// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef DECISION_UTILS__CASCADE_MANAGER_HPP_
#define DECISION_UTILS__CASCADE_MANAGER_HPP_

// C++ headers
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// C++17 headers not support uncrustify yet
#include "cyberdog_utils/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "string_view"

namespace cyberdog_decision {

/*
  以上代码段是在 ROS2 项目中 rclcpp 组件中 lifecycle
  相关的代码。其中包含了一些枚举类型的定义和一个映射表。具体内容如下：

  CallbackReturn_T：回调函数返回值类型定义。
  Manager_Type：管理器类型枚举，包括单一管理器、单一列表和多重列表。
  State_Req：状态请求枚举，包括活跃状态请求和非活跃状态请求。
  ChainState：生命周期状态链枚举，包括空状态、所有组件都处于活跃状态、部分组件处于活跃状态、所有组件都处于非活跃状态和部分组件处于非活跃状态。
  state_map_：生命周期状态映射表，将状态值映射为字符串表示，其中活跃状态映射为
  "activated"，非活跃状态映射为 "deactivated"。
*/

/**
 * @brief 回调函数返回值类型定义
 */
using CallbackReturn_T = cyberdog_utils::CallbackReturn;

/**
 * @brief 管理器类型枚举
 */
enum Manager_Type { SINGLE_MANAGER = 0, SINGLE_LIST = 1, MULTI_LIST = 2 };

/**
 * @brief 状态请求枚举
 */
enum State_Req {
  IS_ACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,     // 活跃状态请求
  IS_DEACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE  // 非活跃状态请求
};

/**
 * @brief 生命周期状态链枚举
 */
enum ChainState {
  STATE_NULL = 0,    // 空状态
  ALL_ACTIVE = 1,    // 所有组件都处于活跃状态
  PART_ACTIVE = 2,   // 部分组件处于活跃状态
  ALL_DEACTIVE = 3,  // 所有组件都处于非活跃状态
  PART_DEACTIVE = 4  // 部分组件处于非活跃状态
};

/**
 * @brief 生命周期状态映射表
 */
static std::unordered_map<uint8_t, std::string> state_map_{
    {lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, std::string("activated")},  // 活跃状态映射
    {lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
     std::string("deactivated")}  // 非活跃状态映射
};

/*
  该代码段定义了一个名为 `CascadeManager` 的类，该类继承自 `cyberdog_utils::LifecycleNode`
  类，用于管理节点的生命周期，包括配置、激活、反激活、清理和关闭等操作。同时还提供了一些常用的函数，如打印信息等。

  具体来说，该类有以下成员：

  - `manager_type`：管理器类型。
  - `chainnodes_state_`：链式节点状态。
  - `manager_configure`：配置函数，用于配置节点列表名称。
  - `manager_activate`：激活函数，用于激活节点。
  - `manager_deactivate`：反激活函数，用于反激活节点。
  - `manager_cleanup`：清理函数，用于清理节点。
  - `manager_shutdown`：关闭函数，用于关闭节点。
  - `manager_error`：错误处理函数，用于处理错误。
  - `message_info`：打印信息函数，用于打印信息。
  - `message_warn`：打印警告函数，用于打印警告。
  - `message_error`：打印错误函数，用于打印错误。
  - `node_list_name_`：节点列表名称。
  - `node_map_`：节点映射表。
  - `node_name_set_`：节点名称集合。
  - `timeout_manager_`：管理器超时时间。
  - `thread_flag_`：线程标志位。
  - `sub_node_checking`：节点检查线程。
  - `node_states_`：节点状态订阅者。
  - `node_state_callback`：节点状态回调函数。
  - `node_status_checking`：节点状态检查函数。
*/

/**
 * @brief CascadeManager 类，继承自 LifecycleNode 类
 * @param node_name 节点名称
 * @param node_list_name 节点列表名称
 * @details
 * 该类用于管理节点的生命周期，包括配置、激活、反激活、清理和关闭等操作。同时还提供了一些常用的函数，如打印信息等。
 */
class CascadeManager : public cyberdog_utils::LifecycleNode {
public:
  explicit CascadeManager(const std::string node_name, const std::string node_list_name);
  ~CascadeManager();

  uint8_t manager_type;       // 管理器类型
  uint8_t chainnodes_state_;  // 链式节点状态

protected:
  /**
   * @brief 配置函数
   * @param node_list_name 节点列表名称
   * @return bool 配置是否成功
   */
  bool manager_configure(const std::string node_list_name = "");
  /**
   * @brief 激活函数
   * @return bool 激活是否成功
   */
  bool manager_activate();
  /**
   * @brief 反激活函数
   * @return bool 反激活是否成功
   */
  bool manager_deactivate();
  /**
   * @brief 清理函数
   * @return bool 清理是否成功
   */
  bool manager_cleanup();
  /**
   * @brief 关闭函数
   * @return bool 关闭是否成功
   */
  bool manager_shutdown();
  /**
   * @brief 错误处理函数
   * @return bool 错误处理是否成功
   */
  bool manager_error();

  /// common funcs
  /**
   * @brief 打印信息函数
   * @param log 信息内容
   */
  void message_info(std::string_view log);
  /**
   * @brief 打印警告函数
   * @param log 警告内容
   */
  void message_warn(std::string_view log);
  /**
   * @brief 打印错误函数
   * @param log 错误内容
   */
  void message_error(std::string_view log);

private:
  /// Variables
  // Parameters
  std::string node_list_name_;                     // 节点列表名称
  std::map<std::string, uint8_t> node_map_;        // 节点映射表
  std::unordered_set<std::string> node_name_set_;  // 节点名称集合
  int timeout_manager_;                            // 管理器超时时间

  /// Threads
  bool thread_flag_;                               // 线程标志位
  std::unique_ptr<std::thread> sub_node_checking;  // 节点检查线程

  // Subscriber for node's topic
  rclcpp::Subscription<cascade_lifecycle_msgs::msg::State>::SharedPtr
      node_states_;  // 节点状态订阅者

  /// Functions
  // Subscription callback
  /**
   * @brief 节点状态回调函数
   * @param msg 节点状态消息指针
   */
  void node_state_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg);
  // common funcs
  /**
   * @brief 节点状态检查函数
   * @param req_type 请求类型
   */
  void node_status_checking(const State_Req req_type);
};
}  // namespace cyberdog_decision

#endif  // DECISION_UTILS__CASCADE_MANAGER_HPP_
