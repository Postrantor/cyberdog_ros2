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

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// C++17 headers not support uncrustify yet
#include "decision_utils/cascade_manager.hpp"
#include "string_view"

namespace cyberdog_decision {

/*
这里的 lifecycle 实际上就是这个库
[](https://github.com/fmrico/cascade_lifecycle/blob/master/README.md)
可以实现对节点生命周期的级联，即相互关联触发。
*/

/*
  这段代码是一个名为 CascadeManager 的类的构造函数和析构函数。该类继承自 LifecycleNode
  类，用于管理节点列表中的节点。在构造函数中，首先根据传入的节点列表名称判断当前节点管理器类型，并声明该节点的参数。在析构函数中，没有任何操作。
*/

/**
 * @brief CascadeManager 类的构造函数，继承自 LifecycleNode 类
 * @param node_name 节点名称
 * @param node_list_name 节点列表名称
 * @details 根据传入的节点列表名称，判断当前节点管理器类型，并声明该节点的参数。
 */
CascadeManager::CascadeManager(const std::string node_name, const std::string node_list_name)
    : cyberdog_utils::LifecycleNode(node_name), node_list_name_(node_list_name) {
  message_info(std::string("Creating ") + this->get_name());

  // Declare this node's parameters
  if (node_list_name_ == std::string("")) {
    manager_type = SINGLE_MANAGER;
  } else if (node_list_name_ == std::string("multi")) {
    manager_type = MULTI_LIST;
    this->declare_parameter("timeout_manager_s", 10);
  } else {
    manager_type = SINGLE_LIST;
    const std::vector<std::string> cascade_nodes_names = {};
    this->declare_parameter(node_list_name_, cascade_nodes_names);
    this->declare_parameter("timeout_manager_s", 10);
  }

  message_info(this->get_name() + std::string(" created."));
}

/**
 * @brief CascadeManager 类的析构函数
 */
CascadeManager::~CascadeManager() {}

/*
  该代码段是一个名为CascadeManager的rclcpp组件中的函数manager_configure。该函数用于配置组件，其输入参数为节点列表名称node_list_name，返回值为bool类型的配置结果。

  在函数中，首先判断manager_type是否为SINGLE_MANAGER。如果不是，则停止线程并清空node_map和chainnodes_state_。接着获取节点列表参数，并将其存储到node_name_list中。然后遍历node_name_list，如果node_map中不存在该节点，则将其插入node_map中。将node_name_list存储到node_name_set中。获取timeout_manager_s参数的值。创建订阅器node_states_，并绑定回调函数node_state_callback。最后返回配置结果。
*/
/**
 * @brief CascadeManager组件的manager_configure函数，用于配置组件
 * @param node_list_name 节点列表名称
 * @return bool 配置结果，成功为true，失败为false
 * @details
 * 1. 如果manager_type不是SINGLE_MANAGER，则停止线程并清空node_map和chainnodes_state_
 * 2. 获取节点列表参数，并将其存储到node_name_list中
 * 3. 遍历node_name_list，如果node_map中不存在该节点，则将其插入node_map中
 * 4. 将node_name_list存储到node_name_set中
 * 5. 获取timeout_manager_s参数的值
 * 6. 创建订阅器node_states_，并绑定回调函数node_state_callback
 * 7. 返回配置结果
 */
bool CascadeManager::manager_configure(const std::string node_list_name) {
  bool rtn_(false);
  if (manager_type != SINGLE_MANAGER) {                // 如果manager_type不是SINGLE_MANAGER
    if (chainnodes_state_ != STATE_NULL) {             // 如果chainnodes_state_不是STATE_NULL
      thread_flag_ = false;                            // 停止线程
      sub_node_checking->join();                       // 等待子线程结束
    }
    chainnodes_state_ = STATE_NULL;                    // 清空chainnodes_state_
    thread_flag_ = false;                              // 停止线程
    node_map_.clear();                                 // 清空node_map
    this->clear_activation();                          // 清空激活状态
    auto node_name_list = manager_type == SINGLE_LIST  // 获取节点列表参数
                              ? this->get_parameter(node_list_name_).as_string_array()
                              : this->get_parameter(node_list_name).as_string_array();

    if (node_name_list.size() != 0) {                        // 如果节点列表不为空
      for (auto& node_name : node_name_list) {               // 遍历节点列表
        if (node_map_.find(node_name) == node_map_.end()) {  // 如果node_map中不存在该节点
          node_map_.insert(std::pair<std::string, uint8_t>(  // 将该节点插入node_map中
              node_name, lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN));
          message_info(                                      // 输出信息
              std::string("Add ") + node_name + std::string(" to node chain list of ") +
              this->get_name());
        }
      }
      node_name_set_.clear();                           // 清空node_name_set
      node_name_set_.insert(
          begin(node_name_list), end(node_name_list));  // 将node_name_list存储到node_name_set中
    }

    timeout_manager_ =
        this->get_parameter("timeout_manager_s").as_int();  // 获取timeout_manager_s参数的值

    node_states_ =
        this->create_subscription<cascade_lifecycle_msgs::msg::State>(  // 创建订阅器node_states_
            "cascade_lifecycle_states", rclcpp::QoS(100),
            std::bind(&CascadeManager::node_state_callback, this, std::placeholders::_1));
    rtn_ = true;  // 设置配置结果为true
  }
  return rtn_;    // 返回配置结果
}

/*
  代码段中定义了CascadeManager类的五个方法，分别用于激活、停止、清除、关闭和出错处理CascadeManager管理的节点。其中，每个方法都会根据manager_type判断是否为单一管理器，如果不是则进行相应的操作。具体而言：

  manager_activate()方法会遍历node_map_中的所有节点并添加激活状态，如果chainnodes_state_不为空，则停止sub_node_checking线程，并创建新的线程进行节点状态检查。
  manager_deactivate()方法会停止sub_node_checking线程，并创建新的线程进行节点状态检查。
  manager_cleanup()方法会停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
  manager_shutdown()方法会停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
  manager_error()方法会停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
  每个方法都返回一个bool类型的值，表示操作是否成功。
*/

/**
 * @brief 激活CascadeManager管理的所有节点
 * @return bool 返回是否激活成功
 * @details
 * 根据manager_type判断是否为单一管理器，如果不是则遍历node_map_中的所有节点并添加激活状态，
 *          如果chainnodes_state_不为空，则停止sub_node_checking线程，并创建新的线程进行节点状态检查。
 */
bool CascadeManager::manager_activate() {
  bool rtn_(false);
  if (manager_type != SINGLE_MANAGER) {  // 判断是否为单一管理器
    for (auto& node : node_map_) {       // 遍历node_map_中的所有节点
      this->add_activation(static_cast<std::string>(node.first));  // 添加激活状态
    }
    if (chainnodes_state_ != STATE_NULL) {  // 如果chainnodes_state_不为空
      thread_flag_ = false;                 // 停止sub_node_checking线程
      sub_node_checking->join();
    }
    thread_flag_ = true;
    sub_node_checking = std::make_unique<std::thread>(
        &CascadeManager::node_status_checking, this, IS_ACTIVE);  // 创建新的线程进行节点状态检查
    rtn_ = true;
  }
  return rtn_;
}

/**
 * @brief 停止CascadeManager管理的所有节点
 * @return bool 返回是否停止成功
 * @details
 * 根据manager_type判断是否为单一管理器，如果不是则停止sub_node_checking线程，并创建新的线程进行节点状态检查。
 */
bool CascadeManager::manager_deactivate() {
  bool rtn_(false);
  if (manager_type != SINGLE_MANAGER) {  // 判断是否为单一管理器
    thread_flag_ = false;                // 停止sub_node_checking线程
    sub_node_checking->join();
    thread_flag_ = true;
    sub_node_checking = std::make_unique<std::thread>(
        &CascadeManager::node_status_checking, this, IS_DEACTIVE);  // 创建新的线程进行节点状态检查
    rtn_ = true;
  }
  return rtn_;
}

/**
 * @brief 清除CascadeManager管理的所有节点
 * @return bool 返回是否清除成功
 * @details
 * 根据manager_type判断是否为单一管理器，如果不是则停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
 */
bool CascadeManager::manager_cleanup() {
  bool rtn_(false);
  thread_flag_ = false;
  if (manager_type != SINGLE_MANAGER) {  // 判断是否为单一管理器
    sub_node_checking->join();           // 停止sub_node_checking线程
    node_map_.clear();                   // 清空node_map_
    node_states_.reset();                // 清空node_states_
    sub_node_checking.reset();           // 清空sub_node_checking
    rtn_ = true;
  }
  return rtn_;
}

/**
 * @brief 关闭CascadeManager管理器
 * @return bool 返回是否关闭成功
 * @details
 * 根据manager_type判断是否为单一管理器，如果不是则停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
 */
bool CascadeManager::manager_shutdown() {
  bool rtn_(false);
  thread_flag_ = false;
  if (manager_type != SINGLE_MANAGER) {  // 判断是否为单一管理器
    sub_node_checking->join();           // 停止sub_node_checking线程
    node_map_.clear();                   // 清空node_map_
    node_states_.reset();                // 清空node_states_
    sub_node_checking.reset();           // 清空sub_node_checking
    rtn_ = true;
  }
  return rtn_;
}

/**
 * @brief CascadeManager管理器出错处理
 * @return bool 返回是否处理成功
 * @details
 * 根据manager_type判断是否为单一管理器，如果不是则停止sub_node_checking线程，并清空node_map_、node_states_和sub_node_checking。
 */
bool CascadeManager::manager_error() {
  bool rtn_(false);
  thread_flag_ = false;
  if (manager_type != SINGLE_MANAGER) {  // 判断是否为单一管理器
    sub_node_checking->join();           // 停止sub_node_checking线程
    node_map_.clear();                   // 清空node_map_
    node_states_.reset();                // 清空node_states_
    sub_node_checking.reset();           // 清空sub_node_checking
    rtn_ = true;
  }
  return rtn_;
}

/*
  CascadeManager 类的 node_state_callback 函数用于更新节点状态，node_status_checking
  函数用于检查节点状态是否符合要求，message_info、message_warn 和 message_error
  函数分别用于输出信息、警告和错误信息。其中，node_status_checking
  函数是核心函数，它通过获取所有激活的节点，并每隔4秒检查一次节点状态是否符合要求，最长等待时间为
  timeout_manager_ 秒。如果所有节点状态都符合要求，则设置 chainnodes_state_ 为 ALL_ACTIVE 或
  ALL_DEACTIVE；否则，设置 chainnodes_state_ 为 PART_ACTIVE 或 PART_DEACTIVE，并输出错误信息。
*/

/**
 * @brief CascadeManager 类的 node_state_callback 函数，用于更新节点状态
 * @param msg 消息指针，包含节点名称和状态
 * @details 如果节点名称在 node_name_set_ 中，则更新 node_map_ 中该节点的状态
 */
void CascadeManager::node_state_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg) {
  if (node_name_set_.find(msg->node_name) != node_name_set_.end()) {
    node_map_[msg->node_name] = msg->state;
  }
}

/**
 * @brief CascadeManager 类的 node_status_checking 函数，用于检查节点状态是否符合要求
 * @param req_type 要求的节点状态类型
 * @details
 * 1. 获取所有激活的节点
 * 2. 每隔4秒检查一次节点状态是否符合要求，最长等待时间为 timeout_manager_ 秒
 * 3. 如果所有节点状态都符合要求，则设置 chainnodes_state_ 为 ALL_ACTIVE 或 ALL_DEACTIVE
 * 4. 否则，设置 chainnodes_state_ 为 PART_ACTIVE 或 PART_DEACTIVE，并输出错误信息
 */
void CascadeManager::node_status_checking(const State_Req req_type) {
  auto activations = this->get_activations();         // 获取所有激活的节点
  rclcpp::WallRate activating_rate(4);                // 定义检查节点状态的频率为4Hz
  rclcpp::Time start_time(this->get_clock()->now());  // 记录开始时间
  std::set<std::string> active_set;                   // 记录已经检查过的节点名称
  auto timeout_count = (this->get_clock()->now() - start_time).seconds();  // 记录已经等待的时间

  while (rclcpp::ok() && thread_flag_ &&
         (this->get_clock()->now() - start_time <= std::chrono::seconds(timeout_manager_))) {
    timeout_count = (this->get_clock()->now() - start_time).seconds();  // 更新已经等待的时间

    if (active_set.size() == node_map_.size()) {  // 如果所有节点都已经检查过
      break;
    }
    if (node_map_.size() == 0) {  // 如果没有节点需要检查
      break;
    }

    for (auto& activation : activations) {                    // 遍历所有激活的节点
      if (node_map_[activation] == req_type &&
          active_set.find(activation) == active_set.end()) {  // 如果节点状态符合要求且未被检查过
        message_info(
            std::string("Node [") + activation + std::string("] is ") +
            state_map_[req_type]);      // 输出信息
        active_set.insert(activation);  // 将该节点名称加入已检查过的集合中
      }
    }
    activating_rate.sleep();  // 等待一段时间再进行下一次检查
  }

  uint8_t unchanged_count_(0);  // 记录状态不符合要求的节点数量
  if (activations.size() != 0) {
    for (auto& activation : activations) {
      if (node_map_[activation] != req_type) {  // 如果节点状态不符合要求
        unchanged_count_ += 1;                  // 计数器加1
        message_error(
            std::string("After ") + std::to_string(timeout_count) + std::string(" seconds. Node ") +
            activation + std::string(" is still not ") + state_map_[req_type]);  // 输出错误信息
      }
    }
  }
  if (unchanged_count_ == 0) {  // 如果所有节点状态都符合要求
    chainnodes_state_ = (req_type == IS_ACTIVE)
                            ? ALL_ACTIVE
                            : ALL_DEACTIVE;  // 设置 chainnodes_state_ 为 ALL_ACTIVE 或 ALL_DEACTIVE
    message_info(std::string("All node/nodes is/are ") + state_map_[req_type]);  // 输出信息
  } else {
    chainnodes_state_ =
        (req_type == IS_ACTIVE)
            ? PART_ACTIVE
            : PART_DEACTIVE;  // 设置 chainnodes_state_ 为 PART_ACTIVE 或 PART_DEACTIVE
  }
}

/**
 * @brief CascadeManager 类的 message_info 函数，用于输出信息
 * @param log 要输出的信息
 */
void CascadeManager::message_info(std::string_view log) {
  RCLCPP_INFO_STREAM(this->get_logger(), log);
}

/**
 * @brief CascadeManager 类的 message_warn 函数，用于输出警告信息
 * @param log 要输出的信息
 */
void CascadeManager::message_warn(std::string_view log) {
  RCLCPP_INFO_STREAM(this->get_logger(), log);
}

/**
 * @brief CascadeManager 类的 message_error 函数，用于输出错误信息
 * @param log 要输出的信息
 */
void CascadeManager::message_error(std::string_view log) {
  RCLCPP_INFO_STREAM(this->get_logger(), log);
}

}  // namespace cyberdog_decision
