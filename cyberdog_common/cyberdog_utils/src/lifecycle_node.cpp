// Copyright 2020 Intelligent Robotics Lab
// Copyright 2021 Homalozoa, Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include "cyberdog_utils/lifecycle_node.hpp"

#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cyberdog_utils  // Modified from rclcpp_cascade_lifecycle
{

using namespace std::chrono_literals;

/**
 * @brief LifecycleNode类的构造函数，初始化节点的生命周期状态和相关参数
 * @param node_name 节点名称
 * @param options 节点选项
 */
LifecycleNode::LifecycleNode(
    const std::string &node_name,  //
    const rclcpp::NodeOptions &options)
    : LifecycleNode(
          node_name,  //
          "",         //
          options) {}

/**
 * @brief LifecycleNode类的构造函数，初始化节点的生命周期状态和相关参数
 * @param node_name 节点名称
 * @param namespace_ 命名空间
 * @param options 节点选项
 * @details
 * 1. 调用rclcpp_lifecycle::LifecycleNode类的构造函数，传入节点名称、命名空间和选项；
 * 2. 初始化governed为false；
 * 3. 创建发布器activations_pub_和states_pub_，分别发布Activation和State消息；
 * 4. 创建订阅器activations_sub_和states_sub_，分别订阅Activation和State消息，并绑定回调函数；
 * 5. 创建定时器timer_，并绑定回调函数；
 * 6. 激活发布器activations_pub_和states_pub_；
 * 7. 注册各个生命周期回调函数。
 */
LifecycleNode::LifecycleNode(
    const std::string &node_name,   //
    const std::string &namespace_,  //
    const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options), governed(false) {
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  activations_pub_ = create_publisher<cascade_lifecycle_msgs::msg::Activation>(
      "cascade_lifecycle_activations", rclcpp::QoS(1000).keep_all().transient_local().reliable());

  states_pub_ = create_publisher<cascade_lifecycle_msgs::msg::State>(
      "cascade_lifecycle_states", rclcpp::QoS(100));

  activations_sub_ = create_subscription<cascade_lifecycle_msgs::msg::Activation>(
      "cascade_lifecycle_activations", rclcpp::QoS(1000).keep_all().transient_local().reliable(),
      std::bind(&LifecycleNode::activations_callback, this, _1));

  states_sub_ = create_subscription<cascade_lifecycle_msgs::msg::State>(
      "cascade_lifecycle_states", rclcpp::QoS(100),
      std::bind(&LifecycleNode::states_callback, this, _1));

  timer_ = create_wall_timer(500ms, std::bind(&LifecycleNode::timer_callback, this));

  activations_pub_->on_activate();
  states_pub_->on_activate();

  register_on_configure(
      std::bind(&LifecycleNode::on_configure_internal, this, std::placeholders::_1));

  register_on_cleanup(std::bind(&LifecycleNode::on_cleanup_internal, this, std::placeholders::_1));

  register_on_shutdown(
      std::bind(&LifecycleNode::on_shutdown_internal, this, std::placeholders::_1));

  register_on_activate(
      std::bind(&LifecycleNode::on_activate_internal, this, std::placeholders::_1));

  register_on_deactivate(
      std::bind(&LifecycleNode::on_deactivate_internal, this, std::placeholders::_1));

  register_on_error(std::bind(&LifecycleNode::on_error_internal, this, std::placeholders::_1));
}

/**
 * @brief LifecycleNode 组件的 activations_callback 函数，用于处理 lifecycle 相关的消息
 * @param msg 生命周期相关的消息
 * @details 根据不同的操作类型进行不同的处理：
 *          1. ADD：添加生命周期激活器，如果是 dictator_ 激活器且当前节点未被管辖，则将 governed
 * 置为 true，并打印日志； 如果是其他激活器，则将其加入 activators_ 集合中，并初始化其状态为
 * PRIMARY_STATE_UNKNOWN。
 *          2. REMOVE：移除生命周期激活器，如果是 dictator_ 激活器且当前节点被管辖，则将 governed
 * 置为 false，并打印日志； 如果是其他激活器，则从 activators_
 * 集合中移除该激活器，并获取其之前的状态； 如果该激活器之前的状态为
 * PRIMARY_STATE_ACTIVE，则判断是否还有其他激活器处于 ACTIVE 状态， 如果没有，则触发
 * TRANSITION_DEACTIVATE 过渡。
 */
void LifecycleNode::activations_callback(
    const cascade_lifecycle_msgs::msg::Activation::SharedPtr msg) {
  // 根据操作类型进行不同的处理
  switch (msg->operation_type) {
    case cascade_lifecycle_msgs::msg::Activation::ADD:
      // 如果激活器是 dictactor_ 并且 activation 是当前节点，则将 governed 设为 true，并打印相关信息
      if (msg->activator == dictator_ && msg->activation == get_name()) {
        governed = true;
        message(std::string("Node ") + get_name() + std::string(" is governed by dictator."));
      } else if (msg->activation == get_name()) {
        // 将该激活器加入 activators_ 集合中，并初始化其状态为 PRIMARY_STATE_UNKNOWN
        activators_.insert(msg->activator);
        if (activators_state_.find(msg->activator) == activators_state_.end()) {
          activators_state_[msg->activator] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
      }
      break;
    case cascade_lifecycle_msgs::msg::Activation::REMOVE:
      // 如果激活器是 dictactor_ 并且 activation 是当前节点，则将 governed 设为
      // false，并打印相关信息
      if (msg->activator == dictator_ && msg->activation == get_name()) {
        governed = false;
        message(std::string("Node ") + get_name() + std::string(" is liberated by dictator."));
      } else if (
          msg->activation == get_name() &&  // NOLINT
          activators_.find(msg->activator) != activators_.end()) {
        // 从 activators_ 中移除该激活器，并删除其状态信息
        uint8_t remover_state = activators_state_[msg->activator];

        activators_.erase(msg->activator);

        if (activators_state_.find(msg->activator) != activators_state_.end()) {
          activators_state_.erase(msg->activator);
        }

        // 如果该激活器的状态为 PRIMARY_STATE_ACTIVE，则检查是否还有其他激活器处于 ACTIVE
        // 状态，如果没有则触发 TRANSITION_DEACTIVATE 过渡
        if (remover_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          bool any_other_activator = false;
          for (const auto &activator : activators_state_) {
            any_other_activator =
                any_other_activator ||
                activator.second == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
          }

          if (!any_other_activator) {
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          }
        }
      }
      break;
  }
}

/*
  以上代码实现了该组件的一些核心功能，包括：

  - `states_callback`
  函数：状态回调函数，用于接收其他节点发送的状态消息，并根据消息更新当前组件的状态。
  - `add_activation` 函数：添加激活器，向 `activations_pub_` 发布一个 ADD 操作类型的 Activation
  消息，表示将该激活器添加到当前组件的激活器列表中。
  - `remove_activation` 函数：移除激活器，向 `activations_pub_` 发布一个 REMOVE 操作类型的
  Activation 消息，表示将该激活器从当前组件的激活器列表中移除。
  - `remove_activation_pub` 函数：移除激活器（仅发布消息），向 `activations_pub_` 发布一个 REMOVE
  操作类型的 Activation 消息，表示将该激活器从当前组件的激活器列表中移除，但不从 `activations_`
  中移除。

  以上函数都是 `LifecycleNode` 组件中的成员函数，使用时需要先创建一个 `LifecycleNode` 对象。其中
  `states_callback` 函数是一个回调函数，需要在初始化时通过 `create_subscription`
  函数注册到对应的话题上。
*/

/**
 * @brief LifecycleNode 组件的状态回调函数
 * @param msg 状态消息指针
 * @details 根据收到的状态消息更新 activators_state_
 * 中对应节点的状态，如果有变化则更新当前组件的状态
 */
void LifecycleNode::states_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg) {
  if (activators_state_.find(msg->node_name) != activators_state_.end() && !governed) {
    if (activators_state_[msg->node_name] != msg->state) {
      activators_state_[msg->node_name] = msg->state;
      update_state();
    }
  }

  if (msg->node_name == dictator_ && governed) {
    update_state(msg->state);
  }
}

/**
 * @brief 添加一个激活器
 * @param node_name 要添加的激活器节点名称
 * @details 向 activations_pub_ 发布一个 ADD 操作类型的 Activation
 * 消息，表示将该激活器添加到当前组件的激活器列表中
 */
void LifecycleNode::add_activation(const std::string &node_name) {
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::ADD;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.insert(node_name);
    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to set an auto activation");
  }
}

/**
 * @brief 移除一个激活器
 * @param node_name 要移除的激活器节点名称
 * @details 向 activations_pub_ 发布一个 REMOVE 操作类型的 Activation
 * 消息，表示将该激活器从当前组件的激活器列表中移除
 */
void LifecycleNode::remove_activation(const std::string &node_name) {
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::REMOVE;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.erase(node_name);
    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to remove & erase an auto activation");
  }
}

/**
 * @brief 移除一个激活器（仅发布消息）
 * @param node_name 要移除的激活器节点名称
 * @details 向 activations_pub_ 发布一个 REMOVE 操作类型的 Activation
 * 消息，表示将该激活器从当前组件的激活器列表中移除，但不从 activations_ 中移除
 */
void LifecycleNode::remove_activation_pub(const std::string &node_name) {
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::REMOVE;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to remove an auto activation");
  }
}

/*
上述代码是在 ros2 项目中 rclcpp 组件中 lifecycle
相关的代码。其中包含了生命周期组件的三个回调函数：on_configure、on_cleanup 和
on_shutdown，以及一个清除激活状态的函数 clear_activation。

在 on_configure、on_cleanup 和 on_shutdown 这三个回调函数中，首先会调用对应的
on_configure、on_cleanup 和 on_shutdown
函数执行配置、清理和关闭操作，然后会发布当前节点的状态信息。这里使用了一个名为
cascade_lifecycle_msgs 的消息类型，通过 states_pub_ 发布节点的状态信息。

clear_activation 函数则是遍历 activations_，调用 remove_activation_pub() 函数进行清除。
*/

/**
 * @brief 清除所有激活状态
 * @details 遍历 activations_，调用 remove_activation_pub() 函数进行清除
 */
void LifecycleNode::clear_activation() {
  for (const auto &activation : activations_) {
    remove_activation_pub(activation);
  }

  activations_.clear();
}

/**
 * @brief 生命周期组件的 on_configure 回调函数
 * @param previous_state 上一个状态
 * @return CallbackReturn::SUCCESS 表示成功
 * @details 调用 on_configure() 函数执行配置操作，然后发布当前节点的状态信息
 */
CallbackReturn LifecycleNode::on_configure_internal(const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_configure(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief 生命周期组件的 on_cleanup 回调函数
 * @param previous_state 上一个状态
 * @return CallbackReturn::SUCCESS 表示成功
 * @details 调用 on_cleanup() 函数执行清理操作，然后发布当前节点的状态信息
 */
CallbackReturn LifecycleNode::on_cleanup_internal(const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_cleanup(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief 生命周期组件的 on_shutdown 回调函数
 * @param previous_state 上一个状态
 * @return CallbackReturn::SUCCESS 表示成功
 * @details 调用 on_shutdown() 函数执行关闭操作，然后发布当前节点的状态信息
 */
CallbackReturn LifecycleNode::on_shutdown_internal(const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_shutdown(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/*
  其中包含了两个函数：on_activate_internal 和
  on_deactivate_internal。这两个函数分别在组件从 INACTIVE 状态转换到 ACTIVE 状态和从 ACTIVE
  状态转换到 INACTIVE 状态时被调用。在这两个函数中，会分别调用 on_activate 和 on_deactivate
  函数，并发布状态消息。

  其中，on_activate_internal 函数的功能为：当组件从 INACTIVE 状态转换到 ACTIVE
  状态时，调用此函数。在此函数中，会调用 on_activate 函数，并发布 ACTIVE
  状态的消息。on_deactivate_internal 函数的功能为：当组件从 ACTIVE 状态转换到 INACTIVE
  状态时，调用此函数。在此函数中，会调用 on_deactivate 函数，并发布 INACTIVE 状态的消息。
*/

/**
 * @brief LifecycleNode 组件的 on_activate_internal 函数
 * @param previous_state 上一个状态
 * @details 当组件从 INACTIVE 状态转换到 ACTIVE 状态时，会调用此函数。在此函数中，会调用 on_activate
 * 函数，并发布 ACTIVE 状态的消息。
 */
CallbackReturn LifecycleNode::on_activate_internal(const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_activate(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief LifecycleNode 组件的 on_deactivate_internal 函数
 * @param previous_state 上一个状态
 * @details 当组件从 ACTIVE 状态转换到 INACTIVE 状态时，会调用此函数。在此函数中，会调用
 * on_deactivate 函数，并发布 INACTIVE 状态的消息。
 */
CallbackReturn LifecycleNode::on_deactivate_internal(
    const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_deactivate(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief LifecycleNode 组件的 on_error_internal 回调函数，用于处理组件状态转换到 ERROR
 * 状态时的逻辑。
 * @param previous_state 上一个状态
 * @return CallbackReturn 返回回调函数的执行结果
 * @details
 * 1. 创建 cascade_lifecycle_msgs::msg::State 类型的 msg 对象；
 * 2. 调用 on_error 函数，获取返回值 ret；
 * 3. 如果 ret 的值为 CallbackReturn::SUCCESS，则将 msg 对象的 state 属性设置为
 * lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED， 将 msg 对象的 node_name
 * 属性设置为当前组件的名称，并通过 states_pub_ 发布该消息；
 * 4. 返回 ret。
 */
CallbackReturn LifecycleNode::on_error_internal(const rclcpp_lifecycle::State &previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_error(previous_state);

  if (ret == CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief 更新组件的状态
 * @param state 组件的新状态
 * @details 根据组件的新状态和父级 activator 的状态更新组件的状态。
 * 如果组件当前处于未知状态或未配置状态，且其父级 activator 处于活动或非活动状态，则触发转换到
 * CONFIGURED 状态。 如果组件当前处于非活动状态，且其父级 activator 处于活动状态，则触发转换到
 * ACTIVE 状态。 如果组件当前处于活动状态，且其父级 activator 都处于非活动状态，则触发转换到
 * INACTIVE 状态。 如果组件当前处于终止状态，则不进行任何操作。
 */
void LifecycleNode::update_state(const uint8_t state) {
  bool parent_inactive = false;                   // 父级 activator 是否有非活动状态
  bool parent_active = false;                     // 父级 activator 是否有活动状态
  auto last_state_id = get_current_state().id();  // 获取当前状态

  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {  // 如果新状态为 ACTIVE
    parent_active = true;
    parent_inactive = false;
  } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {  // 如果新状态为
                                                                             // INACTIVE
    parent_active = false;
    parent_inactive = true;
  } else {                                             // 如果新状态为其他状态
    for (const auto &activator : activators_state_) {  // 遍历所有 activator
      parent_inactive =
          parent_inactive ||
          activator.second ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;  // 如果有非活动状态，则
                                                                   // parent_inactive 为 true
      parent_active =
          parent_active ||
          activator.second ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;  // 如果有活动状态，则 parent_active
                                                                 // 为 true
    }
  }

  switch (last_state_id) {                     // 根据当前状态进行转换
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
      if (parent_active || parent_inactive) {  // 如果父级 activator 有活动或非活动状态
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // 触发转换到 CONFIGURED 状态
        message(
            std::string(
                transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE]) +
            get_name());  // 发送消息
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      if (parent_active || parent_inactive) {  // 如果父级 activator 有活动或非活动状态
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // 触发转换到 CONFIGURED 状态
        message(
            std::string(
                transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE]) +
            get_name());  // 发送消息
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      if (parent_active) {  // 如果父级 activator 有活动状态
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);  // 触发转换到 ACTIVE 状态
        message(
            std::string(
                transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE]) +
            get_name());  // 发送消息
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (!parent_active && parent_inactive) {  // 如果父级 activator 都处于非活动状态
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);  // 触发转换到 INACTIVE 状态
        message(
            std::string(
                transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE]) +
            get_name());  // 发送消息
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      break;
  }
}

/**
 * @brief LifecycleNode 定义了一个生命周期节点，该节点可以发布当前状态并更新状态。
 * @param void
 * @details
 * 该函数是一个定时器回调函数，用于获取节点图形接口中的所有节点名称，并检查活动器是否存在。如果不存在，则从活动器列表中删除它，并且如果当前状态与该活动器的状态相同，则更新状态。最后，将当前状态和节点名称发布到状态发布器中，并更新状态。
 */
void LifecycleNode::timer_callback() {
  // 获取节点图形接口中的所有节点名称
  auto nodes = this->get_node_graph_interface()->get_node_names();
  std::string ns = get_namespace();
  if (ns != std::string("/")) {
    ns = ns + std::string("/");
  }

  // 遍历活动器列表，检查活动器是否存在
  std::set<std::string>::iterator it = activators_.begin();
  while (it != activators_.end()) {
    const auto &node_name = *it;
    if (std::find(nodes.begin(), nodes.end(), ns + node_name) == nodes.end()) {
      // 如果活动器不存在，则从活动器列表中删除它
      RCLCPP_DEBUG(
          get_logger(), "Activator %s is not longer present, removing from activators",
          node_name.c_str());
      it = activators_.erase(it);

      // 如果当前状态与该活动器的状态相同，则更新状态
      if (get_current_state().id() == activators_state_[node_name]) {
        update_state();
      }
      activators_state_.erase(node_name);
    } else {
      it++;
    }
  }

  // 将当前状态和节点名称发布到状态发布器中
  cascade_lifecycle_msgs::msg::State msg;
  msg.state = get_current_state().id();
  msg.node_name = get_name();

  states_pub_->publish(msg);

  // 更新状态
  update_state();
}

/*
  用于执行自动检查操作。该函数根据不同的检查类型，执行对应的生命周期转换操作，并返回检查结果。其中，CHECK_TO_START
  表示启动检查，CHECK_TO_PAUSE 表示暂停检查，CHECK_TO_CLEANUP
  表示清理检查。在执行过程中，会打印一些调试信息，方便开发者进行调试和排错。
*/

/**
 * @brief LifecycleNode 组件的自动检查函数
 * @param check_type 检查类型，包括 CHECK_TO_START、CHECK_TO_PAUSE 和 CHECK_TO_CLEANUP 三种
 * @return 返回布尔值，表示检查是否成功
 * @details 根据不同的检查类型，执行对应的生命周期转换操作，并返回检查结果。
 */
bool LifecycleNode::auto_check(uint8_t check_type) {
  message(std::string("Auto checking ") + get_name());  // 打印调试信息，表示正在进行自动检查

  bool rtn_ = false;                                    // 初始化返回值为 false

  if (check_type == CHECK_TO_START) {                   // 如果检查类型为 CHECK_TO_START
    message(std::string("Auto start..."));  // 打印调试信息，表示正在进行自动启动

    switch (get_current_state().id()) {     // 根据当前状态进行判断
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:  // 如果当前状态为 UNKNOWN
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: {  // 或者当前状态为 UNCONFIGURED
        message(
            transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] +
            get_name());  // 打印调试信息，表示将要执行 CONFIGURE 转换操作
        if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] !=
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)
                .id()) {                  // 如果转换操作失败
          check_type = CHECK_TO_CLEANUP;  // 将检查类型修改为 CHECK_TO_CLEANUP
          break;                          // 跳出 switch 语句
        }
      }
      // fall through
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE: {  // 如果当前状态为 INACTIVE
        message(
            transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE] +
            get_name());  // 打印调试信息，表示将要执行 ACTIVATE 转换操作
        if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE] !=
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)
                .id()) {                  // 如果转换操作失败
          check_type = CHECK_TO_CLEANUP;  // 将检查类型修改为 CHECK_TO_CLEANUP
          break;                          // 跳出 switch 语句
        }
      }
      // fall through
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE: {  // 如果当前状态为 ACTIVE
        message(
            std::string("Bringup ") + get_name() +
            std::string(" succeed."));  // 打印调试信息，表示启动成功
        rtn_ = true;                    // 修改返回值为 true
        break;                          // 跳出 switch 语句
      }
      default:
        break;
    }
  }

  if (check_type == CHECK_TO_PAUSE) {       // 如果检查类型为 CHECK_TO_PAUSE
    message(std::string("Auto pause..."));  // 打印调试信息，表示正在进行自动暂停

    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {  // 如果当前状态为 INACTIVE
      message(
          get_name() + std::string(" already inactive."));  // 打印调试信息，表示已经处于暂停状态
      rtn_ = true;                                          // 修改返回值为 true
    } else if (
        get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {  // 如果当前状态为 UNCONFIGURED
      message(
          transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] +
          get_name());  // 打印调试信息，表示将要执行 CONFIGURE 转换操作
      if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] !=
          trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)
              .id()) {                  // 如果转换操作失败
        check_type = CHECK_TO_CLEANUP;  // 将检查类型修改为 CHECK_TO_CLEANUP
      } else {
        rtn_ = true;                    // 修改返回值为 true
        message(get_name() + std::string(" pause succeed."));  // 打印调试信息，表示暂停成功
      }
    } else if (
        get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {  // 如果当前状态为 ACTIVE
      message(
          get_name() +
          transition_label_map_
              [lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE]);  // 打印调试信息，表示将要执行
                                                                          // DEACTIVATE 转换操作
      if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE] !=
          trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
              .id()) {                  // 如果转换操作失败
        check_type = CHECK_TO_CLEANUP;  // 将检查类型修改为 CHECK_TO_CLEANUP
      } else {
        rtn_ = true;                    // 修改返回值为 true
        message(get_name() + std::string(" pause succeed."));  // 打印调试信息，表示暂停成功
      }
    } else {
      message(std::string("Unknown state to pause."));  // 打印调试信息，表示无法进行暂停操作
    }
  }

  if (check_type == CHECK_TO_CLEANUP) {       // 如果检查类型为 CHECK_TO_CLEANUP
    message(std::string("Auto cleanup..."));  // 打印调试信息，表示正在进行自动清理

    switch (get_current_state().id()) {       // 根据当前状态进行判断
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE: {  // 如果当前状态为 ACTIVE
        message(
            get_name() +
            transition_label_map_
                [lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE]);  // 打印调试信息，表示将要执行
                                                                            // DEACTIVATE 转换操作
        if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE] !=
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
                .id()) {  // 如果转换操作失败
          break;          // 跳出 switch 语句
        }
      }
      // fall through
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE: {  // 如果当前状态为 INACTIVE
        message(
            get_name() +
            transition_label_map_
                [lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP]);  // 打印调试信息，表示将要执行
                                                                         // CLEANUP 转换操作
        if (transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP] !=
            trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)
                .id()) {  // 如果转换操作失败
          break;          // 跳出 switch 语句
        }
      }
      // fall through
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: {  // 如果当前状态为 UNCONFIGURED
        message(get_name() + std::string(" cleanup succeed."));  // 打印调试信息，表示清理成功
        rtn_ = true;                                             // 修改返回值为 true
        break;                                                   // 跳出 switch 语句
      }
      default:
        break;
    }
  }

  return rtn_;  // 返回检查结果
}

}  // namespace cyberdog_utils
