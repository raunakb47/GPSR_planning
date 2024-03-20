// Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

#include <string>
#include <utility>

#include "bt_nodes/GuidePerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_nodes {

using namespace std::chrono_literals;
using namespace std::placeholders;

GuidePerson::GuidePerson(const std::string &xml_tag_name,
                         const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);
}

void GuidePerson::halt() {
  RCLCPP_INFO(node_->get_logger(), "GuidePerson halted");
}

BT::NodeStatus GuidePerson::tick() {

  RCLCPP_INFO(node_->get_logger(), "GuidePerson ticked");
  return BT::NodeStatus::SUCCESS;
}

} // namespace bt_nodes

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<bt_nodes::GuidePerson>("GuidePerson");
}
