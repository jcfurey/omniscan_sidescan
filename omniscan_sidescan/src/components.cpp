// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/sonar_node.hpp"

#include <class_loader/class_loader_core.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace
{
// Construct the loader's retired-factory storage before our static factories.
// If a loader defers dlclose until exit, reverse destruction order then keeps
// this storage alive while the factory deleters remove their registrations.
[[maybe_unused]] const auto & plugin_graveyard = class_loader::impl::getMetaObjectGraveyard();
}  // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(omniscan_sidescan::SonarNode)
RCLCPP_COMPONENTS_REGISTER_NODE(omniscan_sidescan::SonarAllNode)
