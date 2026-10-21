// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include <class_loader/class_loader.hpp>
#include <rclcpp_components/node_factory.hpp>

#include <gtest/gtest.h>
#include <dlfcn.h>

TEST(PluginLifetime, FactoriesCanBeLoadedAndActuallyUnmappedRepeatedly)
{
  for (size_t cycle = 0; cycle < 10; ++cycle) {
    {
      class_loader::ClassLoader loader(OMNISCAN_PLUGIN_PATH);
      const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
      ASSERT_EQ(classes.size(), 2u);
      for (const auto & name : classes) {
        auto factory = loader.createInstance<rclcpp_components::NodeFactory>(name);
        ASSERT_NE(factory, nullptr);
      }
    }
    // isLibraryLoaded() only tracks the loader's bookkeeping; RTLD_NOLOAD
    // checks whether GNU unique symbols have actually pinned the plugin DSO.
    auto * loaded = dlopen(OMNISCAN_PLUGIN_PATH, RTLD_NOW | RTLD_NOLOAD);
    EXPECT_EQ(loaded, nullptr) << "component plugin remained mapped after unloading";
    if (loaded) {dlclose(loaded);}
  }
}
