#include <gtest/gtest.h>
#include <memory>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "smartcar_costmap_plugins/unknown_cost_layer.hpp"

class UnknownCostTest : public ::testing::Test
{
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<nav2_util::LifecycleNode>("test_unknown_cost");
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    map->resizeMap(6, 2, 0.1, 0.0, 0.0);
    layer.initialize(map.get(), "familiarity", nullptr, node, nullptr);
  }
  void TearDown() override {map.reset(); node.reset(); rclcpp::shutdown();}
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  smartcar_costmap_plugins::UnknownCostLayer layer;
};

TEST_F(UnknownCostTest, PreservesEveryKnownCostAndCollisionBoundary)
{
  auto & grid = *map->getCostmap();
  const unsigned char costs[] = {0, 80, 252, 253, 254, 255};
  for (int x = 0; x < 6; ++x) {grid.setCost(x, 0, costs[x]);}
  layer.updateCosts(grid, 0, 0, 6, 1);
  for (int x = 0; x < 5; ++x) {EXPECT_EQ(grid.getCost(x, 0), costs[x]);}
  EXPECT_EQ(grid.getCost(5, 0), 5);
  EXPECT_EQ(grid.getCost(5, 1), 255);  // Update respects its bounds.
  EXPECT_NEAR(1.0 + 5.0 * grid.getCost(5, 0) / 252.0, 1.10, 0.001);
}

TEST_F(UnknownCostTest, FreshlyObservedFreeSpaceLosesTheExplorationPenalty)
{
  auto & grid = *map->getCostmap();
  layer.updateCosts(grid, 0, 0, 6, 2);
  EXPECT_EQ(grid.getCost(2, 1), 5);
  // Upstream layers replace an unknown cell with free space on a later scan.
  grid.setCost(2, 1, 0);
  grid.setCost(3, 1, 254);
  layer.updateCosts(grid, 0, 0, 6, 2);
  EXPECT_EQ(grid.getCost(2, 1), 0);
  EXPECT_EQ(grid.getCost(3, 1), 254);
}
