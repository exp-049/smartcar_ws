#include "smartcar_costmap_plugins/unknown_cost_layer.hpp"
#include <algorithm>
#include <stdexcept>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace smartcar_costmap_plugins
{
void UnknownCostLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("unknown_cost", rclcpp::ParameterValue(5));
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("UnknownCostLayer parent expired");}
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".unknown_cost", unknown_cost_);
  if (unknown_cost_ < 1 || unknown_cost_ > 252) {
    throw std::invalid_argument("unknown_cost must be between 1 and 252");
  }
  current_ = true;
  RCLCPP_INFO(logger_, "Unknown traversal cost: %d (obstacles remain unchanged)", unknown_cost_);
}

void UnknownCostLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {return;}
  // Refresh newly exposed unknown space after rolling or map expansion, and
  // let earlier layers reconstruct their costs before replacing only 255.
  auto * grid = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, grid->getOriginX());
  *min_y = std::min(*min_y, grid->getOriginY());
  *max_x = std::max(*max_x, grid->getOriginX() + grid->getSizeInCellsX() * grid->getResolution());
  *max_y = std::max(*max_y, grid->getOriginY() + grid->getSizeInCellsY() * grid->getResolution());
}

void UnknownCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}
  auto * data = grid.getCharMap();
  const int width = grid.getSizeInCellsX();
  max_i = std::min(max_i, width);
  max_j = std::min(max_j, static_cast<int>(grid.getSizeInCellsY()));
  for (int y = std::max(0, min_j); y < max_j; ++y) {
    for (int x = std::max(0, min_i); x < max_i; ++x) {
      auto & cost = data[y * width + x];
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        cost = static_cast<unsigned char>(unknown_cost_);
      }
    }
  }
}
}  // namespace smartcar_costmap_plugins
PLUGINLIB_EXPORT_CLASS(smartcar_costmap_plugins::UnknownCostLayer, nav2_costmap_2d::Layer)
