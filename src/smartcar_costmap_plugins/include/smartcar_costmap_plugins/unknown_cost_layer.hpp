#pragma once
#include "nav2_costmap_2d/layer.hpp"

namespace smartcar_costmap_plugins
{
// Run last, after static/obstacle/inflation layers. The SLAM /map is untouched;
// only the planner's master grid encodes unknown cells as traversal costs.
class UnknownCostLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override {current_ = true;}
  bool isClearable() override {return false;}
private:
  int unknown_cost_{5};
};
}  // namespace smartcar_costmap_plugins
