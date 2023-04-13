// #include "../include/occupany_grid_parser/costmap_parser.hpp"
#include "occupany_grid_parser/costmap_parser.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace ddg_costmap_parser_plugin
{

CostmapParser::CostmapParser()
// : last_min_x_(-std::numeric_limits<float>::max())
// , last_min_y_(-std::numeric_limits<float>::max())
// , last_max_x_(std::numeric_limits<float>::max())
// , last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void CostmapParser::onInitialize()
{
  auto node = node_.lock();
  // declareParameter("enabled", rclcpp::ParameterValue(true));
  // node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void CostmapParser::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x,
                                 double* min_y, double* max_x, double* max_y)
{
  if (need_recalculation_)
  {
    // last_min_x_ = *min_x;
    // last_min_y_ = *min_y;
    // last_max_x_ = *max_x;
    // last_max_y_ = *max_y;
    // // For some reason when I make these -<double>::max() it does not
    // // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // // -<float>::max() instead.
    // *min_x = -std::numeric_limits<float>::max();
    // *min_y = -std::numeric_limits<float>::max();
    // *max_x = std::numeric_limits<float>::max();
    // *max_y = std::numeric_limits<float>::max();
    // need_recalculation_ = false;

    // PSEUDO code
    // new_map_x_len = delta_x * map_resolution / robot_radius
    // new_map_y_len = delta_y * map_resolution / robot_radius
    // new vector<vector<char>> {x,y}
  }
  else
  {
    // double tmp_min_x = last_min_x_;
    // double tmp_min_y = last_min_y_;
    // double tmp_max_x = last_max_x_;
    // double tmp_max_y = last_max_y_;
    // last_min_x_ = *min_x;
    // last_min_y_ = *min_y;
    // last_max_x_ = *max_x;
    // last_max_y_ = *max_y;
    // *min_x = std::min(tmp_min_x, *min_x);
    // *min_y = std::min(tmp_min_y, *min_y);
    // *max_x = std::max(tmp_max_x, *max_x);
    // *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void CostmapParser::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CostmapParser::onFootprintChanged(): num footprint points: %lu",
               layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void CostmapParser::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  // unsigned char* master_array = master_grid.getCharMap();
  // unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // parseMap(master_grid);

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  if (CBSmap.empty() || CBSmap[0].empty() || CBSmap.size() != size_x || CBSmap[0].size() != size_y)
  {
    // TODO replace this with the actual activev size of the map minx miny
    CBSmap.resize(size_x);
    CBSmap[0].resize(size_y);
  }

  RCLCPP_INFO(logger_, "--------------------------------\n\n\n\n");

  RCLCPP_INFO(logger_, "min_i  = %d, min_j =  %d, max_i = %d , max_j = %d", min_i, min_j, max_i, max_j);

  for (int i = min_i; i < max_i; i++)
  {
    for (int j = min_j; j < max_j; j++)
    {
      int index = master_grid.getIndex(i, j);
      int cost = master_grid.getCost(i, j);

      CBSmap[i][j] = (cost > LETHAL_OBSTACLE) ? '@' : '.';
      std::cout << CBSmap[i][j];
    }
    std::cout << std::endl;
  }
  RCLCPP_INFO(logger_, "--------------------------------\n\n\n\n");
}

}  // namespace ddg_costmap_parser_plugin
// namespace ddg_costmap_parser_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::CostmapParser class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ddg_costmap_parser_plugin::CostmapParser, nav2_costmap_2d::Layer)
