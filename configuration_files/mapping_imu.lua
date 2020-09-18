
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
