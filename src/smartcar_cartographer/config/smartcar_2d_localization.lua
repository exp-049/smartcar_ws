include "smartcar_2d.lua"
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
return options
