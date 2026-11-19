# Ask the walker for a gait. 0 for step_height or cycle_time means "use gait.yaml";
# non-zero values are reserved in version 1 and ignored with a warning.
string gait         # tripod, ripple or wave
float64 step_height # m
float64 cycle_time  # s
