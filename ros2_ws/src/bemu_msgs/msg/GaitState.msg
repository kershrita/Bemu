# What the walker is doing. Legs are in this order: lf, lm, lr, rf, rm, rr.
std_msgs/Header header
string gait               # the gait being walked, or blended toward
string mode               # sitting, standing_up, standing, stopping or sitting_down
float64[6] phase          # each leg's phase in [0, 1)
bool[6] foot_down         # true while the foot is on the ground
float64 stability_margin  # m
