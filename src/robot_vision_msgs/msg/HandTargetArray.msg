# All hands detected in one frame, plus the global safety verdict.
std_msgs/Header header
string safety_state                 # "RUN" / "SLOW" / "STOP" / "WARNING"
HandTarget[] hands
