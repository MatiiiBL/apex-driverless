# The perception evaluation: detection, accuracy, consistency, color and latency on scans from
# the simulator's sensor models along generated tracks (docs/perception.md, "Evaluation").
add_executable(apex_perception_eval
    perception_eval.cpp
    scoring.cpp)
target_link_libraries(apex_perception_eval PRIVATE apex::perception apex::sim apex_warnings)
