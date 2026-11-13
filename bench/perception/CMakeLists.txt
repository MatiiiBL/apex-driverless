# The perception evaluation: detection, accuracy, consistency, color and latency on scans along
# generated tracks (docs/perception.md, "Evaluation"). It scans with the perception tests' exact
# ray caster until the simulator's sensor models are available.
add_executable(apex_perception_eval
    perception_eval.cpp
    scoring.cpp
    ${PROJECT_SOURCE_DIR}/tests/perception/scene.cpp)
target_include_directories(apex_perception_eval PRIVATE ${PROJECT_SOURCE_DIR}/tests/perception)
target_link_libraries(apex_perception_eval PRIVATE apex::perception apex::sim apex_warnings)
