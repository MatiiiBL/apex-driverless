# The `apex` command-line tool (docs/design.md, section 5.6).
add_executable(apex
    apex.cpp
    cli.cpp
    track_command.cpp)
target_link_libraries(apex PRIVATE apex::runner apex_warnings)
