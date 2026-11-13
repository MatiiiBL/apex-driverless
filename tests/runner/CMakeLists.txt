# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(runner
    SOURCES
        centerline_driver.cpp
        config_test.cpp
        delay_queue_test.cpp
        metrics_test.cpp
        oracle_test.cpp
        result_test.cpp
        run_test.cpp
        summary_test.cpp
        sweep_test.cpp
        world_setup_test.cpp
    DEPS apex::runner)

# The `apex` tool, run as a user would: each test checks the exit status or the message.
# apex_cli_test(<name> <regex or WILL_FAIL> <args>...)
function(apex_cli_test name expect)
    add_test(NAME cli.${name} COMMAND apex ${ARGN})
    if(expect STREQUAL "WILL_FAIL")
        set_tests_properties(cli.${name} PROPERTIES WILL_FAIL TRUE LABELS runner)
    else()
        # With a pass expression ctest ignores the exit status, so failure paths can be
        # checked by their message.
        set_tests_properties(cli.${name} PROPERTIES PASS_REGULAR_EXPRESSION "${expect}"
                             LABELS runner)
    endif()
endfunction()

set(cli_out ${CMAKE_CURRENT_BINARY_DIR}/cli)
apex_cli_test(help "Commands:.*track" --help)
apex_cli_test(no_arguments_fail WILL_FAIL)
apex_cli_test(unknown_command "unknown command 'drive'" drive)
apex_cli_test(track_help "--layout NAME" track --help)
apex_cli_test(track_generated "cones; tightest radius" track --seed 3 --out ${cli_out}/gen3.json)
apex_cli_test(track_layout "cones;" track --layout technical --out ${cli_out}/technical.json)
apex_cli_test(track_oval "cones;" track --oval --out ${cli_out}/oval.json)
apex_cli_test(track_one_source "exactly one of --seed" track --seed 3 --oval --out x.json)
apex_cli_test(track_needs_out "--out is required" track --seed 3)
apex_cli_test(track_bad_seed "non-negative integer" track --seed -3 --out x.json)
apex_cli_test(track_unknown_layout "apex track: .*hairpins" track --layout hairpins --out x.json)
apex_cli_test(track_unknown_option "unknown option '--sed'" track --sed 3 --out x.json)
apex_cli_test(track_bad_usage_fails WILL_FAIL track --seed 3)

set(cli_data ${CMAKE_CURRENT_SOURCE_DIR}/data)
apex_cli_test(run_help "--log PATH" run --help)
apex_cli_test(run_time_limit "short: DNF after 0 of 10 laps: time limit"
              run --config ${cli_data}/short_run.json --out ${cli_out}/short.json
              --log ${cli_out}/short.csv --quiet)
apex_cli_test(run_missing_config "cannot open .*missing.json"
              run --config ${cli_data}/missing.json --out ${cli_out}/x.json)
apex_cli_test(run_unknown_key "bad_key_run.json: .*'lap'"
              run --config ${cli_data}/bad_key_run.json --out ${cli_out}/x.json)
apex_cli_test(run_needs_config "--config is required" run --out ${cli_out}/x.json)
apex_cli_test(sweep_help "--threads N" sweep --help)
apex_cli_test(sweep_short "Short sweep: 4 runs on 2 threads"
              sweep --config ${cli_data}/short_sweep.json --out ${cli_out}/sweep --quiet)
apex_cli_test(sweep_bad_threads "non-negative integer"
              sweep --config ${cli_data}/short_sweep.json --out ${cli_out}/sweep --threads x)
apex_cli_test(replay_help "50 ms" replay --help)
apex_cli_test(replay_short "short: DNF"
              replay --config ${cli_data}/short_run.json --out ${cli_out}/replay.json --quiet)
