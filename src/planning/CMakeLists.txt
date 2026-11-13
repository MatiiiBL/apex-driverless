# The planning module (docs/design.md, section 5.4; docs/planning.md).
apex_module(planning
    SOURCES
        config_json.cpp
        delaunay.cpp
        exploration.cpp
        ggv.cpp
        midpoint_search.cpp
        predicates.cpp
        qp_solver.cpp
        raceline.cpp
        smoothing.cpp
        speed_profile.cpp
        track_model.cpp
    PUBLIC_DEPS apex::core)

# The exact predicates' expansion arithmetic needs every product and sum rounded separately:
# no fused multiply-adds, which compilers otherwise form on targets that have them.
set_source_files_properties(predicates.cpp PROPERTIES COMPILE_OPTIONS -ffp-contract=off)
