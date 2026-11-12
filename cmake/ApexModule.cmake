# apex_module(<name> [SOURCES ...] [PUBLIC_DEPS ...] [PRIVATE_DEPS ...])
#
# Declares the library target apex_<name> (alias apex::<name>) for one module of src/. Public
# headers live in include/apex/<name>/; private headers may sit next to the sources. A module
# with no sources yet is an INTERFACE library, so its dependents can link it from day one.
function(apex_module name)
    cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS" ${ARGN})
    set(target apex_${name})
    if(ARG_SOURCES)
        add_library(${target} STATIC ${ARG_SOURCES})
        target_include_directories(${target}
            PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_compile_features(${target} PUBLIC cxx_std_20)
        target_link_libraries(${target}
            PUBLIC ${ARG_PUBLIC_DEPS}
            PRIVATE ${ARG_PRIVATE_DEPS} apex_warnings)
    else()
        add_library(${target} INTERFACE)
        target_include_directories(${target}
            INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
        target_compile_features(${target} INTERFACE cxx_std_20)
        target_link_libraries(${target} INTERFACE ${ARG_PUBLIC_DEPS})
    endif()
    add_library(apex::${name} ALIAS ${target})
endfunction()

# apex_tests(<module> SOURCES ... [DEPS ...])
#
# One GoogleTest executable per module, apex_<module>_tests, whose tests carry the module name
# as a ctest label (ctest -L <module>). APEX_SOURCE_DIR lets tests find files in the repo.
function(apex_tests module)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
    if(NOT ARG_SOURCES)
        return()
    endif()
    set(target apex_${module}_tests)
    add_executable(${target} ${ARG_SOURCES})
    target_link_libraries(${target} PRIVATE ${ARG_DEPS} apex_warnings GTest::gtest_main)
    target_compile_definitions(${target} PRIVATE
        APEX_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        $<$<BOOL:${APEX_SANITIZE}>:APEX_SANITIZE=1>)
    gtest_discover_tests(${target}
        PROPERTIES LABELS ${module}
        DISCOVERY_TIMEOUT 60)
endfunction()
