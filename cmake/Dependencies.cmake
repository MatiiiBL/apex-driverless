include(FetchContent)

# Every dependency is pinned by version and checksum: the build downloads exactly this code,
# every time. SYSTEM keeps their headers out of apex's warning flags.

# Eigen is header-only. SOURCE_SUBDIR points at a directory that does not exist, so
# FetchContent only downloads it and never runs Eigen's own CMake project (tests, docs, BLAS).
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz
    URL_HASH SHA256=b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR do-not-configure
    SYSTEM)
FetchContent_MakeAvailable(eigen)
if(NOT TARGET Eigen3::Eigen)
    add_library(apex_eigen INTERFACE)
    target_include_directories(apex_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
    # Results must not depend on how many threads Eigen happens to use.
    target_compile_definitions(apex_eigen INTERFACE EIGEN_DONT_PARALLELIZE)
    add_library(Eigen3::Eigen ALIAS apex_eigen)
endif()

FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM)
FetchContent_MakeAvailable(nlohmann_json)

if(APEX_WITH_OSQP AND (APEX_BUILD_TESTS OR APEX_BUILD_BENCHMARKS))
    FetchContent_Declare(osqp
        URL https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz
        URL_HASH SHA256=ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM)
    set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
    set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
    set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
    FetchContent_MakeAvailable(osqp)
    # OSQP 1.0.0 calls its linear-system solvers through function pointers of a slightly
    # different type (osqp_api.c), which Clang's UBSan -fsanitize=function reports. It is the
    # reference solver, not apex code, so that one check is off for its sources only.
    if(APEX_SANITIZE)
        target_compile_options(OSQPLIB PRIVATE
            $<$<C_COMPILER_ID:Clang,AppleClang>:-fno-sanitize=function>)
    endif()
endif()
