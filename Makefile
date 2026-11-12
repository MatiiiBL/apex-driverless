# Convenience targets over the CMake presets. `make check` is what every commit must pass.
#
#   make build            configure (once) and build the dev preset
#   make test             build, then run every test
#   make check            format check + build + test
#   make format           reformat every C++ file in place
#   make PRESET=asan test the same with sanitizers (also: tsan, release)

PRESET ?= dev
# Builds share the machine with other work; Eigen-heavy translation units need ~1 GB each.
JOBS ?= 3
CLANG_FORMAT ?= uvx --quiet --from clang-format==21.1.8 clang-format
CXX_SOURCES = $(shell git ls-files --cached --others --exclude-standard '*.cpp' '*.hpp')

.PHONY: build test check format format-check clean

build:
	@test -f build/$(PRESET)/build.ninja || cmake --preset $(PRESET)
	cmake --build --preset $(PRESET) -j $(JOBS)

test: build
	ctest --preset $(PRESET) -j $(JOBS)

check: format-check test

format:
	$(CLANG_FORMAT) -i $(CXX_SOURCES)

format-check:
	$(CLANG_FORMAT) --dry-run --Werror $(CXX_SOURCES)

clean:
	rm -rf build/$(PRESET)
