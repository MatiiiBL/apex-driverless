#include <filesystem>
#include <stdexcept>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"
#include "apex/runner/run.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::runner {
namespace {

TEST(MakeTrack, Oval) {
    TrackSpec spec;
    spec.oval_straight = 30.0;
    spec.oval_radius = 10.0;
    const Track track = make_track(spec);
    EXPECT_EQ(track.centerline.length(), make_oval_track(30.0, 10.0).centerline.length());
}

TEST(MakeTrack, GeneratedFromTheSeed) {
    TrackSpec spec;
    spec.source = TrackSpec::Source::Generated;
    spec.seed = 4;
    const Track track = make_track(spec);
    EXPECT_EQ(track.seed, 4U);
    EXPECT_EQ(track.cones.size(), sim::generate_track(4).cones.size());
    EXPECT_EQ(track.centerline.length(), sim::generate_track(4).centerline.length());
}

TEST(MakeTrack, NamedLayout) {
    TrackSpec spec;
    spec.source = TrackSpec::Source::Layout;
    spec.layout = sim::reference_track_names().front();
    EXPECT_EQ(make_track(spec).centerline.length(),
              sim::reference_track(spec.layout).centerline.length());
    spec.layout = "no such layout";
    EXPECT_THROW((void)make_track(spec), std::invalid_argument);
}

TEST(MakeTrack, FromAFile) {
    const Track oval = make_oval_track(25.0, 9.0);
    const std::filesystem::path path =
        std::filesystem::path(testing::TempDir()) / "apex_make_track_test.json";
    save_track(oval, path);
    TrackSpec spec;
    spec.source = TrackSpec::Source::File;
    spec.file = path.string();
    const Track loaded = make_track(spec);
    std::filesystem::remove(path);
    EXPECT_EQ(loaded.cones.size(), oval.cones.size());
    EXPECT_NEAR(loaded.centerline.length(), oval.centerline.length(), 1e-9);

    spec.file = (std::filesystem::path(testing::TempDir()) / "apex_missing_track.json").string();
    EXPECT_THROW((void)make_track(spec), std::runtime_error);
}

TEST(MakeWorldConfig, TrueCarFrictionAndSeed) {
    RunConfig config;
    config.vehicle.mass = 255.0;
    config.nominal.mass = 230.0;
    config.friction_scale = 0.8;
    config.seed = 42;
    const sim::WorldConfig world = make_world_config(config);
    EXPECT_DOUBLE_EQ(world.vehicle.mass, 255.0);
    EXPECT_DOUBLE_EQ(world.friction_scale, 0.8);
    EXPECT_EQ(world.seed, 42U);
}

}  // namespace
}  // namespace apex::runner
