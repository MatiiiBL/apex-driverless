#include "apex/runner/sweep.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <filesystem>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include "apex/core/json.hpp"

namespace apex::runner {

using nlohmann::json;

namespace {

// Joins every started worker on the way out, also when starting a later one throws.
class Workers {
public:
    Workers() = default;
    Workers(const Workers&) = delete;
    Workers& operator=(const Workers&) = delete;
    ~Workers() {
        for (std::thread& t : threads_) {
            if (t.joinable()) t.join();
        }
    }
    template <class F>
    void start(F&& f) {
        threads_.emplace_back(std::forward<F>(f));
    }

private:
    std::vector<std::thread> threads_;
};

RunConfig parse_run(const json& j, const std::string& context) {
    try {
        RunConfig config = j.get<RunConfig>();
        validate(config);
        return config;
    } catch (const std::exception& e) {
        throw std::invalid_argument(context + ": " + e.what());
    }
}

}  // namespace

void to_json(json& j, const SweepConfig& c) {
    json variants = json::array();
    for (const SweepVariant& v : c.variants) {
        variants.push_back({{"name", v.name}, {"patch", v.patch}});
    }
    j = {{"name", c.name},
         {"base", c.base},
         {"tracks", c.tracks},
         {"seeds", c.seeds},
         {"variants", std::move(variants)},
         {"oracle_variant", c.oracle_variant},
         {"threads", c.threads}};
}

void from_json(const json& j, SweepConfig& c) {
    reject_unknown_keys(
        j, {"name", "base", "tracks", "seeds", "variants", "oracle_variant", "threads"}, "sweep");
    read_optional(j, "name", c.name);
    read_optional(j, "base", c.base);
    if (!c.base.is_object()) throw std::invalid_argument("sweep: base must be an object");
    read_optional(j, "tracks", c.tracks);
    read_optional(j, "seeds", c.seeds);
    if (const auto it = j.find("variants"); it != j.end()) {
        c.variants.clear();
        for (const json& v : *it) {
            reject_unknown_keys(v, {"name", "patch"}, "sweep.variants");
            SweepVariant variant;
            variant.name = v.at("name").get<std::string>();
            read_optional(v, "patch", variant.patch);
            if (!variant.patch.is_object()) {
                throw std::invalid_argument("sweep: the patch of variant '" + variant.name +
                                            "' must be an object");
            }
            c.variants.push_back(std::move(variant));
        }
    }
    read_optional(j, "oracle_variant", c.oracle_variant);
    read_optional(j, "threads", c.threads);
}

std::string track_label(const TrackSpec& spec) {
    switch (spec.source) {
        case TrackSpec::Source::Generated:
            return "gen" + std::to_string(spec.seed);
        case TrackSpec::Source::Layout:
            return spec.layout;
        case TrackSpec::Source::File:
            return std::filesystem::path(spec.file).stem().string();
        case TrackSpec::Source::Oval:
            return "oval";
    }
    return "track";
}

std::vector<SweepRun> expand_sweep(const SweepConfig& config) {
    const RunConfig base = parse_run(config.base, "sweep base");
    std::vector<SweepVariant> variants = config.variants;
    if (variants.empty()) variants.push_back({"base", json::object()});
    const std::vector<TrackSpec> tracks =
        config.tracks.empty() ? std::vector<TrackSpec>{base.track} : config.tracks;
    const std::vector<std::uint64_t> seeds =
        config.seeds.empty() ? std::vector<std::uint64_t>{base.seed} : config.seeds;

    std::set<std::string> names;
    for (const SweepVariant& v : variants) {
        if (v.name.empty()) throw std::invalid_argument("sweep: a variant has no name");
        if (!names.insert(v.name).second) {
            throw std::invalid_argument("sweep: duplicate variant '" + v.name + "'");
        }
    }
    if (!config.oracle_variant.empty() && !names.contains(config.oracle_variant)) {
        throw std::invalid_argument("sweep: oracle_variant '" + config.oracle_variant +
                                    "' is not a variant");
    }

    std::vector<SweepRun> runs;
    runs.reserve(variants.size() * tracks.size() * seeds.size());
    for (const SweepVariant& variant : variants) {
        json patched = config.base;
        patched.merge_patch(variant.patch);
        const RunConfig arm = parse_run(patched, "sweep variant '" + variant.name + "'");
        for (const TrackSpec& track : tracks) {
            for (const std::uint64_t seed : seeds) {
                SweepRun run;
                run.variant = variant.name;
                run.track = track_label(track);
                run.seed = seed;
                run.config = arm;
                run.config.track = track;
                run.config.seed = seed;
                run.config.name = variant.name + "/" + run.track + "/s" + std::to_string(seed);
                try {
                    validate(run.config);
                } catch (const std::exception& e) {
                    throw std::invalid_argument(run.config.name + ": " + e.what());
                }
                runs.push_back(std::move(run));
            }
        }
    }
    return runs;
}

std::vector<RunResult> run_parallel(const std::vector<SweepRun>& runs, const RunFunction& run,
                                    unsigned threads, const RunCallback& on_done) {
    std::vector<RunResult> results(runs.size());
    if (runs.empty()) return results;
    if (threads == 0) threads = std::max(std::thread::hardware_concurrency(), 1U);
    const std::size_t workers = std::min<std::size_t>(threads, runs.size());

    std::atomic<std::size_t> next{0};
    std::mutex callback_mutex;
    // Each worker takes the next run not yet taken; results land in their own slots, so the
    // workers write disjoint elements and need no lock but the callback's.
    const auto work = [&] {
        for (std::size_t i = next.fetch_add(1); i < runs.size(); i = next.fetch_add(1)) {
            const RunConfig& config = runs[i].config;
            RunResult result;
            try {
                result = run(config);
            } catch (const std::exception& e) {
                result = RunResult{};
                result.name = config.name;
                result.seed = config.seed;
                result.laps_target = config.stack.mission.laps;
                result.error = e.what();
            }
            results[i] = std::move(result);
            if (on_done) {
                const std::lock_guard lock(callback_mutex);
                on_done(i, results[i]);
            }
        }
    };
    {
        Workers pool;
        for (std::size_t k = 0; k < workers; ++k) pool.start(work);
    }
    return results;
}

}  // namespace apex::runner
