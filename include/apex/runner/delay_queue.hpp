#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace apex::runner {

// Messages in flight between a sensor and the car's software: each is released on a given tick,
// its stamp plus the latency of the processing that produced it (docs/design.md, section 4.4).
// Messages due on the same tick come out in the order they were pushed, so an older camera frame
// released together with a fresh IMU sample is handed over first. Ticks, not seconds, keep the
// release times exact.
template <class T>
class DelayQueue {
public:
    void push(T item, std::int64_t due_tick) {
        heap_.push_back({due_tick, next_sequence_++, std::move(item)});
        std::push_heap(heap_.begin(), heap_.end(), later);
    }

    // Moves every item due at or before `tick` into `out` (appended), earliest first.
    void pop_due(std::int64_t tick, std::vector<T>& out) {
        while (!heap_.empty() && heap_.front().due <= tick) {
            std::pop_heap(heap_.begin(), heap_.end(), later);
            out.push_back(std::move(heap_.back().item));
            heap_.pop_back();
        }
    }

    [[nodiscard]] bool empty() const { return heap_.empty(); }
    [[nodiscard]] std::size_t size() const { return heap_.size(); }

private:
    struct Entry {
        std::int64_t due;
        std::uint64_t sequence;
        T item;
    };
    // Heap order: the root is the entry due first, ties broken by push order.
    static bool later(const Entry& a, const Entry& b) {
        return a.due != b.due ? a.due > b.due : a.sequence > b.sequence;
    }

    std::vector<Entry> heap_;
    std::uint64_t next_sequence_ = 0;
};

}  // namespace apex::runner
