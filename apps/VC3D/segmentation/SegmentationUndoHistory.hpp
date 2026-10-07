#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "vc/core/util/QuadSurface.hpp"

namespace segmentation
{

// Clone channels as well as points: cv::Mat copies otherwise alias live data.
inline std::shared_ptr<QuadSurface> snapshotSurface(QuadSurface& surface)
{
    auto snapshot = std::make_shared<QuadSurface>(surface.rawPoints(), surface.scale());
    snapshot->id = surface.id;
    snapshot->path = surface.path;
    snapshot->meta = surface.meta;
    snapshot->setDpi(surface.dpi());
    for (const auto& name : surface.channelNames()) {
        const auto channel = surface.channel(name, SURF_CHANNEL_NORESIZE);
        if (!channel.empty()) snapshot->setChannel(name, channel.clone());
    }
    return snapshot;
}

inline void restoreSurface(QuadSurface& target, QuadSurface& snapshot)
{
    snapshot.rawPoints().copyTo(*target.rawPointsPtr());
    for (const auto& name : target.channelNames()) target.removeChannel(name);
    for (const auto& name : snapshot.channelNames()) {
        const auto channel = snapshot.channel(name, SURF_CHANNEL_NORESIZE);
        if (!channel.empty()) target.setChannel(name, channel.clone());
    }
    target.meta = snapshot.meta;
    target.setDpi(snapshot.dpi());
    target.invalidateCache();
}

// Delta-based vertex edit for efficient undo
struct VertexDelta {
    int row{0};
    int col{0};
    cv::Vec3f previousWorld{0.0f, 0.0f, 0.0f};  // Position before the edit
};

class UndoHistory
{
public:
    // Capture full surface state (legacy method - memory intensive)
    bool capture(const cv::Mat_<cv::Vec3f>& points)
    {
        if (points.empty()) {
            return false;
        }

        cv::Mat_<cv::Vec3f> clone = points.clone();
        if (clone.empty()) {
            return false;
        }

        if (_states.size() >= kMaxEntries) {
            _states.pop_front();
        }
        _states.push_back({std::move(clone), {}, {}});
        return true;
    }

    // Capture only changed vertices (delta-based - memory efficient)
    bool captureDelta(const std::vector<VertexDelta>& deltas)
    {
        if (deltas.empty()) {
            return false;
        }

        if (_states.size() >= kMaxEntries) {
            _states.pop_front();
        }
        _states.push_back({cv::Mat_<cv::Vec3f>(), deltas, {}});
        return true;
    }

    // Capture a complete surface for operations such as Grow that can resize
    // the grid and replace ancillary channels/metadata as one transaction.
    bool captureSurface(std::shared_ptr<QuadSurface> surface)
    {
        if (!surface) {
            return false;
        }

        if (_states.size() >= kMaxEntries) {
            _states.pop_front();
        }
        _states.push_back({{}, {}, std::move(surface)});
        return true;
    }

    void discardLast()
    {
        if (!_states.empty()) {
            _states.pop_back();
        }
    }

    [[nodiscard]] std::optional<cv::Mat_<cv::Vec3f>> takeLast()
    {
        if (_states.empty()) {
            return std::nullopt;
        }
        Entry entry = std::move(_states.back());
        _states.pop_back();

        // Return full points if available (legacy)
        if (!entry.points.empty()) {
            return entry.points;
        }

        // Delta-based entry - return empty (caller should use takeLastDelta)
        return std::nullopt;
    }

    // Take the last entry, returning deltas if available
    [[nodiscard]] std::optional<std::vector<VertexDelta>> takeLastDelta()
    {
        if (_states.empty()) {
            return std::nullopt;
        }
        Entry entry = std::move(_states.back());
        _states.pop_back();

        // Return deltas if this was a delta-based entry
        if (!entry.deltas.empty()) {
            return entry.deltas;
        }

        return std::nullopt;
    }

    // Check if the last entry is delta-based (vs full snapshot)
    [[nodiscard]] bool lastIsDelta() const
    {
        if (_states.empty()) {
            return false;
        }
        return !_states.back().deltas.empty();
    }

    [[nodiscard]] bool lastIsSurface() const { return !_states.empty() && static_cast<bool>(_states.back().surface); }

    [[nodiscard]] std::shared_ptr<QuadSurface> takeLastSurface()
    {
        if (_states.empty()) {
            return {};
        }
        Entry entry = std::move(_states.back());
        _states.pop_back();
        return std::move(entry.surface);
    }

    void pushBack(cv::Mat_<cv::Vec3f> points)
    {
        if (points.empty()) {
            return;
        }
        if (_states.size() >= kMaxEntries) {
            _states.pop_front();
        }
        _states.push_back({std::move(points), {}, {}});
    }

    void pushBackSurface(std::shared_ptr<QuadSurface> surface)
    {
        if (!surface) {
            return;
        }
        if (_states.size() >= kMaxEntries) {
            _states.pop_front();
        }
        _states.push_back({{}, {}, std::move(surface)});
    }

    void clear() { _states.clear(); }

    [[nodiscard]] bool empty() const { return _states.empty(); }

    [[nodiscard]] size_t size() const { return _states.size(); }

private:
    struct Entry {
        cv::Mat_<cv::Vec3f> points;            // Full snapshot (legacy, memory intensive)
        std::vector<VertexDelta> deltas;       // Delta-based (memory efficient)
        std::shared_ptr<QuadSurface> surface;  // Full grow transaction, including channels/meta
    };

    static constexpr std::size_t kMaxEntries = 20;

    std::deque<Entry> _states;
};

}  // namespace segmentation
