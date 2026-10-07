#pragma once

#include "SegmentationUndoHistory.hpp"
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace segmentation {

struct GrowthRevision {
    std::filesystem::path path;
    std::string timestamp;
    std::string label;
};

inline std::filesystem::path growthHistoryDirectory(const QuadSurface& surface)
{
    if (surface.path.empty()) throw std::runtime_error("Save the segment before growing it.");
    // Outside the segment: saveOverwrite atomically replaces its directory.
    return surface.path.parent_path() / ".vc3d-growth-history" / surface.path.filename();
}

inline void copyRevisionSidecars(const std::filesystem::path& source,
                                 const std::filesystem::path& destination)
{
    if (!std::filesystem::is_directory(source)) return;
    for (const auto& entry : std::filesystem::directory_iterator(source)) {
        // TIFF channels come from the in-memory snapshot, including removals.
        if (!entry.is_regular_file() || entry.path().extension() == ".tif" ||
            entry.path().filename() == "meta.json") continue;
        const auto target = destination / entry.path().filename();
        if (!std::filesystem::exists(target)) std::filesystem::copy_file(entry.path(), target);
    }
}

inline GrowthRevision saveGrowthRevision(QuadSurface& surface, const std::string& revisionId,
                                         const std::string& timestamp, const std::string& label)
{
    if (revisionId.empty() || revisionId.front() == '.' ||
        std::filesystem::path(revisionId).filename() != revisionId)
        throw std::runtime_error("Invalid revision ID");
    const auto root = growthHistoryDirectory(surface);
    const auto destination = root / revisionId;
    const auto staging = root / (".pending-" + revisionId);
    std::filesystem::create_directories(root);
    if (std::filesystem::exists(destination) || std::filesystem::exists(staging))
        throw std::runtime_error("Revision already exists");
    auto snapshot = snapshotSurface(surface);
    snapshot->meta["vc3d_revision"] = utils::Json{
        {"timestamp", timestamp}, {"label", label}, {"source_segment", surface.id}};
    try {
        snapshot->save(staging, surface.id, false);
        copyRevisionSidecars(surface.path, staging);
        std::filesystem::rename(staging, destination);
    } catch (...) {
        std::error_code ec;
        std::filesystem::remove_all(staging, ec);
        throw;
    }
    return {destination, timestamp, label};
}

inline std::vector<GrowthRevision> listGrowthRevisions(const QuadSurface& surface)
{
    std::vector<GrowthRevision> revisions;
    auto collect = [&](const std::filesystem::path& root, bool legacy) {
        if (!std::filesystem::is_directory(root)) return;
        for (const auto& entry : std::filesystem::directory_iterator(root)) {
            if (!entry.is_directory() || entry.path().filename().string().front() == '.') continue;
            const auto path = entry.path();
            if (!std::filesystem::exists(path / "x.tif") ||
                !std::filesystem::exists(path / "y.tif") ||
                !std::filesystem::exists(path / "z.tif")) continue;
            try {
                const auto meta = utils::Json::parse_file(path / "meta.json");
                if (legacy) {
                    revisions.push_back({path, meta.value("backup_timestamp", std::string()),
                                         "Earlier automatic backup " + path.filename().string()});
                } else if (meta.contains("vc3d_revision")) {
                    const auto& revision = meta["vc3d_revision"];
                    revisions.push_back({path, revision.value("timestamp", std::string()),
                                         revision.value("label", std::string("Before autogrow"))});
                }
            } catch (...) { /* An incomplete/corrupt entry is never offered for restore. */ }
        }
    };
    collect(growthHistoryDirectory(surface), false);
    const auto backupRoot = surface.backupRoot.empty() ? surface.path.parent_path() : surface.backupRoot;
    collect(backupRoot / "backups" / surface.path.filename(), true);
    // Older editing previews used a sibling backup folder instead of the
    // package root. Offer those existing recovery points as well.
    if (backupRoot != surface.path.parent_path())
        collect(surface.path.parent_path() / "backups" / surface.path.filename(), true);
    std::sort(revisions.begin(), revisions.end(), [](const auto& a, const auto& b) {
        if (a.timestamp.empty() && b.timestamp.empty())
            return std::filesystem::last_write_time(a.path) > std::filesystem::last_write_time(b.path);
        return a.timestamp == b.timestamp ? a.path > b.path : a.timestamp > b.timestamp;
    });
    return revisions;
}

// A revision opens as a new segment. The current segment and the checkpoint
// remain available, so revisiting history never destroys subsequent edits.
inline void openGrowthRevisionCopy(const GrowthRevision& revision,
                                   const std::filesystem::path& destination,
                                   const std::string& segmentId)
{
    QuadSurface saved(revision.path);
    saved.ensureLoaded();
    auto copy = snapshotSurface(saved);
    copy->meta["revision_source"] = revision.path.string();
    const auto staging = destination.parent_path() / (".revision-" + segmentId);
    if (std::filesystem::exists(destination) || std::filesystem::exists(staging))
        throw std::runtime_error("Revision copy already exists");
    try {
        copy->save(staging, segmentId, false);
        copyRevisionSidecars(revision.path, staging);
        std::filesystem::rename(staging, destination);
    } catch (...) {
        std::error_code ec;
        std::filesystem::remove_all(staging, ec);
        throw;
    }
}

} // namespace segmentation
