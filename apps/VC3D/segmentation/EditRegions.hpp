#pragma once
// Editing regions of a stitched multi-winding segment: each region is a block of grid cells (one winding) with a label.
// Stored in the segment's meta.json as
//   "editing_regions": [{"label": "w027 (previous winding)", "cols": [0, 700], "rows": [0, 410]}, ...]
//   "editing_region_selected": 1   (or "editing_region_default")
// in the CURRENT grid's cell coordinates; the module keeps them in step with grid padding.
#include <QString>
#include <cmath>
#include <vector>
#include <opencv2/core.hpp>
#include "vc/core/util/QuadSurface.hpp"

struct EditRegion {
    QString label;
    cv::Rect cells;
};

struct EditRegionInfo {
    std::vector<EditRegion> regions;
    int selected{-1};
};

inline int editRegionJsonInt(const utils::Json& value, int fallback)
{
    return value.is_number() ? static_cast<int>(std::lround(value.get_double())) : fallback;
}

inline EditRegionInfo editRegionsFromMeta(QuadSurface* surface)
{
    EditRegionInfo info;
    if (!surface || surface->meta.is_null() || !surface->meta.is_object() || !surface->meta.contains("editing_regions")) {
        return info;
    }
    const auto& list = surface->meta["editing_regions"];
    if (!list.is_array()) {
        return info;
    }
    const cv::Mat_<cv::Vec3f>* points = surface->rawPointsPtr();
    const int rows = points ? points->rows : 0;
    const int cols = points ? points->cols : 0;
    for (size_t i = 0; i < list.size(); ++i) {
        const auto& entry = list[i];
        if (!entry.is_object() || !entry.contains("cols") || !entry["cols"].is_array() || entry["cols"].size() < 2) {
            continue;
        }
        cv::Rect cells(0, 0, cols, rows);
        cells.x = editRegionJsonInt(entry["cols"][static_cast<size_t>(0)], 0);
        cells.width = std::max(0, editRegionJsonInt(entry["cols"][static_cast<size_t>(1)], cols) - cells.x);
        if (entry.contains("rows") && entry["rows"].is_array() && entry["rows"].size() >= 2) {
            cells.y = editRegionJsonInt(entry["rows"][static_cast<size_t>(0)], 0);
            cells.height = std::max(0, editRegionJsonInt(entry["rows"][static_cast<size_t>(1)], rows) - cells.y);
        }
        QString label = QStringLiteral("Region %1").arg(i + 1);
        if (entry.contains("label") && entry["label"].is_string()) {
            label = QString::fromStdString(entry["label"].get_string());
        }
        info.regions.push_back({label, cells});
    }
    if (!info.regions.empty()) {
        if (surface->meta.contains("editing_region_selected")) {
            info.selected = editRegionJsonInt(surface->meta["editing_region_selected"], -1);
        } else if (surface->meta.contains("editing_region_default")) {
            info.selected = editRegionJsonInt(surface->meta["editing_region_default"], -1);
        }
        if (info.selected >= static_cast<int>(info.regions.size())) {
            info.selected = -1;
        }
    }
    return info;
}

inline utils::Json editRegionsToJson(const std::vector<EditRegion>& regions)
{
    auto list = utils::Json::array();
    for (const auto& region : regions) {
        auto entry = utils::Json::object();
        entry["label"] = region.label.toStdString();
        auto cols = utils::Json::array();
        cols.push_back(region.cells.x);
        cols.push_back(region.cells.x + region.cells.width);
        auto rows = utils::Json::array();
        rows.push_back(region.cells.y);
        rows.push_back(region.cells.y + region.cells.height);
        entry["cols"] = cols;
        entry["rows"] = rows;
        list.push_back(entry);
    }
    return list;
}
