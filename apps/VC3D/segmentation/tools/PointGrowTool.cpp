#include "PointGrowTool.hpp"
#include "ThinPlateSpline3d.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <functional>
#include <set>

namespace {
bool valid(const cv::Vec3f& p) {
    return p[0] >= 0 && p[1] >= 0 && p[2] >= 0 &&
           std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}
bool inside(const cv::Mat& m, cv::Point p) {
    return p.x >= 0 && p.y >= 0 && p.x < m.cols && p.y < m.rows;
}
double distanceToLine(cv::Point p, cv::Point a, cv::Point b) {
    const cv::Point2d v = b - a, d = p - a;
    const double t = v.dot(v) > 0 ? std::clamp(d.dot(v) / v.dot(v), 0.0, 1.0) : 0;
    return cv::norm(d - t * v);
}
}

void PointGrowTool::clear() {
    _points.release(); _original.release(); _anchors.clear(); _undo.clear(); _redo.clear(); _offset = {};
}

bool PointGrowTool::begin(const cv::Mat_<cv::Vec3f>& points) {
    clear();
    if (points.empty()) return false;
    _points = points.clone(); _original = points.clone();
    pad(16);
    return true;
}

void PointGrowTool::pad(int amount) {
    auto grow = [amount](cv::Mat_<cv::Vec3f>& m) {
        cv::Mat_<cv::Vec3f> next(m.rows + 2 * amount, m.cols + 2 * amount, cv::Vec3f(-1, -1, -1));
        m.copyTo(next(cv::Rect(amount, amount, m.cols, m.rows)));
        m = std::move(next);
    };
    grow(_points); grow(_original);
    const cv::Point delta(amount, amount);
    _offset += delta;
    if (_editRegion) { _editRegion->x += amount; _editRegion->y += amount; }
    for (auto& a : _anchors) a.grid += delta;
    for (auto* history : {&_undo, &_redo})
        for (auto& u : *history) {
            for (auto& c : u.changes) c.grid += delta;
            for (auto& a : u.anchors) a.grid += delta;
        }
}

std::optional<PointGrowTool::GridKey> PointGrowTool::pickPointNearSlice(
    const cv::Vec3f& world, const cv::Vec3f& normal, float pixelsPerVoxel,
    float radiusPixels, float sliceTolerance) const
{
    const double normalLength = cv::norm(normal);
    if (!valid(world) || !std::isfinite(normalLength) || normalLength < 1e-6 ||
        !std::isfinite(pixelsPerVoxel) || pixelsPerVoxel <= 0 ||
        !std::isfinite(radiusPixels) || radiusPixels <= 0 ||
        !std::isfinite(sliceTolerance) || sliceTolerance < 0) return std::nullopt;
    const cv::Vec3d n = cv::Vec3d(normal) / normalLength;
    const double radius = radiusPixels / pixelsPerVoxel;
    double bestDistance = radius * radius;
    double bestDepth = std::numeric_limits<double>::max();
    std::optional<GridKey> best;
    for (int r = 0; r < _points.rows; ++r) for (int c = 0; c < _points.cols; ++c) {
        if (!valid(_points(r,c))) continue;
        const cv::Vec3d delta = cv::Vec3d(_points(r,c)) - cv::Vec3d(world);
        const double depth = std::abs(delta.dot(n));
        if (depth > sliceTolerance) continue;
        const double distance = std::max(0.0, delta.dot(delta) - depth * depth);
        if (distance < bestDistance || (distance == bestDistance && depth < bestDepth)) {
            best = GridKey{r,c}; bestDistance = distance; bestDepth = depth;
        }
    }
    return best;
}

bool PointGrowTool::removePoint(GridKey key, std::string& status)
{
    const cv::Point grid(key.col, key.row);
    if (!inside(_points, grid) || !valid(_points(grid))) {
        status = "Right-click directly on a mesh point near the current slice.";
        return false;
    }
    Undo undo; undo.anchors = _anchors;
    undo.changes.push_back({grid, _points(grid)});
    _points(grid) = {-1,-1,-1};
    _anchors.erase(std::remove_if(_anchors.begin(), _anchors.end(),
        [&](const Anchor& anchor) { return anchor.grid == grid; }), _anchors.end());
    _undo.push_back(std::move(undo));
    _redo.clear();
    status = "Point deleted. Changes save automatically; Cmd/Ctrl+Z restores it.";
    return true;
}

bool PointGrowTool::addPoint(const cv::Vec3f& world, int width, std::string& status) {
    if (!active() || !valid(world)) { status = "Choose a point inside the CT volume."; return false; }
    std::optional<PointGrowTool> beforePadding;
    bool committed = false;
    struct Rollback { std::function<void()> restore; ~Rollback() { restore(); } } rollback{
        [&]() { if (!committed && beforePadding) *this = std::move(*beforePadding); }
    };
    width = std::clamp(width, 1, 6);
    cv::Point nearest(-1, -1);
    double best = std::numeric_limits<double>::max();
    // A compact imported chart is typically thousands of nodes. This scan has
    // no allocations or surface projections and also works at holes/concavities.
    for (int r = 0; r < _points.rows; ++r) for (int c = 0; c < _points.cols; ++c) {
        const auto& p = _points(r, c);
        if (!valid(p)) continue;
        const auto d = p - world;
        const double d2 = d.dot(d);
        if (d2 < best) { best = d2; nearest = {c, r}; }
    }
    if (nearest.x < 0) { status = "Import a surface with existing geometry first."; return false; }
    if (_editRegion && !_editRegion->contains(nearest)) {
        status = "That click is nearest to the other winding; choose it in the Winding box to edit there.";
        return false;
    }

    // Estimate the inherited chart's two tangents using a small local fit.
    std::vector<cv::Vec3d> rows, values;
    for (int dr = -2; dr <= 2; ++dr) for (int dc = -2; dc <= 2; ++dc) {
        cv::Point p = nearest + cv::Point(dc, dr);
        if (inside(_points, p) && valid(_points(p))) {
            rows.emplace_back(1, dc, dr);
            values.emplace_back(_points(p));
        }
    }
    if (rows.size() < 3) { status = "Click near a wider part of the surface edge."; return false; }
    cv::Mat a(static_cast<int>(rows.size()), 3, CV_64F, rows.data());
    cv::Mat b(static_cast<int>(values.size()), 3, CV_64F, values.data());
    cv::Mat coeff;
    if (!cv::solve(a, b, coeff, cv::DECOMP_SVD)) return false;
    cv::Vec3d du(coeff.at<double>(1,0), coeff.at<double>(1,1), coeff.at<double>(1,2));
    cv::Vec3d dv(coeff.at<double>(2,0), coeff.at<double>(2,1), coeff.at<double>(2,2));
    const double uu = du.dot(du), vv = dv.dot(dv), uv = du.dot(dv), det = uu * vv - uv * uv;
    if (det < 1e-6 || uu < 0.01 || vv < 0.01) {
        status = "The edge has no usable surface tangent; choose another nearby point."; return false;
    }
    cv::Vec3d d = cv::Vec3d(world) - cv::Vec3d(_points(nearest));
    double dc = (d.dot(du) * vv - d.dot(dv) * uv) / det;
    double dr = (d.dot(dv) * uu - d.dot(du) * uv) / det;
    const double step = std::sqrt(std::min(uu, vv));
    const cv::Vec3d normal = du.cross(dv) / std::sqrt(det);
    if (std::hypot(dc, dr) > 12 || cv::norm(d) > 14 * step || std::abs(d.dot(normal)) > 3 * step) {
        status = "Click closer to the edge (within 12 grid steps), then continue outward."; return false;
    }
    cv::Point target = nearest + cv::Point(std::lround(dc), std::lround(dr));
    const bool touchUp = inside(_original, target) && valid(_original(target)) && valid(_points(target));
    if (target.x < width + 4 || target.y < width + 4 ||
        target.x >= _points.cols - width - 4 || target.y >= _points.rows - width - 4) {
        beforePadding = *this;
        pad(16); nearest += cv::Point(16,16); target += cv::Point(16,16);
    }

    std::map<std::pair<int,int>,cv::Vec3f> proposed;
    if (touchUp) {
        // An occupied chart location is a local correction, not a rejected
        // grow click. Apply a compact, smooth displacement to connected vertices
        // only; holes and the rest of the imported chart remain untouched.
        const cv::Vec3f displacement = world - _points(target);
        const double radius = width + 1;
        std::vector<cv::Point> pending{target};
        std::set<std::pair<int,int>> visited{{target.y,target.x}};
        for (size_t i = 0; i < pending.size(); ++i) {
            const auto p = pending[i];
            const double t = cv::norm(p-target) / radius;
            const double falloff = std::pow(1-t,4) * (1+4*t);
            proposed[{p.y,p.x}] = _points(p) + static_cast<float>(falloff) * displacement;
            for (auto delta : {cv::Point(1,0),cv::Point(-1,0),cv::Point(0,1),cv::Point(0,-1)}) {
                const auto q = p + delta;
                if (inside(_points,q) && valid(_points(q)) && cv::norm(q-target) < radius &&
                    visited.emplace(q.y,q.x).second) pending.push_back(q);
            }
        }
    } else {
        const cv::Rect region(std::min(nearest.x,target.x)-width, std::min(nearest.y,target.y)-width,
                             std::abs(nearest.x-target.x)+2*width+1, std::abs(nearest.y-target.y)+2*width+1);
        std::vector<cv::Point> fill;
        for (int r = region.y; r < region.br().y; ++r) for (int c = region.x; c < region.br().x; ++c) {
            cv::Point p(c,r);
            if (!inside(_points,p) || (valid(_original(p)) && valid(_points(p)))) continue;
            // New strips attach to the existing grid; a correction only refits a
            // small neighbourhood of a point added in this session.
            if (distanceToLine(p,nearest,target) <= width &&
                (!valid(_points(p)) || cv::norm(p-target) <= width)) fill.push_back(p);
        }
        if (fill.empty()) { status = "No new surface cells at this point."; return false; }
        std::map<std::pair<int,int>, ThinPlateSpline3d::Sample> samples;
        auto sample = [&](cv::Point p, cv::Vec3f w) {
            samples[{p.y,p.x}] = {cv::Point2d(p),cv::Vec3d(w)};
        };
        // Bound the fit, not the number of original vertices preserved. All original
        // XYZ values are copied verbatim; interpolation only writes the small strip.
        std::vector<cv::Point> candidates;
        for (int r = region.y-3; r < region.br().y+3; ++r) for (int c = region.x-3; c < region.br().x+3; ++c) {
            cv::Point p(c,r);
            if (!inside(_points,p) || !valid(_points(p))) continue;
            if (valid(_original(p)) || cv::norm(p-target) > width) candidates.push_back(p);
        }
        if (candidates.size() > 64) {
            for (size_t i=0; i<64; ++i) { const auto p=candidates[i*candidates.size()/64]; sample(p,_points(p)); }
        } else for (auto p : candidates) sample(p,_points(p));
        for (const auto& anchor : _anchors) if (cv::norm(anchor.grid-target) < 18) sample(anchor.grid,anchor.world);
        sample(target,world);
        std::vector<ThinPlateSpline3d::Sample> fitSamples;
        for (const auto& entry : samples) fitSamples.push_back(entry.second);
        ThinPlateSpline3d tps;
        if (!tps.fit(fitSamples,0.0)) { status = "Cannot fit this extension; click nearer the existing edge."; return false; }

        for (auto p : fill) {
            auto value = tps.evaluate(cv::Point2d(p));
            if (!value || !valid(*value)) { status = "Extension leaves the CT coordinate range."; return false; }
            proposed[{p.y,p.x}] = *value;
        }
    }
    proposed[{target.y,target.x}] = world;
    for (const auto& anchor : _anchors) {
        if (anchor.grid != target && proposed.count({anchor.grid.y,anchor.grid.x}))
            proposed[{anchor.grid.y,anchor.grid.x}] = anchor.world;
    }
    auto at = [&](cv::Point p) -> cv::Vec3f {
        auto it = proposed.find({p.y,p.x});
        return it != proposed.end() ? it->second : inside(_points,p) ? _points(p) : cv::Vec3f(-1,-1,-1);
    };
    // Refuse collapsed/stretched edges and chart folds instead of silently
    // creating geometry that a downstream unwrap cannot use.
    for (const auto& [key,value] : proposed) {
        cv::Point p(key.second,key.first);
        if (!valid(value)) { status = "That correction leaves the CT coordinate range."; return false; }
        for (auto delta : {cv::Point(1,0),cv::Point(-1,0),cv::Point(0,1),cv::Point(0,-1)}) {
            auto q=at(p+delta); if (!valid(q)) continue;
            const double length=cv::norm(q-value), expected=delta.x ? std::sqrt(uu) : std::sqrt(vv);
            if (length < 0.15*expected || length > 3*expected) {
                status = "That point would stretch or collapse the grid. Use a closer point."; return false;
            }
        }
        for (int y=-1;y<=0;++y) for(int x=-1;x<=0;++x) {
            cv::Point o=p+cv::Point(x,y);
            auto p0=at(o), p1=at(o+cv::Point(1,0)), p2=at(o+cv::Point(0,1)), p3=at(o+cv::Point(1,1));
            if (!valid(p0)||!valid(p1)||!valid(p2)||!valid(p3)) continue;
            auto n0 = normal, n1 = normal;
            if (touchUp) {
                // A curved imported chart need not share one tangent plane.
                // Prevent each existing triangle from flipping relative to its
                // own orientation before this click.
                const auto q0=_points(o), q1=_points(o+cv::Point(1,0));
                const auto q2=_points(o+cv::Point(0,1)), q3=_points(o+cv::Point(1,1));
                n0=cv::Vec3d((q1-q0).cross(q2-q0));
                n1=cv::Vec3d((q3-q1).cross(q2-q1));
            }
            if (cv::Vec3d((p1-p0).cross(p2-p0)).dot(n0) <= 0 ||
                cv::Vec3d((p3-p1).cross(p2-p1)).dot(n1) <= 0) {
                status = "That point would fold the surface. Add a closer point along the sheet."; return false;
            }
        }
    }
    Undo undo; undo.anchors = _anchors;
    for (const auto& [key,value] : proposed) {
        cv::Point p(key.second,key.first);
        undo.changes.push_back({p,_points(p)}); _points(p)=value;
    }
    _anchors.erase(std::remove_if(_anchors.begin(),_anchors.end(),[&](const Anchor& a){return a.grid==target;}),_anchors.end());
    _anchors.push_back({target,world}); _undo.push_back(std::move(undo));
    _redo.clear();
    committed = true;
    status = touchUp
        ? "Surface corrected. Changes save automatically; Cmd/Ctrl+Z undoes, Cmd/Ctrl+Shift+Z redoes."
        : "Point added. Changes save automatically; Cmd/Ctrl+Z undoes, Cmd/Ctrl+Shift+Z redoes.";
    return true;
}

bool PointGrowTool::adoptPreview(const cv::Mat_<cv::Vec3f>& points, bool* changed) {
    if (changed) *changed = false;
    if (!active() || points.size() != _points.size()) return false;
    Undo undo; undo.anchors = _anchors;
    for (int r=0;r<points.rows;++r) for (int c=0;c<points.cols;++c) {
        if (points(r,c) == _points(r,c)) continue;
        // Push/pull moves existing vertices; it must not create/delete cells.
        if (!valid(points(r,c)) || !valid(_points(r,c))) return false;
        undo.changes.push_back({{c,r},_points(r,c)});
    }
    if (undo.changes.empty()) return true;
    if (changed) *changed = true;
    points.copyTo(_points);
    // An intentional brush stroke moves old click constraints with the sheet.
    for (auto& anchor : _anchors) anchor.world = _points(anchor.grid);
    _undo.push_back(std::move(undo));
    _redo.clear();
    return true;
}

bool PointGrowTool::moveHistory(std::vector<Undo>& from, std::vector<Undo>& to) {
    if (from.empty()) return false;
    auto entry=std::move(from.back()); from.pop_back();
    // Swap the exact values instead of rerunning the fit/brush. This also
    // preserves constraints and invalid cells after a grid expansion.
    for (auto& c:entry.changes) std::swap(_points(c.grid),c.before);
    _anchors.swap(entry.anchors);
    to.push_back(std::move(entry));
    return true;
}

bool PointGrowTool::undo() { return moveHistory(_undo,_redo); }
bool PointGrowTool::redo() { return moveHistory(_redo,_undo); }

std::vector<PointGrowTool::GridKey> PointGrowTool::changedVertices() const {
    std::vector<GridKey> result;
    for(int r=0;r<_points.rows;++r) for(int c=0;c<_points.cols;++c)
        if (_points(r,c)!=_original(r,c)) result.push_back({r,c});
    return result;
}

std::vector<cv::Vec3f> PointGrowTool::anchors() const {
    std::vector<cv::Vec3f> result;
    for(const auto& a:_anchors) result.push_back(a.world);
    return result;
}
