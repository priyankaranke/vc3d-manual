// Flat-view rotation of the Manual segmentation view (29 Oct 2026): the rotation lives in SurfaceDisplayMap,
// so rendering, clicks (inverse), overlays (gridMap) and pan/zoom all share one exact mapping.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "volume_viewers/SurfaceDisplayMap.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <random>

namespace {
// 21 x 21 flat grid, 10 nominal units per cell: nominal (x, y) -> grid (x/10, y/10).
SurfaceDisplayMap makeMap(float degrees) {
    cv::Mat_<cv::Vec3f> points(21, 21);
    for (int r = 0; r < 21; ++r) for (int c = 0; c < 21; ++c) points(r, c) = cv::Vec3f(c * 10.f, r * 10.f, 500.f);
    cv::Mat_<cv::Vec2f> offsets(21, 21, cv::Vec2f(0, 0));
    return SurfaceDisplayMap(points, offsets, cv::Vec2f(0.1f, 0.1f), cv::Vec3f(0, 0, 0), degrees);
}
} // namespace

TEST_CASE("rotation: inverse undoes forward everywhere inside the grid") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(5.f, 195.f);
    for (float deg : {0.f, 30.f, -72.5f, 180.f, 359.f, 1e-7f}) {
        const auto map = makeMap(deg);
        for (int i = 0; i < 500; ++i) {
            const cv::Vec2f p(u(rng), u(rng));
            const auto back = map.inverse(map.forward(p));
            CHECK(std::abs(back[0] - p[0]) < 1e-3f);
            CHECK(std::abs(back[1] - p[1]) < 1e-3f);
        }
    }
}

TEST_CASE("rotation: positive degrees turn the display clockwise on screen (x right, y down)") {
    const auto map = makeMap(30.f);
    const auto d = map.forward({110.f, 100.f}) - map.forward({100.f, 100.f});
    CHECK(std::abs(d[0] - 10.f * std::cos(30.f * float(CV_PI) / 180.f)) < 1e-4f);
    CHECK(std::abs(d[1] - 10.f * std::sin(30.f * float(CV_PI) / 180.f)) < 1e-4f);
    CHECK(std::abs(cv::norm(d) - 10.0) < 1e-4);
    const auto zero = makeMap(0.f);
    const auto d0 = zero.forward({110.f, 100.f}) - zero.forward({100.f, 100.f});
    CHECK(std::abs(d0[0] - 10.f) < 1e-6f);
    CHECK(std::abs(d0[1]) < 1e-6f);
}

TEST_CASE("rotation: the pixel grid map follows the rotation and keeps full coverage") {
    const cv::Size pixels(100, 60);
    const cv::Vec2f center(100.f, 100.f);
    for (float deg : {0.f, 45.f, 90.f, -135.f}) {
        const auto map = makeMap(deg);
        const auto grid = map.gridMap(pixels, center, 1.0f);
        REQUIRE(grid.size() == pixels);
        const auto mid = grid(30, 50);
        CHECK(std::abs(mid[0] - 10.f) < 0.05f);
        CHECK(std::abs(mid[1] - 10.f) < 0.05f);
        // 20 screen pixels to the right = 20 nominal units along the un-rotated screen-x direction
        const float t = -deg * float(CV_PI) / 180.f;
        const auto right = grid(30, 70);
        CHECK(std::abs(right[0] - (10.f + 2.f * std::cos(t))) < 0.05f);
        CHECK(std::abs(right[1] - (10.f + 2.f * std::sin(t))) < 0.05f);
        int valid = 0;
        for (const auto& g : grid) valid += (g[0] >= 0 && g[1] >= 0);
        CHECK(valid >= int(0.995 * pixels.area()));
    }
}
