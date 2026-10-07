#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/Array3D.hpp"
#include "vc/tracer/Tracer.hpp"
#include "segmentation/SegmentationGrowthOverlay.hpp"
#include <filesystem>
#include <opencv2/core.hpp>
#include <random>

TEST_CASE("real tracer respects steps and fixed vertices and reports blocked frontier")
{
    const auto root = std::filesystem::temp_directory_path() /
        ("vc-growth-workflow-" + std::to_string(std::random_device{}()));
    struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove_all(p); } } cleanup{root};
    Volume::ZarrCreateOptions options;
    options.shapeZYX = {64, 64, 64};
    options.chunkShapeZYX = {32, 32, 32};
    options.numLevels = 1;
    options.fillValue = 255;
    options.voxelSize = 1;
    options.uuid = "test-growth";
    options.name = "test-growth";
    options.compressor = "none";
    auto volume = Volume::New(root / "volume", options);
    Array3D<uint8_t> data({64, 64, 64}, 255);
    volume->writeZYX(data, {0, 0, 0}, 0);
    std::filesystem::create_directories(root / "cache");
    cv::Mat_<cv::Vec3f> points(8, 8, cv::Vec3f(-1, -1, -1));
    cv::Mat_<uint16_t> generations(8, 8, uint16_t(0));
    for (int y = 2; y < 6; ++y) for (int x = 2; x < 6; ++x) {
        points(y, x) = cv::Vec3f(18 + x, 18 + y, 20);
        generations(y, x) = 7;
    }
    points(3, 3)[2] += 0.15f; // an existing hand correction
    points(4, 4) = cv::Vec3f(-1, -1, -1); // interior hole must not block exterior
    generations(4, 4) = 0;
    QuadSurface original(points, cv::Vec2f(1, 1));
    original.id = "synthetic";
    original.setChannel("generations", generations);
    utils::Json params{{"generations", 37}, {"preserve_resume_geometry", true},
        {"step_size", 1.0}, {"snap_weight", 0.0}, {"normal_weight", 0.0}};
    params["growth_directions"] = utils::Json::array();
    params["growth_directions"].push_back("right");
    std::unique_ptr<QuadSurface> grown(tracer(*volume, 1, 0, {20, 20, 20}, params,
        (root / "cache").string(), 1, {}, &original));
    REQUIRE(grown);
    CHECK(grown->meta["growth_report"]["completed_steps"].get_int() == 30);
    CHECK(grown->meta["growth_report"]["added_vertices"].get_int() > 100);
    CHECK(segmentation::preserveExistingGrowthGeometry(original, *grown) == 0);
    auto mask = segmentation::computeLastGrowthMask(original, *grown);
    CHECK(cv::countNonZero(mask) == grown->meta["growth_report"]["added_vertices"].get_int());

    // Resuming a padded saved-style grid must perform exactly one more step.
    params["generations"] = 38;
    std::unique_ptr<QuadSurface> next(tracer(*volume, 1, 0, {20, 20, 20}, params,
        (root / "cache").string(), 1, {}, grown.get()));
    REQUIRE(next);
    CHECK(next->meta["growth_report"]["completed_steps"].get_int() == 1);
    CHECK(next->meta["growth_report"]["added_vertices"].get_int() > 0);
    CHECK(segmentation::preserveExistingGrowthGeometry(*grown, *next) == 0);

    params["generations"] = 68;
    params["x_max"] = 0;
    std::unique_ptr<QuadSurface> blocked(tracer(*volume, 1, 0, {20, 20, 20}, params,
        (root / "cache").string(), 1, {}, next.get()));
    REQUIRE(blocked);
    CHECK(blocked->meta["growth_report"]["added_vertices"].get_int() == 0);
    CHECK(blocked->meta["growth_report"]["bounds_skips"].get_int() > 0);
    CHECK(blocked->meta["growth_report"]["stop_reason"].get_string() == "frontier_exhausted");
}
