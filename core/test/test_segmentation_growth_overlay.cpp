#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "segmentation/SegmentationGrowthOverlay.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <random>

namespace
{

cv::Mat_<cv::Vec3f> validGrid(int rows, int cols, float z = 1.0f)
{
    cv::Mat_<cv::Vec3f> points(rows, cols);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            points(row, col) = cv::Vec3f(
                static_cast<float>(col), static_cast<float>(row), z);
        }
    }
    return points;
}

utils::Json gridOffset(int col, int row)
{
    auto offset = utils::Json::array();
    offset.push_back(col);
    offset.push_back(row);
    return offset;
}

} // namespace

TEST_CASE("last growth mask identifies a newly added right edge")
{
    QuadSurface before(validGrid(3, 3), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(3, 5), cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(0, 0);

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    REQUIRE(mask.rows == 3);
    REQUIRE(mask.cols == 5);
    CHECK(cv::countNonZero(mask) == 6);
    for (int row = 0; row < mask.rows; ++row) {
        CHECK(mask(row, 0) == 0);
        CHECK(mask(row, 2) == 0);
        CHECK(mask(row, 3) == 255);
        CHECK(mask(row, 4) == 255);
    }
}

TEST_CASE("last growth mask honors asymmetric grid expansion")
{
    QuadSurface before(validGrid(2, 2), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(4, 5), cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(2, 1);

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    CHECK(cv::countNonZero(mask) == 16);
    CHECK(mask(1, 2) == 0);
    CHECK(mask(1, 3) == 0);
    CHECK(mask(2, 2) == 0);
    CHECK(mask(2, 3) == 0);
    CHECK(mask(0, 0) == 255);
    CHECK(mask(3, 4) == 255);
}

TEST_CASE("last growth mask identifies newly inpainted holes")
{
    auto beforePoints = validGrid(3, 3);
    beforePoints(1, 1) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    QuadSurface before(beforePoints, cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(3, 3), cv::Vec2f(1.0f, 1.0f));

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    CHECK(cv::countNonZero(mask) == 1);
    CHECK(mask(1, 1) == 255);
}

TEST_CASE("last growth mask ignores points that remain invalid")
{
    auto beforePoints = validGrid(2, 2);
    auto afterPoints = validGrid(2, 3);
    afterPoints(0, 2) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    QuadSurface before(beforePoints, cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(afterPoints, cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(0, 0);

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    CHECK(cv::countNonZero(mask) == 1);
    CHECK(mask(0, 2) == 0);
    CHECK(mask(1, 2) == 255);
}

TEST_CASE("last growth mask rejects stale grid offset metadata")
{
    QuadSurface before(validGrid(3, 4), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(3, 4), cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(1, 0);

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    CHECK(cv::countNonZero(mask) == 0);
}

TEST_CASE("last growth mask uses generation provenance instead of resized padding")
{
    QuadSurface before(validGrid(2, 2), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(4, 4), cv::Vec2f(1.0f, 1.0f));
    cv::Mat_<uint16_t> beforeGenerations(2, 2, static_cast<uint16_t>(7));
    cv::Mat_<uint16_t> afterGenerations(4, 4, static_cast<uint16_t>(7));
    afterGenerations(3, 2) = 8;
    before.setChannel("generations", beforeGenerations);
    after.setChannel("generations", afterGenerations);
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(1, 1);

    const auto mask = segmentation::computeLastGrowthMask(before, after);
    CHECK(cv::countNonZero(mask) == 1);
    CHECK(mask(3, 2) == 255);
}

TEST_CASE("last growth cells place down growth inside the new frontier")
{
    QuadSurface surface(validGrid(4, 4), cv::Vec2f(1.0f, 1.0f));
    cv::Mat_<uint8_t> vertices(4, 4, static_cast<uint8_t>(0));
    for (int col = 0; col < vertices.cols; ++col) {
        vertices(3, col) = 255;
    }

    const auto cells = segmentation::computeLastGrowthCellMask(surface, vertices);
    CHECK(cv::countNonZero(cells) == 3);
    for (int col = 0; col < 3; ++col) {
        CHECK(cells(2, col) == 255);
        CHECK(cells(3, col) == 0);
    }
}

TEST_CASE("last growth cells do not paint unrenderable padding")
{
    auto points = validGrid(3, 3);
    points(2, 0) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    points(2, 1) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    points(2, 2) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    QuadSurface surface(points, cv::Vec2f(1.0f, 1.0f));
    cv::Mat_<uint8_t> vertices(3, 3, static_cast<uint8_t>(0));
    vertices.row(2).setTo(255);

    const auto cells = segmentation::computeLastGrowthCellMask(surface, vertices);
    CHECK(cv::countNonZero(cells) == 0);
}

TEST_CASE("growth displacement mask flags moved existing points")
{
    QuadSurface before(validGrid(3, 4), cv::Vec2f(1.0f, 1.0f));
    auto afterPoints = validGrid(3, 4);
    afterPoints(1, 2)[2] += 1.0f;
    QuadSurface after(afterPoints, cv::Vec2f(1.0f, 1.0f));

    const auto mask = segmentation::computeGrowthDisplacementMask(before, after);
    CHECK(cv::countNonZero(mask) == 1);
    CHECK(mask(1, 2) == 255);
}

TEST_CASE("growth displacement mask rejects stale grid offset metadata")
{
    QuadSurface before(validGrid(3, 4), cv::Vec2f(1.0f, 1.0f));
    auto afterPoints = validGrid(3, 4);
    afterPoints(1, 2)[2] += 1.0f;
    QuadSurface after(afterPoints, cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(1, 0);

    const auto mask = segmentation::computeGrowthDisplacementMask(before, after);
    CHECK(cv::countNonZero(mask) == 1);
    CHECK(mask(1, 2) == 255);
}

TEST_CASE("append-only growth restores every existing vertex exactly")
{
    QuadSurface before(validGrid(2, 3), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(4, 5, 100.0f), cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    after.meta["grid_offset"] = gridOffset(1, 1);

    const cv::Vec3f untouchedNewPoint = after.rawPoints()(0, 0);
    const std::size_t restored =
        segmentation::preserveExistingGrowthGeometry(before, after);

    CHECK(restored == 6);
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 3; ++col) {
            CHECK(after.rawPoints()(row + 1, col + 1) ==
                  before.rawPoints()(row, col));
        }
    }
    CHECK(after.rawPoints()(0, 0) == untouchedNewPoint);
}

TEST_CASE("append-only multi-cycle growth uses cumulative grid offset")
{
    QuadSurface before(validGrid(2, 3), cv::Vec2f(1.0f, 1.0f));
    QuadSurface after(validGrid(6, 3, 100.0f), cv::Vec2f(1.0f, 1.0f));
    after.meta = utils::Json::object();
    // Two upward grow cycles each add one row in this reduced fixture. The
    // final hand-off must align to the invocation origin, not only the last
    // parent-relative grow.
    after.meta["grid_offset"] = gridOffset(0, 2);

    const cv::Vec3f untouchedFirstGrowPoint = after.rawPoints()(0, 0);
    const cv::Vec3f untouchedSecondGrowPoint = after.rawPoints()(1, 0);
    const std::size_t restored = segmentation::preserveExistingGrowthGeometry(before, after);

    CHECK(restored == 6);
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 3; ++col) {
            CHECK(after.rawPoints()(row + 2, col) == before.rawPoints()(row, col));
        }
    }
    CHECK(after.rawPoints()(0, 0) == untouchedFirstGrowPoint);
    CHECK(after.rawPoints()(1, 0) == untouchedSecondGrowPoint);
}

TEST_CASE("last growth channel survives a tifxyz round trip")
{
    QuadSurface surface(validGrid(2, 3), cv::Vec2f(1.0f, 1.0f));
    cv::Mat_<uint8_t> mask(2, 3, static_cast<uint8_t>(0));
    mask(0, 2) = 255;
    mask(1, 2) = 255;
    surface.setChannel(segmentation::kLastGrowthChannel, mask);

    std::mt19937_64 random(std::random_device{}());
    const auto root = std::filesystem::temp_directory_path() /
        ("vc_last_growth_" + std::to_string(random()));
    surface.save(root, "last-growth-test", false);

    QuadSurface restored(root);
    const cv::Mat restoredMask = restored.channel(
        segmentation::kLastGrowthChannel, SURF_CHANNEL_NORESIZE);
    CHECK(restoredMask.type() == CV_8UC1);
    CHECK(restoredMask.rows == 2);
    CHECK(restoredMask.cols == 3);
    CHECK(cv::countNonZero(restoredMask) == 2);

    std::error_code ec;
    std::filesystem::remove_all(root, ec);
}
