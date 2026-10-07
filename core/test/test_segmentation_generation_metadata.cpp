#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "segmentation/SegmentationGenerationMetadata.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <opencv2/core.hpp>

namespace
{

cv::Mat_<cv::Vec3f> validGrid(int rows, int cols)
{
    cv::Mat_<cv::Vec3f> points(rows, cols);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            points(row, col) = cv::Vec3f(
                static_cast<float>(col), static_cast<float>(row), 1.0f);
        }
    }
    return points;
}

} // namespace

TEST_CASE("legacy surface seeds missing generations from metadata")
{
    auto points = validGrid(2, 3);
    points(1, 2) = cv::Vec3f(-1.0f, -1.0f, -1.0f);
    QuadSurface surface(points, cv::Vec2f(1.0f, 1.0f));
    surface.meta = utils::Json::object();
    surface.meta["max_gen"] = 200;

    const auto current = segmentation::ensureGenerationChannel(&surface);
    REQUIRE(current.has_value());
    CHECK(*current == 200);

    const cv::Mat generations = surface.channel(
        "generations", SURF_CHANNEL_NORESIZE);
    REQUIRE(generations.type() == CV_16UC1);
    REQUIRE(generations.rows == 2);
    REQUIRE(generations.cols == 3);
    CHECK(generations.at<uint16_t>(0, 0) == 200);
    CHECK(generations.at<uint16_t>(1, 1) == 200);
    CHECK(generations.at<uint16_t>(1, 2) == 0);
    CHECK(surface.meta["max_gen"].get_int() == 200);
}

TEST_CASE("surface without generation metadata starts at one")
{
    QuadSurface surface(validGrid(2, 2), cv::Vec2f(1.0f, 1.0f));

    const auto current = segmentation::ensureGenerationChannel(&surface);
    REQUIRE(current.has_value());
    CHECK(*current == 1);

    const cv::Mat generations = surface.channel(
        "generations", SURF_CHANNEL_NORESIZE);
    CHECK(cv::countNonZero(generations != 1) == 0);
    CHECK(surface.meta["max_gen"].get_int() == 1);
}

TEST_CASE("existing generation channel overrides stale metadata")
{
    QuadSurface surface(validGrid(2, 2), cv::Vec2f(1.0f, 1.0f));
    cv::Mat_<uint16_t> generations(2, 2, static_cast<uint16_t>(60));
    surface.setChannel("generations", generations);
    surface.meta = utils::Json::object();
    surface.meta["max_gen"] = 200;

    const auto current = segmentation::ensureGenerationChannel(&surface);
    REQUIRE(current.has_value());
    CHECK(*current == 60);
    CHECK(surface.meta["max_gen"].get_int() == 60);
}

TEST_CASE("GUI generation window overrides hidden JSON start and stop values")
{
    utils::Json params = utils::Json::object();
    params["rewind_gen"] = 1;
    params["grow_steps"] = 500;
    params["generations"] = 999;

    const int target = segmentation::applyGuiGenerationWindow(params, 200, 1);

    CHECK(target == 201);
    CHECK(params["rewind_gen"].get_int() == -1);
    CHECK(params["grow_steps"].get_int() == 1);
    CHECK(params["generations"].get_int() == 201);
}
