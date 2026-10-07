#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "segmentation/SegmentationUndoHistory.hpp"
#include "segmentation/SegmentationGrowthHistory.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include <opencv2/core.hpp>
#include <chrono>
#include <fstream>

TEST_CASE("segmentation undo history retains twenty commands")
{
    segmentation::UndoHistory history;
    for (int index = 0; index < 25; ++index) {
        cv::Mat_<cv::Vec3f> points(1, 1);
        points(0, 0) = cv::Vec3f(static_cast<float>(index), 0.0f, 0.0f);
        REQUIRE(history.capture(points));
    }

    CHECK(history.size() == 20);
    for (int expected = 24; expected >= 5; --expected) {
        const auto points = history.takeLast();
        REQUIRE(points.has_value());
        CHECK((*points)(0, 0)[0] == static_cast<float>(expected));
    }
    CHECK(history.empty());
}

TEST_CASE("growth surface snapshot is one undo transaction")
{
    segmentation::UndoHistory history;
    cv::Mat_<cv::Vec3f> points(2, 3, cv::Vec3f(1.0f, 2.0f, 3.0f));
    auto surface = std::make_shared<QuadSurface>(points, cv::Vec2f(1.0f, 1.0f));
    surface->meta = utils::Json{{"area_cm2", 1.25}};
    surface->setChannel("generations", cv::Mat(2, 3, CV_16UC1, cv::Scalar(7)));

    REQUIRE(history.captureSurface(surface));
    CHECK(history.lastIsSurface());
    const auto restored = history.takeLastSurface();
    REQUIRE(restored);
    CHECK(restored.get() == surface.get());
    CHECK(restored->rawPoints().size() == cv::Size(3, 2));
    CHECK(restored->meta.value("area_cm2", 0.0) == 1.25);
    CHECK(history.empty());
}

TEST_CASE("growth undo restores grid dimensions masks metadata and earlier edits")
{
    cv::Mat_<cv::Vec3f> points(3, 4, cv::Vec3f(10, 20, 30));
    points(1, 1) = cv::Vec3f(11, 21, 31); // manually corrected vertex
    QuadSurface surface(points, cv::Vec2f(0.05f, 0.05f));
    surface.meta = utils::Json{{"max_gen", 7}, {"area_cm2", 1.25}};
    surface.setChannel("approval", cv::Mat(3, 4, CV_8UC1, cv::Scalar(255)));
    surface.setChannel("generations", cv::Mat(3, 4, CV_16UC1, cv::Scalar(7)));
    segmentation::UndoHistory history;
    REQUIRE(history.captureDelta({{1, 1, cv::Vec3f(10, 20, 30)}}));
    REQUIRE(history.captureSurface(segmentation::snapshotSurface(surface)));
    surface.channel("approval", SURF_CHANNEL_NORESIZE).setTo(0);
    *surface.rawPointsPtr() = cv::Mat_<cv::Vec3f>(8, 9, cv::Vec3f(99, 99, 99));
    surface.setChannel("last_growth", cv::Mat(8, 9, CV_8UC1, cv::Scalar(255)));
    surface.setChannel("generations", cv::Mat(8, 9, CV_16UC1, cv::Scalar(8)));
    surface.meta["max_gen"] = 8;
    auto snapshot = history.takeLastSurface();
    REQUIRE(snapshot);
    segmentation::restoreSurface(surface, *snapshot);
    CHECK(surface.rawPoints().size() == cv::Size(4, 3));
    CHECK(surface.rawPoints()(1, 1) == cv::Vec3f(11, 21, 31));
    CHECK(surface.channel("approval", SURF_CHANNEL_NORESIZE).at<uint8_t>(1, 1) == 255);
    CHECK(surface.channel("generations", SURF_CHANNEL_NORESIZE).at<uint16_t>(1, 1) == 7);
    CHECK(surface.channel("last_growth", SURF_CHANNEL_NORESIZE).empty());
    CHECK(surface.meta.value("max_gen", 0) == 7);
    REQUIRE(history.lastIsDelta());
    auto delta = history.takeLastDelta();
    REQUIRE(delta);
    CHECK(delta->at(0).previousWorld == cv::Vec3f(10, 20, 30));
}

TEST_CASE("durable growth revisions retain each round and reopen exact editable copies")
{
    namespace fs = std::filesystem;
    const auto root = fs::temp_directory_path() / ("vc3d-history-test-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    struct Cleanup { fs::path path; ~Cleanup() { std::error_code ec; fs::remove_all(path, ec); } } cleanup{root};
    cv::Mat_<cv::Vec3f> points(3, 4, cv::Vec3f(10, 20, 30));
    points(1, 1) = cv::Vec3f(11, 21, 31);
    QuadSurface surface(points, cv::Vec2f(0.05f, 0.05f));
    surface.meta = utils::Json{{"max_gen", 7}};
    surface.setChannel("approval", cv::Mat(3, 4, CV_8UC1, cv::Scalar(255)));
    surface.setChannel("generations", cv::Mat(3, 4, CV_16UC1, cv::Scalar(7)));
    surface.save(root / "segment", "segment", false);
    std::ofstream(surface.path / "corrections.json") << "{\"anchor\":[1,1]}";
    const auto first = segmentation::saveGrowthRevision(surface, "round-1", "2026-10-02T20:00:00.001Z", "Before autogrow");
    CHECK(surface.id == "segment");
    CHECK(surface.path == root / "segment");
    CHECK_FALSE(surface.meta.contains("vc3d_revision"));
    *surface.rawPointsPtr() = cv::Mat_<cv::Vec3f>(8, 9, cv::Vec3f(99, 99, 99));
    surface.setChannel("approval", cv::Mat(8, 9, CV_8UC1, cv::Scalar(0)));
    surface.setChannel("last_growth", cv::Mat(8, 9, CV_8UC1, cv::Scalar(255)));
    surface.meta["max_gen"] = 8;
    surface.invalidateCache();
    surface.saveOverwrite(); // History must survive an atomic segment-directory replacement.
    segmentation::saveGrowthRevision(surface, "round-2", "2026-10-02T20:00:00.002Z", "Before autogrow");
    QuadSurface reopened(root / "segment");
    const auto history = segmentation::listGrowthRevisions(reopened);
    REQUIRE(history.size() >= 2);
    CHECK(history[0].path.filename() == "round-2");
    CHECK(history[1].path == first.path);
    CHECK_THROWS(segmentation::saveGrowthRevision(surface, "round-1", "duplicate", "duplicate"));
    segmentation::openGrowthRevisionCopy(first, root / "revision", "revision");
    QuadSurface restored(root / "revision");
    CHECK(restored.rawPoints().size() == cv::Size(4, 3));
    CHECK(cv::norm(restored.rawPoints(), points, cv::NORM_INF) == 0);
    CHECK(restored.scale() == cv::Vec2f(0.05f, 0.05f));
    CHECK(restored.meta.value("max_gen", 0) == 7);
    CHECK(restored.channel("approval", SURF_CHANNEL_NORESIZE).at<uint8_t>(1, 1) == 255);
    CHECK(restored.channel("generations", SURF_CHANNEL_NORESIZE).at<uint16_t>(1, 1) == 7);
    CHECK(restored.channel("last_growth", SURF_CHANNEL_NORESIZE).empty());
    CHECK(fs::exists(root / "revision" / "corrections.json"));
    CHECK(reopened.rawPoints().size() == cv::Size(9, 8));
    CHECK(fs::exists(first.path / "meta.json"));
}
