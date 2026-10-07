#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "segmentation/tools/SegmentationEditManager.hpp"
#include "ViewerManager.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include <opencv2/core.hpp>

// These tests exercise editing without a viewer or its background index.
SurfacePatchIndex* ViewerManager::surfacePatchIndex() { return nullptr; }

TEST_CASE("manual brush includes newly grown vertices and preserves the cancel baseline")
{
    cv::Mat_<cv::Vec3f> points(9,9);
    for (int r=0;r<9;++r) for (int c=0;c<9;++c) points(r,c)={100+20.f*c,100+20.f*r,100};
    const auto filled=points.clone();
    points(4,4)={-1,-1,-1};
    const auto original=points.clone();
    auto surface=std::make_shared<QuadSurface>(points,cv::Vec2f(.05f,.05f));
    SegmentationEditManager editor;
    REQUIRE(editor.beginSession(surface));
    REQUIRE(editor.setPreviewPointsOnly(filled,{{4,4}},true));
    editor.setAllowNewVertexEdits(true);
    editor.setRadius(2); editor.setSigma(1); editor.setEditScale(1);
    REQUIRE(editor.beginActiveDrag({4,4}));
    REQUIRE(editor.updateActiveDrag(filled(4,4)+cv::Vec3f(0,0,2)));
    REQUIRE(editor.previewPoints()(4,4)==filled(4,4)+cv::Vec3f(0,0,2));
    REQUIRE(editor.previewPoints()(4,5)[2]>100);
    REQUIRE(editor.previewPoints()(4,7)==filled(4,7));
    const auto edits=editor.editedVertices();
    REQUIRE(std::any_of(edits.begin(),edits.end(),[](const auto& v){return v.row==4&&v.col==4&&v.currentWorld[2]==102;}));
    editor.commitActiveDrag();
    editor.resetPreview();
    REQUIRE(cv::norm(editor.previewPoints(),original,cv::NORM_INF)==0);
}

TEST_CASE("edit smoothing preserves dragged boundaries and holes while smoothing interiors")
{
    for (bool hole : {false, true}) {
        cv::Mat_<cv::Vec3f> points(9, 9);
        for (int r = 0; r < 9; ++r)
            for (int c = 0; c < 9; ++c)
                points(r,c) = cv::Vec3f(100+20*c, 100+20*r, 100);
        if (hole) points(2,2) = cv::Vec3f(-1,-1,-1);
        auto surface = std::make_shared<QuadSurface>(points, cv::Vec2f(.05f,.05f));
        SegmentationEditManager editor;
        REQUIRE(editor.beginSession(surface));
        editor.setRadius(100);
        editor.setSigma(100);
        editor.setEditScale(1);
        REQUIRE(editor.beginActiveDrag({4,4}));
        std::vector<cv::Vec3f> targets;
        for (const auto& sample : editor.activeDrag().samples) {
            auto target = sample.baseWorld;
            target[2] += 5; // Intended movement must survive on boundaries.
            if (sample.row == 4 && sample.col == 4) target[2] += 10;
            targets.push_back(target);
        }
        REQUIRE(editor.updateActiveDragTargets(targets));
        const auto dragged = editor.previewPoints().clone();
        REQUIRE(editor.smoothRecentTouched(.4f, 20));
        const auto& result = editor.previewPoints();
        for (int r=0; r<9; ++r) for (int c=0; c<9; ++c) {
            bool boundary = r==0 || r==8 || c==0 || c==8 ||
                (hole && std::abs(r-2)<=1 && std::abs(c-2)<=1);
            if (boundary) CHECK(cv::norm(result(r,c)-dragged(r,c)) == 0);
            if (result(r,c)[0] >= 0) {
                CHECK(std::abs(result(r,c)[0]-points(r,c)[0]) < .001f);
                CHECK(std::abs(result(r,c)[1]-points(r,c)[1]) < .001f);
            }
        }
        CHECK(result(4,4)[2] < dragged(4,4)[2]);
        CHECK(result(4,4)[2] >= 105);
    }
}
