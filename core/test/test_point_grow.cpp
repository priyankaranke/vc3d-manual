#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "segmentation/tools/PointGrowTool.hpp"
#include "segmentation/tools/ThinPlateSpline3d.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include "volume_viewers/SurfacePreviewFootprint.hpp"
#include <chrono>
#include <cstdlib>
#include <cstring>

namespace {
cv::Mat_<cv::Vec3f> plane(int permutation=0) {
    cv::Mat_<cv::Vec3f> p(12,12);
    for(int r=0;r<p.rows;++r) for(int c=0;c<p.cols;++c) {
        cv::Vec3f q(1000+20*c,1000+20*r,1000);
        p(r,c)={q[permutation],q[(permutation+1)%3],q[(permutation+2)%3]};
    }
    return p;
}
void originalUnchanged(const PointGrowTool& tool,const cv::Mat_<cv::Vec3f>& source) {
    const auto o=tool.offset();
    for(int r=0;r<source.rows;++r) for(int c=0;c<source.cols;++c) {
        if(source(r,c)[0]>=0)
            REQUIRE(std::memcmp(&tool.points()(r+o.y,c+o.x),&source(r,c),sizeof(cv::Vec3f))==0);
    }
}
}

TEST_CASE("CT clicks extend every chart edge in all three world orientations") {
    for(int axis=0;axis<3;++axis) for(auto d:{cv::Point(1,0),cv::Point(-1,0),cv::Point(0,1),cv::Point(0,-1)}) {
        auto p=plane(axis); PointGrowTool tool; REQUIRE(tool.begin(p));
        cv::Point edge(d.x<0?0:d.x>0?11:5,d.y<0?0:d.y>0?11:5);
        cv::Vec3f u=p(5,6)-p(5,5),v=p(6,5)-p(5,5);
        const cv::Vec3f target=p(edge)+3.f*(float(d.x)*u+float(d.y)*v);
        std::string status;
        REQUIRE_MESSAGE(tool.addPoint(target,2,status),status);
        originalUnchanged(tool,p);
        REQUIRE(tool.changedVertices().size()>1);
        REQUIRE(cv::norm(tool.anchors().back()-target)==0);
        REQUIRE(tool.undo()); REQUIRE(!tool.changed()); REQUIRE(tool.changedVertices().empty());
    }
}

TEST_CASE("continued growth expands padding and undo restores every original point") {
    auto p=plane(); PointGrowTool tool; REQUIRE(tool.begin(p)); std::string status;
    for(int i=1;i<=15;++i) {
        REQUIRE_MESSAGE(tool.addPoint(cv::Vec3f(1220+60*i,1100,1000),2,status),status);
        originalUnchanged(tool,p);
    }
    REQUIRE(tool.offset().x>16);
    for(int i=0;i<15;++i) REQUIRE(tool.undo());
    REQUIRE(tool.changedVertices().empty()); originalUnchanged(tool,p);
}

TEST_CASE("holes are filled locally and distant clicks leave the surface unchanged") {
    auto p=plane();
    for(int r=4;r<=7;++r) for(int c=4;c<=7;++c) p(r,c)={-1,-1,-1};
    p(1,1)={-1,-1,-1};
    PointGrowTool tool; REQUIRE(tool.begin(p)); std::string status;
    REQUIRE_MESSAGE(tool.addPoint({1100,1100,1002},1,status),status);
    originalUnchanged(tool,p);
    REQUIRE(tool.points()(1+tool.offset().y,1+tool.offset().x)[0]==-1);
    const auto before=tool.points().clone();
    REQUIRE_MESSAGE(tool.addPoint(p(9,9),2,status),status);
    REQUIRE(cv::norm(before,tool.points(),cv::NORM_INF)==0);
    REQUIRE_FALSE(tool.addPoint({9000,9000,9000},2,status));
    REQUIRE(cv::norm(before,tool.points(),cv::NORM_INF)==0);
}

TEST_CASE("clicks on imported geometry correct only a connected local neighbourhood") {
    for(int axis=0;axis<3;++axis) {
        auto p=plane(axis);
        p(6,6)={-1,-1,-1};
        PointGrowTool tool; REQUIRE(tool.begin(p)); std::string status;
        const cv::Point center(5,5), offset=tool.offset();
        cv::Vec3f delta(0,0,0); delta[axis==0?2:axis==1?1:0]=4;
        const auto target=p(center)+delta;
        REQUIRE_MESSAGE(tool.addPoint(target,2,status),status);
        REQUIRE(tool.points()(center+offset)==target);
        REQUIRE(tool.anchors().back()==target);
        REQUIRE(tool.changedVertices().size()>1);
        for(int r=0;r<p.rows;++r) for(int c=0;c<p.cols;++c) {
            if(cv::norm(cv::Point(c,r)-center)>=3 || p(r,c)[0]<0)
                REQUIRE(tool.points()(r+offset.y,c+offset.x)==p(r,c));
        }
        // A nearby second click must retain the first explicit constraint.
        const auto afterFirst=tool.points().clone();
        const cv::Point second(6,5);
        const auto secondTarget=tool.points()(second+offset)+delta;
        REQUIRE_MESSAGE(tool.addPoint(secondTarget,2,status),status);
        REQUIRE(tool.points()(center+offset)==target);
        REQUIRE(tool.points()(second+offset)==secondTarget);
        REQUIRE(tool.undo()); REQUIRE(cv::norm(tool.points(),afterFirst,cv::NORM_INF)==0);
        REQUIRE(tool.undo()); REQUIRE(tool.changedVertices().empty()); originalUnchanged(tool,p);
    }
}

TEST_CASE("an outward click within half a grid step adjusts the edge instead of rejecting it") {
    auto p=plane(); PointGrowTool tool; REQUIRE(tool.begin(p)); std::string status;
    const cv::Vec3f edgeTarget(1224,1100,1002);
    REQUIRE_MESSAGE(tool.addPoint(edgeTarget,2,status),status);
    REQUIRE(tool.points()(cv::Point(11,5)+tool.offset())==edgeTarget);
    // The next outward click still grows a strip and preserves the correction.
    REQUIRE_MESSAGE(tool.addPoint({1280,1100,1002},2,status),status);
    REQUIRE(tool.points()(cv::Point(11,5)+tool.offset())==edgeTarget);
    REQUIRE(tool.anchors().back()==cv::Vec3f(1280,1100,1002));
    REQUIRE(tool.undo()); REQUIRE(tool.undo()); originalUnchanged(tool,p);
}

TEST_CASE("clicked points remain exact while adding and repositioning new points") {
    auto p=plane(); PointGrowTool tool; REQUIRE(tool.begin(p)); std::string status;
    REQUIRE_MESSAGE(tool.addPoint({1280,1100,1002},2,status),status);
    REQUIRE_MESSAGE(tool.addPoint({1340,1100,1003},2,status),status);
    REQUIRE_MESSAGE(tool.addPoint({1340,1100,1006},2,status),status);
    REQUIRE(cv::norm(tool.anchors().front()-cv::Vec3f(1280,1100,1002))==0);
    REQUIRE(cv::norm(tool.anchors().back()-cv::Vec3f(1340,1100,1006))==0);
    originalUnchanged(tool,p);
}

TEST_CASE("one spline factorization still reproduces affine native coordinates") {
    ThinPlateSpline3d tps;
    std::vector<ThinPlateSpline3d::Sample> samples;
    for(int y=0;y<6;++y) for(int x=0;x<6;++x)
        samples.push_back({{double(x),double(y)},{1000+20.*x,2000+20.*y,3000+2.*x+3.*y}});
    REQUIRE(tps.fit(samples,0));
    auto value=tps.evaluate({8,7}); REQUIRE(value.has_value());
    REQUIRE(cv::norm(*value-cv::Vec3f(1160,2140,3037))<.001);
}

TEST_CASE("surface preview never interpolates invalid sentinel borders into CT") {
    auto p=plane();
    p.col(0).setTo(cv::Vec3f(-1,-1,-1));
    p(5,5)={-1,-1,-1};
    QuadSurface surface(p,{.05f,.05f});
    cv::Mat_<cv::Vec3f> coords,normals;
    const cv::Vec3f offset(-surface.center()[0],-surface.center()[1],0);
    surface.gen(&coords,&normals,{240,240},{0,0,0},1,offset);
    guardSurfacePreviewFootprint(surface,coords,normals,1,offset);
    for(int y=0;y<220;++y) for(int x=0;x<220;++x) {
        const int r=y/20,c=x/20;
        const bool supported=p(r,c)[0]>=0 && p(r,c+1)[0]>=0 && p(r+1,c)[0]>=0 && p(r+1,c+1)[0]>=0;
        REQUIRE(std::isfinite(coords(y,x)[0])==supported);
        if(supported) {
            REQUIRE(std::isfinite(normals(y,x)[0]));
            REQUIRE(coords(y,x)[0]>1000);
        }
    }
}

TEST_CASE("real imported tifxyz extends and round trips through the native reader") {
    const char* path=std::getenv("VC3D_TEST_SEGMENT");
    if(!path) return;
    QuadSurface surface(path); const auto p=surface.rawPoints().clone();
    PointGrowTool tool; REQUIRE(tool.begin(p));
    bool added=false; std::string status; cv::Vec3f target;
    const auto started=std::chrono::steady_clock::now();
    for(int r=2;r<p.rows-2&&!added;++r) for(int c=2;c<p.cols-3&&!added;++c) {
        if(p(r,c)[0]<0 || p(r,c-1)[0]<0 || p(r,c+1)[0]>=0 || p(r-1,c)[0]<0 || p(r+1,c)[0]<0) continue;
        target=p(r,c)+(p(r,c)-p(r,c-1));
        added=tool.addPoint(target,2,status);
    }
    REQUIRE_MESSAGE(added,status); originalUnchanged(tool,p);
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    std::cout<<"Real Will grid: "<<p.cols<<" x "<<p.rows<<"; extension search + fit: "<<ms<<" ms\n";
    // A disposable output is supplied only for the requested real-segment check.
    const char* output=std::getenv("VC3D_TEST_OUTPUT");
    if(output) {
        QuadSurface saved(tool.points(),surface.scale()); saved.save(output,"manual-roundtrip",true);
        QuadSurface loaded(output); loaded.ensureLoaded();
        REQUIRE(loaded.scale()==surface.scale());
        REQUIRE(cv::norm(loaded.rawPoints(),tool.points(),cv::NORM_INF)==0);
    }
}
