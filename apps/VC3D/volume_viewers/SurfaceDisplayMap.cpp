#include "SurfaceDisplayMap.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
bool valid(const cv::Vec3f& p) {
    return p[0]>=0 && p[1]>=0 && p[2]>=0 &&
        std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}
float cross(cv::Vec2f a, cv::Vec2f b) { return a[0]*b[1]-a[1]*b[0]; }
bool weights(cv::Vec2f p, cv::Vec2f a, cv::Vec2f b, cv::Vec2f c, cv::Vec3f& w) {
    const float det=cross(b-a,c-a);
    if (!(det>1e-8f)) return false;
    w[1]=cross(p-a,c-a)/det; w[2]=cross(b-a,p-a)/det; w[0]=1-w[1]-w[2];
    return w[0]>=-1e-5f && w[1]>=-1e-5f && w[2]>=-1e-5f;
}
}

cv::Mat_<cv::Vec2f> surface_display::readOffsets(QuadSurface& surface) {
    auto u=surface.channel(uChannel,SURF_CHANNEL_NORESIZE);
    auto v=surface.channel(vChannel,SURF_CHANNEL_NORESIZE);
    if (u.type()!=CV_32FC1 || v.type()!=CV_32FC1 ||
        u.size()!=surface.rawPoints().size() || u.size()!=v.size()) return {};
    cv::Mat_<cv::Vec2f> result;
    cv::merge(std::vector<cv::Mat>{u,v},result);
    if (!cv::checkRange(result)) return {};
    return result;
}

void surface_display::writeOffsets(QuadSurface& surface, const cv::Mat_<cv::Vec2f>& offsets) {
    if (offsets.empty()) return;
    std::vector<cv::Mat> channels;
    cv::split(offsets,channels);
    surface.setChannel(uChannel,channels[0]);
    surface.setChannel(vChannel,channels[1]);
}

cv::Rect surface_display::relax(const cv::Mat_<cv::Vec3f>& points,
                               cv::Mat_<cv::Vec2f>& offsets,
                               cv::Vec2f scale, cv::Rect changed) {
    if (changed.empty() || points.size()!=offsets.size() || scale[0]<=0 || scale[1]<=0) return {};
    constexpr int halo=8;
    const auto region=cv::Rect(changed.x-halo,changed.y-halo,
        changed.width+2*halo,changed.height+2*halo) & cv::Rect(0,0,points.cols,points.rows);
    if (region.width<3 || region.height<3) return {};
    cv::Mat_<cv::Vec2f> uv(region.size()), start(region.size());
    for (int r=0;r<region.height;++r) for(int c=0;c<region.width;++c)
        uv(r,c)=start(r,c)=cv::Vec2f(c/scale[0],r/scale[1])+offsets(r+region.y,c+region.x);

    // Precompute all eight spring lengths. Diagonals constrain shear, which a
    // row/column-only spacing correction cannot remove. Empty cells provide a
    // weak continuation of the chart, not invented papyrus geometry.
    const std::array<cv::Point,8> neighbours={cv::Point(-1,-1),cv::Point(0,-1),
        cv::Point(1,-1),cv::Point(-1,0),cv::Point(1,0),cv::Point(-1,1),cv::Point(0,1),cv::Point(1,1)};
    struct Springs { std::array<float,8> length{}, weight{}; };
    std::vector<Springs> springs(region.area());
    for(int r=1;r<region.height-1;++r) for(int c=1;c<region.width-1;++c) {
        auto& s=springs[r*region.width+c];
        const auto p=points(r+region.y,c+region.x);
        for(int k=0;k<8;++k) {
            const auto d=neighbours[k]; const auto q=points(r+region.y+d.y,c+region.x+d.x);
            const bool real=valid(p)&&valid(q);
            s.length[k]=real ? float(cv::norm(p-q)) : float(cv::norm(start(r,c)-start(r+d.y,c+d.x)));
            s.weight[k]=real ? 1.0f : 0.04f;
        }
    }
    // Fixed work bound on the click path; no global solve, disk or CT sampling.
    for(int iteration=0;iteration<80;++iteration) {
        float movement=0;
        for(int rr=1;rr<region.height-1;++rr) for(int cc=1;cc<region.width-1;++cc) {
            const int r=iteration%2 ? region.height-1-rr : rr;
            const int c=iteration%2 ? region.width-1-cc : cc;
            const auto& springsHere=springs[r*region.width+c];
            cv::Vec2f sum(0,0); float weight=0;
            for(int k=0;k<8;++k) {
                const auto d=neighbours[k]; const auto q=uv(r+d.y,c+d.x);
                auto direction=uv(r,c)-q; const float length=float(cv::norm(direction));
                if (length<1e-6f) continue;
                const float w=springsHere.weight[k];
                sum+=w*(q+direction*(springsHere.length[k]/length)); weight+=w;
            }
            if (weight==0) continue;
            const auto next=sum/weight;
            movement=std::max(movement,float(cv::norm(next-uv(r,c))));
            uv(r,c)=next;
        }
        if (movement<0.0005f) break;
    }
    // A flattening must not introduce folds, even in empty chart cells used
    // for picking. Backtrack the entire update instead of dropping geometry.
    float alpha=1;
    for (;alpha>=1.0f/1024;alpha*=0.5f) {
        bool good=true;
        auto at=[&](int r,int c){return start(r,c)+alpha*(uv(r,c)-start(r,c));};
        for(int r=0;r<region.height-1 && good;++r) for(int c=0;c<region.width-1;++c) {
            const auto a=at(r,c),b=at(r,c+1),d=at(r+1,c),e=at(r+1,c+1);
            const float before0=cross(start(r,c+1)-start(r,c),start(r+1,c+1)-start(r,c));
            const float before1=cross(start(r+1,c+1)-start(r,c),start(r+1,c)-start(r,c));
            if (cross(b-a,e-a)<0.1f*before0 || cross(e-a,d-a)<0.1f*before1) {good=false;break;}
        }
        if(good) break;
    }
    if(alpha<1.0f/1024) return {};
    for(int r=1;r<region.height-1;++r) for(int c=1;c<region.width-1;++c)
        offsets(r+region.y,c+region.x)+=alpha*(uv(r,c)-start(r,c));
    return region;
}

std::shared_ptr<const SurfaceDisplayMap> SurfaceDisplayMap::fromSurface(QuadSurface& surface) {
    auto offsets=surface_display::readOffsets(surface);
    if(offsets.empty() || cv::norm(offsets,cv::NORM_INF)<1e-5) return {};
    return std::make_shared<SurfaceDisplayMap>(surface.rawPoints(),offsets,surface.scale(),surface.center());
}

SurfaceDisplayMap::SurfaceDisplayMap(const cv::Mat_<cv::Vec3f>& points,
    const cv::Mat_<cv::Vec2f>& offsets,cv::Vec2f scale,cv::Vec3f center)
    : _points(points.clone()),_offsets(offsets.clone()),_scale(scale),_center(center[0],center[1]) {
    for(const auto& d:_offsets) for(int k=0;k<2;++k) _maxOffset[k]=std::max(_maxOffset[k],std::abs(d[k]));
}
cv::Vec2f SurfaceDisplayMap::toGrid(cv::Vec2f p) const {return {(p[0]+_center[0])*_scale[0],(p[1]+_center[1])*_scale[1]};}
cv::Vec2f SurfaceDisplayMap::fromGrid(cv::Vec2f p) const {return {p[0]/_scale[0]-_center[0],p[1]/_scale[1]-_center[1]};}
cv::Vec2f SurfaceDisplayMap::vertex(int r,int c) const {return fromGrid({float(c),float(r)})+_offsets(r,c);}

cv::Vec2f SurfaceDisplayMap::forward(cv::Vec2f p) const {
    auto g=toGrid(p); int c=int(std::floor(g[0])),r=int(std::floor(g[1]));
    if(c<0||r<0||c+1>=_points.cols||r+1>=_points.rows) return p;
    const float u=g[0]-c,v=g[1]-r;
    const auto a=_offsets(r,c),b=_offsets(r,c+1),d=_offsets(r+1,c),e=_offsets(r+1,c+1);
    return p+(u>=v ? (1-u)*a+(u-v)*b+v*e : (1-v)*a+u*e+(v-u)*d);
}
cv::Vec2f SurfaceDisplayMap::inverse(cv::Vec2f p) const {
    const auto g=toGrid(p);
    const int rx=int(std::ceil(_maxOffset[0]*_scale[0]))+1,ry=int(std::ceil(_maxOffset[1]*_scale[1]))+1;
    const int c0=std::max(0,int(std::floor(g[0]))-rx),c1=std::min(_points.cols-2,int(std::floor(g[0]))+rx);
    const int r0=std::max(0,int(std::floor(g[1]))-ry),r1=std::min(_points.rows-2,int(std::floor(g[1]))+ry);
    for(int r=r0;r<=r1;++r) for(int c=c0;c<=c1;++c) {
        cv::Vec3f w;
        const auto a=vertex(r,c),b=vertex(r,c+1),d=vertex(r+1,c),e=vertex(r+1,c+1);
        if(weights(p,a,b,e,w)) return fromGrid({c+w[1]+w[2],r+w[2]});
        if(weights(p,a,e,d,w)) return fromGrid({c+w[1],r+w[1]+w[2]});
    }
    return p;
}

cv::Mat_<cv::Vec2f> SurfaceDisplayMap::gridMap(cv::Size pixels,cv::Vec2f nominalCenter,float zoom) const {
    cv::Mat_<cv::Vec2f> map(pixels,cv::Vec2f(-1,-1));
    const auto center=forward(nominalCenter);
    const cv::Vec2f origin=center-cv::Vec2f(pixels.width/(2*zoom),pixels.height/(2*zoom));
    const auto lo=toGrid(origin-_maxOffset),hi=toGrid(origin+cv::Vec2f(pixels.width/zoom,pixels.height/zoom)+_maxOffset);
    const int r0=std::max(0,int(std::floor(lo[1]))),c0=std::max(0,int(std::floor(lo[0])));
    const int r1=std::min(_points.rows-2,int(std::ceil(hi[1]))),c1=std::min(_points.cols-2,int(std::ceil(hi[0])));
    auto raster=[&](cv::Vec2f a,cv::Vec2f b,cv::Vec2f c,cv::Vec2f ga,cv::Vec2f gb,cv::Vec2f gc) {
        a=(a-origin)*zoom;b=(b-origin)*zoom;c=(c-origin)*zoom;
        const int x0=std::max(0,int(std::ceil(std::min({a[0],b[0],c[0]}))));
        const int y0=std::max(0,int(std::ceil(std::min({a[1],b[1],c[1]}))));
        const int x1=std::min(pixels.width-1,int(std::floor(std::max({a[0],b[0],c[0]}))));
        const int y1=std::min(pixels.height-1,int(std::floor(std::max({a[1],b[1],c[1]}))));
        for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
            cv::Vec3f w;
            if(weights({float(x),float(y)},a,b,c,w)) map(y,x)=w[0]*ga+w[1]*gb+w[2]*gc;
        }
    };
    for(int r=r0;r<=r1;++r) for(int c=c0;c<=c1;++c) {
        const auto a=vertex(r,c),b=vertex(r,c+1),d=vertex(r+1,c),e=vertex(r+1,c+1);
        const cv::Vec2f ga(c,r),gb(c+1,r),gd(c,r+1),ge(c+1,r+1);
        raster(a,b,e,ga,gb,ge);raster(a,e,d,ga,ge,gd);
    }
    return map;
}

void SurfaceDisplayMap::gen(cv::Mat_<cv::Vec3f>& coords,cv::Mat_<cv::Vec3f>* normals,
    cv::Size pixels,cv::Vec2f center,float zoom) const {
    const auto map=gridMap(pixels,center,zoom);
    const float nan=std::numeric_limits<float>::quiet_NaN();
    coords=cv::Mat_<cv::Vec3f>(pixels,cv::Vec3f(nan,nan,nan));
    if(normals) *normals=cv::Mat_<cv::Vec3f>(pixels,cv::Vec3f(nan,nan,nan));
    for(int y=0;y<pixels.height;++y) for(int x=0;x<pixels.width;++x) {
        const auto g=map(y,x); const int c=int(std::floor(g[0])),r=int(std::floor(g[1]));
        if(c<0||r<0||c+1>=_points.cols||r+1>=_points.rows) continue;
        const auto a=_points(r,c),b=_points(r,c+1),d=_points(r+1,c),e=_points(r+1,c+1);
        if(!valid(a)||!valid(b)||!valid(d)||!valid(e)) continue;
        const float u=g[0]-c,v=g[1]-r;
        coords(y,x)=(1-v)*((1-u)*a+u*b)+v*((1-u)*d+u*e);
        if(normals) {
            const auto du=(1-v)*(b-a)+v*(e-d),dv=(1-u)*(d-a)+u*(e-b);
            auto n=du.cross(dv); const float length=float(cv::norm(n));
            if(length>1e-8) (*normals)(y,x)=n/length;
        }
    }
}
