#pragma once
#include "0.h"
#include "debug.h"
#include "query.h"
#include "time_log.h"
#include  "poly_contain.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include <random>
#include <chrono>
#include <string>

using namespace std;
struct Boundary { double xMin, xMax, yMin, yMax; };
struct Vertex   { double x, y; };
struct Polygon {
    vector<Vertex> verts;
    int    n;
    double tx, ty;
    double origCx, origCy;
    double hw, hh;
    double cx() const { return origCx + tx; }
    double cy() const { return origCy + ty; }
    Polygon() : n(0), tx(0), ty(0), origCx(0), origCy(0), hw(0), hh(0) {}
};
struct SpatialHash {
    double xMin, yMin, cellW, cellH;
    int nW, nH;
    vector<vector<int>> cells;
    vector<bool> visited;
    void build(const Boundary& bd, const vector<Polygon>& P, double cellSize) {
        xMin=bd.xMin; yMin=bd.yMin; cellW=cellH=cellSize;
        nW=(int)ceil((bd.xMax-bd.xMin)/cellW)+2;
        nH=(int)ceil((bd.yMax-bd.yMin)/cellH)+2;
        cells.assign(nW*nH, vector<int>());
        visited.assign(P.size(), false);
        for (int i=0; i<(int)P.size(); i++) insert(i,P);
    }
    int gx(double x) const { int v=(int)((x-xMin)/cellW); return v<0?0:(v>=nW?nW-1:v); }
    int gy(double y) const { int v=(int)((y-yMin)/cellH); return v<0?0:(v>=nH?nH-1:v); }
    void insert(int idx, const vector<Polygon>& P) {
        int x0=gx(P[idx].cx()-P[idx].hw),x1=gx(P[idx].cx()+P[idx].hw);
        int y0=gy(P[idx].cy()-P[idx].hh),y1=gy(P[idx].cy()+P[idx].hh);
        for (int x=x0;x<=x1;x++) for (int y=y0;y<=y1;y++) cells[x*nH+y].push_back(idx);
    }
    void remove(int idx, const vector<Polygon>& P) {
        int x0=gx(P[idx].cx()-P[idx].hw),x1=gx(P[idx].cx()+P[idx].hw);
        int y0=gy(P[idx].cy()-P[idx].hh),y1=gy(P[idx].cy()+P[idx].hh);
        for (int x=x0;x<=x1;x++) for (int y=y0;y<=y1;y++) {
            auto& c=cells[x*nH+y];
            for (int k=0;k<(int)c.size();k++) if(c[k]==idx){c[k]=c.back();c.pop_back();break;}
        }
    }
    void getCandidates(int idx, const vector<Polygon>& P, vector<int>& out) {
        out.clear();
        int x0=gx(P[idx].cx()-P[idx].hw),x1=gx(P[idx].cx()+P[idx].hw);
        int y0=gy(P[idx].cy()-P[idx].hh),y1=gy(P[idx].cy()+P[idx].hh);
        for (int x=x0;x<=x1;x++) for (int y=y0;y<=y1;y++)
            for (int j:cells[x*nH+y])
                if (j!=idx&&!visited[j]) { visited[j]=true; out.push_back(j); }
        for (int j:out) visited[j]=false;
#ifdef DEBUG_LOG
        g_cntGetCandidates++;
        g_sumCands += (long long)out.size();
#endif
    }
    void rebuild(const Boundary& bd, const vector<Polygon>& P, double cellSize) { build(bd,P,cellSize); }
};
inline nfp::NFPMatrix g_nfps;
inline SpatialHash g_hash;
inline  geom:: ContainPolys g_contains;
inline chrono::steady_clock::time_point g_start;
// inline mt19937 g_rng(42);
inline mt19937 g_rng(std::random_device{}());
inline double elapsed() {
    return chrono::duration<double>(
        chrono::steady_clock::now() - g_start).count();
}
inline double randDouble() {
    static uniform_real_distribution<double> d(0.0, 1.0);
    return d(g_rng);
}
inline int randInt(int n) {
    uniform_int_distribution<int> d(0, n - 1);
    return d(g_rng);
}
inline double cross(const Vertex& o, const Vertex& a, const Vertex& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
static vector<Vertex> expandPolygon(const vector<Vertex>& ply, double expend = EXPAND_EPS) {
    int n = (int)ply.size();
    if (n < 3) return ply;
    const double MITER_LIMIT = 4.0, EPS = 1e-9, SNAP_GRID = 1e5;
    vector<pair<double,double>> normals(n);
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double ex = ply[j].x - ply[i].x, ey = ply[j].y - ply[i].y;
        double len = sqrt(ex*ex + ey*ey);
        if (len < EPS) normals[i] = {0.0, 0.0};
        else           normals[i] = {ey/len, -ex/len};
    }
    vector<Vertex> result; result.reserve(n*2);
    for (int i = 0; i < n; i++) {
        int    prev = (i == 0) ? n-1 : i-1;
        double n1x  = normals[prev].first, n1y = normals[prev].second;
        double n2x  = normals[i].first,    n2y = normals[i].second;
        double dx = n1x+n2x, dy = n1y+n2y, dlen = sqrt(dx*dx+dy*dy);
        if (dlen < EPS) {
            result.push_back({ply[i].x+n1x*expend, ply[i].y+n1y*expend});
            result.push_back({ply[i].x+n2x*expend, ply[i].y+n2y*expend}); continue;
        }
        dx /= dlen; dy /= dlen;
        double sinHalf = dlen/2.0;
        if (fabs(sinHalf) < EPS) {
            result.push_back({ply[i].x+n1x*expend, ply[i].y+n1y*expend});
            result.push_back({ply[i].x+n2x*expend, ply[i].y+n2y*expend}); continue;
        }
        double dist = expend/sinHalf, maxDist = expend*MITER_LIMIT;
        if (dist > maxDist) {
            result.push_back({ply[i].x+n1x*expend, ply[i].y+n1y*expend});
            result.push_back({ply[i].x+n2x*expend, ply[i].y+n2y*expend});
        } else {
            result.push_back({ply[i].x+dx*dist, ply[i].y+dy*dist});
        }
    }
    for (auto& v : result) {
        v.x = round(v.x*SNAP_GRID)/SNAP_GRID;
        v.y = round(v.y*SNAP_GRID)/SNAP_GRID;
    }
    vector<Vertex> cleaned; cleaned.reserve(result.size());
    for (int i = 0; i < (int)result.size(); i++) {
        int j = (i+1)%(int)result.size();
        double ddx = result[j].x-result[i].x, ddy = result[j].y-result[i].y;
        if (sqrt(ddx*ddx+ddy*ddy) >= EPS) cleaned.push_back(result[i]);
    }
    return cleaned.empty() ? result : cleaned;
}
static vector<Vertex> simplify(const vector<Vertex> &ply) {
  return vector<Vertex>{};
}
static vector<Vertex> convexHull(const vector<Vertex> &ply) {
    int n = (int)ply.size();
    if (n <= 3) return ply;
    vector<Vertex> pts = ply;
    sort(pts.begin(), pts.end(), [](const Vertex& a, const Vertex& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    vector<Vertex> lower;
    for (int i = 0; i < n; i++) {
        while (lower.size() >= 2 && cross(lower[lower.size()-2], lower[lower.size()-1], pts[i]) <= 0)
            lower.pop_back();
        lower.push_back(pts[i]);
    }
    vector<Vertex> upper;
    for (int i = n - 1; i >= 0; i--) {
        while (upper.size() >= 2 && cross(upper[upper.size()-2], upper[upper.size()-1], pts[i]) <= 0)
            upper.pop_back();
        upper.push_back(pts[i]);
    }
    lower.insert(lower.end(), upper.begin() + 1, upper.end() - 1);
    return lower;
}
static double calcLPrime(const vector<Polygon>& P) {
    double s = 0.0;
    for (auto& p : P) s += sqrt(p.tx*p.tx + p.ty*p.ty);
    return s;
}
static bool aabbOverlap(const Polygon& a, const Polygon& b) {
    return fabs(a.cx()-b.cx()) < a.hw+b.hw && fabs(a.cy()-b.cy()) < a.hh+b.hh;
}
static bool aabbInBounds(const Polygon& p, const Boundary& bd) {
    return p.cx()-p.hw >= bd.xMin && p.cx()+p.hw <= bd.xMax
        && p.cy()-p.hh >= bd.yMin && p.cy()+p.hh <= bd.yMax;
}
static double aabbOutArea(const Polygon& p, const Boundary& bd) {
    double l = fmax(p.cx()-p.hw,bd.xMin), r = fmin(p.cx()+p.hw,bd.xMax);
    double lo= fmax(p.cy()-p.hh,bd.yMin), hi= fmin(p.cy()+p.hh,bd.yMax);
    if (l >= r || lo >= hi) return 4.0*p.hw*p.hh;
    return fmax(4.0*p.hw*p.hh-(r-l)*(hi-lo), 0.0);
}
inline bool isFeasible(int i, const Vertex& a){
    return g_contains.contain(i, geom::Point(a.x, a.y));
}
static bool exactInBounds(const Polygon& p, const Boundary& bd) {
  const double EPS = 1e-9;
    double cx = p.cx(), cy = p.cy();
    if (cx - p.hw - bd.xMin >=  + EPS &&        cx + p.hw - bd.xMax <=  - EPS &&        cy - p.hh - bd.yMin >=  + EPS &&        cy + p.hh -  bd.yMax <=  - EPS) {        return true;    } else {
      return false;
    }
}
static bool exactInBounds(int i, const Polygon& p, const Boundary& bd) {
    if (!exactInBounds(p, bd)) return false;
    return isFeasible(i, Vertex{p.tx, p.ty});
}
static bool fastOverlap(int i, int j, const Polygon &a, const Polygon &b,
                        Vertex &mtvB, bool &mtvValid, bool &isolated) {
    g_cntFastOverlap++;
    nfp::Point nfpMTV;
    bool result = g_nfps.overlap(i, j, nfp::Point(a.tx, a.ty),
                                 nfp::Point(b.tx, b.ty), nfpMTV, mtvValid, isolated);
    mtvB.x = nfpMTV.x ; mtvB.y = nfpMTV.y ;
    return result;
}
static bool fastOverlap(int i, int j, const Polygon &a, const Polygon &b,
                        bool &isolated) {
    g_cntFastOverlap++;
    bool result = g_nfps.overlap(i, j, nfp::Point(a.tx, a.ty),
                                 nfp::Point(b.tx, b.ty), isolated);
    return result;
}
static bool fastOverlap(int i, int j, const Polygon &a, const Polygon &b,
                        Vertex &mtvB, bool &mtvValid) {
    bool isolated;
    return fastOverlap(i, j, a, b, mtvB, mtvValid, isolated);
}
static bool fastOverlap(int i, int j, const Polygon &a, const Polygon &b) {
    bool isolated;
    return fastOverlap(i, j, a, b, isolated);
}
static void buildNFPMatrix(const vector<Polygon>& P) {
    int n = (int)P.size();
    vector<geom::Polygon> basePolys(n);
    for (int i = 0; i < n; i++) {
        basePolys[i].outer.vertices.resize(P[i].n);
        for (int j = 0; j < P[i].n; j++) {
            basePolys[i].outer.vertices[j].x = P[i].verts[j].x;
            basePolys[i].outer.vertices[j].y = P[i].verts[j].y;
        }
        basePolys[i].init();
    }
    g_nfps.build(basePolys);
}
static void buildContainPolys(const Polygon& feasiblePoly, const vector<Polygon>& P) {
    geom::UnorderedPolygon geomFeasible;
    geomFeasible.segs.reserve(feasiblePoly.n);
    for (int i = 0; i < feasiblePoly.n; ++i) {
        int j = (i + 1) % feasiblePoly.n;
        geomFeasible.segs.push_back(geom::Segment(
            geom::Point(feasiblePoly.verts[i].x, feasiblePoly.verts[i].y),
            geom::Point(feasiblePoly.verts[j].x, feasiblePoly.verts[j].y)
        ));
    }
    geomFeasible.init();
    vector<geom::UnorderedPolygon> geomPolys(P.size());
    for (size_t k = 0; k < P.size(); ++k) {
        const auto& poly = P[k];
        geomPolys[k].segs.reserve(poly.n);
        for (int i = 0; i < poly.n; ++i) {
            int j = (i + 1) % poly.n;
            geomPolys[k].segs.push_back(geom::Segment(
                geom::Point(poly.verts[i].x, poly.verts[i].y),
                geom::Point(poly.verts[j].x, poly.verts[j].y)
            ));
        }
        geomPolys[k].init();
    }
    g_contains.build(geomFeasible, geomPolys);
}
static double buildCellSize(const vector<Polygon>& P) {
    double avgSz=0.0;
    for (auto& p:P) avgSz+=2.0*fmax(p.hw,p.hh);
    avgSz/=(int)P.size();
    return fmax(avgSz*0.6, 0.05);
}
static double singleDist(const Polygon& p) { return sqrt(p.tx*p.tx+p.ty*p.ty); }
static double calcOverlapPenalty(const vector<Polygon>& P, int n) {
    double totalMtv = 0.0;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            Vertex mtv; bool mtvValid;
            if (fastOverlap(i, j, P[i], P[j], mtv, mtvValid)) {
                if (mtvValid) {
                    totalMtv += sqrt(mtv.x * mtv.x + mtv.y * mtv.y);
                } else {
                    double diagA = sqrt(P[i].hw * P[i].hw + P[i].hh * P[i].hh);
                    double diagB = sqrt(P[j].hw * P[j].hw + P[j].hh * P[j].hh);
                    totalMtv += diagA + diagB;
                }
            }
        }
    }
    return totalMtv;
}
static double calcSingleOldPenalty(const vector<Polygon>& P, int idx,
                                   const vector<int>& candidates, double& maxMTV) {
    double oldPenalty = 0.0;
    maxMTV = 0.0;
    for (int j : candidates) {
        Vertex mtv; bool mtvValid;
        if (fastOverlap(idx, j, P[idx], P[j], mtv, mtvValid)) {
            double mtvLen = 0.0;
            if (mtvValid) {
                mtvLen = sqrt(mtv.x * mtv.x + mtv.y * mtv.y);
            } else {
                double diagA = sqrt(P[idx].hw * P[idx].hw + P[idx].hh * P[idx].hh);
                double diagB = sqrt(P[j].hw * P[j].hw + P[j].hh * P[j].hh);
                mtvLen = diagA + diagB;
            }
            oldPenalty += mtvLen;
            if (mtvLen > maxMTV) maxMTV = mtvLen;
        }
    }
    return oldPenalty;
}