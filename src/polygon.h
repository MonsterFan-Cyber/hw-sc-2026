#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <cstdint>

namespace geom {
typedef double Scalar;
const Scalar PI = 3.1415926535897932384626433832795;
const Scalar PI_PI = 2.0 * PI;
const Scalar EPS_ANGLE = std::numeric_limits<Scalar>::epsilon() * PI_PI;
const Scalar EPS_ANGLE_RELAX = 1e2 * EPS_ANGLE;
const Scalar EPS_DISTANCE = 1e-9;
const int64_t SCALE_SNAP_GRID = static_cast<int64_t>(1e5); 
const Scalar EPS_SNAP_GRID = 1.0/SCALE_SNAP_GRID;
inline Scalar toAngle(Scalar a)
{
    while (a < 0)
        a += PI_PI;
    while (a >= PI_PI)
        a -= PI_PI;
    return a;
}
inline Scalar toSignedAngle(Scalar a)
{
    while (a < -PI) a += PI_PI;
    while (a >= PI) a -= PI_PI;
    return a;
}
inline bool eqDist(Scalar a, Scalar b) {   return std::abs(a - b) < EPS_DISTANCE; }
inline bool ltDist(Scalar a, Scalar b) {   return a -b <  -EPS_DISTANCE; }
inline bool gtDist(Scalar a, Scalar b) {   return a -b >  +EPS_DISTANCE; }
inline bool leDist(Scalar a, Scalar b) {   return a -b <  +EPS_DISTANCE; }
inline bool geDist(Scalar a, Scalar b) {   return a -b >  -EPS_DISTANCE; }
struct Point
{
    Point() : x(0), y(0) {}
    Point(Scalar x, Scalar y) : x(x), y(y) {}
    Scalar x, y;
    Point operator+(const Point& other) const { return {x + other.x, y + other.y}; }
    Point operator-(const Point& other) const { return {x - other.x, y - other.y}; }
    Point operator*(Scalar k) const { return {x * k, y * k}; }
    Point operator/(Scalar k) const { return {x / k, y / k}; }
    Scalar dot(const Point& other) const { return x * other.x + y * other.y; }
    Scalar cross(const Point& other) const { return x * other.y - y * other.x; }
    Scalar norm2() const { return x * x + y * y; }
    Scalar norm() const { return std::sqrt(x * x + y * y); }
    Point rotateCW90() const { return {y, -x}; }
    Point rotateCCW90() const { return {-y, x}; }
    bool isEqual(const Point& other, Scalar eps = EPS_DISTANCE) const
    {      return (*this - other).norm() < eps;  }
    bool isPerpendicular(const Point& other, Scalar eps = EPS_ANGLE) const
    {      return std::abs(dot(other)) < eps;  }
    bool isParallel(const Point& other, Scalar eps = EPS_ANGLE) const
    {     return std::abs(cross(other)) < eps;  }
    Scalar angle() const
    {    Scalar a = std::atan2(y, x);
        if (a < 0) a += PI_PI;
        return a;   }
    Scalar angleTo(const Point& other) const
    {   Scalar a1 = angle();
        Scalar a2 = other.angle();
        Scalar diff = a2 - a1;
        if (diff < -PI) diff += PI_PI;
        if (diff >= PI) diff -= PI_PI;
        return diff;
    }
};
struct Segment
{
    Point from;
    Point to;
    Segment() : from(0, 0), to(0, 0) {}
    Segment(const Point& from, const Point& to) : from(from), to(to) {}
};
bool pointOnSegment(const Point& p, const Point& a, const Point& b, Scalar eps = EPS_DISTANCE) {
    Point ab = b - a;
    Point ap = p - a;
    if (std::abs(ab.cross(ap)) > eps) return false;
    Scalar ab2 = ab.norm2();
    if (ab2 <= eps * eps) return (p - a).norm() <= eps;
    Scalar t = ap.dot(ab) / ab2;
    return t > -eps && 1.0 - t > -eps;
};
struct Box
{
    Point min;
    Point max;
    Box() : min{0,0}, max{0,0} {}
    Box(const Point& P1, const Point& P2) {
        min = {std::min(P1.x, P2.x), std::min(P1.y, P2.y)};
        max = {std::max(P1.x, P2.x), std::max(P1.y, P2.y)};
    }
    Box(Scalar xmin, Scalar xmax, Scalar ymin, Scalar ymax): min(xmin, ymin), max(xmax, ymax) {}
    void clear() { min = {0, 0}; max = {0, 0}; }
    void setINF() {
        min = {std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max()};
        max = {-std::numeric_limits<Scalar>::max(), -std::numeric_limits<Scalar>::max()};
    }
    void expand(const Point& p) {
        if (p.x < min.x) min.x = p.x;
        if (p.y < min.y) min.y = p.y;
        if (p.x > max.x) max.x = p.x;
        if (p.y > max.y) max.y = p.y;
    }
    bool contains(const Point &p,  bool inclusive = false , Scalar eps = EPS_DISTANCE) const {
        const Scalar epsIn = inclusive ? -eps : eps;
        return p.x > min.x + epsIn && max.x  >  p.x + epsIn
            && p.y > min.y + epsIn && max.y  >  p.y + epsIn;
    }
};
bool boxesOverlap(const Box& b1, const Box& b2, Scalar eps = EPS_DISTANCE) {
    return !(b1.max.x - b2.min.x < -eps || b2.max.x - b1.min.x < -eps
            || b1.max.y - b2.min.y < -eps || b2.max.y - b1.min.y < -eps);
};
inline Scalar calcSignedArea(const std::vector<Point> &ring){
    Scalar area = 0;
    size_t n = ring.size();
    for (size_t i = 0; i < n; ++i) {
        const Point &a = ring[i];
        const Point &b = ring[(i + 1) % n];
        area += a.x * b.y - a.y * b.x;
    }
    return area * 0.5;
}
template <typename T>
struct XGrid
{
    Scalar cellSizeX, xMin, xMax;    
    int numCellsX;
    std::vector<std::vector<T>> cells;
    void clear() {
        cells.clear();
        numCellsX = 0;
        cellSizeX = xMin = xMax = 0.0;
    }
    int getIndex(Scalar x) const {
        int idx = static_cast<int>(std::floor((x - xMin) / cellSizeX));
        if (idx < 0) idx = 0;
        if (idx >= numCellsX) idx = numCellsX - 1;
        return idx;
    }
};
struct Ring {
    std::vector<Point> vertices;
    Scalar calcSignedArea()const{
        return geom::calcSignedArea(vertices);
    }
    void clear() { vertices.clear(); }
    bool empty() const { return vertices.empty(); }
    void getSumLength(Scalar & xSum, Scalar &ySum) const
    {
        xSum = 0;
        ySum = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Point& a = vertices[i];
            const Point& b = vertices[(i + 1) % vertices.size()];
            xSum += std::abs(b.x - a.x);
            ySum += std::abs(b.y - a.y);
        }
    }
};
bool isEdgesIntersected( const Point&a, const Point&b,const Ring &r1, const Ring &r2) {
  int n1 = static_cast<int>(r1.vertices.size());
  int n2 = static_cast<int>(r2.vertices.size());
  if (n1 < 2 || n2 < 2) return false;
  for (int i = 0; i < n1; ++i) {
    const Point& p1 = r1.vertices[i];
    const Point& p2 = r1.vertices[(i + 1) % n1];
    Point p1Shift = p1 + a;
    Point p2Shift = p2 + a;
    Point r = p2Shift - p1Shift;
    for (int j = 0; j < n2; ++j) {
      const Point& q1 = r2.vertices[j];
      const Point& q2 = r2.vertices[(j + 1) % n2];
      Point q1Shift = q1 + b;
      Point q2Shift = q2 + b;
      Point s = q2Shift - q1Shift;
      Point qp = q1Shift - p1Shift;
      Scalar denom = r.cross(s);
      if (std::abs(denom) > EPS_DISTANCE) {
        Scalar t = qp.cross(s) / denom;
        Scalar u = qp.cross(r) / denom;
        if (t > EPS_DISTANCE && t < 1.0 - EPS_DISTANCE 
            && u > EPS_DISTANCE && u < 1.0 - EPS_DISTANCE) {
          return true;
        }
      }
    }
  }
  return false;
}
struct EdgeIndex{
    int ringIndex;
    int edgeIndex;
};
template <typename SegType>
void segmentsToRing(const std::vector<SegType>& segments, Ring& ring) {
    if (segments.empty()) return;
    ring.vertices.clear();
    int n = static_cast<int>(segments.size());
    ring.vertices.resize(n);
    for (int i = 0; i < n; ++i) {
        ring.vertices[i] = segments[i].from;
    }
}
struct Polygon
{
    Ring outer;
    std::vector<Ring> holes;
    XGrid<EdgeIndex> xGrid;
    Box bBox;
    void clear() {
        outer.clear();
        holes.clear();
        xGrid.clear();
        bBox.clear();
    }
    bool empty() const { return outer.empty(); }
    void translateSelf(const Point& v) {
        for (auto& p : outer.vertices) {
            p = p + v;
        }
        for (auto& hole : holes) {
            for (auto& p : hole.vertices) {
                p = p + v;
            }
        }
        bBox.min = bBox.min + v;
        bBox.max = bBox.max + v;
        xGrid.xMin += v.x;
        xGrid.xMax += v.x;
    }
    Polygon translate(const Point& v) const
    {
        Polygon result = *this;
        result.translateSelf(v);
        return result;
    }
    void initBox(){
        bBox.setINF();
        for (const auto &p : outer.vertices)   {   bBox.expand(p);     }
        for (const auto& hole : holes) {
            for (const auto& p : hole.vertices) {  bBox.expand(p);    }
        }
    }
    void init(){
        initBox();
        buildXGrid();
    }
    void buildXGrid(){
        xGrid.xMin = bBox.min.x;  xGrid.xMax = bBox.max.x;
        Scalar totalEdgeSpanX = 0;  int edgeCount = 0;
        auto accumulateRing = [&](const Ring& ring) {
            if (ring.vertices.size() < 2) return;
            Scalar sumX = 0;
            Scalar sumY = 0;
            ring.getSumLength(sumX, sumY);
            totalEdgeSpanX += sumX;
            edgeCount += static_cast<int>(ring.vertices.size());
        };
        Scalar dxEps = xGrid.cellSizeX*0.01;
        auto addRingEdgesToGrid = [&](const Ring& ring, int ringIndex) {
            const int n = static_cast<int>(ring.vertices.size());
            if (n < 2) return;
            for (int edgeIndex = 0; edgeIndex < n; ++edgeIndex) {
                const Point& a = ring.vertices[edgeIndex];
                const Point& b = ring.vertices[(edgeIndex + 1) % n];
                Scalar edgeMinX = std::min(a.x, b.x);
                Scalar edgeMaxX = std::max(a.x, b.x);
                int cellL = static_cast<int>(std::floor((edgeMinX - dxEps - xGrid.xMin) / xGrid.cellSizeX));
                int cellR = static_cast<int>(std::floor((edgeMaxX + dxEps - xGrid.xMin) / xGrid.cellSizeX));
                cellL = std::clamp(cellL, 0, xGrid.numCellsX - 1);
                cellR = std::clamp(cellR, 0, xGrid.numCellsX - 1);
                if (cellR < cellL) std::swap(cellL, cellR);
                for (int cell = cellL; cell <= cellR; ++cell) {
                    xGrid.cells[cell].push_back({ringIndex, edgeIndex});
                }
            }
        };
        accumulateRing(outer);
        for (const auto& hole : holes) {
            accumulateRing(hole);
        }
        Scalar width = xGrid.xMax - xGrid.xMin;
        Scalar avgEdgeSpanX = edgeCount > 0 ? totalEdgeSpanX / static_cast<Scalar>(edgeCount) : 0;
        Scalar recommendedCellSizeX = std::max(EPS_DISTANCE, avgEdgeSpanX * 0.9);
        int numCellsX = 1;
        if (width > EPS_DISTANCE) {
            numCellsX = static_cast<int>(std::ceil(width / recommendedCellSizeX));
        }
        xGrid.numCellsX = std::clamp(numCellsX, 16, 128);
        xGrid.cellSizeX = width > EPS_DISTANCE ? width / static_cast<Scalar>(xGrid.numCellsX) : 1.0;
        xGrid.cells.clear();
        xGrid.cells.resize(xGrid.numCellsX);
        addRingEdgesToGrid(outer, 0);
        for (int holeIndex = 0; holeIndex < static_cast<int>(holes.size()); ++holeIndex) {
            addRingEdgesToGrid(holes[holeIndex], holeIndex + 1);
        }
    }
    bool containsPointStrict(const Point& p) const
    {
        if (outer.vertices.size() < 3) return false;
        int cellIdx = xGrid.getIndex(p.x);
        const auto& candidates = xGrid.cells[cellIdx];
        Scalar epsScan = 1e-10;
        int crossCount = 0;
        for (const auto& ei : candidates) {
            const auto& ring = (ei.ringIndex == 0) ? outer : holes[ei.ringIndex - 1];
            const auto& v = ring.vertices;
            const Point& a = v[ei.edgeIndex];
            const Point& b = v[(ei.edgeIndex + 1) % v.size()];
            if (pointOnSegment(p, a, b)) return false;
            Scalar edgeMinX = std::min(a.x, b.x);
            Scalar edgeMaxX = std::max(a.x, b.x);
            if (edgeMaxX - edgeMinX < epsScan) continue;            if (p.x < edgeMinX || p.x >= edgeMaxX) continue;
            Scalar yCross = a.y + (p.x - a.x) * (b.y - a.y) / (b.x - a.x + 1e-30);
            if (yCross > p.y) crossCount++;
        }
        return crossCount % 2 == 1;
    }
    bool isOverlapingStrict(const Polygon& other, Scalar eps = EPS_DISTANCE) const
    {
        if (empty() || other.empty()) return false;
        if (!boxesOverlap(bBox, other.bBox)) return false;
        enum class SegRel { None, TouchOnly, ProperCross };
        auto classifySegRelation = [&](const Point& p1, const Point& p2, const Point& q1, const Point& q2) -> SegRel {
            Box bp(p1, p2), bq(q1, q2);
            if (!boxesOverlap(bp, bq)) return SegRel::None;
            Point r = p2 - p1;
            Point s = q2 - q1;
            Point qp = q1 - p1;
            Scalar denom = r.cross(s);
            Scalar qpr = qp.cross(r);
            if (std::abs(denom) <= eps) {
                if (std::abs(qpr) > eps) return SegRel::None;
                Scalar r2 = r.norm2();
                if (r2 <= eps * eps) {
                    return ((p1 - q1).norm() <= eps || (p1 - q2).norm() <= eps) ? SegRel::TouchOnly : SegRel::None;
                }
                Scalar t0 = (q1 - p1).dot(r) / r2;
                Scalar t1 = (q2 - p1).dot(r) / r2;
                if (t0 > t1) std::swap(t0, t1);
                if (t1 < -eps || t0 > 1.0 + eps) return SegRel::None;
                return SegRel::TouchOnly;
            }
            Scalar t = qp.cross(s) / denom;
            Scalar u = qp.cross(r) / denom;
            if (t < -eps || t > 1.0 + eps || u < -eps || u > 1.0 + eps) return SegRel::None;
            bool strict = (t > eps && 1.0 - t > eps && u > eps && 1.0 - u > eps);
            return strict ? SegRel::ProperCross : SegRel::TouchOnly;
        };
        auto getRing = [&](const Polygon& poly, int ringIndex) -> const Ring& {
            return (ringIndex == 0) ? poly.outer : poly.holes[ringIndex - 1];
        };
        auto hasProperCrossing = [&]() -> bool {
            for (int riA = 0; riA <= static_cast<int>(holes.size()); ++riA) {
                const auto& vA = getRing(*this, riA).vertices;
                int nA = static_cast<int>(vA.size());
                for (int iA = 0; iA < nA; ++iA) {
                    const Point& p1 = vA[iA];
                    const Point& p2 = vA[(iA + 1) % nA];
                    Scalar edgeMinX = std::min(p1.x, p2.x);
                    Scalar edgeMaxX = std::max(p1.x, p2.x);
                    int cellL = other.xGrid.getIndex(edgeMinX);
                    int cellR = other.xGrid.getIndex(edgeMaxX);
                    for (int c = cellL; c <= cellR; ++c) {
                        for (const auto& ei : other.xGrid.cells[c]) {
                            const auto& vB = getRing(other, ei.ringIndex).vertices;
                            int nB = static_cast<int>(vB.size());
                            const Point& q1 = vB[ei.edgeIndex];
                            const Point& q2 = vB[(ei.edgeIndex + 1) % nB];
                            if (classifySegRelation(p1, p2, q1, q2) == SegRel::ProperCross)
                                return true;
                        }
                    }
                }
            }
            return false;
        };
        if (hasProperCrossing()) return true;
        if (other.containsPointStrict(outer.vertices[0])) return true;
        if (this->containsPointStrict(other.outer.vertices[0])) return true;
        return false;
    }
};
enum class PointClassify { Inside, Boundary, Outside };
struct UnorderedPolygon
{
    using Index = uint16_t;
    std::vector<Segment> segs;
    Box bBox;
    XGrid<int> xGrid;    
    bool empty() const { return segs.empty(); }
    void pushSegment(const Segment& seg){segs.push_back(seg);}
    void setSegments(const std::vector<Segment>& segmenst){
        segs = segmenst;
        init() ;
    }
    void initBox() {
        bBox.setINF();
        for (const auto& seg : segs) {
            bBox.expand(seg.from);
            bBox.expand(seg.to);
        }
    }
    void buildXGrid() {
        if (segs.empty()) {
            xGrid.xMin = 0;
            xGrid.xMax = 0;
            xGrid.cellSizeX = 0;
            xGrid.numCellsX = 0;
            xGrid.cells.clear();
            return;
        }
        xGrid.xMin = bBox.min.x;
        xGrid.xMax = bBox.max.x;
        Scalar width = xGrid.xMax - xGrid.xMin;
        Scalar recommendedCellSizeX = std::max(EPS_DISTANCE, width / 100.0);
        int numCellsX = 1;
        if (width > EPS_DISTANCE) {
            numCellsX = static_cast<int>(std::ceil(width / recommendedCellSizeX));
        }
        xGrid.numCellsX = std::clamp(numCellsX, 16, 128);
        xGrid.cellSizeX = width > EPS_DISTANCE ? width / static_cast<Scalar>(xGrid.numCellsX) : 1.0;
        xGrid.cells.clear();
        xGrid.cells.resize(xGrid.numCellsX);
        Scalar dxEps = xGrid.cellSizeX*0.01;
        for (size_t i = 0; i < segs.size(); ++i) {
            const auto& seg = segs[i];
            Scalar edgeMinX = std::min(seg.from.x, seg.to.x);
            Scalar edgeMaxX = std::max(seg.from.x, seg.to.x);
            int cellL = static_cast<int>(std::floor((edgeMinX - dxEps - xGrid.xMin) / xGrid.cellSizeX));
            int cellR = static_cast<int>(std::ceil((edgeMaxX + dxEps - xGrid.xMin) / xGrid.cellSizeX));
            cellL = std::clamp(cellL, 0, xGrid.numCellsX - 1);
            cellR = std::clamp(cellR, 0, xGrid.numCellsX - 1);
            if (cellR < cellL) std::swap(cellL, cellR);
            for (int cell = cellL; cell <= cellR; ++cell) {
                xGrid.cells[cell].push_back(static_cast<int>(i));   
            }
        }
    }
    void init() {
        initBox();
        buildXGrid();
    }
    void translateSelf(const Point& v){
        for (auto& seg : segs) {
            seg.from = seg.from + v;
            seg.to = seg.to + v;
        }
        bBox.min = bBox.min + v;
        bBox.max = bBox.max + v;
        xGrid.xMin += v.x;
        xGrid.xMax += v.x;
    }
    UnorderedPolygon translate(const Point& v) const   {
        UnorderedPolygon result = *this;
        result.translateSelf(v);  return result;
    }
    bool contains(const Point& p, Scalar eps = EPS_DISTANCE) const
    {
        if (empty()) return false;
        Scalar eps_scan = 1e-10;
        int cellIdx = xGrid.getIndex(p.x);
        const auto& candidateEdges = xGrid.cells[cellIdx];
        int crossCount = 0;
        for (int edgeIdx : candidateEdges) {
            const Segment& seg = segs[edgeIdx];
            Point pMin = seg.from;            Point pMax = seg.to;            if (pMin.x > pMax.x) std::swap(pMin, pMax);            if(pMax.x - pMin.x < eps_scan) continue;            if (p.x < pMin.x || p.x >= pMax.x) continue; 
            Scalar yCross = pMin.y + (p.x - pMin.x) * (pMax.y - pMin.y) / (pMax.x - pMin.x);
            if (yCross >  p.y) crossCount++;
        }
        return (crossCount % 2) == 1;    }
    PointClassify classifyPoint(const Point& p, Scalar eps = EPS_DISTANCE) const
    {
        if (empty()) return PointClassify::Outside;
        int cellIdx = xGrid.getIndex(p.x);
        for (int edgeIdx : xGrid.cells[cellIdx]) {
            if (pointOnSegment(p, segs[edgeIdx].from, segs[edgeIdx].to, eps))
                return PointClassify::Boundary;
        }
        return contains(p, eps) ? PointClassify::Inside : PointClassify::Outside;
    }
};
void snapGrid(Segment &seg){
    seg.from.x = std::llround(seg.from.x * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
    seg.from.y = std::llround(seg.from.y * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
    seg.to.x   = std::llround(seg.to.x   * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
    seg.to.y   = std::llround(seg.to.y   * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
}
void snapGrid(std::vector<Segment> &segs){
    for (auto& seg : segs)   snapGrid(seg);
}
void snapGrid(UnorderedPolygon& segPoly){
    snapGrid(segPoly.segs);
    segPoly.init();
}
void snapGrid(Ring &ring) {
    for (auto& p : ring.vertices) {
        p.x = std::llround(p.x * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
        p.y = std::llround(p.y * SCALE_SNAP_GRID) * EPS_SNAP_GRID;
    }
};
void removeZeroLengthEdges(Ring& ring) {
    if (ring.vertices.size() < 2) return;
    std::vector<Point> cleaned;
    cleaned.reserve(ring.vertices.size());
    for (const auto& v : ring.vertices) {
        if (cleaned.empty() || !cleaned.back().isEqual(v)) {
            cleaned.push_back(v);
        }
    }
    while (cleaned.size() > 1 && cleaned.front().isEqual(cleaned.back())) {
        cleaned.pop_back();
    }
    ring.vertices = std::move(cleaned);
}
void snapGrid(Polygon &poly) {
    snapGrid(poly.outer);
    removeZeroLengthEdges(poly.outer);
    for (auto &hole : poly.holes) {
        snapGrid(hole);
        removeZeroLengthEdges(hole);
    }
    poly.init();
}
}