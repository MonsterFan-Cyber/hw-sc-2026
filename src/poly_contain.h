#pragma once
#include "polygon.h"
// 如果实现了 ifp 可以将越界检查复杂度降到 O(1)
// 后面会加入 ifp 替换 ContainPolys
// 由于比赛时间有限，只实现了一版复杂度较高的边边检测版本
namespace geom {
class ContainPolys {
    int polyNum = 0;
    UnorderedPolygon feasiblePoly;
    std::vector<UnorderedPolygon> polys;
public:
    void build(const UnorderedPolygon& feasiblePoly, const std::vector<UnorderedPolygon>& polys) {
        this->feasiblePoly = feasiblePoly;
        this->polys = polys;
        this->polyNum = static_cast<int>(polys.size());
    }
    bool contain(int i, const Point& a) {
        if (i < 0 || i >= polyNum) return false;
        const auto& poly = polys[i];
        const auto& segs = poly.segs;
        int n = static_cast<int>(segs.size());
        Box polyBox = poly.bBox;
        polyBox.min = polyBox.min + a;
        polyBox.max = polyBox.max + a;
        if (!feasiblePoly.bBox.contains(polyBox.min) ||
            !feasiblePoly.bBox.contains(polyBox.max)) {
            return false;
        }
        for (int k = 0; k < n; ++k) {
            if (!feasiblePoly.contains(segs[k].from + a)) return false;
        }
        return !hasEdgeCrossing(poly, a);
    }
private:
    bool hasEdgeCrossing(const UnorderedPolygon& poly, const Point& a) const {
        for (const auto& segP : poly.segs) {
            Point p1 = segP.from + a;
            Point p2 = segP.to + a;
            Scalar edgeMinX = std::min(p1.x, p2.x);
            Scalar edgeMaxX = std::max(p1.x, p2.x);
            int cellL = feasiblePoly.xGrid.getIndex(edgeMinX);
            int cellR = feasiblePoly.xGrid.getIndex(edgeMaxX);
            for (int c = cellL; c <= cellR; ++c) {
                for (int edgeIdx : feasiblePoly.xGrid.cells[c]) {
                    const auto& segF = feasiblePoly.segs[edgeIdx];
                    if (segmentsProperCross(p1, p2, segF.from, segF.to)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    bool segmentsProperCross(const Point& p1, const Point& p2,
                            const Point& q1, const Point& q2) const {
        Box bp(p1, p2), bq(q1, q2);
        if (!boxesOverlap(bp, bq)) return false;
        Point r = p2 - p1, s = q2 - q1, qp = q1 - p1;
        Scalar denom = r.cross(s);
        if (std::abs(denom) < EPS_DISTANCE) return false;
        Scalar t = qp.cross(s) / denom;
        Scalar u = qp.cross(r) / denom;
        return t > EPS_DISTANCE && t < 1.0 - EPS_DISTANCE
            && u > EPS_DISTANCE && u < 1.0 - EPS_DISTANCE;
    }
};
}
