#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <cstring>
#include <array>
#include <functional>

using namespace std;
const double EPS = 1e-9;
const double PI = acos(-1.0);
const double EPS_SQ = EPS * EPS;

auto g_startTime = chrono::steady_clock::now();
double elapsedSeconds() {
    return chrono::duration<double>(chrono::steady_clock::now() - g_startTime).count();
}
double g_timeBudget = 55.0;

struct Vector2D {
    double x, y;
    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    Vector2D operator+(const Vector2D& p) const { return Vector2D(x + p.x, y + p.y); }
    Vector2D operator-(const Vector2D& p) const { return Vector2D(x - p.x, y - p.y); }
    Vector2D operator*(double t) const { return Vector2D(x * t, y * t); }
    Vector2D operator/(double t) const { return Vector2D(x / t, y / t); }
    Vector2D& operator+=(const Vector2D& p) { x += p.x; y += p.y; return *this; }
    Vector2D& operator-=(const Vector2D& p) { x -= p.x; y -= p.y; return *this; }
    double dot(const Vector2D& p) const { return x * p.x + y * p.y; }
    double cross(const Vector2D& p) const { return x * p.y - y * p.x; }
    double norm() const { return sqrt(x * x + y * y); }
    double norm2() const { return x * x + y * y; }
    bool operator<(const Vector2D& o) const { return x < o.x || (x == o.x && y < o.y); }
    bool operator==(const Vector2D& o) const { return fabs(x - o.x) < EPS && fabs(y - o.y) < EPS; }
};

struct Box {
    double left, right, bottom, top;
    Box() : left(1e18), right(-1e18), bottom(1e18), top(-1e18) {}
    double width() const { return right - left; }
    double height() const { return top - bottom; }
    bool intersects(const Box& b) const {
        return !(right < b.left - EPS || b.right < left - EPS || top < b.bottom - EPS || b.top < bottom - EPS);
    }
    Vector2D center() const { return Vector2D((left + right) * 0.5, (bottom + top) * 0.5); }
};

struct Polygon {
    vector<Vector2D> pts;
    int cnt;
    mutable double cachedArea = -1;
    mutable Box cachedBox;
    mutable bool boxCached = false;

    Polygon() : cnt(0) {}
    Polygon(const vector<Vector2D>& v) : pts(v), cnt((int)v.size()) {}

    double signedArea() const {
        double a = 0;
        for (int i = 0, j = cnt - 1; i < cnt; ++i) {
            a += (pts[j].x + pts[i].x) * (pts[j].y - pts[i].y);
            j = i;
        }
        return -a * 0.5;
    }
    double area() const { if (cachedArea < 0) cachedArea = fabs(signedArea()); return cachedArea; }
    bool isCCW() const { return signedArea() >= 0; }

    Vector2D centroid() const {
        double cx = 0, cy = 0, a = 0;
        for (int i = 0; i < cnt; i++) {
            int j = (i + 1) % cnt;
            double cr = pts[i].cross(pts[j]);
            a += cr; cx += (pts[i].x + pts[j].x) * cr; cy += (pts[i].y + pts[j].y) * cr;
        }
        a *= 0.5;
        if (fabs(a) < EPS) {
            cx = cy = 0;
            for (int i = 0; i < cnt; i++) { cx += pts[i].x; cy += pts[i].y; }
            return Vector2D(cx / cnt, cy / cnt);
        }
        return Vector2D(cx / (6.0 * a), cy / (6.0 * a));
    }

    const Box& boundingBox() const {
        if (!boxCached) {
            Box b;
            for (int i = 0; i < cnt; i++) {
                b.left = min(b.left, pts[i].x); b.right = max(b.right, pts[i].x);
                b.bottom = min(b.bottom, pts[i].y); b.top = max(b.top, pts[i].y);
            }
            cachedBox = b; boxCached = true;
        }
        return cachedBox;
    }

    Polygon moved(const Vector2D& v) const {
        Polygon res; res.cnt = cnt; res.pts.resize(cnt);
        for (int i = 0; i < cnt; i++) res.pts[i] = pts[i] + v;
        return res;
    }
    void moveBy(const Vector2D& v) {
        cachedArea = -1; boxCached = false;
        for (int i = 0; i < cnt; i++) pts[i] = pts[i] + v;
    }
};

struct DPoint { double x, y; DPoint(double x_ = 0, double y_ = 0) : x(x_), y(y_) {} };

bool segIntersect(const DPoint& p1, const DPoint& p2, const DPoint& p3, const DPoint& p4, DPoint& ip) {
    double d1x = p2.x - p1.x, d1y = p2.y - p1.y;
    double d2x = p4.x - p3.x, d2y = p4.y - p3.y;
    double denom = d1x * d2y - d1y * d2x;
    if (fabs(denom) < 1e-15) return false;
    double t = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / denom;
    double u = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / denom;
    if (t >= -1e-10 && t <= 1.0 + 1e-10 && u >= -1e-10 && u <= 1.0 + 1e-10) {
        ip.x = p1.x + t * d1x; ip.y = p1.y + t * d1y; return true;
    }
    return false;
}

bool pointInPolygon(const DPoint& pt, const Polygon& poly) {
    bool inside = false;
    int n = poly.cnt;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly.pts[i].x, yi = poly.pts[i].y;
        double xj = poly.pts[j].x, yj = poly.pts[j].y;
        if (((yi > pt.y) != (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

vector<DPoint> clipByHalfPlane(const vector<DPoint>& poly, const DPoint& p1, const DPoint& p2) {
    vector<DPoint> output;
    int n = (int)poly.size();
    if (n == 0) return output;
    auto isInside = [&](const DPoint& pt) -> bool {
        return (p2.x - p1.x) * (pt.y - p1.y) - (p2.y - p1.y) * (pt.x - p1.x) >= -1e-10;
    };
    for (int i = 0; i < n; i++) {
        DPoint cur = poly[i], prev = poly[(i - 1 + n) % n];
        bool curIn = isInside(cur), prevIn = isInside(prev);
        if (curIn) {
            if (!prevIn) { DPoint ip; if (segIntersect(prev, cur, p1, p2, ip)) output.push_back(ip); }
            output.push_back(cur);
        } else if (prevIn) {
            DPoint ip; if (segIntersect(prev, cur, p1, p2, ip)) output.push_back(ip);
        }
    }
    return output;
}

vector<DPoint> clipPolygonByConvex(const vector<DPoint>& subject, const Polygon& clipper) {
    vector<DPoint> output = subject;
    for (int i = 0; i < clipper.cnt && !output.empty(); i++) {
        int j = (i + 1) % clipper.cnt;
        output = clipByHalfPlane(output, DPoint(clipper.pts[i].x, clipper.pts[i].y),
                                 DPoint(clipper.pts[j].x, clipper.pts[j].y));
    }
    return output;
}

double dpointArea(const vector<DPoint>& pts) {
    int n = (int)pts.size();
    if (n < 3) return 0.0;
    double a = 0;
    for (int i = 0, j = n - 1; i < n; j = i++) a += pts[j].x * pts[i].y - pts[i].x * pts[j].y;
    return fabs(a) * 0.5;
}

double cross3(const Vector2D& O, const Vector2D& A, const Vector2D& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

struct Triangle { int a, b, c; };

bool isEar(const vector<Vector2D>& pts, const vector<int>& indices, int i) {
    int n = (int)indices.size();
    int prev = indices[(i - 1 + n) % n], curr = indices[i], next = indices[(i + 1) % n];
    Vector2D a = pts[prev], b = pts[curr], c = pts[next];
    double cr = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (cr <= 0) return false;
    for (int j = 0; j < n; j++) {
        if (j == (i - 1 + n) % n || j == i || j == (i + 1) % n) continue;
        Vector2D p = pts[indices[j]];
        double d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        double d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
        double d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
        if (!((d1 < 0) || (d2 < 0) || (d3 < 0)) || !((d1 > 0) || (d2 > 0) || (d3 > 0))) return false;
    }
    return true;
}

vector<Triangle> triangulate(const Polygon& poly) {
    vector<Triangle> triangles;
    int n = poly.cnt;
    if (n < 3) return triangles;
    vector<int> indices(n);
    for (int i = 0; i < n; i++) indices[i] = i;
    if (!poly.isCCW()) reverse(indices.begin(), indices.end());
    int remaining = n, i = 0, safety = n * n * 2;
    while (remaining > 3 && safety-- > 0) {
        if (isEar(poly.pts, indices, i % remaining)) {
            int prev = (i - 1 + remaining) % remaining, next = (i + 1) % remaining;
            triangles.push_back({indices[prev], indices[i % remaining], indices[next]});
            indices.erase(indices.begin() + (i % remaining));
            remaining--; i = prev % remaining;
        } else { i = (i + 1) % remaining; }
    }
    if (remaining == 3) triangles.push_back({indices[0], indices[1], indices[2]});
    return triangles;
}

double intersectionArea(const Polygon& subject, const Polygon& clipper) {
    Box sb = subject.boundingBox(), cb = clipper.boundingBox();
    if (!sb.intersects(cb)) return 0.0;
    vector<Triangle> tris = triangulate(clipper);
    double totalArea = 0.0;
    vector<DPoint> subPts(subject.cnt);
    for (int i = 0; i < subject.cnt; i++) subPts[i] = DPoint(subject.pts[i].x, subject.pts[i].y);
    for (auto& tri : tris) {
        Polygon triPoly; triPoly.cnt = 3;
        triPoly.pts = {clipper.pts[tri.a], clipper.pts[tri.b], clipper.pts[tri.c]};
        vector<DPoint> clipped = clipPolygonByConvex(subPts, triPoly);
        totalArea += dpointArea(clipped);
    }
    return totalArea;
}

vector<Vector2D> convexHull(vector<Vector2D> P) {
    int n = (int)P.size();
    if (n <= 1) return P;
    sort(P.begin(), P.end(), [](const Vector2D& a, const Vector2D& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    vector<Vector2D> H(2 * n);
    int k = 0;
    for (int i = 0; i < n; i++) { while (k >= 2 && cross3(H[k - 2], H[k - 1], P[i]) <= 0) k--; H[k++] = P[i]; }
    for (int i = n - 2, t = k + 1; i >= 0; i--) { while (k >= t && cross3(H[k - 2], H[k - 1], P[i]) <= 0) k--; H[k++] = P[i]; }
    H.resize(k);
    if (k > 1 && H.back().x == H[0].x && H.back().y == H[0].y) H.pop_back();
    return H;
}

// Overlap and boundary checks (strict precision)
bool isInsideBound(const Polygon& poly, const Polygon& bnd) {
    double polyArea = poly.area();
    if (polyArea < EPS) return true;
    double interArea = intersectionArea(poly, bnd);
    double areaDiff = polyArea - interArea;
    // Stricter threshold: absolute difference less than 1e-10 and relative less than 1e-6
    return areaDiff < 1e-10 && areaDiff < polyArea * 1e-6;
}

bool polygonsOverlap(const Polygon& A, const Polygon& B) {
    Box ba = A.boundingBox(), bb = B.boundingBox();
    if (!ba.intersects(bb)) return false;
    double interArea = intersectionArea(A, B);
    if (interArea < 1e-10) return false;  // Stricter threshold
    double minArea = min(A.area(), B.area());
    if (minArea < EPS) return false;
    return interArea >= minArea * 1e-6;  // Stricter relative threshold
}

// Quick overlap check using SAT for early rejection
bool quickOverlapCheck(const Polygon& A, const Polygon& B) {
    Box ba = A.boundingBox(), bb = B.boundingBox();
    if (!ba.intersects(bb)) return false;
    
    // Check edge normals of both polygons
    auto checkAxis = [&](const Vector2D& axis) -> bool {
        double minA = 1e18, maxA = -1e18, minB = 1e18, maxB = -1e18;
        for (auto& p : A.pts) { double proj = p.dot(axis); minA = min(minA, proj); maxA = max(maxA, proj); }
        for (auto& p : B.pts) { double proj = p.dot(axis); minB = min(minB, proj); maxB = max(maxB, proj); }
        return max(minA, minB) <= min(maxA, maxB) + EPS;
    };
    
    for (int i = 0; i < A.cnt; i++) {
        int j = (i + 1) % A.cnt;
        Vector2D edge = A.pts[j] - A.pts[i];
        Vector2D normal(-edge.y, edge.x);
        if (normal.norm2() > EPS_SQ && !checkAxis(normal)) return false;
    }
    for (int i = 0; i < B.cnt; i++) {
        int j = (i + 1) % B.cnt;
        Vector2D edge = B.pts[j] - B.pts[i];
        Vector2D normal(-edge.y, edge.x);
        if (normal.norm2() > EPS_SQ && !checkAxis(normal)) return false;
    }
    return true; // Potential overlap, need detailed check
}

// Minkowski sum / NFP
Polygon minkowskiSumConvex(const Polygon& A, const Polygon& B) {
    int na = A.cnt, nb = B.cnt;
    if (na == 0 || nb == 0) return Polygon();
    vector<Vector2D> a(A.pts.begin(), A.pts.end());
    vector<Vector2D> b(B.pts.begin(), B.pts.end());
    if (!A.isCCW()) reverse(a.begin(), a.end());
    if (!B.isCCW()) reverse(b.begin(), b.end());
    int ia = 0, ib = 0;
    for (int i = 1; i < na; i++) if (a[i].y < a[ia].y || (a[i].y == a[ia].y && a[i].x < a[ia].x)) ia = i;
    for (int i = 1; i < nb; i++) if (b[i].y < b[ib].y || (b[i].y == b[ib].y && b[i].x < b[ib].x)) ib = i;
    vector<Vector2D> result;
    int i = 0, j = 0;
    while (i < na || j < nb) {
        result.push_back(a[(ia + i) % na] + b[(ib + j) % nb]);
        if (i >= na) { j++; continue; }
        if (j >= nb) { i++; continue; }
        Vector2D va = a[(ia + i + 1) % na] - a[(ia + i) % na];
        Vector2D vb = b[(ib + j + 1) % nb] - b[(ib + j) % nb];
        double c = va.cross(vb);
        if (c > EPS) i++; else if (c < -EPS) j++; else { i++; j++; }
    }
    return Polygon(result);
}

Polygon computeNFP(const Polygon& A, const Polygon& B) {
    vector<Vector2D> hullA = convexHull(vector<Vector2D>(A.pts.begin(), A.pts.end()));
    vector<Vector2D> reflectedB(B.cnt);
    for (int i = 0; i < B.cnt; i++) reflectedB[i] = Vector2D(-B.pts[i].x, -B.pts[i].y);
    vector<Vector2D> hullB = convexHull(reflectedB);
    if (hullA.empty() || hullB.empty()) return Polygon();
    return minkowskiSumConvex(Polygon(hullA), Polygon(hullB));
}

// SAT-based minimum translation vector
Vector2D findMinTranslation(const Polygon& A, const Polygon& B) {
    if (!polygonsOverlap(A, B)) return Vector2D(0, 0);
    vector<Vector2D> hullA = convexHull(vector<Vector2D>(A.pts.begin(), A.pts.end()));
    vector<Vector2D> hullB = convexHull(vector<Vector2D>(B.pts.begin(), B.pts.end()));
    double minOverlap = 1e18;
    Vector2D bestAxis(1, 0);
    auto checkAxes = [&](const vector<Vector2D>& hull1, const vector<Vector2D>& hull2) {
        int n = (int)hull1.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            Vector2D edge = hull1[j] - hull1[i];
            Vector2D axis(-edge.y, edge.x);
            double len = axis.norm();
            if (len < EPS) continue;
            axis = axis * (1.0 / len);
            double min1 = 1e18, max1 = -1e18, min2 = 1e18, max2 = -1e18;
            for (auto& p : hull1) { double proj = p.dot(axis); min1 = min(min1, proj); max1 = max(max1, proj); }
            for (auto& p : hull2) { double proj = p.dot(axis); min2 = min(min2, proj); max2 = max(max2, proj); }
            double overlap = min(max1, max2) - max(min1, min2);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                bestAxis = axis;
                Vector2D centerA(0, 0), centerB(0, 0);
                for (auto& p : hullA) centerA = centerA + p;
                for (auto& p : hullB) centerB = centerB + p;
                centerA = centerA * (1.0 / hullA.size());
                centerB = centerB * (1.0 / hullB.size());
                if ((centerB - centerA).dot(bestAxis) < 0) bestAxis = bestAxis * (-1.0);
            }
        }
    };
    checkAxes(hullA, hullB);
    checkAxes(hullB, hullA);
    return bestAxis * (minOverlap + EPS * 10);
}

// Global data
int n = 0;
Polygon bound;
vector<int> vertexNums;
vector<Polygon> polygons;
vector<Vector2D> testOut;

// Validity checks
bool isValidPlacement(int polyIdx, const Vector2D& trans,
                      const vector<int>& placedIndices,
                      const vector<Vector2D>& placedTrans) {
    Polygon movedPoly = polygons[polyIdx].moved(trans);
    if (!isInsideBound(movedPoly, bound)) return false;
    Box movedBox = movedPoly.boundingBox();
    for (int k = 0; k < (int)placedIndices.size(); k++) {
        int otherIdx = placedIndices[k];
        Polygon otherPoly = polygons[otherIdx].moved(placedTrans[k]);
        if (!movedBox.intersects(otherPoly.boundingBox())) continue;
        // Use quick SAT check first before expensive area-based check
        if (quickOverlapCheck(movedPoly, otherPoly)) {
            if (polygonsOverlap(movedPoly, otherPoly)) return false;
        }
    }
    return true;
}

bool isValidPlacementAll(int polyIdx, const Vector2D& trans) {
    Polygon movedPoly = polygons[polyIdx].moved(trans);
    if (!isInsideBound(movedPoly, bound)) return false;
    Box movedBox = movedPoly.boundingBox();
    for (int j = 0; j < n; j++) {
        if (j == polyIdx) continue;
        Polygon otherPoly = polygons[j].moved(testOut[j]);
        if (!movedBox.intersects(otherPoly.boundingBox())) continue;
        // Use quick SAT check first before expensive area-based check
        if (quickOverlapCheck(movedPoly, otherPoly)) {
            if (polygonsOverlap(movedPoly, otherPoly)) return false;
        }
    }
    return true;
}

// Candidate placement finder
Vector2D findBestPlacement(int polyIdx, const vector<int>& placedIndices,
                           const vector<Vector2D>& placedTrans) {
    const Polygon& poly = polygons[polyIdx];
    Box polyBB = poly.boundingBox();
    Box boundBB = bound.boundingBox();

    double feasibleLeft = boundBB.left - polyBB.left;
    double feasibleRight = boundBB.right - polyBB.right;
    double feasibleBottom = boundBB.bottom - polyBB.bottom;
    double feasibleTop = boundBB.top - polyBB.top;

    if (isValidPlacement(polyIdx, Vector2D(0, 0), placedIndices, placedTrans)) return Vector2D(0, 0);

    vector<Vector2D> candidates;

    // NFP-based candidates
    for (int k = 0; k < (int)placedIndices.size(); k++) {
        int otherIdx = placedIndices[k];
        Polygon otherMoved = polygons[otherIdx].moved(placedTrans[k]);
        Polygon nfp = computeNFP(otherMoved, poly);
        for (int i = 0; i < nfp.cnt; i++) {
            int j = (i + 1) % nfp.cnt;
            candidates.push_back(nfp.pts[i]);
            candidates.push_back(Vector2D((nfp.pts[i].x + nfp.pts[j].x) * 0.5,
                                          (nfp.pts[i].y + nfp.pts[j].y) * 0.5));
        }
    }

    // Boundary edge sampling with more points and key positions
    for (int i = 0; i < bound.cnt; i++) {
        int j = (i + 1) % bound.cnt;
        // Sample at 0, 1/4, 1/3, 1/2, 2/3, 3/4, 1 to hit key fractions
        vector<double> keyFractions = {0.0, 0.25, 1.0/3.0, 0.5, 2.0/3.0, 0.75, 1.0};
        for (double t : keyFractions) {
            Vector2D pos = bound.pts[i] + (bound.pts[j] - bound.pts[i]) * t;
            Vector2D cen = poly.centroid();
            candidates.push_back(Vector2D(pos.x - cen.x, pos.y - cen.y));
        }
        // Additional fine sampling
        for (int s = 0; s <= 16; s++) {
            double t = (double)s / 16.0;
            Vector2D pos = bound.pts[i] + (bound.pts[j] - bound.pts[i]) * t;
            Vector2D cen = poly.centroid();
            candidates.push_back(Vector2D(pos.x - cen.x, pos.y - cen.y));
        }
    }

    // Grid positions with higher resolution
    int gridSteps = 40;  // Increased from 20
    for (int i = 0; i <= gridSteps; i++) {
        double t = (double)i / gridSteps;
        double x = feasibleLeft + t * (feasibleRight - feasibleLeft);
        candidates.push_back(Vector2D(x, feasibleBottom));
        candidates.push_back(Vector2D(x, feasibleTop));
        double y = feasibleBottom + t * (feasibleTop - feasibleBottom);
        candidates.push_back(Vector2D(feasibleLeft, y));
        candidates.push_back(Vector2D(feasibleRight, y));
    }
    
    // Add midpoints explicitly
    double midX = (feasibleLeft + feasibleRight) * 0.5;
    double midY = (feasibleBottom + feasibleTop) * 0.5;
    candidates.push_back(Vector2D(midX, feasibleBottom));
    candidates.push_back(Vector2D(midX, feasibleTop));
    candidates.push_back(Vector2D(feasibleLeft, midY));
    candidates.push_back(Vector2D(feasibleRight, midY));
    candidates.push_back(Vector2D(midX, midY));

    // Interior grid with higher resolution
    int innerGrid = 12;  // Increased from 8
    for (int i = 1; i < innerGrid; i++) {
        for (int j = 1; j < innerGrid; j++) {
            double x = feasibleLeft + (double)i / innerGrid * (feasibleRight - feasibleLeft);
            double y = feasibleBottom + (double)j / innerGrid * (feasibleTop - feasibleBottom);
            candidates.push_back(Vector2D(x, y));
        }
    }

    double bestDist = 1e18;
    Vector2D bestVec(0, 0);

    for (auto& cand : candidates) {
        if (cand.x < feasibleLeft - 10 || cand.x > feasibleRight + 10 ||
            cand.y < feasibleBottom - 10 || cand.y > feasibleTop + 10) continue;
        if (!isValidPlacement(polyIdx, cand, placedIndices, placedTrans)) continue;
        double dist = cand.norm();
        if (dist < bestDist) { bestDist = dist; bestVec = cand; }
    }

    // Fallback: iterative separation
    if (bestDist >= 1e17) {
        Vector2D totalTrans(0, 0);
        Polygon curPoly = poly;
        for (int iter = 0; iter < 500; iter++) {
            bool anyOverlap = false;
            for (int k = 0; k < (int)placedIndices.size(); k++) {
                int otherIdx = placedIndices[k];
                Polygon otherPoly = polygons[otherIdx].moved(placedTrans[k]);
                if (polygonsOverlap(otherPoly, curPoly)) {
                    anyOverlap = true;
                    Vector2D mtv = findMinTranslation(otherPoly, curPoly);
                    curPoly.moveBy(mtv);
                    totalTrans = totalTrans + mtv;
                }
            }
            if (!isInsideBound(curPoly, bound)) {
                anyOverlap = true;
                Vector2D toward = (bound.centroid() - curPoly.centroid());
                double tLen = toward.norm();
                if (tLen > EPS) { toward = toward * (1.0 / tLen); curPoly.moveBy(toward * 0.05); totalTrans = totalTrans + toward * 0.05; }
            }
            if (!anyOverlap) break;
        }
        if (isInsideBound(curPoly, bound)) {
            bool noOverlap = true;
            for (int k = 0; k < (int)placedIndices.size(); k++) {
                Polygon otherPoly = polygons[placedIndices[k]].moved(placedTrans[k]);
                if (polygonsOverlap(otherPoly, curPoly)) { noOverlap = false; break; }
            }
            if (noOverlap) bestVec = totalTrans;
        }
    }

    return bestVec;
}

double totalL(const vector<Vector2D>& v) {
    double s = 0;
    for (auto& p : v) s += p.norm();
    return s;
}

void GenSolution()
{
    testOut.resize(n);
    double bestL = 1e18;
    vector<Vector2D> bestOut(n, Vector2D(0, 0));

    // Compute overlap degrees
    vector<double> overlapDegree(n, 0);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (polygonsOverlap(polygons[i], polygons[j])) {
                double area = intersectionArea(polygons[i], polygons[j]);
                overlapDegree[i] += area;
                overlapDegree[j] += area;
            }
        }
    }

    // ===== Strategy 1: Greedy placement with orderings =====
    {
        vector<vector<int>> orderings;

        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            sort(order.begin(), order.end(), [&](int a, int b) { return overlapDegree[a] > overlapDegree[b]; });
            orderings.push_back(order); }
        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            sort(order.begin(), order.end(), [&](int a, int b) { return overlapDegree[a] < overlapDegree[b]; });
            orderings.push_back(order); }
        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            sort(order.begin(), order.end(), [&](int a, int b) { return polygons[a].area() > polygons[b].area(); });
            orderings.push_back(order); }
        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            sort(order.begin(), order.end(), [&](int a, int b) { return polygons[a].area() < polygons[b].area(); });
            orderings.push_back(order); }
        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            Vector2D bc = bound.centroid();
            sort(order.begin(), order.end(), [&](int a, int b) {
                return (polygons[a].centroid() - bc).norm2() < (polygons[b].centroid() - bc).norm2(); });
            orderings.push_back(order); }
        { vector<int> order(n); for (int i = 0; i < n; i++) order[i] = i;
            sort(order.begin(), order.end(), [&](int a, int b) {
                double sa = polygons[a].boundingBox().width() * polygons[a].boundingBox().height();
                double sb = polygons[b].boundingBox().width() * polygons[b].boundingBox().height();
                return sa > sb; });
            orderings.push_back(order); }

        for (int oi = 0; oi < (int)orderings.size(); oi++) {
            if (elapsedSeconds() > g_timeBudget * 0.3) break;
            auto& order = orderings[oi];
            vector<Vector2D> curOut(n, Vector2D(0, 0));
            vector<int> placedIndices;
            vector<Vector2D> placedTrans;

            int idx = order[0];
            Vector2D trans(0, 0);
            if (!isInsideBound(polygons[idx], bound)) {
                Vector2D toward = (bound.centroid() - polygons[idx].centroid());
                trans = toward;
                if (!isInsideBound(polygons[idx].moved(trans), bound)) trans = Vector2D(0, 0);
            }
            curOut[idx] = trans;
            placedIndices.push_back(idx); placedTrans.push_back(trans);

            for (int pi = 1; pi < n; pi++) {
                if (elapsedSeconds() > g_timeBudget * 0.3) break;
                idx = order[pi];
                curOut[idx] = findBestPlacement(idx, placedIndices, placedTrans);
                placedIndices.push_back(idx); placedTrans.push_back(curOut[idx]);
            }

            double curL = totalL(curOut);
            if (curL < bestL) { bestL = curL; bestOut = curOut; }
        }
    }

    // ===== Strategy 2: Iterative resolution from all-zero start =====
    // Only use for small n since it's O(n^2) per iteration
    if (n <= 30 && elapsedSeconds() < g_timeBudget * 0.15) {
        vector<Vector2D> curOut(n, Vector2D(0, 0));
        for (int iter = 0; iter < 100; iter++) {
            if (elapsedSeconds() > g_timeBudget * 0.2) break;
            bool anyOverlap = false;

            for (int i = 0; i < n; i++) {
                Polygon movedI = polygons[i].moved(curOut[i]);
                if (!isInsideBound(movedI, bound)) {
                    anyOverlap = true;
                    Vector2D toward = (bound.centroid() - movedI.centroid());
                    double tLen = toward.norm();
                    if (tLen > EPS) {
                        toward = toward * (1.0 / tLen);
                        double lo = 0, hi = 1.0;
                        for (int s = 0; s < 20; s++) {
                            double mid = (lo + hi) * 0.5;
                            if (isInsideBound(polygons[i].moved(curOut[i] + toward * mid), bound)) hi = mid;
                            else lo = mid;
                        }
                        curOut[i] = curOut[i] + toward * (hi + 0.001);
                    }
                }
            }

            for (int i = 0; i < n; i++) {
                Polygon movedI = polygons[i].moved(curOut[i]);
                for (int j = i + 1; j < n; j++) {
                    Polygon movedJ = polygons[j].moved(curOut[j]);
                    if (polygonsOverlap(movedI, movedJ)) {
                        anyOverlap = true;
                        Vector2D mtv = findMinTranslation(movedI, movedJ);
                        if (curOut[i].norm() <= curOut[j].norm()) {
                            curOut[j] = curOut[j] + mtv;
                            movedJ = polygons[j].moved(curOut[j]);
                        } else {
                            curOut[i] = curOut[i] - mtv;
                            movedI = polygons[i].moved(curOut[i]);
                        }
                    }
                }
            }
            if (!anyOverlap) break;
        }
        double curL = totalL(curOut);
        if (curL < bestL) { bestL = curL; bestOut = curOut; }
    }

    testOut = bestOut;

    // ===== Phase 2: Fix any remaining overlaps =====
    for (int verify = 0; verify < 5; verify++) {
        bool needFix = false;
        for (int i = 0; i < n; i++) {
            Polygon movedI = polygons[i].moved(testOut[i]);
            if (!isInsideBound(movedI, bound)) { needFix = true; break; }
            for (int j = i + 1; j < n; j++) {
                if (polygonsOverlap(movedI, polygons[j].moved(testOut[j]))) { needFix = true; break; }
            }
            if (needFix) break;
        }
        if (!needFix) break;
        for (int i = 0; i < n; i++) {
            Polygon movedI = polygons[i].moved(testOut[i]);
            bool hasIssue = !isInsideBound(movedI, bound);
            if (!hasIssue) {
                for (int j = 0; j < n; j++) {
                    if (j == i) continue;
                    if (polygonsOverlap(movedI, polygons[j].moved(testOut[j]))) { hasIssue = true; break; }
                }
            }
            if (hasIssue) {
                vector<int> otherIndices; vector<Vector2D> otherTrans;
                for (int j = 0; j < n; j++) if (j != i) { otherIndices.push_back(j); otherTrans.push_back(testOut[j]); }
                testOut[i] = findBestPlacement(i, otherIndices, otherTrans);
            }
        }
    }

    // ===== Phase 3: Binary search to reduce each translation =====
    for (int round = 0; round < 5; round++) {  // Increased from 3 to 5
        if (elapsedSeconds() > g_timeBudget * 0.5) break;
        for (int i = 0; i < n; i++) {
            Vector2D curTrans = testOut[i];
            Vector2D dir(-curTrans.x, -curTrans.y);
            double dirLen = dir.norm();
            if (dirLen < EPS) continue;
            dir = dir * (1.0 / dirLen);
            double lo = 0, hi = dirLen;
            bool suc = false;
            for (int step = 0; step < 80; step++) {  // Increased from 50 to 80
                double mid = (lo + hi) * 0.5;
                Vector2D tryTrans = curTrans + dir * mid;
                Polygon movedPoly = polygons[i].moved(tryTrans);
                if (!isInsideBound(movedPoly, bound)) { hi = mid; continue; }
                bool overlaps = false;
                for (int j = 0; j < n; j++) {
                    if (j == i) continue;
                    if (polygonsOverlap(movedPoly, polygons[j].moved(testOut[j]))) { overlaps = true; break; }
                }
                if (overlaps) hi = mid; else { suc = true; lo = mid; }
            }
            if (suc) {
                Vector2D newTrans = curTrans + dir * lo;
                // Use smaller EPS threshold for more precise optimization
                if (newTrans.norm() < curTrans.norm() - 1e-10) testOut[i] = newTrans;
            }
        }
    }

    // ===== Phase 4: Re-place polygons with largest translations =====
    for (int retry = 0; retry < 2; retry++) {
        if (elapsedSeconds() > g_timeBudget * 0.65) break;
        vector<pair<double, int>> byLen;
        for (int i = 0; i < n; i++) byLen.push_back({testOut[i].norm(), i});
        sort(byLen.rbegin(), byLen.rend());
        for (auto& [len, idx] : byLen) {
            if (len < EPS) continue;
            if (elapsedSeconds() > g_timeBudget * 0.65) break;
            vector<int> otherIndices; vector<Vector2D> otherTrans;
            for (int j = 0; j < n; j++) if (j != idx) { otherIndices.push_back(j); otherTrans.push_back(testOut[j]); }
            Vector2D newTrans = findBestPlacement(idx, otherIndices, otherTrans);
            if (isValidPlacementAll(idx, newTrans) && newTrans.norm() < testOut[idx].norm() - EPS) {
                testOut[idx] = newTrans;
            }
        }
    }

    // ===== Phase 5: Multi-direction local search with higher precision =====
    for (int round = 0; round < 10; round++) {  // Increased from 5 to 10
        if (elapsedSeconds() > g_timeBudget * 0.85) break;
        for (int i = 0; i < n; i++) {
            if (elapsedSeconds() > g_timeBudget * 0.85) break;
            Vector2D curTrans = testOut[i];
            double curLen = curTrans.norm();
            if (curLen < EPS) continue;

            for (int d = 0; d < 24; d++) {  // Increased from 16 to 24 directions
                double angle = (d * 2.0 * PI) / 24.0;
                Vector2D dir(cos(angle), sin(angle));
                double step = 0.01;  // Smaller initial step
                while (step > 1e-7) {  // More precise convergence
                    Vector2D tryTrans = curTrans + dir * step;
                    if (tryTrans.norm() >= curLen - 1e-10) { step *= 0.5; continue; }
                    Polygon movedPoly = polygons[i].moved(tryTrans);
                    if (!isInsideBound(movedPoly, bound)) { step *= 0.5; continue; }
                    bool overlaps = false;
                    for (int j = 0; j < n; j++) {
                        if (j == i) continue;
                        if (polygonsOverlap(movedPoly, polygons[j].moved(testOut[j]))) { overlaps = true; break; }
                    }
                    if (!overlaps && tryTrans.norm() < curLen - 1e-10) {
                        curTrans = tryTrans; curLen = tryTrans.norm();
                    } else { step *= 0.5; }
                }
            }
            testOut[i] = curTrans;
        }
    }

    // ===== Phase 6: Final strict verification =====
    for (int i = 0; i < n; i++) {
        Polygon movedPoly = polygons[i].moved(testOut[i]);
        if (!isInsideBound(movedPoly, bound)) {
            Vector2D toward = (bound.centroid() - movedPoly.centroid());
            double tLen = toward.norm();
            if (tLen > EPS) {
                toward = toward * (1.0 / tLen);
                for (int iter = 0; iter < 200; iter++) {
                    testOut[i] = testOut[i] + toward * 0.02;
                    if (isInsideBound(polygons[i].moved(testOut[i]), bound)) break;
                }
            }
        }
        movedPoly = polygons[i].moved(testOut[i]);
        for (int iter = 0; iter < 200; iter++) {
            bool anyOverlap = false;
            for (int j = 0; j < n; j++) {
                if (j == i) continue;
                Polygon movedJ = polygons[j].moved(testOut[j]);
                if (polygonsOverlap(movedPoly, movedJ)) {
                    anyOverlap = true;
                    Vector2D mtv = findMinTranslation(movedJ, movedPoly);
                    movedPoly.moveBy(mtv);
                    testOut[i] = testOut[i] + mtv;
                }
            }
            if (!anyOverlap) break;
        }
    }
}

int main()
{
    int boundVertexCnt;
    cin >> boundVertexCnt;
    bound.pts.resize(boundVertexCnt);
    bound.cnt = boundVertexCnt;
    for (int i = 0; i < boundVertexCnt; ++i) cin >> bound.pts[i].x >> bound.pts[i].y;

    cin >> n;
    if (n <= 2) { cerr << "Input data error" << endl; return 1; }

    if (n > 50) g_timeBudget = 50.0;
    else if (n > 30) g_timeBudget = 52.0;
    else g_timeBudget = 55.0;

    vertexNums.resize(n);
    polygons.resize(n);
    for (int i = 0; i < n; ++i) {
        cin >> vertexNums[i];
        polygons[i].pts.resize(vertexNums[i]);
        polygons[i].cnt = vertexNums[i];
        for (int j = 0; j < vertexNums[i]; ++j) cin >> polygons[i].pts[j].x >> polygons[i].pts[j].y;
    }

    string okResp;
    cin >> okResp;
    if (okResp != "OK") { cerr << "Input data error: waiting for OK" << endl; return 1; }

    GenSolution();

    cout << n << endl;
    cout.flush();
    for (int i = 0; i < n; ++i) {
        cout << fixed << setprecision(5) << testOut[i].x << " " << testOut[i].y << endl;
        cout.flush();
    }
    cout << "OK" << endl;
    cout.flush();

    return 0;
}