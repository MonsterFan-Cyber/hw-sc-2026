#pragma once

#include "nfp.h"
#include "polygon.h"
#include <array>


namespace nfp {
using namespace traj;
const int NFP_GRID_SIZE = 64;
std::vector<Scalar> lineScanY(const UnorderedPolygon &segPoly, Scalar x) {
  std::vector<Scalar> yIntersections;
  if (segPoly.empty()) return yIntersections;
  int cellIdx = segPoly.xGrid.getIndex(x);
  const auto &candidateEdges = segPoly.xGrid.cells[cellIdx];
  const Scalar eps_scan = 1e-10;
  yIntersections.reserve(candidateEdges.size());
  for (int edgeIdx : candidateEdges) {
    const Segment &seg = segPoly.segs[edgeIdx];
    Point pMin = seg.from;
    Point pMax = seg.to;
    if (pMin.x > pMax.x) std::swap(pMin, pMax);
    if (pMax.x - pMin.x < eps_scan) continue;
    if (x < pMin.x || x >= pMax.x) continue;
    Scalar yCross = pMin.y + (x - pMin.x) * (pMax.y - pMin.y) / (pMax.x - pMin.x);
    yIntersections.push_back(yCross);
  }
  std::sort(yIntersections.begin(), yIntersections.end());
  return yIntersections;
}
void lineScanY(const Polygon &segPoly, Scalar x, std::vector<Scalar>& outerLines, std::vector<Scalar>& holesLines) {
  outerLines.clear();
  holesLines.clear();
  if (segPoly.empty()) return;
  int cellIdx = segPoly.xGrid.getIndex(x);
  const auto &candidateEdges = segPoly.xGrid.cells[cellIdx];
  const Scalar eps_scan = 1e-10;
  outerLines.reserve(candidateEdges.size() / 2);
  holesLines.reserve(candidateEdges.size() / 2);
  for (const auto &ei : candidateEdges) {
    int ringId = ei.ringIndex;
    int edgeIdx = ei.edgeIndex;
    const Ring *ring = nullptr;
    if (ringId == 0) {   ring = &segPoly.outer;
    } else if (ringId - 1 < static_cast<int>(segPoly.holes.size())) { ring = &segPoly.holes[ringId - 1];
    } else {   continue;  }
    if (ring->vertices.empty()) continue;
    const std::vector<Point> &vertices = ring->vertices;
    int n = static_cast<int>(vertices.size());
    if (edgeIdx < 0 || edgeIdx >= n) continue;
    const Point &a = vertices[edgeIdx];
    const Point &b = vertices[(edgeIdx + 1) % n];
    Point pMin = a;
    Point pMax = b;
    if (pMin.x > pMax.x) std::swap(pMin, pMax);
    if (pMax.x - pMin.x < eps_scan) continue;
    if (x < pMin.x || x >= pMax.x) continue;
    Scalar yCross = pMin.y + (x - pMin.x) * (pMax.y - pMin.y) / (pMax.x - pMin.x);
    if (ringId == 0) {   outerLines.push_back(yCross);  }
    else { holesLines.push_back(yCross);  }
  }
  std::sort(outerLines.begin(), outerLines.end());
  std::sort(holesLines.begin(), holesLines.end());
}
class BVH {
  struct BVHNode {
    Box aabb;
    int left;
    int right;  };
  const UnorderedPolygon *segPolyPtr = nullptr;
  std::vector<BVHNode> nodes;
  int rootNodeIdx = -1;
  int buildRecursive(std::vector<int> &edgeIds, int start, int end) {
    if (end - start <= 1) {
      BVHNode leaf;
      leaf.aabb = computeAABB(edgeIds, start, end);
      leaf.left = -1;
      leaf.right = edgeIds[start];
      nodes.push_back(leaf);
      return static_cast<int>(nodes.size()) - 1;
    }
    Box aabb = computeAABB(edgeIds, start, end);
    Scalar dx = aabb.max.x - aabb.min.x;
    Scalar dy = aabb.max.y - aabb.min.y;
    int axis = (dx >= dy) ? 0 : 1;
    int mid = start + (end - start) / 2;
    if (axis == 0) {
      std::nth_element(edgeIds.begin() + start, edgeIds.begin() + mid, edgeIds.begin() + end,
        [&](int a, int b) {
          Scalar ca = (segPolyPtr->segs[a].from.x + segPolyPtr->segs[a].to.x) * 0.5;
          Scalar cb = (segPolyPtr->segs[b].from.x + segPolyPtr->segs[b].to.x) * 0.5;
          return ca < cb;
        });
    } else {
      std::nth_element(edgeIds.begin() + start, edgeIds.begin() + mid, edgeIds.begin() + end,
        [&](int a, int b) {
          Scalar ca = (segPolyPtr->segs[a].from.y + segPolyPtr->segs[a].to.y) * 0.5;
          Scalar cb = (segPolyPtr->segs[b].from.y + segPolyPtr->segs[b].to.y) * 0.5;
          return ca < cb;
        });
    }
    int leftIdx = buildRecursive(edgeIds, start, mid);
    int rightIdx = buildRecursive(edgeIds, mid, end);
    BVHNode internal;
    internal.aabb = aabb;
    internal.left = leftIdx;
    internal.right = rightIdx;
    nodes.push_back(internal);
    return static_cast<int>(nodes.size()) - 1;
  }
  Box computeAABB(const std::vector<int> &edgeIds, int start, int end) const {
    Box aabb;
    aabb.setINF();
    for (int k = start; k < end; ++k) {
      const Segment &seg = segPolyPtr->segs[edgeIds[k]];
      aabb.expand(seg.from);
      aabb.expand(seg.to);
    }
    return aabb;
  }
  void queryNearest(int nodeIdx, const Point &q,
                    int &bestEdge, Scalar &bestDist2) const {
    const BVHNode &node = nodes[nodeIdx];
    Scalar dist2B = dist2ToAABB(q, node.aabb);
    if (dist2B >= bestDist2) return;
    if (node.left < 0) {
      const Segment &seg = segPolyPtr->segs[node.right];
      Scalar d2 = dist2ToSegment(q, seg);
      if (d2 < bestDist2) {
        bestDist2 = d2;
        bestEdge = node.right;
      }
      return;
    }
    Scalar d2L = dist2ToAABB(q, nodes[node.left].aabb);
    Scalar d2R = dist2ToAABB(q, nodes[node.right].aabb);
    if (d2L <= d2R) {
      queryNearest(node.left, q, bestEdge, bestDist2);
      queryNearest(node.right, q, bestEdge, bestDist2);
    } else {
      queryNearest(node.right, q, bestEdge, bestDist2);
      queryNearest(node.left, q, bestEdge, bestDist2);
    }
  }
public:
  static Scalar dist2ToSegment(const Point &q, const Segment &seg) {
    Point d = seg.to - seg.from;
    Scalar len2 = d.norm2();
    if (len2 < EPS_DISTANCE * EPS_DISTANCE) return (q - seg.from).norm2();
    Scalar t = (q - seg.from).dot(d) / len2;
    t = std::clamp(t, 0.0, 1.0);
    Point closest = seg.from + d * t;
    return (q - closest).norm2();
  }
  static Scalar dist2ToAABB(const Point &q, const Box &aabb) {
    Scalar dx = 0, dy = 0;
    if (q.x < aabb.min.x) dx = q.x - aabb.min.x;
    else if (q.x > aabb.max.x) dx = q.x - aabb.max.x;
    if (q.y < aabb.min.y) dy = q.y - aabb.min.y;
    else if (q.y > aabb.max.y) dy = q.y - aabb.max.y;
    return dx * dx + dy * dy;
  }
  static Point nearestPointOnSegment(const Point &q, const Segment &seg) {
    Point d = seg.to - seg.from;
    Scalar len2 = d.norm2();
    if (len2 < EPS_DISTANCE * EPS_DISTANCE) return seg.from;
    Scalar t = (q - seg.from).dot(d) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return seg.from + d * t;
  }
  void build(const UnorderedPolygon &segPoly) {
    segPolyPtr = &segPoly;
    nodes.clear();
    int n = static_cast<int>(segPoly.segs.size());
    if (n == 0) { rootNodeIdx = -1; return; }
    std::vector<int> edgeIds(n);
    for (int k = 0; k < n; ++k) edgeIds[k] = k;
    rootNodeIdx = buildRecursive(edgeIds, 0, n);
  }
  int qureyMTV(const Point &q, Point &outMTV) const {
    if (rootNodeIdx < 0) { outMTV = {0, 0}; return -1; }
    int bestEdge = -1;
    Scalar bestDist2 = std::numeric_limits<Scalar>::max();
    queryNearest(rootNodeIdx, q, bestEdge, bestDist2);
    if (bestEdge < 0) { outMTV = {0, 0}; return -1; }
    Point nearestPt = nearestPointOnSegment(q, segPolyPtr->segs[bestEdge]);
    outMTV = nearestPt - q;
    return bestEdge;
  }
  void clear() {
    segPolyPtr = nullptr;
    nodes.clear();
    rootNodeIdx = -1;
  }
  bool empty() const {
    return rootNodeIdx < 0;
  }
  int size() const {
    return static_cast<int>(nodes.size());
  }
};
enum CellState {
  UNKNOWN_HOLES = -2,  UNKNOWN = -1,
  INSIDE = 0,
  BORDER = 1,
  OUTSIDE = 2,
  OUTSIDE_HOLES = 3};
using PointState = CellState;
class FastGrid {
  const UnorderedPolygon *segPolyPtr = nullptr;
  const Polygon *polyPtr = nullptr;  int numCellsX = 0;
  int numCellsY = 0;
  Scalar xMin = 0, yMin = 0, xMax = 0, yMax = 0;
  Scalar xCell = 1, yCell = 1;
  std::vector<int8_t> stateGrid;  std::vector<int16_t> edgeGrid;  void initGridRange() {
    xMin = segPolyPtr->bBox.min.x;
    yMin = segPolyPtr->bBox.min.y;
    xMax = segPolyPtr->bBox.max.x;
    yMax = segPolyPtr->bBox.max.y;
    numCellsX = NFP_GRID_SIZE;
    numCellsY = NFP_GRID_SIZE;
    xCell = (xMax - xMin) / static_cast<Scalar>(numCellsX);
    yCell = (yMax - yMin) / static_cast<Scalar>(numCellsY);
    if (xCell < EPS_DISTANCE) xCell = 1.0;
    if (yCell < EPS_DISTANCE) yCell = 1.0;
    stateGrid.resize(static_cast<size_t>(numCellsX) * numCellsY);
  }
  void fillInside() {
    std::vector<Scalar> outerLines, holesLines;
    for (int i = 0; i < numCellsX; ++i) {
      Scalar scanX = xMin + (i + 0.5) * xCell;
      lineScanY(*polyPtr, scanX, outerLines, holesLines);
      if (outerLines.size() >= 2) {
        for (size_t k = 0; k + 1 < outerLines.size(); k += 2) {
          Scalar yLow = outerLines[k];
          Scalar yHigh = outerLines[k + 1];
          int jStart = static_cast<int>(std::floor((yLow - yMin) / yCell));
          int jEnd = static_cast<int>(std::ceil((yHigh - yMin) / yCell)) - 1;
          jStart = std::clamp(jStart, 0, numCellsY - 1);
          jEnd = std::clamp(jEnd, 0, numCellsY - 1);
          for (int j = jStart; j <= jEnd; ++j) {
            stateGrid[static_cast<size_t>(j) + static_cast<size_t>(i) * numCellsY] = static_cast<int8_t>(INSIDE);
          }
        }
      }
      if (holesLines.size() >= 2) {
        for (size_t k = 0; k + 1 < holesLines.size(); k += 2) {
          Scalar yLow = holesLines[k];
          Scalar yHigh = holesLines[k + 1];
          int jStart = static_cast<int>(std::floor((yLow - yMin) / yCell));
          int jEnd = static_cast<int>(std::ceil((yHigh - yMin) / yCell)) - 1;
          jStart = std::clamp(jStart, 0, numCellsY - 1);
          jEnd = std::clamp(jEnd, 0, numCellsY - 1);
          for (int j = jStart; j <= jEnd; ++j) {
            stateGrid[static_cast<size_t>(j) + static_cast<size_t>(i) * numCellsY] = static_cast<int8_t>(OUTSIDE_HOLES);
          }
        }
      }
    }
  }
  void rasterizeSegment(const Segment &seg, std::unordered_set<int> &hitCells) const {
    Scalar x0 = seg.from.x, y0 = seg.from.y;
    Scalar x1 = seg.to.x, y1 = seg.to.y;
    Scalar dx = x1 - x0, dy = y1 - y0;
    Scalar segLen = std::sqrt(dx * dx + dy * dy);
    if (segLen < EPS_DISTANCE) {
      int ci = static_cast<int>(std::floor((x0 - xMin) / xCell));
      int cj = static_cast<int>(std::floor((y0 - yMin) / yCell));
      ci = std::clamp(ci, 0, numCellsX - 1);
      cj = std::clamp(cj, 0, numCellsY - 1);
      hitCells.insert(cj + ci * numCellsY);
      return;
    }
    Scalar xLoAll = std::min(x0, x1), xHiAll = std::max(x0, x1);
    Scalar yLoAll = std::min(y0, y1), yHiAll = std::max(y0, y1);
    int iL = static_cast<int>(std::floor((xLoAll - xMin) / xCell));
    int iR = static_cast<int>(std::floor((xHiAll - xMin) / xCell));
    iL = std::clamp(iL, 0, numCellsX - 1);
    iR = std::clamp(iR, 0, numCellsX - 1);
    if (iL > iR) std::swap(iL, iR);
    bool nearlyVertical = (std::abs(dx) < EPS_DISTANCE);
    for (int i = iL; i <= iR; ++i) {
      Scalar xSliceLo = std::max(xMin + i * xCell, xLoAll);
      Scalar xSliceHi = std::min(xMin + (i + 1) * xCell, xHiAll);
      Scalar ySegLo, ySegHi;
      if (nearlyVertical) {
        ySegLo = yLoAll;
        ySegHi = yHiAll;
      } else {
        Scalar tLo = (xSliceLo - x0) / dx;
        Scalar tHi = (xSliceHi - x0) / dx;
        Scalar yAtLo = y0 + dy * tLo;
        Scalar yAtHi = y0 + dy * tHi;
        ySegLo = std::min(yAtLo, yAtHi);
        ySegHi = std::max(yAtLo, yAtHi);
      }
      int jLo = static_cast<int>(std::floor((ySegLo - yMin) / yCell));
      int jHi = static_cast<int>(std::ceil((ySegHi - yMin) / yCell)) - 1;
      jLo = std::clamp(jLo, 0, numCellsY - 1);
      jHi = std::clamp(jHi, 0, numCellsY - 1);
      for (int j = jLo; j <= jHi; ++j) {
        hitCells.insert(j + i * numCellsY);
      }
    }
  }
  void markBorderBand() {
    std::unordered_set<int> hitCells;
    {
      hitCells.clear();
      hitCells.reserve(polyPtr->outer.vertices.size() * 4);
      int n = static_cast<int>(polyPtr->outer.vertices.size());
      for (int k = 0; k < n; ++k) {
        Segment seg(polyPtr->outer.vertices[k], polyPtr->outer.vertices[(k + 1) % n]);
        rasterizeSegment(seg, hitCells);
      }
      for (int idx : hitCells) {
        int ci = idx / numCellsY;
        int cj = idx % numCellsY;
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            int ni = ci + di;
            int nj = cj + dj;
            if (ni >= 0 && ni < numCellsX && nj >= 0 && nj < numCellsY) {
              stateGrid[static_cast<size_t>(nj) + static_cast<size_t>(ni) * numCellsY] = static_cast<int8_t>(UNKNOWN);
            }
          }
        }
      }
    }
    for (const auto &hole : polyPtr->holes) {
      hitCells.clear();
      hitCells.reserve(hole.vertices.size() * 4);
      int n = static_cast<int>(hole.vertices.size());
      for (int k = 0; k < n; ++k) {
        Segment seg(hole.vertices[k], hole.vertices[(k + 1) % n]);
        rasterizeSegment(seg, hitCells);
      }
      for (int idx : hitCells) {
        int ci = idx / numCellsY;
        int cj = idx % numCellsY;
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            int ni = ci + di;
            int nj = cj + dj;
            if (ni >= 0 && ni < numCellsX && nj >= 0 && nj < numCellsY) {
              stateGrid[static_cast<size_t>(nj) + static_cast<size_t>(ni) * numCellsY] = static_cast<int8_t>(UNKNOWN_HOLES);
            }
          }
        }
      }
    }
  }
  void fillEdgeGrid() {
    BVH bvh;
    bvh.build(*segPolyPtr);
    edgeGrid.resize(static_cast<size_t>(numCellsX) * numCellsY, -1);
    for (int i = 0; i < numCellsX; ++i) {
      for (int j = 0; j < numCellsY; ++j) {
        int idx = j + i * numCellsY;
        if (stateGrid[idx] == static_cast<int8_t>(OUTSIDE) || stateGrid[idx] == static_cast<int8_t>(OUTSIDE_HOLES)) {
          edgeGrid[idx] = -1;
        } else {
          Scalar cx = xMin + (i + 0.5) * xCell;
          Scalar cy = yMin + (j + 0.5) * yCell;
          Point center(cx, cy);
          Point dummyMTV;
          edgeGrid[idx] = static_cast<int16_t>(bvh.qureyMTV(center, dummyMTV));
        }
      }
    }
  }
public:
  void build(Polygon &segPoly, UnorderedPolygon &segPolyU) {
    polyPtr = &segPoly;
    segPolyPtr = &segPolyU;
    initGridRange();
    std::fill(stateGrid.begin(), stateGrid.end(), static_cast<int8_t>(OUTSIDE));
    fillInside();
    markBorderBand();
    fillEdgeGrid();
  }
  PointState queryState(const Point &p) const {
    int idx = getIndex(p);
    int8_t state = stateGrid[idx];
    if (state == INSIDE) return PointState::INSIDE;
    if (state == OUTSIDE) return PointState::OUTSIDE;
    if (state == OUTSIDE_HOLES) return PointState::OUTSIDE_HOLES;
    if (state == UNKNOWN_HOLES) {
      return segPolyPtr->contains(p) ? PointState::INSIDE : PointState::OUTSIDE_HOLES;
    }
    return segPolyPtr->contains(p) ? PointState::INSIDE : PointState::OUTSIDE;
  }
  int qureyMTV(const Point &p, Point &outMTV) const {
    int idx = getIndex(p);
    int edgeId = edgeGrid[idx];
    if (edgeId < 0) { outMTV = {0, 0}; return -1; }
    const Segment &seg = segPolyPtr->segs[edgeId];
    Point nearestPt = BVH::nearestPointOnSegment(p, seg);
    outMTV = nearestPt - p;
    return edgeId;
  }
  void clear() {
    segPolyPtr = nullptr;
    polyPtr = nullptr;
    numCellsX = numCellsY = 0;
    xMin = yMin = xMax = yMax = 0;
    xCell = yCell = 1;
    stateGrid.clear();
    edgeGrid.clear();
  }
  int getEdgeId(int i, int j) const {
    return edgeGrid[j + i * numCellsY];
  }
  const std::vector<int8_t>& getStateGrid() const { return stateGrid; }
  Scalar getXCell() const { return xCell; }
  Scalar getYCell() const { return yCell; }
  int getIndex(const Point& p) const {
    int i = static_cast<int>(std::floor((p.x - xMin) / xCell));
    int j = static_cast<int>(std::floor((p.y - yMin) / yCell));
    i = std::clamp(i, 0, numCellsX - 1);
    j = std::clamp(j, 0, numCellsY - 1);
    return j + i * numCellsY;
  }
  Scalar getXMin() const { return xMin; }
  Scalar getYMin() const { return yMin; }
  Scalar getXMax() const { return xMax; }
  Scalar getYMax() const { return yMax; }
  int getNumCellsX() const { return numCellsX; }
  int getNumCellsY() const { return numCellsY; }
};
inline bool isOpenInner(const Polygon &pa, const Polygon &pb) {
  Scalar sA = (pa.bBox.max.x - pa.bBox.min.x)*(pa.bBox.max.y - pa.bBox.min.y);
  Scalar sB = (pb.bBox.max.x - pb.bBox.min.x)*(pb.bBox.max.y - pb.bBox.min.y);
  int vertexCountA = static_cast<int>(pa.outer.vertices.size());
  int vertexCountB = static_cast<int>(pb.outer.vertices.size());
  bool sizeCondition = sA < sB*OPEN_INNER_AREA_RATIO ||  sB < sA*OPEN_INNER_AREA_RATIO ;
  bool vertexCondition = vertexCountA*vertexCountB <= OPEN_INNER_VERTEX_NUMBER;
  return sizeCondition || vertexCondition;
}
class NFPMatrix {
  int polyNum = 0;
  std::vector<UnorderedPolygon> nfpMatrix;
  std::vector<Polygon> nfpPolysMatrix;
  std::vector<FastGrid> gridMatrix;
  std::vector<Polygon> polys;
  int failuresNum = 0;
  mutable std::vector<bool> accessed;  void computeUpperTriangle(const std::vector<Polygon>& polys) {
    NFPSolver solver;
    for (int i = 0; i < polyNum; ++i) {
      for (int j = i + 1; j < polyNum; ++j) {
        bool openInner = OPEN_INNER && isOpenInner(polys[i], polys[j]) ;
        nfpPolysMatrix[getIndex(i, j)] = solver.genNFP(polys[i].outer, polys[j].outer, openInner);
        std::vector<Segment> nfpSegs = solver.getNFPSegments();
        UnorderedPolygon segPoly;
        segPoly.setSegments(nfpSegs);
        if(segPoly.empty()) failuresNum++;
        nfpMatrix[getIndex(i, j)] = std::move(segPoly);
      }
    }
  }
  void buildGridIndex() {
    for (int i = 0; i < polyNum; ++i) {
      for (int j = i + 1; j < polyNum; ++j) {
        gridMatrix[getIndex(i, j)].build(nfpPolysMatrix[getIndex(i, j)], nfpMatrix[getIndex(i, j)]);
      }
    }
  }
  bool overlapByRay(int i, int j, const Point &a, const Point &b) const {
    if (i == j) return false;
    if (i < 0 || j < 0 || i >= polyNum || j >= polyNum) return false;
    const Polygon& polyI = polys[i];
    const Polygon& polyJ = polys[j];
    Box boxI = polyI.bBox;
    Box boxJ = polyJ.bBox;
    boxI.min = boxI.min + a; boxI.max = boxI.max + a;
    boxJ.min = boxJ.min + b; boxJ.max = boxJ.max + b;
    if (!boxesOverlap(boxI, boxJ)) return false;
    Point shift = a - b;
    for (const auto& v : polyI.outer.vertices) {
      Point pTest = v + shift;
      if (polyJ.containsPointStrict(pTest)) return true;
    }
    for (const auto& v : polyJ.outer.vertices) {
      Point pTest = v - shift;
      if (polyI.containsPointStrict(pTest)) return true;
    }
    if (isEdgesIntersected(a, b, polyI.outer, polyJ.outer)) 
        return true;
    return false;
  }
  inline void accessedNFP(int idx) const {
    // 访问标记
    }
public:
  void build(const std::vector<Polygon> &polys) {
    this->polys = polys;
    polyNum = static_cast<int>(polys.size());
    nfpMatrix.resize(polyNum * polyNum);
    nfpPolysMatrix.resize(polyNum * polyNum);
    gridMatrix.resize(polyNum * polyNum);
    accessed.resize(polyNum * polyNum, false);
    computeUpperTriangle(polys);
    buildGridIndex();
  }
  int getIndex(int i, int j) const {
    return j + i * polyNum;
  }
  bool contains(int i, int j, const Point& p) const {
    if (i == j) return false;
    if (i < 0 || j < 0 || i >= polyNum || j >= polyNum) return false;
    if (i < j) {
      int idx = getIndex(i, j);
      accessedNFP(idx);
      return gridMatrix[idx].queryState(p) == PointState::INSIDE;
    } else {
      int idx = getIndex(j, i);
      accessedNFP(idx);
      Point negP(-p.x, -p.y);
      return gridMatrix[idx].queryState(negP) == PointState::INSIDE;
    }
  }
  bool containsTrans(int i, int j,
                     const Point &a, const Point &b,
                     const Point p = {0.0, 0.0}) const {
    if (i == j) return false;
    if (i < 0 || j < 0 || i >= polyNum || j >= polyNum) return false;
    int idx = (i < j) ? getIndex(i, j) : getIndex(j, i);
    if (!nfpMatrix[idx].empty()) {
      Point pShift = p - (a - b);
      return contains(i, j, pShift);
    }
    Point pInI = p - a;
    Point pInJ = p - b;
    bool inPolyI = polys[i].containsPointStrict(pInI);
    bool inPolyJ = polys[j].containsPointStrict(pInJ);
    return inPolyI && inPolyJ; 
  }
  int getfailuresNum()const{ return failuresNum;}
  bool hasValidNFP(int i, int j) const {
    if (i == j) return false;
    if (i < 0 || j < 0 || i >= polyNum || j >= polyNum) return false;
    int idx = (i < j) ? getIndex(i, j) : getIndex(j, i);
    return !nfpMatrix[idx].empty();
  }
  bool overlap(int i, int j, const Point &a, const Point &b, Point &mtv, bool &mtvValid, bool &isolated) {
    if (i == j) { mtv = {0, 0}; mtvValid = true; isolated = false; return false; }
    int idx = (i < j) ? getIndex(i, j) : getIndex(j, i);
    if (nfpMatrix[idx].empty()) {
      mtvValid = false;
      isolated = false;
      bool isOverlap = overlapByRay(i,j,a,b);
      if (!isOverlap) mtv = {0, 0};
      return isOverlap;
    }
    Point q = (b-a);
    Point queryPt;
    const FastGrid *grid;
    if (i < j) {
      int gridIdx = getIndex(i, j);
      accessedNFP(gridIdx);
      queryPt = q;
      grid = &gridMatrix[gridIdx];
    } else {
      int gridIdx = getIndex(j, i);
      accessedNFP(gridIdx);
      queryPt = Point(-q.x, -q.y);
      grid = &gridMatrix[gridIdx];
    }
    PointState state = grid->queryState(queryPt);
    if (state == PointState::INSIDE) {
      isolated = false;
      grid->qureyMTV(queryPt, mtv);
      if (i > j) { mtv = Point(-mtv.x, -mtv.y); }
      mtvValid = true;
      return true;
    }
    mtv = {0, 0}; mtvValid = true;
    isolated = (state == PointState::OUTSIDE_HOLES);
    return false;
  }
  bool overlap(int i, int j, const Point &a, const Point &b, bool &isolated) {
    if (i == j) { isolated = false; return false; }
    int idx = (i < j) ? getIndex(i, j) : getIndex(j, i);
    if (nfpMatrix[idx].empty()) {
      isolated = false;
      return overlapByRay(i,j,a,b);
    }
    Point q = (b-a);
    Point queryPt;
    const FastGrid *grid;
    if (i < j) {
      int gridIdx = getIndex(i, j);
      accessedNFP(gridIdx);
      queryPt = q;
      grid = &gridMatrix[gridIdx];
    } else {
      int gridIdx = getIndex(j, i);
      accessedNFP(gridIdx);
      queryPt = Point(-q.x, -q.y);
      grid = &gridMatrix[gridIdx];
    }
    PointState state = grid->queryState(queryPt);
    if (state == PointState::INSIDE) { isolated = false; return true; }
    isolated = (state == PointState::OUTSIDE_HOLES);
    return false;
  }
  const UnorderedPolygon& getNFP(int i, int j) const {
    return nfpMatrix[getIndex(i, j)];
  }
  const FastGrid& getGrid(int i, int j) const {
    return gridMatrix[getIndex(i, j)];
  }
  void clear() {
    polyNum = 0;
    nfpMatrix.clear();
    nfpPolysMatrix.clear();
    gridMatrix.clear();
  }
  int getPolyNum() const { return polyNum; }
  int getTotalConstructionCount() const { return polyNum * (polyNum - 1) / 2; }
  int getAccessedCount() const {
    int count = 0;
    for (int i = 0; i < polyNum; ++i) {
      for (int j = i + 1; j < polyNum; ++j) {
        if (accessed[getIndex(i, j)]) ++count;
      }
    }
    return count;
  }
  double getAccessRate() const {
    int total = getTotalConstructionCount();
    if (total == 0) return 0.0;
    return static_cast<double>(getAccessedCount()) / total;
  }
};
}
