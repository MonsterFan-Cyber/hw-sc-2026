// Merged by merge.py on 2026-05-16T03:43:45.265224Z
// Source dir: /home/zyh/桌面/xianchang_V3/hw_sec_2026_v3
// Merged files:
//  - src/0.h
//  - src/debug.h
//  - src/off_log.h
//  - src/polygon.h
//  - src/trajgrid.h
//  - src/trajgraph.h
//  - src/nfp.h
//  - src/query.h
//  - src/poly_contain.h
//  - src/lg.h
//  - src/op.h
//  - src/sa.h
//  - src/main.cpp

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ==== Begin src/0.h ====
const double EXPAND_EPS    = 4e-5;
const double AABB_PREC     = 1e-4;
const double EXACT_PREC    = 1e-5;
const double SA_PERM_ALPHA = 0.9999;
const double SA_TMIN       = 1e-6;
const double WALL_EPS      = 1e-4;
const double TIME_SCALE = 1;
const double JUMP_SIZE_WEIGHT = 1.0;
const double T_SA_PERM      = 22.0*TIME_SCALE ;
const double T_AABB_PULL    = 23.0*TIME_SCALE;
const double T_SA_HIGH_END  = 46.0*TIME_SCALE;
const double T_EXACT_MID    = 47.5*TIME_SCALE;
const double T_SA_LOW_END   = 57.0*TIME_SCALE;
const double T_EXACT_FINAL  = 59.5*TIME_SCALE;
const double JUMP_BEGIN_PROG_HIGH = 0.5;
const double JUMP_BACK_PROB_HIGH = 0.1;
const double MOVE_BACK_PROB_HIGH = 0.25;
const int RONND_PRE_CYC = 2500;
const bool OPEN_INNER = true;
const double OPEN_INNER_AREA_RATIO = 0.1;
const double OPEN_INNER_VERTEX_NUMBER = 50*50;
// ==== End src/0.h ====

// ==== Begin src/debug.h ====
extern double elapsed();
extern long long g_cntGetCandidates, g_sumCands, g_cntFastOverlap, g_cntExactOverlap, g_cntShelfPlace;
inline long long g_cntExactOverlap  = 0;
inline long long g_cntFastOverlap   = 0;
inline long long g_cntGetCandidates = 0;
inline long long g_sumCands         = 0;
inline long long g_cntShelfPlace    = 0;
#ifdef DEBUG_LOG
struct DebugSAStats {
    long long dbgIter;
    double    dbgBestL;
    long long snapSwapTrials, snapSwapAccepts;
    long long snapMoveTrials, snapMoveAccepts;
    long long snapSlideTrials, snapSlideAccepts;
    long long snapJumpTrials, snapJumpAccepts;
    long long snapRandomJumpTrials, snapRandomJumpAccepts;
    long long snapIters;
    double    lastT, lastStep;
    double    lastSnapshotTime;
    const char* phaseName;
};
#endif
#ifdef DEBUG_LOG
  #define DEBUG_SA_STATS_PARAM  , DebugSAStats& dbg
  #define DEBUG_SA_STATS_ARG    , dbg
#else
  #define DEBUG_SA_STATS_PARAM
  #define DEBUG_SA_STATS_ARG
#endif
#ifdef DEBUG_LOG
static FILE* g_logFile = nullptr;
static void logInitFile() {
    g_logFile = fopen("debug_log.txt", "w");
    if (!g_logFile) { fprintf(stderr, "[DEBUG_LOG] 无法创建 debug_log.txt\n"); return; }
    fprintf(g_logFile,
        "# debug_log.txt — 多边形重叠消除调试日志 v3\n"
        "# phase / iter / round / elapsed / cur_L / best_L / tag\n#\n");
    fflush(g_logFile);
}
static void logCloseFile() {
    if (g_logFile) { fflush(g_logFile); fclose(g_logFile); g_logFile = nullptr; }
}
static void logPhaseHeader(const char* phaseName) {
    if (!g_logFile) return;
    fprintf(g_logFile,
        "\n================================================\nPhase: %s\n================================================\n",
        phaseName);
    fflush(g_logFile);
}
static void logLine(const char* phase, long long iter, int round,
                    double elapsedSec, double curL, double bestL, const char* tag) {
    if (!g_logFile) return;
    if (iter >= 0 && round >= 0)
        fprintf(g_logFile, "phase=%-24s  iter=%-10lld  round=%-4d  elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, iter, round, elapsedSec, curL, bestL, tag);
    else if (iter >= 0)
        fprintf(g_logFile, "phase=%-24s  iter=%-10lld             elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, iter, elapsedSec, curL, bestL, tag);
    else
        fprintf(g_logFile, "phase=%-24s  round=%-4d               elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, round, elapsedSec, curL, bestL, tag);
    fflush(g_logFile);
}
static void logPhaseSummary(const char* phaseName, long long totalIter, int totalRound, double elapsedSec) {
    if (!g_logFile) return;
    if (totalIter >= 0)
        fprintf(g_logFile, ">>> %s  total_iters=%lld  elapsed_at_end=%.4f\n", phaseName, totalIter, elapsedSec);
    else
        fprintf(g_logFile, ">>> %s  total_rounds=%d  elapsed_at_end=%.4f\n", phaseName, totalRound, elapsedSec);
    fflush(g_logFile);
}
static FILE* g_timerFile = nullptr;
template<typename B>
static void timerInitFile(int n, const B& bd) {
    g_timerFile = fopen("timer_log.txt", "w");
    if (!g_timerFile) { fprintf(stderr, "[DEBUG_LOG] 无法创建 timer_log.txt\n"); return; }
    time_t now_t = time(nullptr);
    struct tm* tm_info = localtime(&now_t);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info);
    double bw = bd.xMax - bd.xMin;
    double bh = bd.yMax - bd.yMin;
    fprintf(g_timerFile,
        "\n"
        "════════════════════════════════════════════════════════\n"
        "  Run start : %s\n"
        "  n         = %d 个多边形\n"
        "  boundary  = x∈[%.4f, %.4f]  y∈[%.4f, %.4f]\n"
        "  宽×高     = %.4f × %.4f  面积=%.4f\n"
        "════════════════════════════════════════════════════════\n"
        "%-12s  %-28s  %s\n"
        "%-12s  %-28s  %s\n",
        time_str,
        n,
        bd.xMin, bd.xMax, bd.yMin, bd.yMax,
        bw, bh, bw * bh,
        "elapsed(s)", "event", "detail",
        "------------", "----------------------------", "------"
    );
    fflush(g_timerFile);
}
static void timerCloseFile() {
    if (g_timerFile) { fflush(g_timerFile); fclose(g_timerFile); g_timerFile = nullptr; }
}
static void timerPhaseStart(const char* name, double lBefore) {
    if (!g_timerFile) return;
    fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s  L'_before=%12.6f\n",
            elapsed(), name, lBefore);
    fflush(g_timerFile);
}
static void timerPhaseEnd(const char* name, double tStart, double lBefore, double lAfter,
                          long long iters = -1, int rounds = -1) {
    if (!g_timerFile) return;
    double now = elapsed();
    double cost = now - tStart;
    double gain = lBefore - lAfter;
    double gainPct = (lBefore > 1e-9) ? gain / lBefore * 100.0 : 0.0;
    if (iters >= 0)
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)  iters=%lld\n",
            now, name, cost, lBefore, lAfter, gain, gainPct, iters);
    else if (rounds >= 0)
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)  rounds=%d\n",
            now, name, cost, lBefore, lAfter, gain, gainPct, rounds);
    else
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)\n",
            now, name, cost, lBefore, lAfter, gain, gainPct);
    fflush(g_timerFile);
}
static void timerRound(const char* funcName, int round,
                       double tRoundStart, double lBefore, double lAfter) {
    if (!g_timerFile) return;
    double now = elapsed();
    double roundCost = now - tRoundStart;
    double gain = lBefore - lAfter;
    fprintf(g_timerFile,
        "%12.4f  [ROUND  r=%-4d] %-20s  round_cost=%6.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f\n",
        now, round, funcName, roundCost, lBefore, lAfter, gain);
    fflush(g_timerFile);
}
static void timerSASnapshot(const char* phaseName, double T, double step,
                            double curCost, double bestCost, long long iters,
                            long long swapTrials, long long swapAccepts,
                            long long moveTrials, long long moveAccepts,
                            long long slideTrials = 0, long long slideAccepts = 0,
                            long long jumpTrials = 0, long long jumpAccepts = 0) {
    if (!g_timerFile) return;
    double swapRate  = (swapTrials  > 0) ? swapAccepts  * 100.0 / swapTrials  : 0.0;
    double moveRate  = (moveTrials  > 0) ? moveAccepts  * 100.0 / moveTrials  : 0.0;
    double slideRate = (slideTrials > 0) ? slideAccepts * 100.0 / slideTrials : 0.0;
    double jumpRate  = (jumpTrials  > 0) ? jumpAccepts  * 100.0 / jumpTrials  : 0.0;
    fprintf(g_timerFile,
        "%12.4f  [SNAPSHOT     ] %-20s"
        "  T=%10.6f  step=%8.6f"
        "  curCost=%12.6f  bestCost=%12.6f  iters=%9lld"
        "  swap_acc=%.1f%%(%lld/%lld)"
        "  move_acc=%.1f%%(%lld/%lld)"
        "  slide_acc=%.1f%%(%lld/%lld)"
        "  jump_acc=%.1f%%(%lld/%lld)\n",
        elapsed(), phaseName,
        T, step,
        curCost, bestCost, iters,
        swapRate, swapAccepts, swapTrials,
        moveRate, moveAccepts, moveTrials,
        slideRate, slideAccepts, slideTrials,
        jumpRate, jumpAccepts, jumpTrials);
    fflush(g_timerFile);
}
static void timerHotspotSummary() {
    if (!g_timerFile) return;
    double avgCands = (g_cntGetCandidates > 0) ? (double)g_sumCands / g_cntGetCandidates : 0.0;
    fprintf(g_timerFile,
            "\n%s\n"
            "# [HOTSPOT] 热点操作统计\n"
            "%s\n"
            "  fastOverlap   calls : %lld\n"
            "  exactOverlap  calls : %lld\n"
            "  getCandidates calls : %lld  (avg_cands_per_call=%.2f)\n"
            "  shelfPlace    calls : %lld\n",
            "================================================",
            "================================================",
            g_cntFastOverlap,
        g_cntExactOverlap,
        g_cntGetCandidates, avgCands,
        g_cntShelfPlace);
    fflush(g_timerFile);
}
#endif// ==== End src/debug.h ====

// ==== Begin src/off_log.h ====
#define TL_SET_OUTPUT(path) (void)0
#define TL_START(label) (void)0
#define TL_ATTACH(label) (void)0
#define TL_END() (void)0
#define TL_SCOPE(label) (void)0
// ==== End src/off_log.h ====

// ==== Begin src/polygon.h ====
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
inline bool geDist(Scalar a, Scalar b) {   return a -b >   -EPS_DISTANCE; }
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
// ==== End src/polygon.h ====

// ==== Begin src/trajgrid.h ====
namespace traj {
using namespace geom;
const Scalar kCellSizeX = 0.02, kCellSizeY = 0.02;
const Scalar kCellSizeFactorXColor = 0.1, kCellSizeFactorYColor = 0.1;
const Scalar kCellSizeFactorXOuter = 0.6, kCellSizeFactorYOuter = 0.6;
const int kMaxGridCellsNumX = 128, kMaxGridCellsNumY = 128; 
const int kSimpleSegmentThreshold = 1000;
const int kColorMethodThreshold = 1000000;
struct IntersectionInfo
{
    int id;    Scalar t1;
    Scalar t2;
    Scalar angle;
    bool used {false};
};
struct DirectedSegment
{
    DirectedSegment()
        : from(0, 0), to(0, 0), mid(0, 0), 
          bdBox(from, to), dir(0, 0), len(0), len2(0), alpha(0) {}
    DirectedSegment(const Point& f, const Point& t)
        : from(f), to(t), mid((f + t) * 0.5), 
          bdBox(from, to), dir(t - f), len(dir.norm()), len2(dir.norm2()), alpha(dir.angle()) {}
    Point from;
    Point to;
    Box bdBox;
    Point dir;
    Point mid;
    Scalar len;
    Scalar len2;
    Scalar alpha;
    Point direction() const { return dir; }
    Point midpoint() const { return mid; }
    Scalar length() const { return len; }
    Scalar angle() const { return alpha; }
    Scalar angleTo(const DirectedSegment& other) const
    {
        Scalar diff = other.alpha - alpha;
        if (diff < -PI) diff += PI_PI;
        if (diff >= PI) diff -= PI_PI;
        return diff;
    }
    DirectedSegment getFromRange(Scalar tStart, Scalar tEnd) const { return DirectedSegment(from + dir * tStart, from + dir * tEnd); }
    bool hasPointOnLine(const Point& p, Scalar& t, Scalar eps = EPS_DISTANCE) const
    {
        Point ap = p - from;
        Point ab = dir;
        t = ap.dot(ab) / len2;
        Point proj = from + ab * t;
        return (proj - p).norm() < eps;
    }
    bool isIntersecting(const DirectedSegment& other, Scalar& t1, Scalar& t2, Scalar eps_angle = EPS_ANGLE, Scalar eps_distance = EPS_DISTANCE) const
    {
        if (bdBox.max.x - other.bdBox.min.x < -eps_distance || other.bdBox.max.x - bdBox.min.x < -eps_distance
            || bdBox.max.y - other.bdBox.min.y < -eps_distance || other.bdBox.max.y - bdBox.min.y < -eps_distance)
        {
            return false;        }
        Point d1 = dir;
        Point d2 = other.dir;
        Point delta = other.from - from;
        Scalar denom = d1.cross(d2);
        if (std::abs(denom) < eps_angle)
        {
            if (hasPointOnLine(other.to, t1, eps_distance) && t1 - 1.0 > eps_distance)
            {
                if (other.hasPointOnLine(to, t2, eps_distance) && t2 > -eps_distance && 1.0 - t2 > eps_distance)
                {
                    t1 = 1.0;
                    return true;
                }
                else return false;
            }
            else
                return false;
        }
        t1 = delta.cross(d2) / denom;
        t2 = delta.cross(d1) / denom;
        return (t1 > -eps_distance && 1.0 - t1 > -eps_distance
                && t2 > -eps_distance && 1.0 - t2 > -eps_distance);
    }
    Scalar distanceToPoint(const Point& p) const
    {
        Point ap = p - from;
        Point ab = dir;
        if (len < EPS_DISTANCE) {
            return ap.norm();        }
        Scalar t = ap.dot(ab) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        Point proj = from + ab * t;
        return (proj - p).norm();
    }
    DirectedSegment getOverlapSegmentWithBox(const Box& box, bool& isOverlapped) const
    {
        isOverlapped = false;
        if (box.contains(from, true) && box.contains(to, true)) {
            isOverlapped = true;
            return *this;
        }
        Scalar tMin = 0.0, tMax = 1.0;
        Point d = dir;
        if (std::abs(d.x) > EPS_DISTANCE) {
            Scalar tx1 = (box.min.x - from.x) / d.x;
            Scalar tx2 = (box.max.x - from.x) / d.x;
            if (tx1 > tx2) std::swap(tx1, tx2);
            tMin = std::max(tMin, tx1);
            tMax = std::min(tMax, tx2);
        } else if (from.x < box.min.x || from.x > box.max.x) {
            return DirectedSegment();        }
        if (std::abs(d.y) > EPS_DISTANCE) {
            Scalar ty1 = (box.min.y - from.y) / d.y;
            Scalar ty2 = (box.max.y - from.y) / d.y;
            if (ty1 > ty2) std::swap(ty1, ty2);
            tMin = std::max(tMin, ty1);
            tMax = std::min(tMax, ty2);
        } else if (from.y < box.min.y || from.y > box.max.y) {
            return DirectedSegment();        }
        if (tMin > tMax || tMin > 1.0 || tMax < 0.0) {
            return DirectedSegment();        }
        isOverlapped = true;
        return getFromRange(std::max(tMin, 0.0), std::min(tMax, 1.0));
    }
    Box boundingBox() const
    {
        return bdBox;
    }
    Scalar xCutWithLine(Scalar x) const
    {
        if (std::abs(dir.x) < EPS_DISTANCE) return std::numeric_limits<Scalar>::infinity();        Scalar t = (x - from.x) / dir.x;
        return from.y + dir.y * t;
    }
    Scalar yCutWithLine(Scalar y) const
    {
        if (std::abs(dir.y) < EPS_DISTANCE) return std::numeric_limits<Scalar>::infinity();        Scalar t = (y - from.y) / dir.y;
        return from.x + dir.x * t;
    }
    bool isPointOnLeftSide(const Point& p, Scalar eps = EPS_ANGLE) const
    {
        Point ap = p - from;
        Point ab = dir;
        Scalar cross = ab.cross(ap);
        if (cross > eps) return true;        if (cross < -eps) return false;        return false;    }
    DirectedSegment reverse() const
    {
        return DirectedSegment(to, from);
    }
    Point pointAt(Scalar t) const
    {
        return from + dir * t;
    }
};
class Grid2D
{
public:
    enum INDEX_MODE
    {
        LOWER,
        ROUND,
        UPPER
    };
    Grid2D(): boundingBox(), cellSizeX(0), cellSizeY(0), nx(0), ny(0), totalSize(0) {}
    void init(const Box& box, Scalar dx, Scalar dy);
    void clear() { boundingBox.clear(); cellSizeX = cellSizeY = 0; nx = ny = totalSize = 0; xCoords.clear(); yCoords.clear(); }
    void segmentToCells(std::vector<int>& cells, const DirectedSegment& segment) const;
    int getXIndex(Scalar x, INDEX_MODE mode) const
    {
        int roundIndex = static_cast<int>(std::round((x - boundingBox.min.x) / cellSizeX));
        if (roundIndex < 0 || roundIndex > nx) {
            return -1;        }
        int index;
        switch (mode) {
            case LOWER:
            {
                Scalar roundX = getX(roundIndex);
                if (roundX - x > -EPS_DISTANCE)
                {
                    return roundIndex;
                }
                else
                {
                    return roundIndex + 1;
                }
            }
            case ROUND:
                return roundIndex;
            case UPPER:
            {
                Scalar roundX = getX(roundIndex);
                if (x - roundX > -EPS_DISTANCE)
                {
                    return roundIndex;
                }
                else
                {
                    return roundIndex - 1;
                }
            }
            default:
                return roundIndex;
        }
    }
    int getYIndex(Scalar y, INDEX_MODE mode) const
    {
        int roundIndex = static_cast<int>(std::round((y - boundingBox.min.y) / cellSizeY));
        if (roundIndex < 0 || roundIndex > ny) {
            return -1;        }
        switch (mode) {
            case LOWER:
            {
                Scalar roundY = getY(roundIndex);
                if (roundY - y > -EPS_DISTANCE)
                {
                    return roundIndex;
                }
                else
                {
                    return roundIndex + 1;
                }
            }
            case ROUND:
                return roundIndex;
            case UPPER:
            {
                Scalar roundY = getY(roundIndex);
                if (y - roundY > -EPS_DISTANCE)
                {
                    return roundIndex;
                }
                else
                {
                    return roundIndex - 1;
                }
            }
            default:
                return roundIndex;
        }
    }
    Point getCellCenter(int cellIndex) const
    {
        int i = cellIndex / ny;
        int j = cellIndex % ny;
        return {getX(i) + cellSizeX * 0.5, getY(j) + cellSizeY * 0.5};
    }
    int getCellIndex(int i, int j) const
    {
        if (i < 0 || i >= nx || j < 0 || j >= ny) {
            return -1;        }
        return i * ny + j;
    }
    Box getCellBox(int cellIndex) const
    {
        int i = cellIndex / ny;
        int j = cellIndex % ny;
        return Box{getX(i), getX(i + 1), getY(j), getY(j + 1)};
    }
    Scalar getX(int i) const { return xCoords[i]; }
    Scalar getY(int j) const { return yCoords[j]; }
    int size() const { return totalSize; }
    int sizeX() const { return nx; }
    int sizeY() const { return ny; }
private:
    Box boundingBox;
    Scalar cellSizeX;
    Scalar cellSizeY;
    int nx;
    int ny;
    std::vector<Scalar> xCoords;
    std::vector<Scalar> yCoords;
    int totalSize;
};
void Grid2D::init(const Box& box, Scalar dx, Scalar dy)
{
    boundingBox = box;
    cellSizeX = dx;
    cellSizeY = dy;
    nx = static_cast<int>(std::ceil((boundingBox.max.x - boundingBox.min.x) / cellSizeX));
    ny = static_cast<int>(std::ceil((boundingBox.max.y - boundingBox.min.y) / cellSizeY));
    totalSize = nx * ny;
    cellSizeX = (boundingBox.max.x - boundingBox.min.x) / static_cast<Scalar>(nx);
    cellSizeY = (boundingBox.max.y - boundingBox.min.y) / static_cast<Scalar>(ny);
    xCoords.resize(nx + 1);
    yCoords.resize(ny + 1);
    for (int i = 0; i <= nx; ++i) {
        xCoords[i] = boundingBox.min.x + i * cellSizeX;
    }
    for (int j = 0; j <= ny; ++j) {
        yCoords[j] = boundingBox.min.y + j * cellSizeY;
    }
}
void Grid2D::segmentToCells(std::vector<int>& cells, const DirectedSegment& segment) const
{
    cells.clear();
    if (nx <= 0 || ny <= 0) return;
    const Point& a = segment.from;
    const Point& b = segment.to;
    const Scalar dx = b.x - a.x;
    const Scalar dy = b.y - a.y;
    std::unordered_set<int> uniqueCells;
    uniqueCells.reserve(32);
    auto addCell = [&](int i, int j) {
        if (i < 0 || i >= nx || j < 0 || j >= ny) return;
        uniqueCells.insert(i * ny + j);
    };
    auto lineIndex = [&](Scalar coord, Scalar minCoord, Scalar h, int maxLineIdx) -> int {
        Scalar r = (coord - minCoord) / h;
        int k = static_cast<int>(std::llround(r));
        if (k < 0 || k > maxLineIdx) return -1;
        Scalar onLineCoord = minCoord + static_cast<Scalar>(k) * h;
        if (std::abs(coord - onLineCoord) <= EPS_DISTANCE) return k;
        return -1;
    };
    auto cellIndexFromCoord = [&](Scalar coord, Scalar minCoord, Scalar h, int nCell) -> int {
        int idx = static_cast<int>(std::floor((coord - minCoord) / h));
        if (idx < 0) idx = 0;
        if (idx >= nCell) idx = nCell - 1;
        return idx;
    };
    auto cellIndexFromCoordWithDir = [&](Scalar coord, Scalar minCoord, Scalar h, int nCell, int lineMaxIdx, Scalar dir) -> int {
        int k = lineIndex(coord, minCoord, h, lineMaxIdx);
        if (k == -1) return cellIndexFromCoord(coord, minCoord, h, nCell);
        int idx;
        if (dir > EPS_DISTANCE) {
            idx = k;
        } else if (dir < -EPS_DISTANCE) {
            idx = k - 1;
        } else {
            idx = (k == nCell) ? (nCell - 1) : k;
        }
        if (idx < 0) idx = 0;
        if (idx >= nCell) idx = nCell - 1;
        return idx;
    };
    auto addPointTouchCells = [&](const Point& p) {
        int ix = lineIndex(p.x, boundingBox.min.x, cellSizeX, nx);
        int iy = lineIndex(p.y, boundingBox.min.y, cellSizeY, ny);
        if (ix == -1 && iy == -1) {
            int i = cellIndexFromCoord(p.x, boundingBox.min.x, cellSizeX, nx);
            int j = cellIndexFromCoord(p.y, boundingBox.min.y, cellSizeY, ny);
            addCell(i, j);
            return;
        }
        if (ix != -1 && iy == -1) {
            int j = cellIndexFromCoord(p.y, boundingBox.min.y, cellSizeY, ny);
            addCell(ix - 1, j);
            addCell(ix, j);
            return;
        }
        if (ix == -1 && iy != -1) {
            int i = cellIndexFromCoord(p.x, boundingBox.min.x, cellSizeX, nx);
            addCell(i, iy - 1);
            addCell(i, iy);
            return;
        }
        addCell(ix - 1, iy - 1);
        addCell(ix - 1, iy);
        addCell(ix, iy - 1);
        addCell(ix, iy);
    };
    if (segment.length() <= EPS_DISTANCE) {
        addPointTouchCells(a);
        cells.assign(uniqueCells.begin(), uniqueCells.end());
        return;
    }
    if (std::abs(dx) <= EPS_DISTANCE) {
        int ix = lineIndex(a.x, boundingBox.min.x, cellSizeX, nx);
        Scalar yMin = std::min(a.y, b.y);
        Scalar yMax = std::max(a.y, b.y);
        int j0 = cellIndexFromCoord(yMin, boundingBox.min.y, cellSizeY, ny);
        int j1 = cellIndexFromCoord(yMax, boundingBox.min.y, cellSizeY, ny);
        int yMinLine = lineIndex(yMin, boundingBox.min.y, cellSizeY, ny);
        if (yMinLine != -1) j0 -= 1;
        if (j0 < 0) j0 = 0;
        if (j1 >= ny) j1 = ny - 1;
        if (ix != -1) {
            for (int j = j0; j <= j1; ++j) {
                addCell(ix - 1, j);
                addCell(ix, j);
            }
        } else {
            int i = cellIndexFromCoord(a.x, boundingBox.min.x, cellSizeX, nx);
            for (int j = j0; j <= j1; ++j) {
                addCell(i, j);
            }
        }
        addPointTouchCells(a);
        addPointTouchCells(b);
        cells.assign(uniqueCells.begin(), uniqueCells.end());
        return;
    }
    if (std::abs(dy) <= EPS_DISTANCE) {
        int iy = lineIndex(a.y, boundingBox.min.y, cellSizeY, ny);
        Scalar xMin = std::min(a.x, b.x);
        Scalar xMax = std::max(a.x, b.x);
        int i0 = cellIndexFromCoord(xMin, boundingBox.min.x, cellSizeX, nx);
        int i1 = cellIndexFromCoord(xMax, boundingBox.min.x, cellSizeX, nx);
        int xMinLine = lineIndex(xMin, boundingBox.min.x, cellSizeX, nx);
        if (xMinLine != -1) i0 -= 1;
        if (i0 < 0) i0 = 0;
        if (i1 >= nx) i1 = nx - 1;
        if (iy != -1) {
            for (int i = i0; i <= i1; ++i) {
                addCell(i, iy - 1);
                addCell(i, iy);
            }
        } else {
            int j = cellIndexFromCoord(a.y, boundingBox.min.y, cellSizeY, ny);
            for (int i = i0; i <= i1; ++i) {
                addCell(i, j);
            }
        }
        addPointTouchCells(a);
        addPointTouchCells(b);
        cells.assign(uniqueCells.begin(), uniqueCells.end());
        return;
    }
    int stepX = (dx > 0) ? 1 : -1;
    int stepY = (dy > 0) ? 1 : -1;
    int i = cellIndexFromCoordWithDir(a.x, boundingBox.min.x, cellSizeX, nx, nx, dx);
    int j = cellIndexFromCoordWithDir(a.y, boundingBox.min.y, cellSizeY, ny, ny, dy);
    Scalar nextX = (stepX > 0) ? getX(i + 1) : getX(i);
    Scalar nextY = (stepY > 0) ? getY(j + 1) : getY(j);
    Scalar tMaxX = (nextX - a.x) / dx;
    Scalar tMaxY = (nextY - a.y) / dy;
    Scalar tDeltaX = cellSizeX / std::abs(dx);
    Scalar tDeltaY = cellSizeY / std::abs(dy);
    const int maxSteps = nx + ny + 16;
    int steps = 0;
    while (steps++ < maxSteps) {
        addCell(i, j);
        Scalar tEvent = std::min(tMaxX, tMaxY);
        if (tEvent - 1.0 > EPS_DISTANCE) break;
        if (tMaxX + EPS_DISTANCE < tMaxY) {
            i += stepX;
            tMaxX += tDeltaX;
            continue;
        }
        if (tMaxY + EPS_DISTANCE < tMaxX) {
            j += stepY;
            tMaxY += tDeltaY;
            continue;
        }
        int nextI = i + stepX;
        int nextJ = j + stepY;
        addCell(nextI, j);
        addCell(i, nextJ);
        addCell(nextI, nextJ);
        i = nextI;
        j = nextJ;
        tMaxX += tDeltaX;
        tMaxY += tDeltaY;
    }
    addPointTouchCells(a);
    addPointTouchCells(b);
    cells.assign(uniqueCells.begin(), uniqueCells.end());
}
}
// ==== End src/trajgrid.h ====

// ==== Begin src/trajgraph.h ====
namespace fs = std::filesystem;
namespace traj {
class TraceSegmentGraph
{
public:
    TraceSegmentGraph() : startSegment(0), totalSegLenX(0), totalSegLenY(0), innerLoopExtractingSwitch(true) {};
    void addSegment(const DirectedSegment &segment);
    void updateBoundingBox(const DirectedSegment& segment);
    void getCellSize(Scalar& cellSizeX, Scalar& cellSizeY) const;
    void initUpdateCurrentSegmentIndex(const DirectedSegment& segment);
    void buildGridIndex(Scalar cellSizeX, Scalar cellSizeY);
    void buildGridColor();
    void buildIntersectionMap();
    void buildSegmentIntersectionInfo(int segmentIndex, std::vector<IntersectionInfo>& intersectionInfos);
    void clear();
    void reserve(size_t n) { segments.reserve(n); }
    void outputSegments(const std::vector<DirectedSegment>& segmentList, const std::string& outputFilePath) const;
    void extractOuterLoop(std::vector<DirectedSegment>& loopSegments);
    void getIntersectingCandidates(std::vector<int>& candidateIndices, int segmentIndex) const;
    void extractInnerLoops(std::vector<std::vector<DirectedSegment>>& innerLoops);
    void extractInnerLoopsByTraversing(std::vector<std::vector<DirectedSegment>>& innerLoops);
    void extractInnerLoopsByEmptyCells(std::vector<std::vector<DirectedSegment>>& innerLoops);
    void extractLoopStartAtSegment(int startSegmentIndex, std::vector<DirectedSegment>& loopSegments, Scalar& totalAngle, bool isInner);
    size_t getSegmentCount() const { return segments.size(); }
    void setInnerLoopExtractingSwitch(bool isEnabled) { innerLoopExtractingSwitch = isEnabled; }
private:
    std::vector<DirectedSegment> segments;    Box boundingBox;
    Grid2D grid;
    std::vector<std::vector<int>> cellHoldSegments;    std::vector<std::vector<int>> cellHoldSegmentsStrict;    std::vector<std::vector<int>> segmentCells;    std::vector<std::vector<IntersectionInfo>> segIntersections;    std::vector<int> gridColor;
    std::unordered_map<int, std::vector<int>> gridColorMap; 
    int startSegment;
    Scalar totalSegLenX;
    Scalar totalSegLenY;
    bool innerLoopExtractingSwitch;
};
void TraceSegmentGraph::getCellSize(Scalar& cellSizeX, Scalar& cellSizeY) const
{
    Scalar segLenX = 0, segLenY = 0;
    for (const auto& segment : segments) {
        segLenX += std::abs(segment.to.x - segment.from.x);
        segLenY += std::abs(segment.to.y - segment.from.y);
    }
    segLenX = segments.empty() ? 0 : segLenX / static_cast<Scalar>(segments.size());
    segLenY = segments.empty() ? 0 : segLenY / static_cast<Scalar>(segments.size());
    Scalar kCellSizeFactorX = kCellSizeFactorXOuter;
    Scalar kCellSizeFactorY = kCellSizeFactorYOuter;
    if (segments.size() >= kSimpleSegmentThreshold && segments.size() < kColorMethodThreshold)
    {
        kCellSizeFactorX = kCellSizeFactorXColor;
        kCellSizeFactorY = kCellSizeFactorYColor;
    }
    cellSizeX = std::max(kCellSizeX, segLenX * kCellSizeFactorX);
    cellSizeY = std::max(kCellSizeY, segLenY * kCellSizeFactorY);
    int maxCellsX = static_cast<int>(std::ceil((boundingBox.max.x - boundingBox.min.x) / cellSizeX));
    int maxCellsY = static_cast<int>(std::ceil((boundingBox.max.y - boundingBox.min.y) / cellSizeY));
    if (maxCellsX > kMaxGridCellsNumX) {
        cellSizeX = (boundingBox.max.x - boundingBox.min.x) / static_cast<Scalar>(kMaxGridCellsNumX);
    }
    if (maxCellsY > kMaxGridCellsNumY) {
        cellSizeY = (boundingBox.max.y - boundingBox.min.y) / static_cast<Scalar>(kMaxGridCellsNumY);
    }
}
void TraceSegmentGraph::getIntersectingCandidates(std::vector<int>& candidateIndices, int segmentIndex) const
{
    candidateIndices.clear();
    if (segmentIndex < 0 || segmentIndex >= static_cast<int>(segments.size())) return;
    std::unordered_set<int> uniqueCandidates;
    for (int cell : segmentCells[segmentIndex]) {
        uniqueCandidates.insert(cellHoldSegments[cell].begin(), cellHoldSegments[cell].end());
    }
    uniqueCandidates.erase(segmentIndex);    candidateIndices.assign(uniqueCandidates.begin(), uniqueCandidates.end());
}
void TraceSegmentGraph::buildSegmentIntersectionInfo(int segmentIndex, std::vector<IntersectionInfo>& intersectionInfos)
{
    const auto &segCurrent = segments[segmentIndex];
    auto compareForSort = [&] (const IntersectionInfo& a, const IntersectionInfo& b) {
                                if (std::abs(a.t1 - b.t1) > EPS_DISTANCE) {
                                    return a.t1 < b.t1;                                } else if (std::abs(a.angle - b.angle) > EPS_ANGLE_RELAX) {
                                    return a.angle < b.angle;                                } else {
                                    Scalar leftA = (segments[a.id].to - segCurrent.pointAt(a.t1)).norm();
                                    Scalar leftB = (segments[b.id].to - segCurrent.pointAt(b.t1)).norm();   
                                    if (std::abs(leftA - leftB) > EPS_DISTANCE)
                                        return leftA > leftB;
                                    else
                                        return a.id < b.id;                                }
                            };
    std::vector<int> intersectingCandidates;
    getIntersectingCandidates(intersectingCandidates, segmentIndex);
    intersectionInfos.reserve(intersectingCandidates.size());
    for (int candidate : intersectingCandidates) {
        DirectedSegment& segCandidate = segments[candidate];
        Scalar angle = segCurrent.angleTo(segCandidate);
        if (angle > EPS_ANGLE && 
            (std::abs(segCurrent.to.x - segCandidate.from.x) > EPS_DISTANCE 
                || std::abs(segCurrent.to.y - segCandidate.from.y) > EPS_DISTANCE)) continue;
        Scalar t1, t2;
        if (segCurrent.isIntersecting(segCandidate, t1, t2)) {
            if (t1 < EPS_DISTANCE) continue;
            if (t2 - 1.0 > -EPS_DISTANCE) continue;
            if (angle > EPS_ANGLE && 1.0 - t1 > EPS_DISTANCE) continue;
            if (std::abs(1.0 - t1) < EPS_DISTANCE) t1 = 1.0;
            if (std::abs(t2) < EPS_DISTANCE) t2 = 0.0;
            if (std::abs(1.0 - t2) < EPS_DISTANCE) t2 = 1.0;
            intersectionInfos.push_back({candidate, t1, t2, angle, false});
        }
    }
    std::sort(intersectionInfos.begin(), intersectionInfos.end(), compareForSort);
    auto it = std::unique(intersectionInfos.begin(), intersectionInfos.end(), 
                [&](const IntersectionInfo& a, const IntersectionInfo& b) {
                    return std::abs(a.t1 - b.t1) < EPS_DISTANCE;
                });
    intersectionInfos.erase(it, intersectionInfos.end());
}
void TraceSegmentGraph::extractOuterLoop(std::vector<DirectedSegment>& loopSegments)
{
    Scalar totalAngle = 0.0;
    extractLoopStartAtSegment(startSegment, loopSegments, totalAngle, false);
    if (loopSegments.size() > 1) {
        if (std::abs(totalAngle - PI_PI) > EPS_ANGLE_RELAX) {
        }
    } else {
    }
}
void TraceSegmentGraph::extractInnerLoops(std::vector<std::vector<DirectedSegment>>& innerLoops)
{
    if (!innerLoopExtractingSwitch) return;
    if (segments.size() < kSimpleSegmentThreshold) {
        extractInnerLoopsByTraversing(innerLoops);
    } 
    else if (segments.size() < kColorMethodThreshold) {
        buildGridColor();
        extractInnerLoopsByEmptyCells(innerLoops);
    }
    else {
    }
}
void TraceSegmentGraph::extractInnerLoopsByEmptyCells(std::vector<std::vector<DirectedSegment>>& innerLoops)
{
    if (segments.empty()) return;
    int nx = grid.sizeX();
    int ny = grid.sizeY();
    for (const auto& colorCellsPair: gridColorMap) {
        int colorId = colorCellsPair.first;
    }
    for (const auto& colorCellsPair: gridColorMap) {
        if (colorCellsPair.first == 0) continue;        if (colorCellsPair.second.empty()) continue;
        int colorId = colorCellsPair.first;
        const std::vector<int>& coloredCells = colorCellsPair.second;
        int nearestNonEmptyCellIndex = -1;
        int targetCell = coloredCells[0];        int cellIndex = targetCell;
        while (cellIndex % ny != 0) {
            cellIndex -= 1;
            if (gridColor[cellIndex] == -1) {
                nearestNonEmptyCellIndex = cellIndex;
                break;
            }
        }
        if (nearestNonEmptyCellIndex == -1) {
            continue;        }
        Box cellBox = grid.getCellBox(nearestNonEmptyCellIndex);
        Point targetPoint = grid.getCellCenter(targetCell);
        Box targetBox = grid.getCellBox(targetCell);
        int nearestSegmentIndex = -1;
        Scalar nearestSegmentDistance = std::numeric_limits<Scalar>::max();
        bool isOverlapped;
        for (const auto& segIndex: cellHoldSegments[nearestNonEmptyCellIndex]) {
            DirectedSegment cutSegment = segments[segIndex].getOverlapSegmentWithBox(cellBox, isOverlapped);
            if (!isOverlapped) continue;
            Scalar segmentDistance = cutSegment.distanceToPoint(targetPoint);
            if (segmentDistance < nearestSegmentDistance) {
                nearestSegmentDistance = segmentDistance;
                nearestSegmentIndex = segIndex;
            }
        }
        if (nearestSegmentIndex == -1) {
            continue;        }
        const auto& nearestSeg = segments[nearestSegmentIndex];
        if (nearestSeg.isPointOnLeftSide(targetPoint)) {
            continue;
        }
        std::vector<DirectedSegment> innerLoopSegments;
        Scalar totalAngle = 0.0;
        extractLoopStartAtSegment(nearestSegmentIndex, innerLoopSegments, totalAngle, true);
        if (innerLoopSegments.empty())
        {
            continue;        }
        bool isInnerLoop = true;
        if (!(std::abs(totalAngle + PI_PI) < EPS_ANGLE_RELAX))
        {
            isInnerLoop = false;        }
        if (isInnerLoop) 
        {
            innerLoops.push_back(std::move(innerLoopSegments));
        }
    }
}
void TraceSegmentGraph::extractInnerLoopsByTraversing(std::vector<std::vector<DirectedSegment>>& innerLoops)
{
    if (segments.empty()) return;
    int segSize = segments.size();
    for (int i = 0; i < segSize; ++i) {
        if (segments[i].angle() - PI > -EPS_ANGLE) continue;
        if (!segIntersections[i].empty() && segIntersections[i][0].used) continue;        std::vector<DirectedSegment> innerLoopSegments;
        Scalar totalAngle = 0.0;
        extractLoopStartAtSegment(i, innerLoopSegments, totalAngle, true);
        if (innerLoopSegments.empty()) continue;
        bool isInnerLoop = true;
        if (!(std::abs(totalAngle + PI_PI) < EPS_ANGLE_RELAX))
        {
            isInnerLoop = false;        }
        if (isInnerLoop) innerLoops.push_back(std::move(innerLoopSegments));
    }
}
void TraceSegmentGraph::extractLoopStartAtSegment(int startSegmentIndex, std::vector<DirectedSegment>& loopSegments, Scalar& totalAngle, bool isInner)
{
    loopSegments.clear();
    totalAngle = 0.0;
    if (segments.empty()) return;
    int segSize = segments.size();
    std::vector<Scalar> tStarts(segSize, 0.0);
    std::vector<Scalar> tEnds(segSize, 0.0);
    std::vector<std::vector<std::pair<Scalar, int>>> segVisitedPositions(segSize);
    std::vector<int> loopSegmentIndices;
    std::vector<Scalar> loopTStarts;
    std::vector<Scalar> loopTEnds;
    std::vector<Scalar> loopAngles;
    loopSegmentIndices.reserve(segSize);
    loopTStarts.reserve(segSize);
    loopTEnds.reserve(segSize);
    loopAngles.reserve(segSize);
    int currentSegment = startSegmentIndex;
    int nextSegment = -1, lastSegment = -1;
    int startPosition = -1;
    Scalar t1, t2, tmin, tnext, angleMin;
    bool isLoopCompleted = false;
    while(true)
    {
        tmin = 1.0;
        angleMin = std::numeric_limits<Scalar>::infinity();
        DirectedSegment& segCurrent = segments[currentSegment];
        std::vector<IntersectionInfo>& intersectionInfos = segIntersections[currentSegment];
        if (intersectionInfos.empty())
        {
            buildSegmentIntersectionInfo(currentSegment, intersectionInfos);
        }
        if (!intersectionInfos.empty())
        {
            for (auto& intersectInfo : intersectionInfos) {
                int otherSegIndex = intersectInfo.id;
                if (otherSegIndex != lastSegment) {
                    t1 = intersectInfo.t1;
                    t2 = intersectInfo.t2;
                    if (t1 - tStarts[currentSegment] < EPS_DISTANCE)
                    {
                        continue;                    }
                    if (intersectInfo.used)
                    {
                        if (!segVisitedPositions[currentSegment].empty())
                        {
                            for (const auto& visitedPos : segVisitedPositions[currentSegment]) {
                                if (std::abs(visitedPos.first - t1) < EPS_DISTANCE) {
                                    startPosition = visitedPos.second;
                                    loopTStarts[startPosition] = tStarts[currentSegment];
                                    isLoopCompleted = true;
                                    break;
                                }
                            }                        
                        }
                    }
                    else
                    {
                        tmin = t1;
                        tnext = t2;
                        nextSegment = otherSegIndex;
                        angleMin = intersectInfo.angle;
                        intersectInfo.used = true;                    }
                    break;
                }
            }
        }
        if (!isLoopCompleted) {
            if (nextSegment == -1) {
                break;            }
            segVisitedPositions[currentSegment].emplace_back(tmin, loopSegmentIndices.size());
            loopSegmentIndices.push_back(currentSegment);
            loopTStarts.push_back(tStarts[currentSegment]);
            loopTEnds.push_back(tmin);            loopAngles.push_back(angleMin);
            tStarts[nextSegment] = tnext;            lastSegment = currentSegment;
            currentSegment = nextSegment;
            nextSegment = -1;
            tEnds[lastSegment] = tmin;
        }
        else
        {
            break;
        }
    }
    if (currentSegment == -1 || startPosition == -1) return;
    loopSegments.reserve(loopSegmentIndices.size());
    for (int i = startPosition; i < loopSegmentIndices.size(); ++i) {
        loopSegments.push_back(segments[loopSegmentIndices[i]].getFromRange(loopTStarts[i], loopTEnds[i]));
        totalAngle += loopAngles[i];
    }
}
void TraceSegmentGraph::buildGridColor()
{
    int gridSize = grid.size();
    cellHoldSegmentsStrict.clear();
    cellHoldSegmentsStrict.resize(gridSize);
    bool isOverlapped;
    for (int i = 0; i < gridSize; ++i)
    {
        cellHoldSegmentsStrict[i].reserve(cellHoldSegments[i].size());
        for (int segIndex: cellHoldSegments[i])
        {
            const DirectedSegment& seg = segments[segIndex];
            if (seg.getOverlapSegmentWithBox(grid.getCellBox(i), isOverlapped).length() > EPS_DISTANCE) {
                cellHoldSegmentsStrict[i].push_back(static_cast<int>(segIndex));
            }
        }
    }
    int nx = grid.sizeX();
    int ny = grid.sizeY();
    gridColor.clear();
    gridColor.assign(gridSize, -1);
    gridColorMap.clear();
    int colorId = 1;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j)
        {
            int cellIndex = i * ny + j;
            if (cellHoldSegmentsStrict[cellIndex].empty())
            {
                if (j == 0 || i == 0 || j == ny - 1 || i == nx - 1) {
                    gridColor[cellIndex] = 0;                    gridColorMap[0].push_back(cellIndex);
                }
                int lowerIndex = cellIndex - 1;
                if (j > 0 && gridColor[lowerIndex] != -1) {
                    if (gridColor[cellIndex] != -1) {
                        if (gridColor[cellIndex] != gridColor[lowerIndex]) {
                            int oldColor = gridColor[cellIndex] < gridColor[lowerIndex] ? gridColor[cellIndex] : gridColor[lowerIndex];
                            int newColor = gridColor[lowerIndex] > gridColor[cellIndex] ? gridColor[lowerIndex] : gridColor[cellIndex];
                            for (int idx : gridColorMap[newColor]) {
                                gridColor[idx] = oldColor;
                                gridColorMap[oldColor].push_back(idx);
                            }
                            gridColorMap.erase(newColor);
                        }
                    } else {
                        gridColor[cellIndex] = gridColor[lowerIndex];
                        gridColorMap[gridColor[cellIndex]].push_back(cellIndex);
                    }
                }
                int leftIndex = cellIndex - ny;
                if (i > 0 && gridColor[leftIndex] != -1) {
                    if (gridColor[cellIndex] != -1) {
                        if (gridColor[cellIndex] != gridColor[leftIndex]) {
                            int oldColor = gridColor[cellIndex] < gridColor[leftIndex] ? gridColor[cellIndex] : gridColor[leftIndex];
                            int newColor = gridColor[leftIndex] > gridColor[cellIndex] ? gridColor[leftIndex] : gridColor[cellIndex];
                            for (int idx : gridColorMap[newColor]) {
                                gridColor[idx] = oldColor;
                                gridColorMap[oldColor].push_back(idx);
                            }
                            gridColorMap.erase(newColor);
                        }
                    } else {
                        gridColor[cellIndex] = gridColor[leftIndex];
                        gridColorMap[gridColor[cellIndex]].push_back(cellIndex);
                    }
                }
                if (gridColor[cellIndex] == -1) {
                    gridColor[cellIndex] = colorId;
                    colorId++;
                    gridColorMap[gridColor[cellIndex]].push_back(cellIndex);
                }
            }
        }
    }
}
void TraceSegmentGraph::buildIntersectionMap()
{
    int gridSize = grid.size();
    int segSize = segments.size();
    segIntersections.clear();
    segIntersections.resize(segSize);
    std::vector<std::unordered_set<int>> tempSegmentVisited(segSize);
    Scalar t1, t2;
    for (int i = 0; i < gridSize; ++i) {
        int cellSegSize = cellHoldSegments[i].size();
        for (int j = 0; j < cellSegSize; ++j) {
            for (int k = j + 1; k < cellSegSize; ++k) {
                int segIndex1 = cellHoldSegments[i][j];
                int segIndex2 = cellHoldSegments[i][k];
                if (tempSegmentVisited[segIndex1].count(segIndex2) > 0
                    || tempSegmentVisited[segIndex2].count(segIndex1) > 0) continue;                tempSegmentVisited[segIndex1].insert(segIndex2);
                if (segments[segIndex1].isIntersecting(segments[segIndex2], t1, t2)) {
                    segIntersections[segIndex1].push_back({segIndex2, t1, t2, segments[segIndex1].angleTo(segments[segIndex2])});
                    segIntersections[segIndex2].push_back({segIndex1, t2, t1, segments[segIndex2].angleTo(segments[segIndex1])});
                }
            }
        }
    }
}
void TraceSegmentGraph::buildGridIndex(Scalar cellSizeX, Scalar cellSizeY)
{
    grid.init(boundingBox, cellSizeX, cellSizeY);
    int gridSize = grid.size();
    int segSize = segments.size();
    cellHoldSegments.clear();
    cellHoldSegments.resize(gridSize);
    segmentCells.clear();
    segmentCells.resize(segSize);
    for (size_t i = 0; i < segSize; ++i) {
        const auto& seg = segments[i];
        grid.segmentToCells(segmentCells[i], seg);
        for (int cell : segmentCells[i]) {
            cellHoldSegments[cell].push_back(static_cast<int>(i));
        }
    }
    segIntersections.clear();
    segIntersections.resize(segSize);}
void TraceSegmentGraph::clear()
{
    segments.clear();
    cellHoldSegments.clear();
    segmentCells.clear();
    startSegment = 0;
    boundingBox.clear();
    grid.clear();
}
void TraceSegmentGraph::addSegment(const DirectedSegment& segment)
{
    segments.push_back(segment);
    updateBoundingBox(segment);
    initUpdateCurrentSegmentIndex(segment);
}
void TraceSegmentGraph::initUpdateCurrentSegmentIndex(const DirectedSegment& segment)
{
    const Point& currentStart = segments[startSegment].from;
    if (segment.from.y - currentStart.y < -EPS_DISTANCE || (std::abs(segment.from.y - currentStart.y) <= EPS_DISTANCE && segment.angle() - segments[startSegment].angle() < -EPS_ANGLE)) {
        startSegment = static_cast<int>(segments.size()) - 1;
    }
}
void TraceSegmentGraph::updateBoundingBox(const DirectedSegment& segment)
{
    const Point& from = segment.from;
    const Point& to = segment.to;
    if (segments.size() == 1) {
        boundingBox.min = from;
        boundingBox.max = from;
    }
    if (from.x < boundingBox.min.x) boundingBox.min.x = from.x;
    if (from.y < boundingBox.min.y) boundingBox.min.y = from.y;
    if (from.x > boundingBox.max.x) boundingBox.max.x = from.x;
    if (from.y > boundingBox.max.y) boundingBox.max.y = from.y;
    if (to.x < boundingBox.min.x) boundingBox.min.x = to.x;
    if (to.y < boundingBox.min.y) boundingBox.min.y = to.y;
    if (to.x > boundingBox.max.x) boundingBox.max.x = to.x;
    if (to.y > boundingBox.max.y) boundingBox.max.y = to.y;
}
void TraceSegmentGraph::outputSegments(const std::vector<DirectedSegment>& segmentList, const std::string& outputFilePath) const
{
    std::ofstream outFile(outputFilePath);
    if (!outFile.is_open()) {
        return;
    }
    outFile << segmentList.size() << "\n";
    for (const auto& seg : segmentList) {
        outFile << std::fixed << std::setprecision(5) << seg.from.x << " " << seg.from.y << " "
                << std::fixed << std::setprecision(5) << seg.to.x << " " << seg.to.y << "\n";
    }
    outFile.close();
}
}
// ==== End src/trajgraph.h ====

// ==== Begin src/nfp.h ====
namespace nfp {
using namespace traj;
class NFPSolver
{
public:
    Polygon genNFP(const Ring& RingA, const Ring& RingB, bool outInner = false);
    void outputNFP(const std::string& outputFilePath) const;
    void clear();
    Polygon getNFP(){return nfp;}
    std::vector<Segment> getNFPSegments(){return nfpSegments;};
private:
    void setPolygon(const Ring &RingA, const Ring &RingB);
    void generateTraceSegments();
    bool mayVertexTouchEdge(Scalar angleStart, Scalar angleEnd, Scalar angleLine) const;
    void dataToNFP();
    int na, nb;
    Polygon plyA;
    Polygon plyB;
    Polygon nfp;
    Point refPointB{0.0, 0.0};
    TraceSegmentGraph traceGraph;
    std::vector<Segment> nfpSegments;    std::vector<Segment> nfpOuterLoop;
    std::vector<std::vector<Segment>> nfpInnerLoops;
    std::vector<Scalar> loopAreas;
    std::vector<bool> loopInsideMeansOverlap;
};
bool NFPSolver::mayVertexTouchEdge(Scalar angleStart, Scalar angleEnd, Scalar angleLine) const
{
    Scalar span = toAngle(angleEnd - angleStart);
    Scalar rel = toAngle(angleLine - angleStart);
    if (span - rel > -EPS_ANGLE) return true;
    return false;
}
void NFPSolver::generateTraceSegments()
{
    traceGraph.clear();
    traceGraph.reserve(na * nb * 2);
    const auto& A = plyA.outer.vertices;
    const auto& B = plyB.outer.vertices;
    std::vector<Point> edgeA(na);
    std::vector<Point> edgeB(nb);
    for (int ia = 0; ia < na; ++ia)
    {  edgeA[ia] = A[(ia + 1) % na] - A[ia];  }
    for (int ib = 0; ib < nb; ++ib)
    {  edgeB[ib] = B[(ib + 1) % nb] - B[ib];  }
    int concaveCountA = 0, concaveCountB = 0;
    for (int ia = 0; ia < na; ++ia) {
        if (edgeA[(ia - 1 + na) % na].cross(edgeA[ia]) < -EPS_ANGLE) {
            concaveCountA++;
        }
    }
    for (int ib = 0; ib < nb; ++ib) {
        if (edgeB[(ib - 1 + nb) % nb].cross(edgeB[ib]) < -EPS_ANGLE) {
            concaveCountB++;
        }
    }
    if (concaveCountA < 2 && concaveCountB < 2)
    {
        traceGraph.setInnerLoopExtractingSwitch(false);
    }
    for (int ib = 0; ib < nb; ++ib) {
        Scalar angleStart = edgeB[(ib - 1 + nb) % nb].rotateCCW90().angle();
        Scalar angleEnd = edgeB[ib].rotateCCW90().angle();
        if (edgeB[(ib - 1 + nb) % nb].cross(edgeB[ib]) < -EPS_ANGLE)  continue;
        for (int ia = 0; ia < na; ++ia) {
            Scalar angleLine = edgeA[ia].rotateCW90().angle();
            if (!mayVertexTouchEdge(angleStart, angleEnd, angleLine)) continue;
            Point offset = refPointB - B[ib];
            traceGraph.addSegment({A[ia] + offset, A[(ia + 1) % na] + offset});
        }
    }
    for (int ia = 0; ia < na; ++ia) {
        Scalar angleStart = edgeA[(ia - 1 + na) % na].rotateCCW90().angle();
        Scalar angleEnd = edgeA[ia].rotateCCW90().angle();
        if (edgeA[(ia - 1 + na) % na].cross(edgeA[ia]) < -EPS_ANGLE)  continue;
        for (int ib = 0; ib < nb; ++ib) {
            Scalar angleLine = edgeB[ib].rotateCW90().angle();
            if (!mayVertexTouchEdge(angleStart, angleEnd, angleLine)) continue;
            Point offset = refPointB + A[ia];
            traceGraph.addSegment({offset - B[ib], offset - B[(ib + 1) % nb]});
        }
    }
}
void NFPSolver::outputNFP(const std::string& outputFilePath) const
{
    std::ofstream fout(outputFilePath);
    if (!fout.is_open()) return;
    fout << std::fixed << std::setprecision(5);
    fout << "# NFP loops generated by trace-line algorithm\n";
    fout << "loop_count " << nfpInnerLoops.size() + 1 << "\n";
    fout << "loop" << " " << 0 << " size " << nfpOuterLoop.size() << " overlap_inside " << 1 << "\n";
    for (const auto& seg: nfpOuterLoop) {
        fout << seg.from.x << " " << seg.from.y << "\n";
    }
    for (size_t i = 0; i < nfpInnerLoops.size(); ++i) {
        fout << "loop" << " " << i + 1 << " size " << nfpInnerLoops[i].size() << " overlap_inside " << 0 << "\n";
        for (const auto& seg: nfpInnerLoops[i]) {
            fout << seg.from.x << " " << seg.from.y << "\n";
        }
    }
}
void NFPSolver::clear() {
    na = 0;
    nb = 0;
    plyA.clear();
    plyB.clear();
    nfp.clear();
    refPointB = {0.0, 0.0};
    traceGraph.clear();
    nfpSegments.clear();
    nfpOuterLoop.clear();
    nfpInnerLoops.clear();
    loopAreas.clear();
    loopInsideMeansOverlap.clear();
}
void NFPSolver::setPolygon(const Ring& RingA, const Ring& RingB) {
    plyA.outer = RingA;
    plyB.outer = RingB;
    na = static_cast<int>(RingA.vertices.size());
    nb = static_cast<int>(RingB.vertices.size());
    plyA.init();
    plyB.init();
}
void NFPSolver::dataToNFP() {
    segmentsToRing(nfpOuterLoop, nfp.outer);
    nfp.holes.resize(nfpInnerLoops.size());
    for(int i = 0; i < nfpInnerLoops.size(); ++i) 
        segmentsToRing(nfpInnerLoops[i], nfp.holes[i]);
    snapGrid(nfp);
    int nfpSegsize = static_cast<int>(nfp.outer.vertices.size());
    for (const auto& hole : nfp.holes) nfpSegsize += static_cast<int>(hole.vertices.size());
    nfpSegments.reserve(nfpSegsize);
    const auto& outerV = nfp.outer.vertices;
    for (int i = 0; i < static_cast<int>(outerV.size()); ++i) {
        nfpSegments.push_back({outerV[i], outerV[(i + 1) % outerV.size()]});
    }
    for (const auto& hole : nfp.holes) {
        const auto& holeV = hole.vertices;
        for (int i = 0; i < static_cast<int>(holeV.size()); ++i) {
            nfpSegments.push_back({holeV[i], holeV[(i + 1) % holeV.size()]});
        }
    }
};
Polygon NFPSolver::genNFP(const Ring& RingA, const Ring& RingB, bool outInner)
{
    clear();
    setPolygon(RingA, RingB);
    generateTraceSegments();
    Scalar cellSizeX, cellSizeY;
    traceGraph.getCellSize(cellSizeX, cellSizeY);
    traceGraph.buildGridIndex(cellSizeX, cellSizeY);
    {
        std::vector<DirectedSegment> outerLoopDirected;
        traceGraph.extractOuterLoop(outerLoopDirected);
        nfpOuterLoop.reserve(outerLoopDirected.size());
        for (const auto& ds : outerLoopDirected)
            nfpOuterLoop.push_back({ds.from, ds.to});
    }
    if(outInner)
    {
        std::vector<std::vector<DirectedSegment>> innerLoopsDirected;
        traceGraph.extractInnerLoops(innerLoopsDirected);
        nfpInnerLoops.reserve(innerLoopsDirected.size());
        for (auto& loop : innerLoopsDirected) {
            std::vector<Segment> innerLoop;
            innerLoop.reserve(loop.size());
            for (const auto& ds : loop)
                innerLoop.push_back({ds.from, ds.to});
            nfpInnerLoops.push_back(std::move(innerLoop));
        }
        for (auto it = nfpInnerLoops.begin(); it != nfpInnerLoops.end(); ) {
            Point offset = ((*it)[0].from + (*it)[0].to) * 0.5 - refPointB;
            if (plyA.isOverlapingStrict(plyB.translate(offset)))
            {    it = nfpInnerLoops.erase(it);  }
            else  {   ++it; }
        }
    }
    dataToNFP();
    return nfp;
}
}
// ==== End src/nfp.h ====

// ==== Begin src/query.h ====
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
    if (ringId == 0) {
      ring = &segPoly.outer;
    } else if (ringId - 1 < static_cast<int>(segPoly.holes.size())) {
      ring = &segPoly.holes[ringId - 1];
    } else {
      continue;
    }
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
    if (ringId == 0) {
      outerLines.push_back(yCross);
    } else {
      holesLines.push_back(yCross);
    }
  }
  std::sort(outerLines.begin(), outerLines.end());
  std::sort(holesLines.begin(), holesLines.end());
}
class BVH {
  struct BVHNode {
    Box aabb;
    int left;    int right;  };
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
  std::vector<UnorderedPolygon> nfpMatrix;  std::vector<Polygon> nfpPolysMatrix;  std::vector<FastGrid> gridMatrix;
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
// ==== End src/query.h ====

// ==== Begin src/poly_contain.h ====
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
// ==== End src/poly_contain.h ====

// ==== Begin src/lg.h ====
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
inline mt19937 g_rng(42);
// inline mt19937 g_rng(std::random_device{}());
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
// ==== End src/lg.h ====

// ==== Begin src/op.h ====
static bool saSwapOp(vector<Polygon>& P, int n, const Boundary& bd,
                     double T, double progress,
                     unordered_set<int>& overlappingSet, bool globalFeasible, double C,
                     double& curCost, double& bestCost,
                     vector<double>& bestTx, vector<double>& bestTy,
                     vector<int>& candidates
                     DEBUG_SA_STATS_PARAM)
{
#ifdef DEBUG_LOG
    dbg.snapSwapTrials++;
#endif
    int ia = -1, ib = -1;
    if (progress < 0.35) {
        ia = 0;
        double maxD = singleDist(P[0]);
        for (int k = 1; k < n; k++) {
            double d = singleDist(P[k]);
            if (d > maxD) { maxD = d; ia = k; }
        }
        ib = randInt(n);
        while (ib == ia) ib = randInt(n);
    } else if (progress < 0.65) {        double totalW = 0.0;
        for (int k = 0; k < n; k++)
            totalW += pow(singleDist(P[k]), 1.5) + 1e-9;
        auto weightedPick = [&](int exclude) -> int {
            double r = randDouble() * totalW;
            double acc = 0.0;
            for (int k = 0; k < n; k++) {
                if (k == exclude) continue;
                acc += pow(singleDist(P[k]), 1.5) + 1e-9;
                if (acc >= r) return k;
            }
            return (exclude == 0) ? 1 : 0;
        };
        ia = weightedPick(-1);
        ib = weightedPick(ia);
    } else {
        vector<double> dists(n);
        for (int k = 0; k < n; k++) dists[k] = singleDist(P[k]);
        vector<double> sortedDists = dists;
        sort(sortedDists.begin(), sortedDists.end());
        double medianDist = sortedDists[n / 2];
        vector<int> candidates_ia;
        candidates_ia.reserve(n);
        for (int k = 0; k < n; k++)
            if (dists[k] >= medianDist) candidates_ia.push_back(k);
        if (candidates_ia.empty()) {
            ia = randInt(n);
        } else {
            ia = candidates_ia[randInt((int)candidates_ia.size())];
        }
        double sizeA = P[ia].hw + P[ia].hh;
        ib = -1;
        for (int attempt = 0; attempt < 8; attempt++) {
            int cand = randInt(n);
            if (cand == ia) continue;
            double sizeB = P[cand].hw + P[cand].hh;
            double larger  = fmax(sizeA, sizeB);
            double smaller = fmin(sizeA, sizeB);
            double sizeRatio = (larger > 1e-9) ? smaller / larger : 1.0;
            if (sizeRatio > 0.6 && dists[cand] > medianDist * 0.3) {
                ib = cand;
                break;
            }
        }
        if (ib == -1) {
            return false;
        }
    }
    {
        double oldPenaltyA = 0.0, oldPenaltyB = 0.0;
        if (!globalFeasible && overlappingSet.count(ia)) {
            g_hash.getCandidates(ia, P, candidates);
            double maxMTV_unused;
            oldPenaltyA = calcSingleOldPenalty(P, ia, candidates, maxMTV_unused);
        }
        if (!globalFeasible && overlappingSet.count(ib)) {
            g_hash.getCandidates(ib, P, candidates);
            double maxMTV_unused;
            oldPenaltyB = calcSingleOldPenalty(P, ib, candidates, maxMTV_unused);
        }
        double otxa=P[ia].tx, otya=P[ia].ty, otxb=P[ib].tx, otyb=P[ib].ty;
        double oldCost2 = singleDist(P[ia]) + singleDist(P[ib]);
        g_hash.remove(ia,P); g_hash.remove(ib,P);
        double old_cxa = P[ia].origCx + otxa, old_cya = P[ia].origCy + otya;
        double old_cxb = P[ib].origCx + otxb, old_cyb = P[ib].origCy + otyb;
        P[ia].tx = old_cxb - P[ia].origCx;  P[ia].ty = old_cyb - P[ia].origCy;
        P[ib].tx = old_cxa - P[ib].origCx;  P[ib].ty = old_cya - P[ib].origCy;
        Vertex mtvA={0,0}; bool mtvAValid=false;
        Vertex mtvB={0,0}; bool mtvBValid=false;
        bool valid = exactInBounds(ia, P[ia], bd) && exactInBounds(ib, P[ib], bd);
        if (valid && fastOverlap(ia,ib,P[ia],P[ib],mtvA,mtvAValid)) {
            valid=false;
        }
        if (valid) {
            g_hash.getCandidates(ia,P,candidates);
            for (int k:candidates) if(fastOverlap(ia,k,P[ia],P[k],mtvA,mtvAValid)){valid=false;break;}
        }
        if (valid) {
            g_hash.getCandidates(ib,P,candidates);
            for (int k:candidates) if(fastOverlap(ib,k,P[ib],P[k],mtvB,mtvBValid)){valid=false;break;}
        }
        if (valid) {
            double newCost2 = singleDist(P[ia]) + singleDist(P[ib]);
            double delta2   = (newCost2 - oldCost2) - C * (oldPenaltyA + oldPenaltyB);
            double swapT = fmax(T, oldCost2 * 0.05);
            if (delta2<0.0 || randDouble()<exp(-delta2/fmax(swapT, 1e-15))) {
                curCost += delta2;
                overlappingSet.erase(ia);
                overlappingSet.erase(ib);
#ifdef DEBUG_LOG
                dbg.snapSwapAccepts++;
#endif
                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
#ifdef DEBUG_LOG
                    dbg.dbgBestL=bestCost;
                    logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),bestCost,dbg.dbgBestL,"best_update");
#endif
                }
            } else { P[ia].tx=otxa;P[ia].ty=otya;P[ib].tx=otxb;P[ib].ty=otyb; }
        } else {
            bool mtvFixed = false;
            bool bothInBounds = exactInBounds(ia, P[ia], bd) && exactInBounds(ib, P[ib], bd);
            if (bothInBounds && (mtvAValid || mtvBValid)) {
                double mtvALen=sqrt(mtvA.x*mtvA.x+mtvA.y*mtvA.y);
                if (mtvAValid && mtvALen>1e-7) {
                    P[ia].tx-=mtvA.x*1.001; P[ia].ty-=mtvA.y*1.001;
                }
                bool iaOk=exactInBounds(ia, P[ia], bd);
                if (iaOk && fastOverlap(ia,ib,P[ia],P[ib])) iaOk=false;
                if (iaOk) {
                    g_hash.getCandidates(ia,P,candidates);
                    for (int k:candidates) if(fastOverlap(ia,k,P[ia],P[k])){iaOk=false;break;}
                }
                if (iaOk) {
                    Vertex mtvB2={0,0}; bool mtvB2Valid=false;
                    if (fastOverlap(ib,ia,P[ib],P[ia],mtvB2,mtvB2Valid) && mtvB2Valid) {
                    } else {
                        g_hash.getCandidates(ib,P,candidates);
                        for (int k:candidates) {
                            if (k==ia) continue;
                            if (fastOverlap(ib,k,P[ib],P[k],mtvB2,mtvB2Valid) && mtvB2Valid) break;
                        }
                    }
                    double mtvB2Len=sqrt(mtvB2.x*mtvB2.x+mtvB2.y*mtvB2.y);
                    if (mtvB2Valid && mtvB2Len>1e-7) {
                        P[ib].tx-=mtvB2.x*1.001; P[ib].ty-=mtvB2.y*1.001;
                    }
                    bool ibOk=exactInBounds(ib, P[ib], bd);
                    if (ibOk && fastOverlap(ib,ia,P[ib],P[ia])) ibOk=false;
                    if (ibOk) {
                        g_hash.getCandidates(ib,P,candidates);
                        for (int k:candidates) if(fastOverlap(ib,k,P[ib],P[k])){ibOk=false;break;}
                    }
                    if (ibOk) {
                        double newCost2=singleDist(P[ia])+singleDist(P[ib]);
                        double delta2=(newCost2-oldCost2) - C * (oldPenaltyA + oldPenaltyB);
                        double swapT=T*exp(-(double)n/25.0);
                        if (delta2<0.0||randDouble()<exp(-delta2/fmax(swapT,1e-15))) {
                            mtvFixed=true;
                            curCost+=delta2;
                            overlappingSet.erase(ia);
                            overlappingSet.erase(ib);
#ifdef DEBUG_LOG
                            dbg.snapSwapAccepts++;
#endif
                            if (curCost<bestCost) {
                                bestCost=curCost;
                                for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
#ifdef DEBUG_LOG
                                dbg.dbgBestL=bestCost;
                                logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),bestCost,dbg.dbgBestL,"best_update(mtv_swap)");
#endif
                            }
                        }
                    }
                }
            }
            if (!mtvFixed) { P[ia].tx=otxa;P[ia].ty=otya;P[ib].tx=otxb;P[ib].ty=otyb; }
        }
        g_hash.insert(ia,P); g_hash.insert(ib,P);
    }
#ifdef DEBUG_LOG
    {
        double nowSnap = elapsed();
        if (nowSnap - dbg.lastSnapshotTime >= 1.0) {
            timerSASnapshot(dbg.phaseName, dbg.lastT, dbg.lastStep, curCost, bestCost, dbg.dbgIter,
                            dbg.snapSwapTrials, dbg.snapSwapAccepts,
                            dbg.snapMoveTrials, dbg.snapMoveAccepts,
                            dbg.snapSlideTrials, dbg.snapSlideAccepts);
            dbg.lastSnapshotTime = nowSnap;
            dbg.snapSwapTrials=dbg.snapSwapAccepts=0;
            dbg.snapMoveTrials=dbg.snapMoveAccepts=0;
            dbg.snapSlideTrials=dbg.snapSlideAccepts=0;
            dbg.snapIters=0;
        }
        if (dbg.dbgIter%1000==0) logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),curCost,dbg.dbgBestL,"checkpoint");
    }
#endif
    return true;
}
static bool saMoveOp(vector<Polygon>& P, int n, const Boundary& bd,
                     double T, double step, double progress,
                     double towardOriginProb, bool enableMTVSlide,
                     unordered_set<int>& overlappingSet, bool globalFeasible, double C,
                     double& curCost, double& bestCost,
                     vector<double>& bestTx, vector<double>& bestTy,
                     vector<int>& candidates
                     DEBUG_SA_STATS_PARAM)
{
    const double FAN_FULL_RAD = 2.0 * 60.0 * 3.14159265358979323846 / 180.0;
#ifdef DEBUG_LOG
    dbg.snapMoveTrials++;
#endif
    int    idx = randInt(n);
    double otx = P[idx].tx, oty = P[idx].ty;
    double oldLen = singleDist(P[idx]);
    double oldPenalty = 0.0;
    if (!globalFeasible && overlappingSet.count(idx)) {
        g_hash.getCandidates(idx, P, candidates);
        double maxMTV_unused;
        oldPenalty = calcSingleOldPenalty(P, idx, candidates, maxMTV_unused);
    }
    double dx, dy;
    double dynamicTowardProb = towardOriginProb *(0.5 + 0.5 * progress);
    bool useFanSampling = (progress >= 0.3);
    double mag = step * (0.5 + 0.5 * randDouble());
    if (randDouble() < dynamicTowardProb && oldLen > 1e-6) {
        if (useFanSampling) {
            double baseAng  = atan2(-oty, -otx);
            double deltaAng = (randDouble() - 0.5) * FAN_FULL_RAD;
            double ang      = baseAng + deltaAng;
            dx = cos(ang) * mag;
            dy = sin(ang) * mag;
        } else {
            dx = -otx / oldLen * mag;
            dy = -oty / oldLen * mag;
        }
    } else {
        double ang = randDouble() * 6.283185307;
        dx = cos(ang) * mag;
        dy = sin(ang) * mag;
    }
    bool onLeft  = (P[idx].cx()-P[idx].hw < bd.xMin+WALL_EPS);
    bool onRight = (P[idx].cx()+P[idx].hw > bd.xMax-WALL_EPS);
    bool onBot   = (P[idx].cy()-P[idx].hh < bd.yMin+WALL_EPS);
    bool onTop   = (P[idx].cy()+P[idx].hh > bd.yMax-WALL_EPS);
    if (onLeft  && dx<0.0) dx=0.0;
    if (onRight && dx>0.0) dx=0.0;
    if (onBot   && dy<0.0) dy=0.0;
    if (onTop   && dy>0.0) dy=0.0;
    if (fabs(dx)<1e-9 && fabs(dy)<1e-9) {
#ifdef DEBUG_LOG
        if (dbg.dbgIter%1000==0) logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),curCost,dbg.dbgBestL,"checkpoint");
#endif
        return false;
    }
    g_hash.remove(idx, P);
    P[idx].tx = otx + dx;
    P[idx].ty = oty + dy;
    bool accepted = false;
    if (exactInBounds(idx, P[idx], bd)) {
        g_hash.getCandidates(idx, P, candidates);
        bool conflict = false;
        for (int j : candidates)
            if (fastOverlap(idx, j, P[idx], P[j])) { conflict = true; break; }
        if (!conflict) {
            double newLen = singleDist(P[idx]);
            double delta  = (newLen - oldLen) - C * oldPenalty;
            if (delta<0.0 || randDouble()<exp(-delta/T)) {
                accepted = true;
                curCost += delta;
                overlappingSet.erase(idx);
#ifdef DEBUG_LOG
                dbg.snapMoveAccepts++;
#endif
                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
#ifdef DEBUG_LOG
                    dbg.dbgBestL=bestCost;
                    logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),bestCost,dbg.dbgBestL,"best_update");
#endif
                }
            }
        } else if (enableMTVSlide && globalFeasible) {
#ifdef DEBUG_LOG
            dbg.snapSlideTrials++;
#endif
            Vertex mtv      = {0.0, 0.0};
            bool   mtvValid = false;
            for (int j : candidates) {
                if (fastOverlap(idx, j, P[idx], P[j], mtv, mtvValid)) {
                    if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7))
                        break;
                }
            }
            if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7)) {
                double norm = sqrt(mtv.x*mtv.x + mtv.y*mtv.y);
                double tx1 = -mtv.y/norm,  ty1 =  mtv.x/norm;
                double tx2 =  mtv.y/norm,  ty2 = -mtv.x/norm;
                double dot1 = tx1*dx + ty1*dy;
                double dot2 = tx2*dx + ty2*dy;
                double sx, sy;
                if (dot1 >= dot2) { sx = tx1; sy = ty1; }
                else              { sx = tx2; sy = ty2; }
                P[idx].tx = otx + sx * step;
                P[idx].ty = oty + sy * step;
                bool slideOk = exactInBounds(idx, P[idx], bd);
                if (slideOk) {
                    g_hash.getCandidates(idx, P, candidates);
                    for (int j : candidates)
                        if (fastOverlap(idx, j, P[idx], P[j])) { slideOk = false; break; }
                }
                if (slideOk) {
                    double newLen = singleDist(P[idx]);
                    double delta  = newLen - oldLen;
                    if (delta < 0.0 || randDouble() < exp(-delta/T)) {
                        accepted  = true;
                        curCost  += delta;
#ifdef DEBUG_LOG
                        dbg.snapSlideAccepts++;
#endif
                        if (curCost < bestCost) {
                            bestCost = curCost;
                            for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
#ifdef DEBUG_LOG
                            dbg.dbgBestL=bestCost;
                            logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),bestCost,dbg.dbgBestL,"best_update(slide)");
#endif
                        }
                    }
                }
            }
        } else if (!globalFeasible) {
            Vertex mtv = {0.0, 0.0};
            bool mtvValid = false;
            for (int j : candidates) {
                if (fastOverlap(j, idx, P[j], P[idx], mtv, mtvValid)) {
                    if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7))
                        break;
                }
            }
            if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7)) {
                double corrTx = P[idx].tx + mtv.x * 1.001;
                double corrTy = P[idx].ty + mtv.y * 1.001;
                P[idx].tx = corrTx;
                P[idx].ty = corrTy;
                if (exactInBounds(idx, P[idx], bd)) {
                    g_hash.getCandidates(idx, P, candidates);
                    bool still = false;
                    for (int j : candidates)
                        if (fastOverlap(idx, j, P[idx], P[j])) { still = true; break; }
                    if (!still) {
                        double newLen = singleDist(P[idx]);
                        double delta = (newLen - oldLen) - C * oldPenalty;
                        if (delta < 0.0 || randDouble() < exp(-delta / T)) {
                            accepted = true;
                            curCost += delta;
                            overlappingSet.erase(idx);
                            if (curCost < bestCost) {
                                bestCost = curCost;
                                for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
#ifdef DEBUG_LOG
                                dbg.dbgBestL = bestCost;
                                logLine(dbg.phaseName, dbg.dbgIter, -1, elapsed(), bestCost, dbg.dbgBestL, "best_update(mtv_push_move)");
#endif
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx=otx; P[idx].ty=oty; }
    g_hash.insert(idx, P);
#ifdef DEBUG_LOG
    double nowSnap = elapsed();
    if (nowSnap - dbg.lastSnapshotTime >= 1.0) {
        timerSASnapshot(dbg.phaseName, dbg.lastT, dbg.lastStep, curCost, bestCost, dbg.dbgIter,
                        dbg.snapSwapTrials, dbg.snapSwapAccepts,
                        dbg.snapMoveTrials, dbg.snapMoveAccepts,
                        dbg.snapSlideTrials, dbg.snapSlideAccepts);
        dbg.lastSnapshotTime = nowSnap;
        dbg.snapSwapTrials=dbg.snapSwapAccepts=0;
        dbg.snapMoveTrials=dbg.snapMoveAccepts=0;
        dbg.snapSlideTrials=dbg.snapSlideAccepts=0;
        dbg.snapIters=0;
    }
    if (dbg.dbgIter%1000==0) logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),curCost,dbg.dbgBestL,"checkpoint");
#endif
    return accepted;
}
static bool saJumpOp(vector<Polygon>& P, int n, const Boundary& bd,
                      double T, double T_init, double avgSize, double progress,
                      unordered_set<int>& overlappingSet, bool globalFeasible, double C,
                      double& curCost, double& bestCost,
                      vector<double>& bestTx, vector<double>& bestTy,
                      vector<int>& candidates
                      DEBUG_SA_STATS_PARAM)
{
    const double FAN_HALF_RAD = 45.0 * 3.14159265358979323846 / 180.0;
    const double JUMP_TEMP_FACTOR = 2.0;
    const double JUMP_MIN_DIST_FACTOR = 0.05;
#ifdef DEBUG_LOG
    dbg.snapJumpTrials++;
#endif
    vector<double> weights(n);
    double totalW = 0.0;
    for (int k = 0; k < n; k++) {
        double d = singleDist(P[k]);
        double s = P[k].hw + P[k].hh;
        if (s < 1e-9) s = 1e-9;
        weights[k] = d / pow(s, JUMP_SIZE_WEIGHT) + 1e-9;
        totalW += weights[k];
    }
    double rPick = randDouble() * totalW;
    double acc = 0.0;
    int idx = 0;
    for (int k = 0; k < n; k++) {
        acc += weights[k];
        if (acc >= rPick) { idx = k; break; }
    }
    double otx = P[idx].tx, oty = P[idx].ty;
    double dist = sqrt(otx*otx + oty*oty) ;    if (dist < avgSize * JUMP_MIN_DIST_FACTOR) return false;
    double oldPenalty = 0.0;
    if (!globalFeasible && overlappingSet.count(idx)) {
        g_hash.getCandidates(idx, P, candidates);
        double maxMTV_unused;
        oldPenalty = calcSingleOldPenalty(P, idx, candidates, maxMTV_unused);
    }
    bool useDirectedJump = true;
    double jumpAng;
    double jumpDist;
    if (useDirectedJump) {
        double baseAng = atan2(-oty, -otx);
        double deltaAng = (randDouble() - 0.5) * 2.0 * FAN_HALF_RAD;
        jumpAng = baseAng + deltaAng;
        jumpDist = dist * (0.3 + 0.7 * randDouble());
    } else {
        g_hash.getCandidates(idx, P, candidates);
        double nearAvgSize = 0.0;
        double maxObstacleSize = 0.0;
        int nearCount = 0;
        for (int j : candidates) {
            double dx = P[j].cx() - P[idx].cx();
            double dy = P[j].cy() - P[idx].cy();
            double distSq = dx*dx + dy*dy;
            double range = 4.0 * (P[idx].hw + P[idx].hh);
            if (distSq < range * range) {
                double objSize = 2.0 * fmax(P[j].hw, P[j].hh);
                nearAvgSize += objSize;
                if (objSize > maxObstacleSize) maxObstacleSize = objSize;
                nearCount++;
            }
        }
        if (nearCount > 0) nearAvgSize /= nearCount;
        jumpAng = randDouble() * 6.283185307;
        double minJump = fmax(nearAvgSize, avgSize * 0.25);
        double maxJump = fmax(maxObstacleSize * 1.2, avgSize * 0.5);
        if (nearCount == 0) {
            minJump = avgSize * 0.3;
            maxJump = avgSize * 0.8;
        }
        double temperatureFactor = T / T_init;
        double randomFactor = randDouble();
        double positionInRange = temperatureFactor * 0.7 + randomFactor * 0.3;
        positionInRange = fmax(0.0, fmin(1.0, positionInRange));
        jumpDist = minJump + (maxJump - minJump) * positionInRange;
        double absoluteMax = avgSize * 2.0;
        if (jumpDist > absoluteMax) jumpDist = absoluteMax;
    }
    double dirX = cos(jumpAng), dirY = sin(jumpAng);
    double maxDist = 1e18;
    if (dirX > 1e-9)  maxDist = fmin(maxDist, (bd.xMax - P[idx].hw - P[idx].cx()) / dirX);
    if (dirX < -1e-9) maxDist = fmin(maxDist, (bd.xMin + P[idx].hw - P[idx].cx()) / dirX);
    if (dirY > 1e-9)  maxDist = fmin(maxDist, (bd.yMax - P[idx].hh - P[idx].cy()) / dirY);
    if (dirY < -1e-9) maxDist = fmin(maxDist, (bd.yMin + P[idx].hh - P[idx].cy()) / dirY);
    if (maxDist < 1e-6) return false;
    jumpDist = fmin(jumpDist, maxDist * 0.98);    if (jumpDist < 1e-6) return false;
    double newTx = otx + dirX * jumpDist;
    double newTy = oty + dirY * jumpDist;
    g_hash.remove(idx, P);
    P[idx].tx = newTx;
    P[idx].ty = newTy;
    double oldLen = dist;
    bool accepted = false;
    if (exactInBounds(idx, P[idx], bd)) {
        g_hash.getCandidates(idx, P, candidates);
        bool conflict = false;
        for (int j : candidates)
            if (fastOverlap(idx, j, P[idx], P[j])) { conflict = true; break; }
        if (!conflict) {
            double newLen = singleDist(P[idx]);
            double delta = (newLen - oldLen) - C * oldPenalty;
            double jumpT = T * JUMP_TEMP_FACTOR;
            if (delta < 0.0 || randDouble() < exp(-delta / fmax(jumpT, 1e-15))) {
                accepted = true;
                curCost += delta;
                overlappingSet.erase(idx);
#ifdef DEBUG_LOG
                dbg.snapJumpAccepts++;
#endif
                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
#ifdef DEBUG_LOG
                    dbg.dbgBestL = bestCost;
                    logLine(dbg.phaseName, dbg.dbgIter, -1, elapsed(), bestCost, dbg.dbgBestL, "best_update(jump)");
#endif
                }
            }
        } else {
            Vertex mtv = {0.0, 0.0};
            bool mtvValid = false;
            for (int j : candidates) {
                if (fastOverlap(j, idx, P[j], P[idx], mtv, mtvValid)) {
                    if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7))
                        break;
                }
            }
            if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7)) {
                double corrTx = P[idx].tx + mtv.x * 1.001;
                double corrTy = P[idx].ty + mtv.y * 1.001;
                P[idx].tx = corrTx;
                P[idx].ty = corrTy;
                if (exactInBounds(idx, P[idx], bd)) {
                    g_hash.getCandidates(idx, P, candidates);
                    bool still = false;
                    for (int j : candidates)
                        if (fastOverlap(idx, j, P[idx], P[j])) { still = true; break; }
                    if (!still) {
                        double newLen = singleDist(P[idx]);
                        double delta = (newLen - oldLen) - C * oldPenalty;
                        double jumpT = T * JUMP_TEMP_FACTOR;
                        if (delta < 0.0 || randDouble() < exp(-delta / fmax(jumpT, 1e-15))) {
                            accepted = true;
                            curCost += delta;
                            overlappingSet.erase(idx);
#ifdef DEBUG_LOG
                            dbg.snapJumpAccepts++;
#endif
                            if (curCost < bestCost) {
                                bestCost = curCost;
                                for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
#ifdef DEBUG_LOG
                                dbg.dbgBestL = bestCost;
                                logLine(dbg.phaseName, dbg.dbgIter, -1, elapsed(), bestCost, dbg.dbgBestL, "best_update(jump_mtv)");
#endif
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx = otx; P[idx].ty = oty; }
    g_hash.insert(idx, P);
#ifdef DEBUG_LOG
    double nowSnap = elapsed();
    if (nowSnap - dbg.lastSnapshotTime >= 1.0) {
        timerSASnapshot(dbg.phaseName, dbg.lastT, dbg.lastStep, curCost, bestCost, dbg.dbgIter,
                        dbg.snapSwapTrials, dbg.snapSwapAccepts,
                        dbg.snapMoveTrials, dbg.snapMoveAccepts,
                        dbg.snapSlideTrials, dbg.snapSlideAccepts,
                        dbg.snapJumpTrials, dbg.snapJumpAccepts);
        dbg.lastSnapshotTime = nowSnap;
        dbg.snapSwapTrials=dbg.snapSwapAccepts=0;
        dbg.snapMoveTrials=dbg.snapMoveAccepts=0;
        dbg.snapSlideTrials=dbg.snapSlideAccepts=0;
        dbg.snapJumpTrials=dbg.snapJumpAccepts=0;
        dbg.snapIters=0;
    }
    if (dbg.dbgIter%1000==0) logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),curCost,dbg.dbgBestL,"checkpoint");
#endif
    return accepted;
}
static double globalJumpProb(double progress) {
    if (progress < 0.3) {
        return 0.6;
    } else if (progress < 0.7) {
        return 0.6 - 0.4 * (progress - 0.3) / 0.4;
    } else {
        return 0.2;
    }
}
static bool randomJumpOp(vector<Polygon>& P, int n, const Boundary& bd,
                          double T, double avgSize, double progress,
                          unordered_set<int>& overlappingSet, double C,
                          double& curCost, double& bestCost,
                          vector<double>& bestTx, vector<double>& bestTy,
                          vector<int>& candidates
                          DEBUG_SA_STATS_PARAM)
{
    if (overlappingSet.empty()) return false;
#ifdef DEBUG_LOG
    dbg.snapRandomJumpTrials++;
#endif
    int idx = -1;
    double oldPenalty = 0.0;
    double maxMTV = 0.0;
    for (int i : overlappingSet) {
        g_hash.getCandidates(i, P, candidates);
        double maxMTV_i;
        double penalty_i = calcSingleOldPenalty(P, i, candidates, maxMTV_i);
        if (penalty_i > oldPenalty) {
            oldPenalty = penalty_i;
            idx = i;
            maxMTV = maxMTV_i;
        }
    }
    if (idx < 0) return false;
    bool useGlobalJump = (randDouble() < globalJumpProb(progress));
    double jumpDist;
    double dirX, dirY;
    if (useGlobalJump) {
        double validXMin = bd.xMin + P[idx].hw;
        double validXMax = bd.xMax - P[idx].hw;
        double validYMin = bd.yMin + P[idx].hh;
        double validYMax = bd.yMax - P[idx].hh;
        if (validXMin > validXMax || validYMin > validYMax) {
            return false;
        }
        double targetCx = 0, targetCy = 0;
        bool found = false;
        for (int attempt = 0; attempt < 100; ++attempt) {
            double cx = validXMin + randDouble() * (validXMax - validXMin);
            double cy = validYMin + randDouble() * (validYMax - validYMin);
            if (isFeasible(idx, {cx - P[idx].origCx, cy - P[idx].origCy})) {
                targetCx = cx;
                targetCy = cy;
                found = true;
                break;
            }
        }
        if (!found) return false;
        double curCx = P[idx].cx();
        double curCy = P[idx].cy();
        double dx = targetCx - curCx;
        double dy = targetCy - curCy;
        jumpDist = sqrt(dx*dx + dy*dy);
        if (jumpDist < 1e-6) {
            return false;
        }
        dirX = dx / jumpDist;
        dirY = dy / jumpDist;
    } else {
        double jumpAng = randDouble() * 6.283185307;
        dirX = cos(jumpAng);
        dirY = sin(jumpAng);
        if (randDouble() < 0.5) {
            if (maxMTV < 1e-9) maxMTV = avgSize * 0.5;
            double k = 1.1 + 0.4 * randDouble();
            jumpDist = k * maxMTV;
        } else {
            g_hash.getCandidates(idx, P, candidates);
            double nearSizeSum = 0.0;
            int nearCount = 0;
            double range = 4.0 * (P[idx].hw + P[idx].hh);
            for (int j : candidates) {
                double dx = P[j].cx() - P[idx].cx();
                double dy = P[j].cy() - P[idx].cy();
                if (dx * dx + dy * dy < range * range) {
                    nearSizeSum += 2.0 * fmax(P[j].hw, P[j].hh);
                    nearCount++;
                }
            }
            double avgNearSize = (nearCount > 0) ? nearSizeSum / nearCount : avgSize;
            if (avgNearSize < 1e-9) avgNearSize = avgSize;
            double k = 1.0 + 1.0 * randDouble();
            jumpDist = k * avgNearSize;
        }
    }
    double maxDist = 1e18;
    if (dirX > 1e-9)  maxDist = fmin(maxDist, (bd.xMax - P[idx].hw - P[idx].cx()) / dirX);
    if (dirX < -1e-9) maxDist = fmin(maxDist, (bd.xMin + P[idx].hw - P[idx].cx()) / dirX);
    if (dirY > 1e-9)  maxDist = fmin(maxDist, (bd.yMax - P[idx].hh - P[idx].cy()) / dirY);
    if (dirY < -1e-9) maxDist = fmin(maxDist, (bd.yMin + P[idx].hh - P[idx].cy()) / dirY);
    if (maxDist < 1e-6) return false;
    jumpDist = fmin(jumpDist, maxDist * 0.98);
    if (jumpDist < 1e-6) return false;
    double otx = P[idx].tx, oty = P[idx].ty;
    double oldLen = singleDist(P[idx]);
    g_hash.remove(idx, P);
    P[idx].tx = otx + dirX * jumpDist;
    P[idx].ty = oty + dirY * jumpDist;
    bool accepted = false;
    if (exactInBounds(idx, P[idx], bd)) {
        g_hash.getCandidates(idx, P, candidates);
        bool conflict = false;
        for (int j : candidates)
            if (fastOverlap(idx, j, P[idx], P[j])) { conflict = true; break; }
        if (!conflict) {
            double newLen = singleDist(P[idx]);
            double delta = (newLen - oldLen) - C * oldPenalty;
            if (delta < 0.0 || randDouble() < exp(-delta / fmax(T, 1e-15))) {
                accepted = true;
                curCost += delta;
                overlappingSet.erase(idx);
#ifdef DEBUG_LOG
                dbg.snapRandomJumpAccepts++;
#endif
                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
#ifdef DEBUG_LOG
                    dbg.dbgBestL = bestCost;
                    logLine(dbg.phaseName, dbg.dbgIter, -1, elapsed(), bestCost, dbg.dbgBestL, "best_update(rjump)");
#endif
                }
            }
        } else {
            Vertex mtv = {0.0, 0.0};
            bool mtvValid = false;
            for (int j : candidates) {
                if (fastOverlap(j, idx, P[j], P[idx], mtv, mtvValid)) {
                    if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7))
                        break;
                }
            }
            if (mtvValid && (fabs(mtv.x) > 1e-7 || fabs(mtv.y) > 1e-7)) {
                double corrTx = P[idx].tx + mtv.x * 1.001;
                double corrTy = P[idx].ty + mtv.y * 1.001;
                P[idx].tx = corrTx;
                P[idx].ty = corrTy;
                if (exactInBounds(idx, P[idx], bd)) {
                    g_hash.getCandidates(idx, P, candidates);
                    bool still = false;
                    for (int j : candidates)
                        if (fastOverlap(idx, j, P[idx], P[j])) { still = true; break; }
                    if (!still) {
                        double newLen = singleDist(P[idx]);
                        double delta = (newLen - oldLen) - C * oldPenalty;
                        if (delta < 0.0 || randDouble() < exp(-delta / fmax(T, 1e-15))) {
                            accepted = true;
                            curCost += delta;
                            overlappingSet.erase(idx);
#ifdef DEBUG_LOG
                            dbg.snapRandomJumpAccepts++;
#endif
                            if (curCost < bestCost) {
                                bestCost = curCost;
                                for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
#ifdef DEBUG_LOG
                                dbg.dbgBestL = bestCost;
                                logLine(dbg.phaseName, dbg.dbgIter, -1, elapsed(), bestCost, dbg.dbgBestL, "best_update(rjump_mtv)");
#endif
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx = otx; P[idx].ty = oty; }
    g_hash.insert(idx, P);
#ifdef DEBUG_LOG
    double nowSnap = elapsed();
    if (nowSnap - dbg.lastSnapshotTime >= 1.0) {
        timerSASnapshot(dbg.phaseName, dbg.lastT, dbg.lastStep, curCost, bestCost, dbg.dbgIter,
                        dbg.snapSwapTrials, dbg.snapSwapAccepts,
                        dbg.snapMoveTrials, dbg.snapMoveAccepts,
                        dbg.snapSlideTrials, dbg.snapSlideAccepts,
                        dbg.snapJumpTrials, dbg.snapJumpAccepts);
        dbg.lastSnapshotTime = nowSnap;
        dbg.snapSwapTrials=dbg.snapSwapAccepts=0;
        dbg.snapMoveTrials=dbg.snapMoveAccepts=0;
        dbg.snapSlideTrials=dbg.snapSlideAccepts=0;
        dbg.snapJumpTrials=dbg.snapJumpAccepts=0;
        dbg.snapIters=0;
    }
    if (dbg.dbgIter%1000==0) logLine(dbg.phaseName,dbg.dbgIter,-1,elapsed(),curCost,dbg.dbgBestL,"checkpoint");
#endif
    return accepted;
}
// ==== End src/op.h ====

// ==== Begin src/sa.h ====
#ifdef DEBUG_LOG
#endif
static const double PULL_ANGLES_DEG[] = {
    0.0,
    12.0, -12.0,
    24.0, -24.0,
    36.0, -36.0,
    48.0, -48.0,
    60.0, -60.0,
    72.0, -72.0,
    84.0, -84.0
};
static const int N_PULL_ANGLES = 15;
static const double PULL_PI       = 3.14159265358979323846;
static void shelfPlace_BL(const vector<int>& perm, vector<Polygon>& P, const Boundary& bd) {
    int n=(int)perm.size();
    double colX=bd.xMin, curY=bd.yMin, colWidth=0.0;
    for (int k=0; k<n; k++) {
        int i=perm[k];
        double h=2.0*P[i].hh, w=2.0*P[i].hw;
        if (curY+h > bd.yMax+1e-9) { colX+=colWidth; curY=bd.yMin; colWidth=0.0; }
        P[i].tx=(colX+P[i].hw)-P[i].origCx;
        P[i].ty=(curY+P[i].hh)-P[i].origCy;
        curY+=h; if (w>colWidth) colWidth=w;
    }
}
static void shelfPlace_LB(const vector<int>& perm, vector<Polygon>& P, const Boundary& bd) {
    int n=(int)perm.size();
    double rowY=bd.yMin, curX=bd.xMin, rowHeight=0.0;
    for (int k=0; k<n; k++) {
        int i=perm[k];
        double h=2.0*P[i].hh, w=2.0*P[i].hw;
        if (curX+w > bd.xMax+1e-9) { rowY+=rowHeight; curX=bd.xMin; rowHeight=0.0; }
        P[i].tx=(curX+P[i].hw)-P[i].origCx;
        P[i].ty=(rowY+P[i].hh)-P[i].origCy;
        curX+=w; if (h>rowHeight) rowHeight=h;
    }
}
static void shelfPlace(const vector<int>& perm, vector<Polygon>& P, const Boundary& bd) {
#ifdef DEBUG_LOG
    g_cntShelfPlace++;
#endif
    int n=(int)P.size();
    shelfPlace_BL(perm, P, bd);
    double L_BL = calcLPrime(P);
    vector<double> tx_BL(n), ty_BL(n);
    for (int i=0; i<n; i++) { tx_BL[i]=P[i].tx; ty_BL[i]=P[i].ty; }
    shelfPlace_LB(perm, P, bd);
    double L_LB = calcLPrime(P);
    if (L_BL < L_LB) {
        for (int i=0; i<n; i++) { P[i].tx=tx_BL[i]; P[i].ty=ty_BL[i]; }
    }
}
static double permCost(const vector<Polygon>& P, const Boundary& bd) {
    int n=(int)P.size(); double cost=0.0;
    double totalArea=(bd.xMax-bd.xMin)*(bd.yMax-bd.yMin);
    for (int i=0; i<n; i++) {
        double dx=P[i].tx, dy=P[i].ty;
        cost += sqrt(dx*dx+dy*dy);
        cost += totalArea*100.0*aabbOutArea(P[i], bd);
    }
    return cost;
}
static double evalAvgFreeMove(vector<Polygon>& P, const Boundary& bd) {
    int n = (int)P.size();
    static const double DIRS[8][2] = {
        {1,0},{-1,0},{0,1},{0,-1},
        {0.707,0.707},{-0.707,0.707},{0.707,-0.707},{-0.707,-0.707}
    };
    const int dirs = 8;
    const double PROBE_PREC = 5e-4;
    const double PROBE_MAX  = 1.0;
    vector<int> cands;
    double totalFree = 0.0;
    double minFree   = 1e18;
    double maxFree   = 0.0;
#ifdef DEBUG_LOG
    double t0 = elapsed();
    if (g_timerFile) {
        fprintf(g_timerFile,
            "%12.4f  [TIGHTNESS   ] %-20s  n=%d  dirs=%d  prec=%.0e  probing...\n",
            t0, "evalAvgFreeMove", n, dirs, PROBE_PREC);
        fflush(g_timerFile);
    }
#endif
    for (int idx = 0; idx < n; idx++) {
        double origTx = P[idx].tx, origTy = P[idx].ty;
        double polyFree = 0.0;
        g_hash.remove(idx, P);
        for (int d = 0; d < dirs; d++) {
            double nx = DIRS[d][0], ny = DIRS[d][1];
            double lo = 0.0, hi = PROBE_MAX;
            P[idx].tx = origTx + nx * hi;
            P[idx].ty = origTy + ny * hi;
            bool maxOk = exactInBounds(idx,P[idx], bd);
            if (maxOk) {
                g_hash.getCandidates(idx, P, cands);
                for (int j : cands)
                    if (fastOverlap(idx, j, P[idx], P[j])) { maxOk = false; break; }
            }
            if (maxOk) {
                lo = PROBE_MAX;
            } else {
                while (hi - lo > PROBE_PREC) {
                    double mid = (lo + hi) * 0.5;
                    P[idx].tx = origTx + nx * mid;
                    P[idx].ty = origTy + ny * mid;
                    bool ok = exactInBounds(idx,P[idx], bd);
                    if (ok) {
                        g_hash.getCandidates(idx, P, cands);
                        for (int j : cands)
                            if (fastOverlap(idx, j, P[idx], P[j])) { ok = false; break; }
                    }
                    if (ok) lo = mid; else hi = mid;
                }
            }
            polyFree += lo;
        }
        P[idx].tx = origTx;
        P[idx].ty = origTy;
        g_hash.insert(idx, P);
        double avg4 = polyFree / dirs;
        totalFree += avg4;
        if (avg4 < minFree) minFree = avg4;
        if (avg4 > maxFree) maxFree = avg4;
    }
    double avgFree = totalFree / n;
#ifdef DEBUG_LOG
    if (g_timerFile) {
        fprintf(g_timerFile,
            "%12.4f  [TIGHTNESS   ] %-20s"
            "  avgFreeMove=%10.6f"
            "  minFreeMove=%10.6f"
            "  maxFreeMove=%10.6f"
            "  cost=%.3fs\n",
            elapsed(), "evalAvgFreeMove",
            avgFree, minFree, maxFree,
            elapsed() - t0);
        fflush(g_timerFile);
    }
#endif
    return avgFree;
}
static void saOnPerm(vector<Polygon>& P, vector<int>& perm, const Boundary& bd) {
    int n=(int)perm.size();
    shelfPlace(perm, P, bd);
    double curCost=permCost(P,bd), bestCost=curCost;
    vector<int> bestPerm=perm;
    double avgSize=0.0;
    for (int i=0; i<n; i++) avgSize+=P[i].hw+P[i].hh;
    avgSize/=n;
    double T=fmax(curCost*0.3, avgSize*0.5);
#ifdef DEBUG_LOG
    logPhaseHeader("saOnPerm");
    long long dbgIter=0; double dbgBestL=calcLPrime(P);
    logLine("saOnPerm",0,-1,elapsed(),dbgBestL,dbgBestL,"snapshot");
    double lastSnap = elapsed();
    const double SNAP_INTERVAL = 1.0;
    long long snapIter = 0;
    double snapCostSum = 0.0;
#endif
    int iter=0;
    vector<double> savedTx(n), savedTy(n);
    while (T > SA_TMIN) {
        iter++;
        if (iter%500==0 && elapsed()>T_SA_PERM) break;
#ifdef DEBUG_LOG
        dbgIter++;
        snapIter++;
#endif
        vector<int> savedPerm=perm;
        for (int i=0;i<n;i++){savedTx[i]=P[i].tx;savedTy[i]=P[i].ty;}
        double r=randDouble();
        if (r < 1.0/3.0) {
            int i=randInt(n), j=randInt(n);
            while (n>=2&&j==i) j=randInt(n);
            swap(perm[i],perm[j]);
        } else if (r < 2.0/3.0) {
            int i=randInt(n), j=randInt(n);
            if (i>j) swap(i,j);
            reverse(perm.begin()+i, perm.begin()+j+1);
        } else {
            int i=randInt(n), j=randInt(n), val=perm[i];
            perm.erase(perm.begin()+i);
            if (j>=(int)perm.size()) j=(int)perm.size();
            perm.insert(perm.begin()+j, val);
        }
        shelfPlace(perm, P, bd);
        double newCost=permCost(P,bd), delta=newCost-curCost;
        if (delta<0.0 || randDouble()<exp(-delta/T)) {
            curCost=newCost;
            if (curCost<bestCost) {
                bestCost=curCost; bestPerm=perm;
#ifdef DEBUG_LOG
                double curL=calcLPrime(P); dbgBestL=fmin(dbgBestL,curL);
                logLine("saOnPerm",dbgIter,-1,elapsed(),curL,dbgBestL,"best_update");
#endif
            }
        } else { perm=savedPerm; for (int i=0;i<n;i++){P[i].tx=savedTx[i];P[i].ty=savedTy[i];} }
#ifdef DEBUG_LOG
        if (dbgIter%1000==0) {
            double curL=calcLPrime(P);
            logLine("saOnPerm",dbgIter,-1,elapsed(),curL,dbgBestL,"checkpoint");
        }
        double nowSnap = elapsed();
        if (nowSnap - lastSnap >= SNAP_INTERVAL) {
            double curL = calcLPrime(P);
            if (g_timerFile) {
                fprintf(g_timerFile,
                    "%12.4f  [SNAPSHOT     ] %-20s  T=%10.4f  curCost(perm)=%12.6f  curL'=%12.6f  bestL'=%12.6f  iters_in_snap=%lld\n",
                    nowSnap, "saOnPerm", T, curCost, curL, dbgBestL, snapIter);
                fflush(g_timerFile);
            }
            lastSnap = nowSnap;
            snapIter = 0;
        }
#endif
        T *= SA_PERM_ALPHA;
    }
    perm=bestPerm; shelfPlace(perm,P,bd);
#ifdef DEBUG_LOG
    logPhaseSummary("saOnPerm",dbgIter,-1,elapsed());
#endif
}
static void aabbPullback(vector<Polygon>& P, const Boundary& bd) {
    int n=(int)P.size();
    vector<int> order(n); for (int i=0;i<n;i++) order[i]=i;
#ifdef DEBUG_LOG
    logPhaseHeader("aabbPullback");
    int dbgRound=0; double dbgBestL=calcLPrime(P);
    logLine("aabbPullback",-1,0,elapsed(),dbgBestL,dbgBestL,"snapshot");
#endif
    bool improved=true;
    while (improved && elapsed()<T_AABB_PULL) {
        improved=false;
#ifdef DEBUG_LOG
        dbgRound++;
        double tRoundStart = elapsed();
        double lBeforeRound = calcLPrime(P);
#endif
        sort(order.begin(),order.end(),[&](int a,int b){return singleDist(P[a])>singleDist(P[b]);});
        for (int idx:order) {
            if (elapsed()>T_AABB_PULL) break;
            double dist=singleDist(P[idx]); if (dist<AABB_PREC) continue;
            double nx=-P[idx].tx/dist, ny=-P[idx].ty/dist;
            double lo=0.0, hi=dist, origTx=P[idx].tx, origTy=P[idx].ty;
            while (hi-lo>AABB_PREC) {
                double mid=(lo+hi)*0.5;
                P[idx].tx=origTx+nx*mid; P[idx].ty=origTy+ny*mid;
                bool ok=exactInBounds(idx, P[idx], bd);
                for (int j=0;j<n&&ok;j++) if(j!=idx&&aabbOverlap(P[idx],P[j])) ok=false;
                if (ok) lo=mid; else hi=mid;
            }
            P[idx].tx=origTx+nx*lo; P[idx].ty=origTy+ny*lo;
            if (lo>AABB_PREC) improved=true;
        }
#ifdef DEBUG_LOG
        double curL=calcLPrime(P); dbgBestL=fmin(dbgBestL,curL);
        logLine("aabbPullback",-1,dbgRound,elapsed(),curL,dbgBestL,"round");
        timerRound("aabbPullback", dbgRound, tRoundStart, lBeforeRound, curL);
#endif
    }
#ifdef DEBUG_LOG
    logPhaseSummary("aabbPullback",-1,dbgRound,elapsed());
#endif
}
static void exactPullback(vector<Polygon>& P, const Boundary& bd,
                          double timeLimit, const char* phaseName = "exactPullback") {
    int n=(int)P.size();
    vector<int> order(n); for (int i=0;i<n;i++) order[i]=i;
    vector<int> cands; cands.reserve(32);
#ifdef DEBUG_LOG
    logPhaseHeader(phaseName);
    int dbgRound=0; double dbgBestL=calcLPrime(P);
    logLine(phaseName,-1,0,elapsed(),dbgBestL,dbgBestL,"snapshot");
#endif
    bool improved=true;
    while (improved && elapsed()<timeLimit) {
        improved=false;
#ifdef DEBUG_LOG
        dbgRound++;
        double tRoundStart = elapsed();
        double lBeforeRound = calcLPrime(P);
#endif
        sort(order.begin(),order.end(),[&](int a,int b){return singleDist(P[a])>singleDist(P[b]);});
        for (int idx:order) {
            if (elapsed()>timeLimit) break;
            double origTx=P[idx].tx, origTy=P[idx].ty;
            double dist=sqrt(origTx*origTx+origTy*origTy);
            if (dist<EXACT_PREC) continue;
            double bx=-origTx/dist, by=-origTy/dist;
            double bestGain=EXACT_PREC, bestNewTx=origTx, bestNewTy=origTy;
            g_hash.remove(idx,P);
            for (int ai=0; ai<N_PULL_ANGLES; ai++) {
                double angle=PULL_ANGLES_DEG[ai]*PULL_PI/180.0;
                double nx=bx*cos(angle)-by*sin(angle), ny=bx*sin(angle)+by*cos(angle);
                double lo=0.0, hi=dist*1.2;
                while (hi-lo>EXACT_PREC) {
                    double mid=(lo+hi)*0.5;
                    P[idx].tx=origTx+nx*mid; P[idx].ty=origTy+ny*mid;
                    g_hash.getCandidates(idx,P,cands);
                    bool ok=exactInBounds(idx, P[idx],bd);
                    for (int j:cands) { if(!ok) break; if(fastOverlap(idx,j,P[idx],P[j])) ok=false; }
                    if (ok) lo=mid; else hi=mid;
                }
                double finalTx = origTx + nx * lo;
                double finalTy = origTy + ny * lo;
                P[idx].tx = origTx + nx * hi;
                P[idx].ty = origTy + ny * hi;
                g_hash.getCandidates(idx, P, cands);
                Vertex mtv = {0, 0};
                bool mtvValid = false;
                bool foundBlocker = false;
                for (int j : cands) {
                    if (fastOverlap(j, idx, P[j], P[idx], mtv, mtvValid)) {
                        if (mtvValid && (abs(mtv.x) > 1e-6 || abs(mtv.y) > 1e-6)) {
                            foundBlocker = true;
                            break;
                        }
                    }
                }
                if (foundBlocker) {
                    struct MtvBuf { double nx, ny, dist; };
                    MtvBuf mtvs[4]; int mtvCnt = 0;
                    for (int j : cands) {
                        Vertex mv; bool mvValid = false;
                        if (fastOverlap(j, idx, P[j], P[idx], mv, mvValid) && mvValid) {
                            double d = sqrt(mv.x*mv.x + mv.y*mv.y);
                            if (d > 1e-7 && mtvCnt < 4)
                                mtvs[mtvCnt++] = {mv.x/d, mv.y/d, d};
                        }
                    }
                    double cx = 0.0, cy = 0.0; bool hasCorr = false;
                    if (mtvCnt == 1) {
                        cx = mtvs[0].nx * mtvs[0].dist;
                        cy = mtvs[0].ny * mtvs[0].dist;
                        hasCorr = true;
                    } else if (mtvCnt >= 2) {
                        double g   = mtvs[0].nx*mtvs[1].nx + mtvs[0].ny*mtvs[1].ny;
                        double den = 1.0 - g*g;
                        if (den < 1e-4) {
                            if (g > 0) {
                                MtvBuf& m = (mtvs[0].dist >= mtvs[1].dist) ? mtvs[0] : mtvs[1];
                                cx = m.nx*m.dist; cy = m.ny*m.dist; hasCorr = true;
                            }
                        } else {
                            double l0 = (mtvs[0].dist - mtvs[1].dist*g) / den;
                            double l1 = (mtvs[1].dist - mtvs[0].dist*g) / den;
                            if      (l0 < 0) { cx = mtvs[1].nx*mtvs[1].dist; cy = mtvs[1].ny*mtvs[1].dist; }
                            else if (l1 < 0) { cx = mtvs[0].nx*mtvs[0].dist; cy = mtvs[0].ny*mtvs[0].dist; }
                            else             { cx = l0*mtvs[0].nx+l1*mtvs[1].nx; cy = l0*mtvs[0].ny+l1*mtvs[1].ny; }
                            hasCorr = true;
                        }
                    }
                    if (hasCorr && cx*cx+cy*cy > 1e-14) {
                        double corrTx = P[idx].tx + cx * 1.001;
                        double corrTy = P[idx].ty + cy * 1.001;
                        P[idx].tx = corrTx; P[idx].ty = corrTy;
                        if (exactInBounds(idx,P[idx], bd)) {
                            g_hash.getCandidates(idx, P, cands);
                            bool still = false;
                            for (int j : cands)
                                if (fastOverlap(idx, j, P[idx], P[j])) { still = true; break; }
                            if (!still) {
                                double gainCorr = dist - sqrt(corrTx*corrTx + corrTy*corrTy);
                                double gainBin  = dist - sqrt(finalTx*finalTx + finalTy*finalTy);
                                if (gainCorr > gainBin) {
                                    finalTx = corrTx;
                                    finalTy = corrTy;
                                }
                            }
                        }
                    }
                }
                P[idx].tx=origTx; P[idx].ty=origTy;
                double gain=dist-sqrt(finalTx*finalTx+finalTy*finalTy);
                if (gain>bestGain) { bestGain=gain; bestNewTx=finalTx; bestNewTy=finalTy; }
            }
            if (bestGain>EXACT_PREC) { P[idx].tx=bestNewTx; P[idx].ty=bestNewTy; improved=true; }
            g_hash.insert(idx,P);
        }
#ifdef DEBUG_LOG
        double curL=calcLPrime(P); dbgBestL=fmin(dbgBestL,curL);
        logLine(phaseName,-1,dbgRound,elapsed(),curL,dbgBestL,"round");
        timerRound(phaseName, dbgRound, tRoundStart, lBeforeRound, curL);
#endif
    }
#ifdef DEBUG_LOG
    logPhaseSummary(phaseName,-1,dbgRound,elapsed());
#endif
}
static void positionSAHigh(vector<Polygon>& P, const Boundary& bd,
                           double T_init, double T_end, double time_limit)
{
    const double towardOriginProb = MOVE_BACK_PROB_HIGH;
    const char*  phaseName        = "positionSA_high";
    const double stepInitFactor   = 0.20;
    const bool   enableMTVSlide   = true;
    int n = (int)P.size();
    int iterPerRound = max((int)pow((double)n, 1.25), 400);
    int roundPerCyc = RONND_PRE_CYC;
    long long iterPerCycle = (long long)roundPerCyc * iterPerRound;
    long long iteration = 0;
    double T_init_cycle = T_init;
    double T_end_cycle = T_end;
    g_hash.rebuild(bd, P, buildCellSize(P));
    unordered_set<int> overlappingSet;
    bool globalFeasible = false;
    double C = 1.0;
    const double C_min = 3.0;
    double bestFeasibleCost = 1e18;
    vector<double> bestFeasibleTx(n), bestFeasibleTy(n);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (fastOverlap(i, j, P[i], P[j])) {
                overlappingSet.insert(i);
                overlappingSet.insert(j);
            }
        }
    }
    globalFeasible = overlappingSet.empty();
    double penaltyValue = calcOverlapPenalty(P, n);
    double curCost  = calcLPrime(P) + C * penaltyValue;
    double bestCost = curCost;
    vector<double> bestTx(n), bestTy(n);
    for (int i=0; i<n; i++) { bestTx[i]=P[i].tx; bestTy[i]=P[i].ty; }
    if (globalFeasible) {
        bestFeasibleCost = calcLPrime(P);
        for (int i=0; i<n; i++) { bestFeasibleTx[i]=P[i].tx; bestFeasibleTy[i]=P[i].ty; }
    }
    double tStart = elapsed();
    double tTotal = time_limit - tStart;
    if (tTotal <= 0.0) return;
    double avgSize = 0.0;
    for (auto& p : P) avgSize += 2.0 * fmax(p.hw, p.hh);
    avgSize /= n;
    double stepInit = avgSize * stepInitFactor;
    vector<int> candidates; candidates.reserve(32);
#ifdef DEBUG_LOG
    logPhaseHeader(phaseName);
    DebugSAStats dbg;
    dbg.dbgIter = 0;
    dbg.dbgBestL = curCost;
    dbg.snapSwapTrials = 0; dbg.snapSwapAccepts = 0;
    dbg.snapMoveTrials = 0; dbg.snapMoveAccepts = 0;
    dbg.snapSlideTrials = 0; dbg.snapSlideAccepts = 0;
    dbg.snapJumpTrials = 0; dbg.snapJumpAccepts = 0;
    dbg.snapRandomJumpTrials = 0; dbg.snapRandomJumpAccepts = 0;
    dbg.snapIters = 0;
    dbg.lastT = T_init;
    dbg.lastStep = stepInit;
    dbg.lastSnapshotTime = tStart;
    dbg.phaseName = phaseName;
    logLine(phaseName, 0, -1, elapsed(), curCost, dbg.dbgBestL, "snapshot");
    if (g_logFile) {
        fprintf(g_logFile,
            "# T_init=%.6f  T_end=%.2e  time_limit=%.1f"
            "  towardOriginProb=%.2f  avgSize=%.4f"
            "  stepInitFactor=%.4f  stepInit=%.6f"
            "  enableMTVSlide=%d  JUMP_SIZE_WEIGHT=%.2f\n",
            T_init, T_end, time_limit,
            towardOriginProb, avgSize,
            stepInitFactor, stepInit,
            (int)enableMTVSlide, JUMP_SIZE_WEIGHT);
        fflush(g_logFile);
    }
    if (g_timerFile) {
        fprintf(g_timerFile,
            "%12.4f  [SA_PARAMS   ] %-20s"
            "  stepInitFactor=%.4f  stepInit=%.6f"
            "  T_init=%.6f  T_end=%.2e"
            "  enableMTVSlide=%d  JUMP_SIZE_WEIGHT=%.2f\n",
            elapsed(), phaseName,
            stepInitFactor, stepInit,
            T_init, T_end,
            (int)enableMTVSlide, JUMP_SIZE_WEIGHT);
        fflush(g_timerFile);
    }
#endif
    int iterCheck = 0;
    double nowElapsed = elapsed();
    while (true) {
        iterCheck++;
        if (iterCheck % 10000 == 0) {
            nowElapsed = elapsed();
            if (nowElapsed >= time_limit) break;
        }
        double progress = (nowElapsed - tStart) / tTotal;
        if (progress >= 1.0) break;
        iteration++;
        long long iteration_in_cycle = (iteration - 1) % iterPerCycle + 1;
        if (iteration_in_cycle == 1) {
            g_rng.seed(std::random_device{}());
            double L_current = calcLPrime(P);
            T_init_cycle = L_current * 0.03;
            T_end_cycle = L_current * 0.001;
            if (!globalFeasible) {
                double avgDisplacement = L_current / n;
                double totalMTV = 0.0;
                int overlapPairCount = 0;
                for (int i = 0; i < n; i++) {
                    for (int j = i + 1; j < n; j++) {
                        Vertex mtv; bool mtvValid;
                        if (fastOverlap(i, j, P[i], P[j], mtv, mtvValid)) {
                            overlapPairCount++;
                            if (mtvValid) totalMTV += sqrt(mtv.x * mtv.x + mtv.y * mtv.y);
                        }
                    }
                }
                if (overlapPairCount > 0) {
                    double avgMTV = totalMTV / overlapPairCount;
                    C = fmax(C_min, avgDisplacement / avgMTV);
                } else {
                    C = C_min;
                }
            }
        }
        double progress_cycle = iteration_in_cycle / (double)iterPerCycle;
        double T = T_init_cycle * pow(fmax(T_end_cycle / T_init_cycle, 1e-15), progress_cycle);
        double step = fmax(avgSize * 0.005,
                           avgSize * stepInitFactor * (1.0 - progress_cycle * 0.975));
#ifdef DEBUG_LOG
        dbg.dbgIter++;
        dbg.snapIters++;
        dbg.lastT    = T;
        dbg.lastStep = step;
#endif
        double jumpProb = 0.0;
        if (progress_cycle > JUMP_BEGIN_PROG_HIGH) {
            jumpProb = JUMP_BACK_PROB_HIGH * (progress_cycle - JUMP_BEGIN_PROG_HIGH) / (1-JUMP_BEGIN_PROG_HIGH);
        }
        double randomJumpProb = 0.0;
        if (progress > 0.05) {
            if (!globalFeasible && !overlappingSet.empty()) {
                double overlapRatio = (double)overlappingSet.size() / n;
                randomJumpProb = fmin(0.2, 0.1 + 0.1 * overlapRatio);
        }          
        }
        double randV = randDouble();
        if (randV < 0.20 && n >= 2) {
            saSwapOp(P, n, bd, T, progress_cycle,
                     overlappingSet, globalFeasible, C,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        } else if (randV < 0.20 + jumpProb) {
            saJumpOp(P, n, bd, T, T_init, avgSize, progress_cycle,
                     overlappingSet, globalFeasible, C,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        } else if (randV < 0.20 + jumpProb + randomJumpProb) {
            randomJumpOp(P, n, bd, T, avgSize, progress_cycle,
                         overlappingSet, C,
                         curCost, bestCost, bestTx, bestTy, candidates
                         DEBUG_SA_STATS_ARG);
        } else {
            saMoveOp(P, n, bd, T, step, progress_cycle,
                     towardOriginProb, enableMTVSlide,
                     overlappingSet, globalFeasible, C,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        }
        if (!globalFeasible && overlappingSet.empty()) {
            globalFeasible = true;
        }
        if (globalFeasible && overlappingSet.empty()) {
            double curL = calcLPrime(P);
            if (curL < bestFeasibleCost) {
                bestFeasibleCost = curL;
                for (int i = 0; i < n; i++) { bestFeasibleTx[i] = P[i].tx; bestFeasibleTy[i] = P[i].ty; }
            }
        }
    }
    bool hasResidual = false;
    for (int i = 0; i < n && !hasResidual; i++) {
        for (int j = i + 1; j < n; j++) {
            if (fastOverlap(i, j, P[i], P[j])) { hasResidual = true; break; }
        }
    }
    if (hasResidual) {
        bool sweepOk = false;
        for (int iter = 0; iter < 20; iter++) {
            bool found = false;
            for (int i = 0; i < n && !found; i++) {
                for (int j = i + 1; j < n; j++) {
                    Vertex mtv; bool mtvValid;
                    if (fastOverlap(i, j, P[i], P[j], mtv, mtvValid) && mtvValid) {
                        double mtvLen = sqrt(mtv.x * mtv.x + mtv.y * mtv.y);
                        if (mtvLen > 1e-7) {
                            double savedJtx = P[j].tx, savedJty = P[j].ty;
                            P[j].tx += mtv.x * 1.001;
                            P[j].ty += mtv.y * 1.001;
                            if (!exactInBounds(j, P[j], bd)) {
                                P[j].tx = savedJtx; P[j].ty = savedJty;
                                double savedItx = P[i].tx, savedIty = P[i].ty;
                                P[i].tx -= mtv.x * 1.001;
                                P[i].ty -= mtv.y * 1.001;
                                if (!exactInBounds(i, P[i], bd)) {
                                    P[i].tx = savedItx; P[i].ty = savedIty;
                                }
                            }
                        }
                        found = true;
                        break;
                    }
                }
            }
            if (!found) { sweepOk = true; break; }
            bool stillHas = false;
            for (int i = 0; i < n && !stillHas; i++) {
                for (int j = i + 1; j < n; j++) {
                    if (fastOverlap(i, j, P[i], P[j])) { stillHas = true; break; }
                }
            }
            if (!stillHas) { sweepOk = true; break; }
        }
        if (sweepOk) {
            bool allInBounds = true;
            for (int i = 0; i < n && allInBounds; i++) {
                if (!exactInBounds(i, P[i], bd)) allInBounds = false;
            }
            if (allInBounds) {
                double L = calcLPrime(P);
                if (L < bestFeasibleCost) {
                    bestFeasibleCost = L;
                    for (int i = 0; i < n; i++) { bestFeasibleTx[i] = P[i].tx; bestFeasibleTy[i] = P[i].ty; }
                }
            } else {
                if (bestFeasibleCost < 1e18) {
                    for (int i = 0; i < n; i++) { P[i].tx = bestFeasibleTx[i]; P[i].ty = bestFeasibleTy[i]; }
                }
            }
        } else {
            if (bestFeasibleCost < 1e18) {
                for (int i = 0; i < n; i++) { P[i].tx = bestFeasibleTx[i]; P[i].ty = bestFeasibleTy[i]; }
            } else {
                for (int iter = 0; iter < 80; iter++) {
                    bool found = false;
                    for (int i = 0; i < n && !found; i++) {
                        for (int j = i + 1; j < n; j++) {
                            Vertex mtv; bool mtvValid;
                            if (fastOverlap(i, j, P[i], P[j], mtv, mtvValid) && mtvValid) {
                                double mtvLen = sqrt(mtv.x * mtv.x + mtv.y * mtv.y);
                                if (mtvLen > 1e-7) {
                                    P[j].tx += mtv.x * 1.001;
                                    P[j].ty += mtv.y * 1.001;
                                }
                                found = true; break;
                            }
                        }
                    }
                    if (!found) break;
                }
            }
        }
    }
    if (bestFeasibleCost < 1e18) {
        for (int i = 0; i < n; i++) { P[i].tx = bestFeasibleTx[i]; P[i].ty = bestFeasibleTy[i]; }
    } else {
        for (int i = 0; i < n; i++) { P[i].tx = bestTx[i]; P[i].ty = bestTy[i]; }
    }
#ifdef DEBUG_LOG
    logPhaseSummary(phaseName, dbg.dbgIter, -1, elapsed());
#endif
}
static void positionSALow(vector<Polygon>& P, const Boundary& bd,
                          double T_init, double T_end, double time_limit,
                          double stepInitFactor)
{
    const double towardOriginProb = 0.60;
    const char*  phaseName        = "positionSA_low";
    const bool   enableMTVSlide   = true;
    int n = (int)P.size();
    g_hash.rebuild(bd, P, buildCellSize(P));
    unordered_set<int> overlappingSetLow;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (fastOverlap(i, j, P[i], P[j])) {
                overlappingSetLow.insert(i);
                overlappingSetLow.insert(j);
            }
        }
    }
    bool globalFeasibleLow = overlappingSetLow.empty();
    double CLow = globalFeasibleLow ? 1.0 : 3.0;
    double curCost  = calcLPrime(P) + (globalFeasibleLow ? 0.0 : CLow * calcOverlapPenalty(P, n));
    double bestCost = curCost;
    vector<double> bestTx(n), bestTy(n);
    for (int i=0; i<n; i++) { bestTx[i]=P[i].tx; bestTy[i]=P[i].ty; }
    double tStart = elapsed();
    double tTotal = time_limit - tStart;
    if (tTotal <= 0.0) return;
    double avgSize = 0.0;
    for (auto& p : P) avgSize += 2.0 * fmax(p.hw, p.hh);
    avgSize /= n;
    double stepInit = avgSize * stepInitFactor;
    vector<int> candidates; candidates.reserve(32);
#ifdef DEBUG_LOG
    logPhaseHeader(phaseName);
    DebugSAStats dbg;
    dbg.dbgIter = 0;
    dbg.dbgBestL = curCost;
    dbg.snapSwapTrials = 0; dbg.snapSwapAccepts = 0;
    dbg.snapMoveTrials = 0; dbg.snapMoveAccepts = 0;
    dbg.snapSlideTrials = 0; dbg.snapSlideAccepts = 0;
    dbg.snapJumpTrials = 0; dbg.snapJumpAccepts = 0;
    dbg.snapRandomJumpTrials = 0; dbg.snapRandomJumpAccepts = 0;
    dbg.snapIters = 0;
    dbg.lastT = T_init;
    dbg.lastStep = stepInit;
    dbg.lastSnapshotTime = tStart;
    dbg.phaseName = phaseName;
    logLine(phaseName, 0, -1, elapsed(), curCost, dbg.dbgBestL, "snapshot");
    if (g_logFile) {
        fprintf(g_logFile,
            "# T_init=%.6f  T_end=%.2e  time_limit=%.1f"
            "  towardOriginProb=%.2f  avgSize=%.4f"
            "  stepInitFactor=%.4f  stepInit=%.6f"
            "  enableMTVSlide=%d  JUMP_SIZE_WEIGHT=%.2f\n",
            T_init, T_end, time_limit,
            towardOriginProb, avgSize,
            stepInitFactor, stepInit,
            (int)enableMTVSlide, JUMP_SIZE_WEIGHT);
        fflush(g_logFile);
    }
    if (g_timerFile) {
        fprintf(g_timerFile,
            "%12.4f  [SA_PARAMS   ] %-20s"
            "  stepInitFactor=%.4f  stepInit=%.6f"
            "  T_init=%.6f  T_end=%.2e"
            "  enableMTVSlide=%d  JUMP_SIZE_WEIGHT=%.2f\n",
            elapsed(), phaseName,
            stepInitFactor, stepInit,
            T_init, T_end,
            (int)enableMTVSlide, JUMP_SIZE_WEIGHT);
        fflush(g_timerFile);
    }
#endif
    int iterCheck = 0;
    double nowElapsed = elapsed();
    while (true) {
        iterCheck++;
        if (iterCheck % 10000 == 0) {
            nowElapsed = elapsed();
            if (nowElapsed >= time_limit) break;
        }
        double progress = (nowElapsed - tStart) / tTotal;
        if (progress >= 1.0) break;
        double T = T_init * pow(fmax(T_end / T_init, 1e-15), progress);
        double step = fmax(avgSize * 0.005,
                           avgSize * stepInitFactor * (1.0 - progress * 0.975));
#ifdef DEBUG_LOG
        dbg.dbgIter++;
        dbg.snapIters++;
        dbg.lastT    = T;
        dbg.lastStep = step;
#endif
        double jumpProb = 0.05 + 0.10 * (T / T_init);
        double randV = randDouble();
        if (randV < 0.20 && n >= 2) {
            saSwapOp(P, n, bd, T, progress,
                     overlappingSetLow, globalFeasibleLow, CLow,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        } else if (randV < 0.20 + jumpProb) {
            saJumpOp(P, n, bd, T, T_init, avgSize, progress,
                     overlappingSetLow, globalFeasibleLow, CLow,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        } else {
            saMoveOp(P, n, bd, T, step, progress,
                     towardOriginProb, enableMTVSlide,
                     overlappingSetLow, globalFeasibleLow, CLow,
                     curCost, bestCost, bestTx, bestTy, candidates
                     DEBUG_SA_STATS_ARG);
        }
    }
    for (int i=0;i<n;i++){P[i].tx=bestTx[i];P[i].ty=bestTy[i];}
#ifdef DEBUG_LOG
    logPhaseSummary(phaseName, dbg.dbgIter, -1, elapsed());
#endif
}
// ==== End src/sa.h ====

// ==== Begin src/main.cpp ====
static void readInput(Polygon& feasiblePoly, Boundary& bd, vector<Polygon>& polys) {
    int n_feasible;
    if (scanf("%d", &n_feasible) != 1) exit(1);
    feasiblePoly.verts.resize(n_feasible);
    feasiblePoly.n = n_feasible;
    feasiblePoly.tx = feasiblePoly.ty = 0.0;
    for (int i = 0; i < n_feasible; ++i) {
        if (scanf("%lf %lf", &feasiblePoly.verts[i].x, &feasiblePoly.verts[i].y) != 2) exit(1);
    }
    double xlo = feasiblePoly.verts[0].x, xhi = xlo;
    double ylo = feasiblePoly.verts[0].y, yhi = ylo;
    for (int i = 1; i < n_feasible; ++i) {
        xlo = fmin(xlo, feasiblePoly.verts[i].x);
        xhi = fmax(xhi, feasiblePoly.verts[i].x);
        ylo = fmin(ylo, feasiblePoly.verts[i].y);
        yhi = fmax(yhi, feasiblePoly.verts[i].y);
    }
    feasiblePoly.origCx = (xlo + xhi) / 2.0;
    feasiblePoly.origCy = (ylo + yhi) / 2.0;
    feasiblePoly.hw = (xhi - xlo) / 2.0;
    feasiblePoly.hh = (yhi - ylo) / 2.0;
    bd.xMin = xlo; bd.xMax = xhi;
    bd.yMin = ylo; bd.yMax = yhi;
    int n;
    if (scanf("%d", &n) != 1) exit(1);
    polys.resize(n);
    double xSpanMax = 0, ySpanMax = 0;
    for (int i = 0; i < n; i++) {
        int nv; if (scanf("%d", &nv) != 1) exit(1);
        polys[i].verts.resize(nv); polys[i].n=nv; polys[i].tx=polys[i].ty=0.0;
        for (int j=0; j<nv; j++)
            if (scanf("%lf %lf", &polys[i].verts[j].x, &polys[i].verts[j].y) != 2) exit(1);
        polys[i].verts = expandPolygon(polys[i].verts, EXPAND_EPS);
        polys[i].n = (int)polys[i].verts.size();
        double xlo=polys[i].verts[0].x, xhi=xlo, ylo=polys[i].verts[0].y, yhi=ylo;
        for (int j=1; j<polys[i].n; j++) {
            xlo=fmin(xlo,polys[i].verts[j].x); xhi=fmax(xhi,polys[i].verts[j].x);
            ylo=fmin(ylo,polys[i].verts[j].y); yhi=fmax(yhi,polys[i].verts[j].y);
        }
        polys[i].origCx=(xlo+xhi)/2.0; polys[i].origCy=(ylo+yhi)/2.0;
        polys[i].hw = (xhi - xlo) / 2.0;
        polys[i].hh = (yhi - ylo) / 2.0;
        if (xSpanMax*ySpanMax < (xhi - xlo)*(yhi - ylo))
            xSpanMax = (xhi - xlo), ySpanMax = (yhi - ylo);
    }
    for (int i = 0; i < n; i++) {
        double width = polys[i].hw * 2.0;
        double height = polys[i].hh * 2.0;
        if ( 50<polys[i].n && width < xSpanMax * 0.1 && height < ySpanMax * 0.1) {
            polys[i].verts = convexHull(polys[i].verts);
            polys[i].n = (int)polys[i].verts.size();
            double xlo=polys[i].verts[0].x, xhi=xlo, ylo=polys[i].verts[0].y, yhi=ylo;
            for (int j=1; j<polys[i].n; j++) {
                xlo=fmin(xlo,polys[i].verts[j].x); xhi=fmax(xhi,polys[i].verts[j].x);
                ylo=fmin(ylo,polys[i].verts[j].y); yhi=fmax(yhi,polys[i].verts[j].y);
            }
            polys[i].origCx=(xlo+xhi)/2.0; polys[i].origCy=(ylo+yhi)/2.0;
            polys[i].hw = (xhi - xlo) / 2.0;
            polys[i].hh = (yhi - ylo) / 2.0;
        }
    }
    char ok[16]; if (scanf("%15s", ok) != 1) exit(1);
    buildContainPolys(feasiblePoly, polys);
}
static void clampAll(vector<Polygon>& P, const Boundary& bd) {
    double refCx = (bd.xMin + bd.xMax) * 0.5;
    double refCy = (bd.yMin + bd.yMax) * 0.5;
    for (int i = 0; i < (int)P.size(); i++) {
        double mnx = P[i].origCx + P[i].tx - P[i].hw;
        double mxx = P[i].origCx + P[i].tx + P[i].hw;
        double mny = P[i].origCy + P[i].ty - P[i].hh;
        double mxy = P[i].origCy + P[i].ty + P[i].hh;
        if (mnx < bd.xMin) P[i].tx += bd.xMin - mnx;
        if (mxx > bd.xMax) P[i].tx -= mxx - bd.xMax;
        if (mny < bd.yMin) P[i].ty += bd.yMin - mny;
        if (mxy > bd.yMax) P[i].ty -= mxy - bd.yMax;
        if (isFeasible(i, Vertex{P[i].tx, P[i].ty})) continue;
        double tgtTx = refCx - P[i].origCx;
        double tgtTy = refCy - P[i].origCy;
        if (!isFeasible(i, Vertex{tgtTx, tgtTy})) {
            bool found = false;
            double vxMin = bd.xMin + P[i].hw, vxMax = bd.xMax - P[i].hw;
            double vyMin = bd.yMin + P[i].hh, vyMax = bd.yMax - P[i].hh;
            for (int attempt = 0; attempt < 50 && !found; ++attempt) {
                double cx = vxMin + randDouble() * (vxMax - vxMin);
                double cy = vyMin + randDouble() * (vyMax - vyMin);
                double tx = cx - P[i].origCx, ty = cy - P[i].origCy;
                if (isFeasible(i, Vertex{tx, ty})) {
                    tgtTx = tx; tgtTy = ty; found = true;
                }
            }
            if (!found) continue;        }
        double loTx = P[i].tx, loTy = P[i].ty;        double hiTx = tgtTx,   hiTy = tgtTy;        for (int iter = 0; iter < 20; ++iter) {
            double midTx = (loTx + hiTx) * 0.5;
            double midTy = (loTy + hiTy) * 0.5;
            if (isFeasible(i, Vertex{midTx, midTy}))
                hiTx = midTx, hiTy = midTy;
            else
                loTx = midTx, loTy = midTy;
        }
        P[i].tx = hiTx;
        P[i].ty = hiTy;
    }
}
int main(int argc, char *argv[]) {
#ifdef __linux__
    {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    }
#endif
  TL_SET_OUTPUT("dist/log/time/time_log.txt");
#if LOCAL_RUN
    if (argc > 1) {
        FILE* fp = freopen(argv[1], "r", stdin);
        if (!fp) {
            fprintf(stderr, "无法打开输入文件: %s\n", argv[1]);
            return 1;
        }
    }
#endif
    g_start = chrono::steady_clock::now();
#ifdef DEBUG_LOG
    logInitFile();
#endif
    Polygon feasiblePoly;
    Boundary bd; vector<Polygon> P;
    readInput(feasiblePoly, bd, P);
    int n = (int)P.size();
#ifdef DEBUG_LOG
    timerInitFile(n, bd);
    if (g_timerFile) {
        fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s  n=%d  boundary=[%.3f,%.3f]x[%.3f,%.3f]\n",
                elapsed(), "readInput_done", n,
                bd.xMin, bd.xMax, bd.yMin, bd.yMax);
        fflush(g_timerFile);
    }
#endif
    vector<int> perm(n);
    for (int i=0; i<n; i++) perm[i]=i;
    stable_sort(perm.begin(), perm.end(), [&](int a, int b) {
        return P[a].hw*P[a].hh < P[b].hw*P[b].hh;
    });
#ifdef DEBUG_LOG
    logPhaseHeader("init_shelf");
    double initL = calcLPrime(P);
    logLine("init_shelf", 0, -1, elapsed(), initL, initL, "snapshot");
#endif
#ifdef DEBUG_LOG
    double tPhase = elapsed();
    double lBefore = calcLPrime(P);
    timerPhaseStart("saOnPerm", lBefore);
#endif
    saOnPerm(P, perm, bd);
    clampAll(P, bd);
#ifdef DEBUG_LOG
    {
        double lAfter = calcLPrime(P);
        timerPhaseEnd("saOnPerm", tPhase, lBefore, lAfter, g_cntShelfPlace, -1);
    }
#endif
#ifdef DEBUG_LOG
    tPhase = elapsed();
    lBefore = calcLPrime(P);
    timerPhaseStart("aabbPullback", lBefore);
#endif
    aabbPullback(P, bd);
#ifdef DEBUG_LOG
    {
        double lAfter = calcLPrime(P);
        timerPhaseEnd("aabbPullback", tPhase, lBefore, lAfter);
    }
    logPhaseHeader("after_aabbPullback");
    logLine("after_aabbPullback", 0, -1, elapsed(), calcLPrime(P), calcLPrime(P), "snapshot");
#endif
#ifdef DEBUG_LOG
    {
        double tNfp = elapsed();
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s  nfp_pairs=%d\n",
                    tNfp, "buildNFPMatrix", n*(n-1)/2);
            fflush(g_timerFile);
        }
#endif
    TL_START("buildNFPMatrix ");
    buildNFPMatrix(P);
    int nfpFailures = g_nfps.getfailuresNum();
    TL_ATTACH("n = " + to_string(n * (n - 1) / 2));
    TL_ATTACH("Failures = " + to_string(nfpFailures));
    if (nfpFailures) {
        TL_ATTACH("ERROR: The generation of nfp failed.");
    }
    TL_END();
#ifdef DEBUG_LOG
    if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f        Failures NFP number: %d",elapsed(),nfpFailures);
            fprintf(g_timerFile, "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs\n",
                    elapsed(), "buildNFPMatrix", elapsed() - tNfp);
            fflush(g_timerFile);
        }
    }
#endif
    {
        double L0 = calcLPrime(P);
#ifdef DEBUG_LOG
        tPhase = elapsed(); lBefore = L0;
        timerPhaseStart("positionSA_high", lBefore);
        long long itersBeforeSA = g_cntExactOverlap;
        (void)itersBeforeSA;
#endif
        TL_START("positionSA");
        positionSAHigh(P, bd,
                       L0 * 0.03, 
                       L0 * 0.001,
                       T_SA_HIGH_END);
        TL_ATTACH("fastOverlap calls: " + to_string(g_cntFastOverlap));
        TL_ATTACH("exactOverlap calls: " + to_string(g_cntExactOverlap));
        TL_END();
#ifdef DEBUG_LOG
        {
            double lAfter = calcLPrime(P);
            timerPhaseEnd("positionSA_high", tPhase, lBefore, lAfter);
        }
#endif
    }
#ifdef DEBUG_LOG
    {
        double tRebuild = elapsed();
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s\n", tRebuild, "hash_rebuild_1");
            fflush(g_timerFile);
        }
#endif
        g_hash.rebuild(bd, P, buildCellSize(P));
#ifdef DEBUG_LOG
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs\n",
                    elapsed(), "hash_rebuild_1", elapsed() - tRebuild);
            fflush(g_timerFile);
        }
    }
#endif
#ifdef DEBUG_LOG
    tPhase = elapsed(); lBefore = calcLPrime(P);
    timerPhaseStart("exactPullback_mid", lBefore);
#endif
    exactPullback(P, bd, T_EXACT_MID, "exactPullback_mid");
#ifdef DEBUG_LOG
    {
        double lAfter = calcLPrime(P);
        timerPhaseEnd("exactPullback_mid", tPhase, lBefore, lAfter);
    }
#endif
    {
#ifdef DEBUG_LOG
        double tRebuild = elapsed();
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s\n",
                    tRebuild, "hash_rebuild_pre_tightness");
            fflush(g_timerFile);
        }
#endif
        g_hash.rebuild(bd, P, buildCellSize(P));
#ifdef DEBUG_LOG
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs\n",
                    elapsed(), "hash_rebuild_pre_tightness",
                    elapsed() - tRebuild);
            fflush(g_timerFile);
        }
#endif
    }
    {
        double avgFreeMove = evalAvgFreeMove(P, bd);
        double avgSzTmp = 0.0;
        for (int i = 0; i < n; i++) avgSzTmp += 2.0 * fmax(P[i].hw, P[i].hh);
        avgSzTmp /= n;
        double stepInit   = fmax(avgFreeMove * 0.8, avgSzTmp * 0.003);
        double stepFactor = stepInit / avgSzTmp;
        double L0_low     = calcLPrime(P);
        double T_low_init = fmax(stepInit * 0.36, L0_low * 0.0005);
#ifdef DEBUG_LOG
        if (g_timerFile) {
            fprintf(g_timerFile,
                "%12.4f  [DYNAMIC_PAR ] %-20s"
                "  avgFreeMove=%.6f"
                "  avgSz=%.6f"
                "  free/sz=%.4f"
                "  stepInit=%.6f"
                "  stepFactor=%.4f"
                "  T_low_init=%.6f"
                "  L0=%.6f\n",
                elapsed(), "positionSA_low_prep",
                avgFreeMove,
                avgSzTmp,
                avgFreeMove / (avgSzTmp > 1e-9 ? avgSzTmp : 1.0),
                stepInit,
                stepFactor,
                T_low_init,
                L0_low);
            fflush(g_timerFile);
        }
        tPhase = elapsed(); lBefore = L0_low;
        timerPhaseStart("positionSA_low", lBefore);
#endif
        positionSALow(P, bd,
                      T_low_init,
                      SA_TMIN,
                      T_SA_LOW_END,
                      stepFactor);
#ifdef DEBUG_LOG
        {
            double lAfter = calcLPrime(P);
            timerPhaseEnd("positionSA_low", tPhase, lBefore, lAfter);
        }
#endif
    }
#ifdef DEBUG_LOG
    {
        double tRebuild = elapsed();
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s\n", tRebuild, "hash_rebuild_2");
            fflush(g_timerFile);
        }
#endif
        g_hash.rebuild(bd, P, buildCellSize(P));
#ifdef DEBUG_LOG
        if (g_timerFile) {
            fprintf(g_timerFile, "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs\n",
                    elapsed(), "hash_rebuild_2", elapsed() - tRebuild);
            fflush(g_timerFile);
        }
    }
#endif
#ifdef DEBUG_LOG
    tPhase = elapsed(); lBefore = calcLPrime(P);
    timerPhaseStart("exactPullback_final", lBefore);
#endif
    exactPullback(P, bd, T_EXACT_FINAL, "exactPullback_final");
#ifdef DEBUG_LOG
    {
        double lAfter = calcLPrime(P);
        timerPhaseEnd("exactPullback_final", tPhase, lBefore, lAfter);
    }
#endif
#ifdef DEBUG_LOG
    if (g_timerFile) {
        fprintf(g_timerFile,
            "\n%12.4f  [FINAL       ] L'_final=%12.6f\n",
            elapsed(), calcLPrime(P));
        int totalNFP = g_nfps.getTotalConstructionCount();
        int accessedNFP = g_nfps.getAccessedCount();
        double accessRate = g_nfps.getAccessRate();
        fprintf(g_timerFile,
            "%12.4f  [NFP_ACCESS  ] total=%d  accessed=%d  rate=%.4f\n",
            elapsed(), totalNFP, accessedNFP, accessRate);
        fflush(g_timerFile);
    }
    timerHotspotSummary();
    logPhaseHeader("final");
    double finalL = calcLPrime(P);
    logLine("final", 0, -1, elapsed(), finalL, finalL, "snapshot");
    logCloseFile();
    timerCloseFile();
#endif
    TL_START("Summary");
    TL_ATTACH("fastOverlap all calls: " + to_string(g_cntFastOverlap));    TL_ATTACH("exactOverlap all calls: " + to_string(g_cntExactOverlap));    printf("%d\n", n);
    for (int i=0; i<n; i++) {
        printf("%.5f %.5f\n", P[i].tx, P[i].ty);
        fflush(stdout);
    }
    printf("OK\n");
    fflush(stdout);
    TL_END();
    return 0;
}
// ==== End src/main.cpp ====

