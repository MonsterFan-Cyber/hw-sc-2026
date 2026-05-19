#pragma once
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <limits>
#include <map>
#include <set>
#include <iomanip>
#include <iostream>
#include <cmath>
#include "time_log.h"
#include "polygon.h"

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
        {   return false;        }
        Point d1 = dir;
        Point d2 = other.dir;
        Point delta = other.from - from;
        Scalar denom = d1.cross(d2);
        if (std::abs(denom) < eps_angle)
        {
            if (hasPointOnLine(other.to, t1, eps_distance) && t1 - 1.0 > eps_distance)
            {
                if (other.hasPointOnLine(to, t2, eps_distance) && t2 > -eps_distance && 1.0 - t2 > eps_distance)
                {    t1 = 1.0;   return true;  }
                else {return false;}
            }
            else{ return false;}
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
                {    return roundIndex;   }
                else
                {    return roundIndex + 1;  }
            }
            case ROUND:
                return roundIndex;
            case UPPER:
            {
                Scalar roundX = getX(roundIndex);
                if (x - roundX > -EPS_DISTANCE)
                {   return roundIndex;  }
                else
                {  return roundIndex - 1; }
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
                {    return roundIndex;   }
                else
                {    return roundIndex + 1; }
            }
            case ROUND:
                return roundIndex;
            case UPPER:
            {
                Scalar roundY = getY(roundIndex);
                if (y - roundY > -EPS_DISTANCE)
                {   return roundIndex; }
                else
                {  return roundIndex - 1; }
            }
            default:
                return roundIndex;
        }
    }
    Point getCellCenter(int cellIndex) const
    {
        int i = cellIndex / ny;  int j = cellIndex % ny;
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
            for (int i = i0; i <= i1; ++i) { addCell(i, iy - 1);  addCell(i, iy);  }
        } else {
            int j = cellIndexFromCoord(a.y, boundingBox.min.y, cellSizeY, ny);
            for (int i = i0; i <= i1; ++i) { addCell(i, j); }
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
            i += stepX;  tMaxX += tDeltaX;  continue;
        }
        if (tMaxY + EPS_DISTANCE < tMaxX) {
            j += stepY;  tMaxY += tDeltaY; continue;
        }
        int nextI = i + stepX, nextJ = j + stepY;
        addCell(nextI, j);
        addCell(i, nextJ);
        addCell(nextI, nextJ);
        i = nextI;  j = nextJ;
        tMaxX += tDeltaX;
        tMaxY += tDeltaY;
    }
    addPointTouchCells(a);
    addPointTouchCells(b);
    cells.assign(uniqueCells.begin(), uniqueCells.end());
}
}

