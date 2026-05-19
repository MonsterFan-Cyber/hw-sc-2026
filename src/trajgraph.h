#pragma once
#include "trajgrid.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <filesystem>

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
        if (nearestSegmentIndex == -1) {   continue;        }
        const auto& nearestSeg = segments[nearestSegmentIndex];
        if (nearestSeg.isPointOnLeftSide(targetPoint)) {   continue;  }
        std::vector<DirectedSegment> innerLoopSegments;
        Scalar totalAngle = 0.0;
        extractLoopStartAtSegment(nearestSegmentIndex, innerLoopSegments, totalAngle, true);
        if (innerLoopSegments.empty())
        {   continue;        }
        bool isInnerLoop = true;
        if (!(std::abs(totalAngle + PI_PI) < EPS_ANGLE_RELAX))
        {   isInnerLoop = false;        }
        if (isInnerLoop) 
        {   innerLoops.push_back(std::move(innerLoopSegments));  }
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