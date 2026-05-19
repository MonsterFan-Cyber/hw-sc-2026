#pragma once
#include "0.h"
#include "trajgraph.h"

namespace nfp {
using namespace traj;
class NFPSolver
{
public:
    Polygon genNFP(const Ring& RingA, const Ring& RingB, bool outInner = false);
    void clear();
    Polygon getNFP(){return nfp;}
    std::vector<Segment> getNFPSegments(){return nfpSegments;};
private:
    void setPolygon(const Ring &RingA, const Ring &RingB);
    void genTraceSegments();
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
void NFPSolver::genTraceSegments()
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
    genTraceSegments();
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