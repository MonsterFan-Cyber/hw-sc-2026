#pragma once

#include "lg.h"
#include "op.h"
#include "snapshoot.h"
#include <unordered_set>
#include <set>

void initSnapshoot(const Polygon& feasiblePoly, const vector<Polygon>& P) {
    // 格式转换，在调用 snap::g_snapshoot.setPoly 前将 Polygon 转换为 snap::Snapshoot::Poly
    snap::Snapshoot::Poly snapFeasiblePoly;
    vector<snap::Snapshoot::Poly> snapPolys;
    for (const auto& vertex : feasiblePoly.verts) {
        snapFeasiblePoly.push_back({static_cast<float>(vertex.x), static_cast<float>(vertex.y)});
    }
    for (const auto& poly : P) {
        snap::Snapshoot::Poly snapPoly;
        for (const auto& vertex : poly.verts) {
            snapPoly.push_back({static_cast<float>(vertex.x), static_cast<float>(vertex.y)});
        }
        snapPolys.push_back(snapPoly);
    }
    snap::g_snapshoot.setPoly(snapFeasiblePoly, snapPolys);

}

void addSnapshootFrame(const vector<Polygon>& P, const std::unordered_set<int> &overlappingSet) {
    vector<array<float, 2>> frame;
    vector<bool> legal;
    for (const auto& poly : P) {
        frame.push_back({static_cast<float>(poly.tx), static_cast<float>(poly.ty)});
        // 如果当前多边形在 overlappingSet 中，则认为它的位置不合法
        legal.push_back(overlappingSet.find(&poly - &P[0]) == overlappingSet.end());
    }
    snap::g_snapshoot.addFrame(frame, legal);
}

void outputSnapshootData() {
    snap::g_snapshoot.write("dist/data/snapshoot_data.snpst");
}

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

    int iter=0;
    vector<double> savedTx(n), savedTy(n);
    while (T > SA_TMIN) {
        iter++;
        if (iter%500==0 && elapsed()>T_SA_PERM) break;

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

            }
        } else { perm=savedPerm; for (int i=0;i<n;i++){P[i].tx=savedTx[i];P[i].ty=savedTy[i];} }

        T *= SA_PERM_ALPHA;
        if (iter % 2000 == 0)
            addSnapshootFrame(P, {});
    }
    perm=bestPerm; shelfPlace(perm,P,bd);

}
static void aabbPullback(vector<Polygon>& P, const Boundary& bd) {
    int n=(int)P.size();
    vector<int> order(n); for (int i=0;i<n;i++) order[i]=i;

    bool improved=true;
    while (improved && elapsed()<T_AABB_PULL) {
        improved=false;

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

    }

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
static void exactPullback(vector<Polygon>& P, const Boundary& bd,
                          double timeLimit, const char* phaseName = "exactPullback") {
    int n=(int)P.size();
    vector<int> order(n); for (int i=0;i<n;i++) order[i]=i;
    vector<int> cands; cands.reserve(32);

    bool improved = true;
    int iterCheck = 0;
    while (improved && elapsed()<timeLimit) {
        improved=false;
        iterCheck++;
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
        if (iterCheck % 1000 == 0)
            addSnapshootFrame(P, {});
    }

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
        // 每 5000 次迭代记录一次快照数据
        if (iteration % 10000 == 0)
            addSnapshootFrame(P, overlappingSet);
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
        if (iterCheck % 1000 == 0)
            addSnapshootFrame(P, {});
    }
    for (int i=0;i<n;i++){P[i].tx=bestTx[i];P[i].ty=bestTy[i];}

}