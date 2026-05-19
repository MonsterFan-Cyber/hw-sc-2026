#pragma once
#include "lg.h"
#include <unordered_set>

static bool saSwapOp(vector<Polygon>& P, int n, const Boundary& bd,
                     double T, double progress,
                     unordered_set<int>& overlappingSet, bool globalFeasible, double C,
                     double& curCost, double& bestCost,
                     vector<double>& bestTx, vector<double>& bestTy,
                     vector<int>& candidates
                     DEBUG_SA_STATS_PARAM)
{

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

                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}

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
                            if (curCost<bestCost) {
                                bestCost=curCost;
                                for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
                            }
                        }
                    }
                }
            }
            if (!mtvFixed) { P[ia].tx=otxa;P[ia].ty=otya;P[ib].tx=otxb;P[ib].ty=otyb; }
        }
        g_hash.insert(ia,P); g_hash.insert(ib,P);
    }

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

                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
                }
            }
        } else if (enableMTVSlide && globalFeasible) {

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

                        if (curCost < bestCost) {
                            bestCost = curCost;
                            for (int i=0;i<n;i++){bestTx[i]=P[i].tx;bestTy[i]=P[i].ty;}
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
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx=otx; P[idx].ty=oty; }
    g_hash.insert(idx, P);

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

                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }

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
                            if (curCost < bestCost) {
                                bestCost = curCost;
                                for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx = otx; P[idx].ty = oty; }
    g_hash.insert(idx, P);
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

                if (curCost < bestCost) {
                    bestCost = curCost;
                    for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
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
                            if (curCost < bestCost) {
                                bestCost = curCost;
                                for (int i = 0; i < n; i++) { bestTx[i] = P[i].tx; bestTy[i] = P[i].ty; }
                            }
                        }
                    }
                }
            }
        }
    }
    if (!accepted) { P[idx].tx = otx; P[idx].ty = oty; }
    g_hash.insert(idx, P);
    return accepted;
}