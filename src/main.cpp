#include "sa.h"

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