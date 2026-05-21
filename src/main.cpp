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

int main(int argc, char *argv[]) {
#ifdef __linux__
    {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    }
#endif

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

    Polygon feasiblePoly; Boundary bd; vector<Polygon> P;
    readInput(feasiblePoly, bd, P);
    initSnapshoot(feasiblePoly, P); 
    int n = (int)P.size();
    vector<int> perm(n);
    for (int i=0; i<n; i++) perm[i]=i;
    stable_sort(perm.begin(), perm.end(), [&](int a, int b) {
        return P[a].hw*P[a].hh < P[b].hw*P[b].hh;
    });

    saOnPerm(P, perm, bd);
    clampAll(P, bd);
    aabbPullback(P, bd);
    buildNFPMatrix(P);
    int nfpFailures = g_nfps.getfailuresNum();

    {
        double L0 = calcLPrime(P);
        positionSAHigh(P, bd,
                       L0 * 0.03, 
                       L0 * 0.001,
                       T_SA_HIGH_END);
    }
    g_hash.rebuild(bd, P, buildCellSize(P));
    exactPullback(P, bd, T_EXACT_MID, "exactPullback_mid");
    g_hash.rebuild(bd, P, buildCellSize(P));
    
    {
        double avgFreeMove = evalAvgFreeMove(P, bd);
        double avgSzTmp = 0.0;
        for (int i = 0; i < n; i++) avgSzTmp += 2.0 * fmax(P[i].hw, P[i].hh);
        avgSzTmp /= n;
        double stepInit   = fmax(avgFreeMove * 0.8, avgSzTmp * 0.003);
        double stepFactor = stepInit / avgSzTmp;
        double L0_low     = calcLPrime(P);
        double T_low_init = fmax(stepInit * 0.36, L0_low * 0.0005);
        positionSALow(P, bd,
                      T_low_init,
                      SA_TMIN,
                      T_SA_LOW_END,
                      stepFactor);
    }

    g_hash.rebuild(bd, P, buildCellSize(P));
    exactPullback(P, bd, T_EXACT_FINAL, "exactPullback_final");

    printf("%d\n", n);
    for (int i=0; i<n; i++) 
        printf("%.5f %.5f\n", P[i].tx, P[i].ty);
    printf("OK\n");
    fflush(stdout);
    // 输出 snapshoot 数据
    outputSnapshootData();
    return 0;
}