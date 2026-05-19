#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

const double EPS = 1e-9;
const double PI = 3.14159265358979323846;
const double EPS_SQ = 1e-18;

/* ---- Timing ---- */
#ifdef _WIN32
static LARGE_INTEGER g_startTimeFreq, g_startTimeVal;
static void initTimer() { QueryPerformanceFrequency(&g_startTimeFreq); QueryPerformanceCounter(&g_startTimeVal); }
static double elapsedSeconds() {
    LARGE_INTEGER now; QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - g_startTimeVal.QuadPart) / (double)g_startTimeFreq.QuadPart;
}
#else
static struct timeval g_startTimeVal;
static void initTimer() { gettimeofday(&g_startTimeVal, NULL); }
static double elapsedSeconds() {
    struct timeval now; gettimeofday(&now, NULL);
    return (now.tv_sec - g_startTimeVal.tv_sec) + (now.tv_usec - g_startTimeVal.tv_usec) * 1e-6;
}
#endif

static double g_timeBudget = 55.0;

/* ---- Vector2D ---- */
typedef struct { double x, y; } Vector2D;

static Vector2D vec2(double x, double y) { Vector2D v; v.x = x; v.y = y; return v; }
static Vector2D vec2_add(Vector2D a, Vector2D b) { return vec2(a.x + b.x, a.y + b.y); }
static Vector2D vec2_sub(Vector2D a, Vector2D b) { return vec2(a.x - b.x, a.y - b.y); }
static Vector2D vec2_mul(Vector2D a, double t) { return vec2(a.x * t, a.y * t); }
static Vector2D vec2_div(Vector2D a, double t) { return vec2(a.x / t, a.y / t); }
static double vec2_dot(Vector2D a, Vector2D b) { return a.x * b.x + a.y * b.y; }
static double vec2_cross(Vector2D a, Vector2D b) { return a.x * b.y - a.y * b.x; }
static double vec2_norm(Vector2D a) { return sqrt(a.x * a.x + a.y * a.y); }
static double vec2_norm2(Vector2D a) { return a.x * a.x + a.y * a.y; }
static int vec2_lt(Vector2D a, Vector2D b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }
static int vec2_eq(Vector2D a, Vector2D b) { return fabs(a.x - b.x) < EPS && fabs(a.y - b.y) < EPS; }

/* ---- Box ---- */
typedef struct { double left, right, bottom, top; } Box;

static Box box_new() { Box b; b.left = 1e18; b.right = -1e18; b.bottom = 1e18; b.top = -1e18; return b; }
static double box_width(Box *b) { return b->right - b->left; }
static double box_height(Box *b) { return b->top - b->bottom; }
static int box_intersects(Box *a, Box *b) {
    return !(a->right < b->left - EPS || b->right < a->left - EPS ||
             a->top < b->bottom - EPS || b->top < a->bottom - EPS);
}
static Vector2D box_center(Box *b) { return vec2((b->left + b->right) * 0.5, (b->bottom + b->top) * 0.5); }

/* ---- MyPolygon ---- */
typedef struct {
    Vector2D *pts;
    int cnt;
    double cachedArea;
    Box cachedBox;
    int boxCached;
} MyPolygon;

static MyPolygon poly_new() { MyPolygon p; p.pts = NULL; p.cnt = 0; p.cachedArea = -1; p.boxCached = 0; return p; }
static MyPolygon poly_from_arr(Vector2D *v, int n) {
    MyPolygon p; p.pts = (Vector2D*)malloc(n * sizeof(Vector2D));
    memcpy(p.pts, v, n * sizeof(Vector2D)); p.cnt = n; p.cachedArea = -1; p.boxCached = 0; return p;
}
static void poly_free(MyPolygon *p) { if (p->pts) free(p->pts); p->pts = NULL; }

static double poly_signedArea(MyPolygon *p) {
    double a = 0;
    for (int i = 0, j = p->cnt - 1; i < p->cnt; ++i) {
        a += (p->pts[j].x + p->pts[i].x) * (p->pts[j].y - p->pts[i].y);
        j = i;
    }
    return -a * 0.5;
}
static double poly_area(MyPolygon *p) {
    if (p->cachedArea < 0) p->cachedArea = fabs(poly_signedArea(p));
    return p->cachedArea;
}
static int poly_isCCW(MyPolygon *p) { return poly_signedArea(p) >= 0; }

static Vector2D poly_centroid(MyPolygon *p) {
    double cx = 0, cy = 0, a = 0;
    for (int i = 0; i < p->cnt; i++) {
        int j = (i + 1) % p->cnt;
        double cr = vec2_cross(p->pts[i], p->pts[j]);
        a += cr; cx += (p->pts[i].x + p->pts[j].x) * cr; cy += (p->pts[i].y + p->pts[j].y) * cr;
    }
    a *= 0.5;
    if (fabs(a) < EPS) {
        cx = cy = 0;
        for (int i = 0; i < p->cnt; i++) { cx += p->pts[i].x; cy += p->pts[i].y; }
        return vec2(cx / p->cnt, cy / p->cnt);
    }
    return vec2(cx / (6.0 * a), cy / (6.0 * a));
}

static Box poly_boundingBox(MyPolygon *p) {
    if (!p->boxCached) {
        Box b = box_new();
        for (int i = 0; i < p->cnt; i++) {
            if (p->pts[i].x < b.left) b.left = p->pts[i].x;
            if (p->pts[i].x > b.right) b.right = p->pts[i].x;
            if (p->pts[i].y < b.bottom) b.bottom = p->pts[i].y;
            if (p->pts[i].y > b.top) b.top = p->pts[i].y;
        }
        p->cachedBox = b; p->boxCached = 1;
    }
    return p->cachedBox;
}

static MyPolygon poly_moved(MyPolygon *p, Vector2D v) {
    MyPolygon res = poly_new(); res.cnt = p->cnt;
    res.pts = (Vector2D*)malloc(p->cnt * sizeof(Vector2D));
    for (int i = 0; i < p->cnt; i++) res.pts[i] = vec2_add(p->pts[i], v);
    return res;
}
static void poly_moveBy(MyPolygon *p, Vector2D v) {
    p->cachedArea = -1; p->boxCached = 0;
    for (int i = 0; i < p->cnt; i++) p->pts[i] = vec2_add(p->pts[i], v);
}

/* ---- DPoint ---- */
typedef struct { double x, y; } DPoint;
static DPoint dpt(double x, double y) { DPoint d; d.x = x; d.y = y; return d; }

static int segIntersect(DPoint p1, DPoint p2, DPoint p3, DPoint p4, DPoint *ip) {
    double d1x = p2.x - p1.x, d1y = p2.y - p1.y;
    double d2x = p4.x - p3.x, d2y = p4.y - p3.y;
    double denom = d1x * d2y - d1y * d2x;
    if (fabs(denom) < 1e-15) return 0;
    double t = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / denom;
    double u = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / denom;
    if (t >= -1e-10 && t <= 1.0 + 1e-10 && u >= -1e-10 && u <= 1.0 + 1e-10) {
        ip->x = p1.x + t * d1x; ip->y = p1.y + t * d1y; return 1;
    }
    return 0;
}

static int pointInPolygon(DPoint pt, MyPolygon *poly) {
    int inside = 0, n = poly->cnt;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly->pts[i].x, yi = poly->pts[i].y;
        double xj = poly->pts[j].x, yj = poly->pts[j].y;
        if (((yi > pt.y) != (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

/* ---- Clip by half-plane ---- */
typedef struct { DPoint *data; int cnt, cap; } DPointVec;

static DPointVec dpvec_new() { DPointVec v; v.data = NULL; v.cnt = 0; v.cap = 0; return v; }
static void dpvec_push(DPointVec *v, DPoint p) {
    if (v->cnt >= v->cap) { v->cap = v->cap ? v->cap * 2 : 16; v->data = (DPoint*)realloc(v->data, v->cap * sizeof(DPoint)); }
    v->data[v->cnt++] = p;
}
static void dpvec_free(DPointVec *v) { if (v->data) free(v->data); v->data = NULL; v->cnt = 0; v->cap = 0; }

static DPointVec clipByHalfPlane(DPoint *poly, int n, DPoint p1, DPoint p2) {
    DPointVec output = dpvec_new();
    if (n == 0) return output;
    for (int i = 0; i < n; i++) {
        DPoint cur = poly[i], prev = poly[(i - 1 + n) % n];
        double curSide = (p2.x - p1.x) * (cur.y - p1.y) - (p2.y - p1.y) * (cur.x - p1.x);
        double prevSide = (p2.x - p1.x) * (prev.y - p1.y) - (p2.y - p1.y) * (prev.x - p1.x);
        int curIn = curSide >= -1e-10;
        int prevIn = prevSide >= -1e-10;
        if (curIn) {
            if (!prevIn) { DPoint ip; if (segIntersect(prev, cur, p1, p2, &ip)) dpvec_push(&output, ip); }
            dpvec_push(&output, cur);
        } else if (prevIn) {
            DPoint ip; if (segIntersect(prev, cur, p1, p2, &ip)) dpvec_push(&output, ip);
        }
    }
    return output;
}

static DPointVec clipPolygonByConvex(DPoint *subject, int subjCnt, MyPolygon *clipper) {
    DPointVec output = dpvec_new();
    if (subjCnt == 0) return output;
    output.data = (DPoint*)malloc(subjCnt * sizeof(DPoint));
    output.cap = subjCnt; output.cnt = subjCnt;
    memcpy(output.data, subject, subjCnt * sizeof(DPoint));

    for (int i = 0; i < clipper->cnt && output.cnt > 0; i++) {
        int j = (i + 1) % clipper->cnt;
        DPoint p1 = dpt(clipper->pts[i].x, clipper->pts[i].y);
        DPoint p2 = dpt(clipper->pts[j].x, clipper->pts[j].y);
        DPointVec next = clipByHalfPlane(output.data, output.cnt, p1, p2);
        dpvec_free(&output);
        output = next;
    }
    return output;
}

static double dpointArea(DPoint *pts, int n) {
    if (n < 3) return 0.0;
    double a = 0;
    for (int i = 0, j = n - 1; i < n; j = i++) a += pts[j].x * pts[i].y - pts[i].x * pts[j].y;
    return fabs(a) * 0.5;
}

static double cross3(Vector2D O, Vector2D A, Vector2D B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

/* ---- Triangulation ---- */
typedef struct { int a, b, c; } Triangle;

static int isEar(Vector2D *pts, int *indices, int idx, int n) {
    int prev = indices[(idx - 1 + n) % n], curr = indices[idx], next = indices[(idx + 1) % n];
    Vector2D a = pts[prev], b = pts[curr], c = pts[next];
    double cr = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (cr <= 0) return 0;
    for (int j = 0; j < n; j++) {
        if (j == (idx - 1 + n) % n || j == idx || j == (idx + 1) % n) continue;
        Vector2D p = pts[indices[j]];
        double d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        double d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
        double d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
        if (!((d1 < 0) || (d2 < 0) || (d3 < 0)) || !((d1 > 0) || (d2 > 0) || (d3 > 0))) return 0;
    }
    return 1;
}

typedef struct { Triangle *data; int cnt, cap; } TriVec;
static TriVec trivec_new() { TriVec v; v.data = NULL; v.cnt = 0; v.cap = 0; return v; }
static void trivec_push(TriVec *v, Triangle t) {
    if (v->cnt >= v->cap) { v->cap = v->cap ? v->cap * 2 : 16; v->data = (Triangle*)realloc(v->data, v->cap * sizeof(Triangle)); }
    v->data[v->cnt++] = t;
}
static void trivec_free(TriVec *v) { if (v->data) free(v->data); v->data = NULL; v->cnt = 0; v->cap = 0; }

static TriVec triangulate(MyPolygon *poly) {
    TriVec triangles = trivec_new();
    int n = poly->cnt;
    if (n < 3) return triangles;
    int *indices = (int*)malloc(n * sizeof(int));
    for (int i = 0; i < n; i++) indices[i] = i;
    if (!poly_isCCW(poly)) {
        for (int i = 0; i < n / 2; i++) { int tmp = indices[i]; indices[i] = indices[n - 1 - i]; indices[n - 1 - i] = tmp; }
    }
    int remaining = n, i = 0, safety = n * n * 2;
    while (remaining > 3 && safety-- > 0) {
        if (isEar(poly->pts, indices, i % remaining, remaining)) {
            int prev = (i - 1 + remaining) % remaining, next = (i + 1) % remaining;
            Triangle t; t.a = indices[prev]; t.b = indices[i % remaining]; t.c = indices[next];
            trivec_push(&triangles, t);
            for (int k = i % remaining; k < remaining - 1; k++) indices[k] = indices[k + 1];
            remaining--; i = prev % remaining;
        } else { i = (i + 1) % remaining; }
    }
    if (remaining == 3) { Triangle t; t.a = indices[0]; t.b = indices[1]; t.c = indices[2]; trivec_push(&triangles, t); }
    free(indices);
    return triangles;
}

static double intersectionArea(MyPolygon *subject, MyPolygon *clipper) {
    Box sb = poly_boundingBox(subject), cb = poly_boundingBox(clipper);
    if (!box_intersects(&sb, &cb)) return 0.0;
    TriVec tris = triangulate(clipper);
    double totalArea = 0.0;
    DPoint *subPts = (DPoint*)malloc(subject->cnt * sizeof(DPoint));
    for (int i = 0; i < subject->cnt; i++) subPts[i] = dpt(subject->pts[i].x, subject->pts[i].y);
    for (int t = 0; t < tris.cnt; t++) {
        MyPolygon triPoly; triPoly.cnt = 3;
        Vector2D triPts[3] = {clipper->pts[tris.data[t].a], clipper->pts[tris.data[t].b], clipper->pts[tris.data[t].c]};
        triPoly.pts = triPts; triPoly.cachedArea = -1; triPoly.boxCached = 0;
        DPointVec clipped = clipPolygonByConvex(subPts, subject->cnt, &triPoly);
        totalArea += dpointArea(clipped.data, clipped.cnt);
        dpvec_free(&clipped);
    }
    free(subPts); trivec_free(&tris);
    return totalArea;
}

/* ---- Convex Hull ---- */
static int vec2_cmp(const void *a, const void *b) {
    Vector2D *va = (Vector2D*)a, *vb = (Vector2D*)b;
    if (va->x < vb->x) return -1; if (va->x > vb->x) return 1;
    if (va->y < vb->y) return -1; if (va->y > vb->y) return 1;
    return 0;
}

static Vector2D* convexHull(Vector2D *P, int n, int *outN) {
    if (n <= 1) { *outN = n; return P; }
    qsort(P, n, sizeof(Vector2D), vec2_cmp);
    Vector2D *H = (Vector2D*)malloc(2 * n * sizeof(Vector2D));
    int k = 0;
    for (int i = 0; i < n; i++) { while (k >= 2 && cross3(H[k-2], H[k-1], P[i]) <= 0) k--; H[k++] = P[i]; }
    for (int i = n - 2, t = k + 1; i >= 0; i--) { while (k >= t && cross3(H[k-2], H[k-1], P[i]) <= 0) k--; H[k++] = P[i]; }
    *outN = k;
    if (k > 1 && H[k-1].x == H[0].x && H[k-1].y == H[0].y) (*outN)--;
    return H;
}

/* ---- Overlap and boundary checks ---- */
static int isInsideBound(MyPolygon *poly, MyPolygon *bnd) {
    double polyArea = poly_area(poly);
    if (polyArea < EPS) return 1;
    double interArea = intersectionArea(poly, bnd);
    double areaDiff = polyArea - interArea;
    return areaDiff < 1e-10 && areaDiff < polyArea * 1e-6;
}

static int polygonsOverlap(MyPolygon *A, MyPolygon *B) {
    Box ba = poly_boundingBox(A), bb = poly_boundingBox(B);
    if (!box_intersects(&ba, &bb)) return 0;
    double interArea = intersectionArea(A, B);
    if (interArea < 1e-10) return 0;
    double minArea = poly_area(A); double bArea = poly_area(B);
    if (bArea < minArea) minArea = bArea;
    if (minArea < EPS) return 0;
    return interArea >= minArea * 1e-6;
}

static int quickOverlapCheck(MyPolygon *A, MyPolygon *B) {
    Box ba = poly_boundingBox(A), bb = poly_boundingBox(B);
    if (!box_intersects(&ba, &bb)) return 0;
    for (int i = 0; i < A->cnt; i++) {
        int j = (i + 1) % A->cnt;
        Vector2D edge = vec2_sub(A->pts[j], A->pts[i]);
        Vector2D normal = vec2(-edge.y, edge.x);
        if (vec2_norm2(normal) > EPS_SQ) {
            double minA = 1e18, maxA = -1e18, minB = 1e18, maxB = -1e18;
            for (int k = 0; k < A->cnt; k++) { double proj = vec2_dot(A->pts[k], normal); if (proj < minA) minA = proj; if (proj > maxA) maxA = proj; }
            for (int k = 0; k < B->cnt; k++) { double proj = vec2_dot(B->pts[k], normal); if (proj < minB) minB = proj; if (proj > maxB) maxB = proj; }
            double maxMin = minA > minB ? minA : minB;
            double minMax = maxA < maxB ? maxA : maxB;
            if (maxMin > minMax + EPS) return 0;
        }
    }
    for (int i = 0; i < B->cnt; i++) {
        int j = (i + 1) % B->cnt;
        Vector2D edge = vec2_sub(B->pts[j], B->pts[i]);
        Vector2D normal = vec2(-edge.y, edge.x);
        if (vec2_norm2(normal) > EPS_SQ) {
            double minA = 1e18, maxA = -1e18, minB = 1e18, maxB = -1e18;
            for (int k = 0; k < A->cnt; k++) { double proj = vec2_dot(A->pts[k], normal); if (proj < minA) minA = proj; if (proj > maxA) maxA = proj; }
            for (int k = 0; k < B->cnt; k++) { double proj = vec2_dot(B->pts[k], normal); if (proj < minB) minB = proj; if (proj > maxB) maxB = proj; }
            double maxMin = minA > minB ? minA : minB;
            double minMax = maxA < maxB ? maxA : maxB;
            if (maxMin > minMax + EPS) return 0;
        }
    }
    return 1;
}

/* ---- Minkowski sum ---- */
static MyPolygon minkowskiSumConvex(MyPolygon *A, MyPolygon *B) {
    int na = A->cnt, nb = B->cnt;
    if (na == 0 || nb == 0) return poly_new();
    Vector2D *a = (Vector2D*)malloc(na * sizeof(Vector2D));
    Vector2D *b = (Vector2D*)malloc(nb * sizeof(Vector2D));
    memcpy(a, A->pts, na * sizeof(Vector2D));
    memcpy(b, B->pts, nb * sizeof(Vector2D));
    if (!poly_isCCW(A)) { for (int i = 0; i < na/2; i++) { Vector2D tmp = a[i]; a[i] = a[na-1-i]; a[na-1-i] = tmp; } }
    if (!poly_isCCW(B)) { for (int i = 0; i < nb/2; i++) { Vector2D tmp = b[i]; b[i] = b[nb-1-i]; b[nb-1-i] = tmp; } }
    int ia = 0, ib = 0;
    for (int i = 1; i < na; i++) if (a[i].y < a[ia].y || (a[i].y == a[ia].y && a[i].x < a[ia].x)) ia = i;
    for (int i = 1; i < nb; i++) if (b[i].y < b[ib].y || (b[i].y == b[ib].y && b[i].x < b[ib].x)) ib = i;
    Vector2D *result = (Vector2D*)malloc((na + nb) * sizeof(Vector2D));
    int rcnt = 0, i = 0, j = 0;
    while (i < na || j < nb) {
        result[rcnt++] = vec2_add(a[(ia + i) % na], b[(ib + j) % nb]);
        if (i >= na) { j++; continue; }
        if (j >= nb) { i++; continue; }
        Vector2D va = vec2_sub(a[(ia + i + 1) % na], a[(ia + i) % na]);
        Vector2D vb = vec2_sub(b[(ib + j + 1) % nb], b[(ib + j) % nb]);
        double c = vec2_cross(va, vb);
        if (c > EPS) i++; else if (c < -EPS) j++; else { i++; j++; }
    }
    free(a); free(b);
    MyPolygon res = poly_from_arr(result, rcnt);
    free(result);
    return res;
}

static MyPolygon computeNFP(MyPolygon *A, MyPolygon *B) {
    int hAn;
    Vector2D *aCopy = (Vector2D*)malloc(A->cnt * sizeof(Vector2D));
    memcpy(aCopy, A->pts, A->cnt * sizeof(Vector2D));
    Vector2D *hullA = convexHull(aCopy, A->cnt, &hAn);
    Vector2D *reflectedB = (Vector2D*)malloc(B->cnt * sizeof(Vector2D));
    for (int i = 0; i < B->cnt; i++) reflectedB[i] = vec2(-B->pts[i].x, -B->pts[i].y);
    int hBn;
    Vector2D *hullB = convexHull(reflectedB, B->cnt, &hBn);
    if (hAn == 0 || hBn == 0) { free(hullA); free(hullB); return poly_new(); }
    MyPolygon pA = poly_from_arr(hullA, hAn);
    MyPolygon pB = poly_from_arr(hullB, hBn);
    MyPolygon res = minkowskiSumConvex(&pA, &pB);
    poly_free(&pA); poly_free(&pB);
    free(hullA); free(hullB);
    return res;
}

/* ---- SAT-based minimum translation vector ---- */
static Vector2D findMinTranslation(MyPolygon *A, MyPolygon *B) {
    if (!polygonsOverlap(A, B)) return vec2(0, 0);
    int hAn, hBn;
    Vector2D *aCopy = (Vector2D*)malloc(A->cnt * sizeof(Vector2D));
    memcpy(aCopy, A->pts, A->cnt * sizeof(Vector2D));
    Vector2D *bCopy = (Vector2D*)malloc(B->cnt * sizeof(Vector2D));
    memcpy(bCopy, B->pts, B->cnt * sizeof(Vector2D));
    Vector2D *hullA = convexHull(aCopy, A->cnt, &hAn);
    Vector2D *hullB = convexHull(bCopy, B->cnt, &hBn);
    double minOverlap = 1e18;
    Vector2D bestAxis = vec2(1, 0);
    Vector2D centerA = vec2(0, 0), centerB = vec2(0, 0);
    for (int i = 0; i < hAn; i++) centerA = vec2_add(centerA, hullA[i]);
    for (int i = 0; i < hBn; i++) centerB = vec2_add(centerB, hullB[i]);
    centerA = vec2_div(centerA, hAn);
    centerB = vec2_div(centerB, hBn);

    for (int pass = 0; pass < 2; pass++) {
        Vector2D *h1 = pass == 0 ? hullA : hullB;
        Vector2D *h2 = pass == 0 ? hullB : hullA;
        int n1 = pass == 0 ? hAn : hBn;
        for (int i = 0; i < n1; i++) {
            int j = (i + 1) % n1;
            Vector2D edge = vec2_sub(h1[j], h1[i]);
            Vector2D axis = vec2(-edge.y, edge.x);
            double len = vec2_norm(axis);
            if (len < EPS) continue;
            axis = vec2_mul(axis, 1.0 / len);
            double min1 = 1e18, max1 = -1e18, min2 = 1e18, max2 = -1e18;
            for (int k = 0; k < hAn; k++) { double proj = vec2_dot(hullA[k], axis); if (proj < min1) min1 = proj; if (proj > max1) max1 = proj; }
            for (int k = 0; k < hBn; k++) { double proj = vec2_dot(hullB[k], axis); if (proj < min2) min2 = proj; if (proj > max2) max2 = proj; }
            double overlap = (max1 < max2 ? max1 : max2) - (min1 > min2 ? min1 : min2);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                bestAxis = axis;
                if (vec2_dot(vec2_sub(centerB, centerA), bestAxis) < 0) bestAxis = vec2_mul(bestAxis, -1.0);
            }
        }
    }
    free(hullA); free(hullB);
    return vec2_mul(bestAxis, minOverlap + EPS * 10);
}

/* ---- Global data ---- */
static int n = 0;
static MyPolygon bound;
static int *vertexNums = NULL;
static MyPolygon *polygons = NULL;
static Vector2D *testOut = NULL;

/* ---- Validity checks ---- */
static int isValidPlacement(int polyIdx, Vector2D trans, int *placedIndices, Vector2D *placedTrans, int placedCnt) {
    MyPolygon movedPoly = poly_moved(&polygons[polyIdx], trans);
    int res = isInsideBound(&movedPoly, &bound);
    if (res) {
        Box movedBox = poly_boundingBox(&movedPoly);
        for (int k = 0; k < placedCnt; k++) {
            int otherIdx = placedIndices[k];
            MyPolygon otherPoly = poly_moved(&polygons[otherIdx], placedTrans[k]);
            Box otherBox = poly_boundingBox(&otherPoly);
            if (!box_intersects(&movedBox, &otherBox)) { poly_free(&otherPoly); continue; }
            if (quickOverlapCheck(&movedPoly, &otherPoly)) {
                if (polygonsOverlap(&movedPoly, &otherPoly)) { poly_free(&otherPoly); res = 0; break; }
            }
            poly_free(&otherPoly);
        }
    }
    poly_free(&movedPoly);
    return res;
}

static int isValidPlacementAll(int polyIdx, Vector2D trans) {
    MyPolygon movedPoly = poly_moved(&polygons[polyIdx], trans);
    int res = isInsideBound(&movedPoly, &bound);
    if (res) {
        Box movedBox = poly_boundingBox(&movedPoly);
        for (int j = 0; j < n; j++) {
            if (j == polyIdx) continue;
            MyPolygon otherPoly = poly_moved(&polygons[j], testOut[j]);
            Box otherBox = poly_boundingBox(&otherPoly);
            if (!box_intersects(&movedBox, &otherBox)) { poly_free(&otherPoly); continue; }
            if (quickOverlapCheck(&movedPoly, &otherPoly)) {
                if (polygonsOverlap(&movedPoly, &otherPoly)) { poly_free(&otherPoly); res = 0; break; }
            }
            poly_free(&otherPoly);
        }
    }
    poly_free(&movedPoly);
    return res;
}

/* ---- Candidate placement finder ---- */
static Vector2D findBestPlacement(int polyIdx, int *placedIndices, Vector2D *placedTrans, int placedCnt) {
    MyPolygon *poly = &polygons[polyIdx];
    Box polyBB = poly_boundingBox(poly);
    Box boundBB = poly_boundingBox(&bound);
    double feasibleLeft = boundBB.left - polyBB.left;
    double feasibleRight = boundBB.right - polyBB.right;
    double feasibleBottom = boundBB.bottom - polyBB.bottom;
    double feasibleTop = boundBB.top - polyBB.top;

    if (isValidPlacement(polyIdx, vec2(0, 0), placedIndices, placedTrans, placedCnt)) return vec2(0, 0);

    typedef struct { Vector2D *data; int cnt, cap; } Vec2Vec;
    Vec2Vec candidates; candidates.data = NULL; candidates.cnt = 0; candidates.cap = 0;
    #define VEC_PUSH(v, val) do { if ((v).cnt >= (v).cap) { (v).cap = (v).cap ? (v).cap * 2 : 64; (v).data = (Vector2D*)realloc((v).data, (v).cap * sizeof(Vector2D)); } (v).data[(v).cnt++] = (val); } while(0)

    /* NFP-based candidates */
    for (int k = 0; k < placedCnt; k++) {
        int otherIdx = placedIndices[k];
        MyPolygon otherMoved = poly_moved(&polygons[otherIdx], placedTrans[k]);
        MyPolygon nfp = computeNFP(&otherMoved, poly);
        for (int i = 0; i < nfp.cnt; i++) {
            int j = (i + 1) % nfp.cnt;
            VEC_PUSH(candidates, nfp.pts[i]);
            VEC_PUSH(candidates, vec2((nfp.pts[i].x + nfp.pts[j].x) * 0.5, (nfp.pts[i].y + nfp.pts[j].y) * 0.5));
        }
        poly_free(&otherMoved); poly_free(&nfp);
    }

    /* Boundary edge sampling */
    Vector2D cen = poly_centroid(poly);
    for (int i = 0; i < bound.cnt; i++) {
        int j = (i + 1) % bound.cnt;
        double keyFractions[] = {0.0, 0.25, 1.0/3.0, 0.5, 2.0/3.0, 0.75, 1.0};
        for (int fi = 0; fi < 7; fi++) {
            double t = keyFractions[fi];
            Vector2D pos = vec2_add(bound.pts[i], vec2_mul(vec2_sub(bound.pts[j], bound.pts[i]), t));
            VEC_PUSH(candidates, vec2(pos.x - cen.x, pos.y - cen.y));
        }
        for (int s = 0; s <= 16; s++) {
            double t = (double)s / 16.0;
            Vector2D pos = vec2_add(bound.pts[i], vec2_mul(vec2_sub(bound.pts[j], bound.pts[i]), t));
            VEC_PUSH(candidates, vec2(pos.x - cen.x, pos.y - cen.y));
        }
    }

    /* Grid positions */
    int gridSteps = 40;
    for (int i = 0; i <= gridSteps; i++) {
        double t = (double)i / gridSteps;
        double x = feasibleLeft + t * (feasibleRight - feasibleLeft);
        VEC_PUSH(candidates, vec2(x, feasibleBottom));
        VEC_PUSH(candidates, vec2(x, feasibleTop));
        double y = feasibleBottom + t * (feasibleTop - feasibleBottom);
        VEC_PUSH(candidates, vec2(feasibleLeft, y));
        VEC_PUSH(candidates, vec2(feasibleRight, y));
    }
    double midX = (feasibleLeft + feasibleRight) * 0.5;
    double midY = (feasibleBottom + feasibleTop) * 0.5;
    VEC_PUSH(candidates, vec2(midX, feasibleBottom));
    VEC_PUSH(candidates, vec2(midX, feasibleTop));
    VEC_PUSH(candidates, vec2(feasibleLeft, midY));
    VEC_PUSH(candidates, vec2(feasibleRight, midY));
    VEC_PUSH(candidates, vec2(midX, midY));

    /* Interior grid */
    int innerGrid = 12;
    for (int i = 1; i < innerGrid; i++) {
        for (int j = 1; j < innerGrid; j++) {
            double x = feasibleLeft + (double)i / innerGrid * (feasibleRight - feasibleLeft);
            double y = feasibleBottom + (double)j / innerGrid * (feasibleTop - feasibleBottom);
            VEC_PUSH(candidates, vec2(x, y));
        }
    }

    double bestDist = 1e18;
    Vector2D bestVec = vec2(0, 0);
    for (int ci = 0; ci < candidates.cnt; ci++) {
        Vector2D cand = candidates.data[ci];
        if (cand.x < feasibleLeft - 10 || cand.x > feasibleRight + 10 ||
            cand.y < feasibleBottom - 10 || cand.y > feasibleTop + 10) continue;
        if (!isValidPlacement(polyIdx, cand, placedIndices, placedTrans, placedCnt)) continue;
        double dist = vec2_norm(cand);
        if (dist < bestDist) { bestDist = dist; bestVec = cand; }
    }

    /* Fallback */
    if (bestDist >= 1e17) {
        Vector2D totalTrans = vec2(0, 0);
        MyPolygon curPoly = poly_moved(poly, vec2(0, 0));
        int maxIter = n > 30 ? 1000 : 500;
        for (int iter = 0; iter < maxIter; iter++) {
            int anyOverlap = 0;
            for (int k = 0; k < placedCnt; k++) {
                int otherIdx = placedIndices[k];
                MyPolygon otherPoly = poly_moved(&polygons[otherIdx], placedTrans[k]);
                if (polygonsOverlap(&otherPoly, &curPoly)) {
                    anyOverlap = 1;
                    Vector2D mtv = findMinTranslation(&otherPoly, &curPoly);
                    poly_moveBy(&curPoly, mtv);
                    totalTrans = vec2_add(totalTrans, mtv);
                }
                poly_free(&otherPoly);
            }
            if (!isInsideBound(&curPoly, &bound)) {
                anyOverlap = 1;
                Vector2D toward = vec2_sub(poly_centroid(&bound), poly_centroid(&curPoly));
                double tLen = vec2_norm(toward);
                if (tLen > EPS) { toward = vec2_mul(vec2_div(toward, tLen), 0.05); poly_moveBy(&curPoly, toward); totalTrans = vec2_add(totalTrans, toward); }
            }
            if (!anyOverlap) break;
        }
        if (isInsideBound(&curPoly, &bound)) {
            int noOverlap = 1;
            for (int k = 0; k < placedCnt; k++) {
                MyPolygon otherPoly = poly_moved(&polygons[placedIndices[k]], placedTrans[k]);
                if (polygonsOverlap(&otherPoly, &curPoly)) { noOverlap = 0; poly_free(&otherPoly); break; }
                poly_free(&otherPoly);
            }
            if (noOverlap) bestVec = totalTrans;
        }
        poly_free(&curPoly);
    }

    free(candidates.data);
    return bestVec;
}

static double totalL(Vector2D *v, int cnt) {
    double s = 0;
    for (int i = 0; i < cnt; i++) s += vec2_norm(v[i]);
    return s;
}

static int int_cmp_desc(const void *a, const void *b) { return *(int*)b - *(int*)a; }

static void GenSolution() {
    testOut = (Vector2D*)malloc(n * sizeof(Vector2D));
    double bestL = 1e18;
    Vector2D *bestOut = (Vector2D*)malloc(n * sizeof(Vector2D));
    for (int i = 0; i < n; i++) bestOut[i] = vec2(0, 0);

    /* Compute overlap degrees */
    double *overlapDegree = (double*)calloc(n, sizeof(double));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (polygonsOverlap(&polygons[i], &polygons[j])) {
                double area = intersectionArea(&polygons[i], &polygons[j]);
                overlapDegree[i] += area; overlapDegree[j] += area;
            }
        }
    }

    /* Strategy 1: Greedy placement with orderings */
    {
        /* 6 orderings */
        int orderings[6][100]; /* assuming n <= 100 */
        int numOrderings = 0;

        /* overlap desc */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) if (overlapDegree[order[i]] < overlapDegree[order[j]]) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }
        /* overlap asc */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) if (overlapDegree[order[i]] > overlapDegree[order[j]]) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }
        /* area desc */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) if (poly_area(&polygons[order[i]]) < poly_area(&polygons[order[j]])) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }
        /* area asc */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) if (poly_area(&polygons[order[i]]) > poly_area(&polygons[order[j]])) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }
        /* dist to bound centroid */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          Vector2D bc = poly_centroid(&bound);
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) {
              double da = vec2_norm2(vec2_sub(poly_centroid(&polygons[order[i]]), bc));
              double db = vec2_norm2(vec2_sub(poly_centroid(&polygons[order[j]]), bc));
              if (da > db) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }
        /* bbox area desc */
        { int order[100]; for (int i = 0; i < n; i++) order[i] = i;
          for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) {
              Box bi = poly_boundingBox(&polygons[order[i]]);
              Box bj = poly_boundingBox(&polygons[order[j]]);
              double sa = box_width(&bi) * box_height(&bi);
              double sb = box_width(&bj) * box_height(&bj);
              if (sa < sb) { int tmp = order[i]; order[i] = order[j]; order[j] = tmp; }
          }
          memcpy(orderings[numOrderings++], order, n * sizeof(int)); }

        for (int oi = 0; oi < numOrderings; oi++) {
            if (elapsedSeconds() > g_timeBudget * 0.3) break;
            int *order = orderings[oi];
            Vector2D *curOut = (Vector2D*)malloc(n * sizeof(Vector2D));
            for (int i = 0; i < n; i++) curOut[i] = vec2(0, 0);
            int *placedIndices = (int*)malloc(n * sizeof(int));
            Vector2D *placedTrans = (Vector2D*)malloc(n * sizeof(Vector2D));
            int placedCnt = 0;

            int idx = order[0];
            Vector2D trans = vec2(0, 0);
            if (!isInsideBound(&polygons[idx], &bound)) {
                Vector2D toward = vec2_sub(poly_centroid(&bound), poly_centroid(&polygons[idx]));
                double tLen = vec2_norm(toward);
                trans = vec2(0, 0);
                if (tLen > EPS) {
                    toward = vec2_div(toward, tLen);
                    double lo = 0, hi = 1.0;
                    for (int s = 0; s < 30; s++) {
                        double mid = (lo + hi) * 0.5;
                        MyPolygon tmp = poly_moved(&polygons[idx], vec2_mul(toward, mid));
                        if (isInsideBound(&tmp, &bound)) hi = mid; else lo = mid;
                        poly_free(&tmp);
                    }
                    trans = vec2_mul(toward, hi + 0.001);
                    MyPolygon tmp = poly_moved(&polygons[idx], trans);
                    if (!isInsideBound(&tmp, &bound)) trans = vec2(0, 0);
                    poly_free(&tmp);
                }
            }
            curOut[idx] = trans;
            placedIndices[placedCnt] = idx; placedTrans[placedCnt] = trans; placedCnt++;

            for (int pi = 1; pi < n; pi++) {
                if (elapsedSeconds() > g_timeBudget * 0.3) break;
                idx = order[pi];
                curOut[idx] = findBestPlacement(idx, placedIndices, placedTrans, placedCnt);
                placedIndices[placedCnt] = idx; placedTrans[placedCnt] = curOut[idx]; placedCnt++;
            }

            double curL = totalL(curOut, n);
            if (curL < bestL) { bestL = curL; memcpy(bestOut, curOut, n * sizeof(Vector2D)); }
            free(curOut); free(placedIndices); free(placedTrans);
        }
    }

    /* Strategy 2: Iterative resolution from all-zero start */
    if (elapsedSeconds() < g_timeBudget * 0.15) {
        Vector2D *curOut = (Vector2D*)malloc(n * sizeof(Vector2D));
        for (int i = 0; i < n; i++) curOut[i] = vec2(0, 0);
        int maxIter = n > 30 ? 50 : 100;
        for (int iter = 0; iter < maxIter; iter++) {
            if (elapsedSeconds() > g_timeBudget * 0.25) break;
            int anyOverlap = 0;
            for (int i = 0; i < n; i++) {
                MyPolygon movedI = poly_moved(&polygons[i], curOut[i]);
                if (!isInsideBound(&movedI, &bound)) {
                    anyOverlap = 1;
                    Vector2D toward = vec2_sub(poly_centroid(&bound), poly_centroid(&movedI));
                    double tLen = vec2_norm(toward);
                    if (tLen > EPS) {
                        toward = vec2_div(toward, tLen);
                        double lo = 0, hi = 1.0;
                        for (int s = 0; s < 20; s++) {
                            double mid = (lo + hi) * 0.5;
                            MyPolygon tmp = poly_moved(&polygons[i], vec2_add(curOut[i], vec2_mul(toward, mid)));
                            if (isInsideBound(&tmp, &bound)) hi = mid; else lo = mid;
                            poly_free(&tmp);
                        }
                        curOut[i] = vec2_add(curOut[i], vec2_mul(toward, hi + 0.001));
                    }
                }
                poly_free(&movedI);
            }
            for (int i = 0; i < n; i++) {
                MyPolygon movedI = poly_moved(&polygons[i], curOut[i]);
                for (int j = i + 1; j < n; j++) {
                    MyPolygon movedJ = poly_moved(&polygons[j], curOut[j]);
                    if (polygonsOverlap(&movedI, &movedJ)) {
                        anyOverlap = 1;
                        Vector2D mtv = findMinTranslation(&movedI, &movedJ);
                        if (vec2_norm(curOut[i]) <= vec2_norm(curOut[j])) {
                            curOut[j] = vec2_add(curOut[j], mtv);
                            poly_free(&movedJ); movedJ = poly_moved(&polygons[j], curOut[j]);
                        } else {
                            curOut[i] = vec2_sub(curOut[i], mtv);
                            poly_free(&movedI); movedI = poly_moved(&polygons[i], curOut[i]);
                        }
                    }
                    poly_free(&movedJ);
                }
                poly_free(&movedI);
            }
            if (!anyOverlap) break;
        }
        double curL = totalL(curOut, n);
        if (curL < bestL) { bestL = curL; memcpy(bestOut, curOut, n * sizeof(Vector2D)); }
        free(curOut);
    }

    memcpy(testOut, bestOut, n * sizeof(Vector2D));

    /* Phase 2: Fix remaining overlaps */
    int maxVerifyRounds = n > 30 ? 10 : 5;
    for (int verify = 0; verify < maxVerifyRounds; verify++) {
        int needFix = 0;
        for (int i = 0; i < n; i++) {
            MyPolygon movedI = poly_moved(&polygons[i], testOut[i]);
            if (!isInsideBound(&movedI, &bound)) { needFix = 1; poly_free(&movedI); break; }
            int brk = 0;
            for (int j = i + 1; j < n; j++) {
                MyPolygon movedJ = poly_moved(&polygons[j], testOut[j]);
                if (polygonsOverlap(&movedI, &movedJ)) { needFix = 1; brk = 1; poly_free(&movedJ); break; }
                poly_free(&movedJ);
            }
            poly_free(&movedI);
            if (needFix) break;
        }
        if (!needFix) break;
        for (int i = 0; i < n; i++) {
            MyPolygon movedI = poly_moved(&polygons[i], testOut[i]);
            int hasIssue = !isInsideBound(&movedI, &bound);
            poly_free(&movedI);
            if (!hasIssue) {
                for (int j = 0; j < n; j++) {
                    if (j == i) continue;
                    MyPolygon movedI2 = poly_moved(&polygons[i], testOut[i]);
                    MyPolygon movedJ = poly_moved(&polygons[j], testOut[j]);
                    if (polygonsOverlap(&movedI2, &movedJ)) { hasIssue = 1; poly_free(&movedI2); poly_free(&movedJ); break; }
                    poly_free(&movedI2); poly_free(&movedJ);
                }
            }
            if (hasIssue) {
                int *otherIndices = (int*)malloc((n - 1) * sizeof(int));
                Vector2D *otherTrans = (Vector2D*)malloc((n - 1) * sizeof(Vector2D));
                int oc = 0;
                for (int j = 0; j < n; j++) if (j != i) { otherIndices[oc] = j; otherTrans[oc] = testOut[j]; oc++; }
                testOut[i] = findBestPlacement(i, otherIndices, otherTrans, oc);
                free(otherIndices); free(otherTrans);
            }
        }
    }

    /* Phase 3: Binary search to reduce each translation */
    for (int round = 0; round < 5; round++) {
        if (elapsedSeconds() > g_timeBudget * 0.5) break;
        for (int i = 0; i < n; i++) {
            Vector2D curTrans = testOut[i];
            Vector2D dir = vec2(-curTrans.x, -curTrans.y);
            double dirLen = vec2_norm(dir);
            if (dirLen < EPS) continue;
            dir = vec2_div(dir, dirLen);
            double lo = 0, hi = dirLen;
            int suc = 0;
            for (int step = 0; step < 80; step++) {
                double mid = (lo + hi) * 0.5;
                Vector2D tryTrans = vec2_add(curTrans, vec2_mul(dir, mid));
                MyPolygon movedPoly = poly_moved(&polygons[i], tryTrans);
                if (!isInsideBound(&movedPoly, &bound)) { hi = mid; poly_free(&movedPoly); continue; }
                int overlaps = 0;
                for (int j = 0; j < n; j++) {
                    if (j == i) continue;
                    MyPolygon movedJ = poly_moved(&polygons[j], testOut[j]);
                    if (polygonsOverlap(&movedPoly, &movedJ)) { overlaps = 1; poly_free(&movedJ); break; }
                    poly_free(&movedJ);
                }
                if (overlaps) hi = mid; else { suc = 1; lo = mid; }
                poly_free(&movedPoly);
            }
            if (suc) {
                Vector2D newTrans = vec2_add(curTrans, vec2_mul(dir, lo));
                if (vec2_norm(newTrans) < vec2_norm(curTrans) - 1e-10) testOut[i] = newTrans;
            }
        }
    }

    /* Phase 4: Re-place polygons with largest translations */
    for (int retry = 0; retry < 2; retry++) {
        if (elapsedSeconds() > g_timeBudget * 0.65) break;
        typedef struct { double len; int idx; } LenIdx;
        LenIdx *byLen = (LenIdx*)malloc(n * sizeof(LenIdx));
        for (int i = 0; i < n; i++) { byLen[i].len = vec2_norm(testOut[i]); byLen[i].idx = i; }
        for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) if (byLen[i].len < byLen[j].len) { LenIdx tmp = byLen[i]; byLen[i] = byLen[j]; byLen[j] = tmp; }
        for (int bi = 0; bi < n; bi++) {
            if (byLen[bi].len < EPS) continue;
            if (elapsedSeconds() > g_timeBudget * 0.65) break;
            int idx = byLen[bi].idx;
            int *otherIndices = (int*)malloc((n - 1) * sizeof(int));
            Vector2D *otherTrans = (Vector2D*)malloc((n - 1) * sizeof(Vector2D));
            int oc = 0;
            for (int j = 0; j < n; j++) if (j != idx) { otherIndices[oc] = j; otherTrans[oc] = testOut[j]; oc++; }
            Vector2D newTrans = findBestPlacement(idx, otherIndices, otherTrans, oc);
            if (isValidPlacementAll(idx, newTrans) && vec2_norm(newTrans) < vec2_norm(testOut[idx]) - EPS)
                testOut[idx] = newTrans;
            free(otherIndices); free(otherTrans);
        }
        free(byLen);
    }

    /* Phase 5: Multi-direction local search */
    for (int round = 0; round < 10; round++) {
        if (elapsedSeconds() > g_timeBudget * 0.85) break;
        for (int i = 0; i < n; i++) {
            if (elapsedSeconds() > g_timeBudget * 0.85) break;
            Vector2D curTrans = testOut[i];
            double curLen = vec2_norm(curTrans);
            if (curLen < EPS) continue;
            for (int d = 0; d < 24; d++) {
                double angle = d * 2.0 * PI / 24.0;
                Vector2D dir = vec2(cos(angle), sin(angle));
                double step = 0.01;
                while (step > 1e-7) {
                    Vector2D tryTrans = vec2_add(curTrans, vec2_mul(dir, step));
                    if (vec2_norm(tryTrans) >= curLen - 1e-10) { step *= 0.5; continue; }
                    MyPolygon movedPoly = poly_moved(&polygons[i], tryTrans);
                    if (!isInsideBound(&movedPoly, &bound)) { step *= 0.5; poly_free(&movedPoly); continue; }
                    int overlaps = 0;
                    for (int j = 0; j < n; j++) {
                        if (j == i) continue;
                        MyPolygon movedJ = poly_moved(&polygons[j], testOut[j]);
                        if (polygonsOverlap(&movedPoly, &movedJ)) { overlaps = 1; poly_free(&movedJ); break; }
                        poly_free(&movedJ);
                    }
                    if (!overlaps && vec2_norm(tryTrans) < curLen - 1e-10) {
                        curTrans = tryTrans; curLen = vec2_norm(tryTrans);
                    } else { step *= 0.5; }
                    poly_free(&movedPoly);
                }
            }
            testOut[i] = curTrans;
        }
    }

    /* Phase 6: Final strict verification */
    for (int i = 0; i < n; i++) {
        MyPolygon movedPoly = poly_moved(&polygons[i], testOut[i]);
        if (!isInsideBound(&movedPoly, &bound)) {
            Vector2D toward = vec2_sub(poly_centroid(&bound), poly_centroid(&movedPoly));
            double tLen = vec2_norm(toward);
            if (tLen > EPS) {
                toward = vec2_div(toward, tLen);
                for (int iter = 0; iter < 200; iter++) {
                    testOut[i] = vec2_add(testOut[i], vec2_mul(toward, 0.02));
                    MyPolygon tmp = poly_moved(&polygons[i], testOut[i]);
                    if (isInsideBound(&tmp, &bound)) { poly_free(&tmp); break; }
                    poly_free(&tmp);
                }
            }
        }
        poly_free(&movedPoly);
        movedPoly = poly_moved(&polygons[i], testOut[i]);
        for (int iter = 0; iter < 200; iter++) {
            int anyOverlap = 0;
            for (int j = 0; j < n; j++) {
                if (j == i) continue;
                MyPolygon movedJ = poly_moved(&polygons[j], testOut[j]);
                if (polygonsOverlap(&movedPoly, &movedJ)) {
                    anyOverlap = 1;
                    Vector2D mtv = findMinTranslation(&movedJ, &movedPoly);
                    poly_moveBy(&movedPoly, mtv);
                    testOut[i] = vec2_add(testOut[i], mtv);
                    poly_free(&movedJ);
                } else {
                    poly_free(&movedJ);
                }
            }
            if (!anyOverlap) break;
        }
        poly_free(&movedPoly);
    }

    free(bestOut); free(overlapDegree);
}

int main() {
    initTimer();
    int boundVertexCnt;
    scanf("%d", &boundVertexCnt);
    bound.pts = (Vector2D*)malloc(boundVertexCnt * sizeof(Vector2D));
    bound.cnt = boundVertexCnt; bound.cachedArea = -1; bound.boxCached = 0;
    for (int i = 0; i < boundVertexCnt; ++i) scanf("%lf %lf", &bound.pts[i].x, &bound.pts[i].y);

    scanf("%d", &n);
    if (n <= 2) { fprintf(stderr, "Input data error\n"); return 1; }

    if (n > 40) g_timeBudget = 58.0; else if (n > 20) g_timeBudget = 55.0; else g_timeBudget = 55.0;

    vertexNums = (int*)malloc(n * sizeof(int));
    polygons = (MyPolygon*)malloc(n * sizeof(MyPolygon));
    for (int i = 0; i < n; ++i) {
        scanf("%d", &vertexNums[i]);
        polygons[i].pts = (Vector2D*)malloc(vertexNums[i] * sizeof(Vector2D));
        polygons[i].cnt = vertexNums[i]; polygons[i].cachedArea = -1; polygons[i].boxCached = 0;
        for (int j = 0; j < vertexNums[i]; ++j) scanf("%lf %lf", &polygons[i].pts[j].x, &polygons[i].pts[j].y);
    }

    char okResp[16];
    scanf("%s", okResp);
    if (strcmp(okResp, "OK") != 0) { fprintf(stderr, "Input data error: waiting for OK\n"); return 1; }

    GenSolution();

    printf("%d\n", n);
    fflush(stdout);
    for (int i = 0; i < n; ++i) {
        printf("%.5f %.5f\n", testOut[i].x, testOut[i].y);
        fflush(stdout);
    }
    printf("OK\n");
    fflush(stdout);

    for (int i = 0; i < n; i++) poly_free(&polygons[i]);
    free(polygons); free(vertexNums); free(testOut); poly_free(&bound);
    return 0;
}
