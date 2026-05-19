import java.io.*;
import java.util.*;

public class Solution {
    static final double EPS = 1e-9;
    static final double PI = Math.acos(-1.0);
    static final double EPS_SQ = EPS * EPS;

    static long g_startTime = System.nanoTime();
    static double elapsedSeconds() { return (System.nanoTime() - g_startTime) * 1e-9; }
    static double g_timeBudget = 55.0;

    static class Vector2D {
        double x, y;
        Vector2D(double x, double y) { this.x = x; this.y = y; }
        Vector2D add(Vector2D p) { return new Vector2D(x + p.x, y + p.y); }
        Vector2D sub(Vector2D p) { return new Vector2D(x - p.x, y - p.y); }
        Vector2D mul(double t) { return new Vector2D(x * t, y * t); }
        Vector2D div(double t) { return new Vector2D(x / t, y / t); }
        double dot(Vector2D p) { return x * p.x + y * p.y; }
        double cross(Vector2D p) { return x * p.y - y * p.x; }
        double norm() { return Math.sqrt(x * x + y * y); }
        double norm2() { return x * x + y * y; }
    }

    static class Box {
        double left = 1e18, right = -1e18, bottom = 1e18, top = -1e18;
        double width() { return right - left; }
        double height() { return top - bottom; }
        boolean intersects(Box b) {
            return !(right < b.left - EPS || b.right < left - EPS || top < b.bottom - EPS || b.top < bottom - EPS);
        }
        Vector2D center() { return new Vector2D((left + right) * 0.5, (bottom + top) * 0.5); }
    }

    static class Polygon {
        List<Vector2D> pts;
        int cnt;
        double cachedArea = -1;
        Box cachedBox = null;

        Polygon() { pts = new ArrayList<>(); cnt = 0; }
        Polygon(List<Vector2D> v) { pts = new ArrayList<>(v); cnt = v.size(); }

        double signedArea() {
            double a = 0;
            for (int i = 0, j = cnt - 1; i < cnt; ++i) {
                a += (pts.get(j).x + pts.get(i).x) * (pts.get(j).y - pts.get(i).y);
                j = i;
            }
            return -a * 0.5;
        }
        double area() { if (cachedArea < 0) cachedArea = Math.abs(signedArea()); return cachedArea; }
        boolean isCCW() { return signedArea() >= 0; }

        Vector2D centroid() {
            double cx = 0, cy = 0, a = 0;
            for (int i = 0; i < cnt; i++) {
                int j = (i + 1) % cnt;
                double cr = pts.get(i).cross(pts.get(j));
                a += cr; cx += (pts.get(i).x + pts.get(j).x) * cr; cy += (pts.get(i).y + pts.get(j).y) * cr;
            }
            a *= 0.5;
            if (Math.abs(a) < EPS) {
                cx = cy = 0;
                for (Vector2D p : pts) { cx += p.x; cy += p.y; }
                return new Vector2D(cx / cnt, cy / cnt);
            }
            return new Vector2D(cx / (6.0 * a), cy / (6.0 * a));
        }

        Box boundingBox() {
            if (cachedBox == null) {
                Box b = new Box();
                for (Vector2D p : pts) {
                    b.left = Math.min(b.left, p.x); b.right = Math.max(b.right, p.x);
                    b.bottom = Math.min(b.bottom, p.y); b.top = Math.max(b.top, p.y);
                }
                cachedBox = b;
            }
            return cachedBox;
        }

        Polygon moved(Vector2D v) {
            List<Vector2D> newPts = new ArrayList<>(cnt);
            for (Vector2D p : pts) newPts.add(p.add(v));
            return new Polygon(newPts);
        }
        void moveBy(Vector2D v) {
            cachedArea = -1; cachedBox = null;
            for (int i = 0; i < cnt; i++) pts.set(i, pts.get(i).add(v));
        }
    }

    static class DPoint {
        double x, y;
        DPoint(double x, double y) { this.x = x; this.y = y; }
    }

    static DPoint segIntersect(DPoint p1, DPoint p2, DPoint p3, DPoint p4) {
        double d1x = p2.x - p1.x, d1y = p2.y - p1.y;
        double d2x = p4.x - p3.x, d2y = p4.y - p3.y;
        double denom = d1x * d2y - d1y * d2x;
        if (Math.abs(denom) < 1e-15) return null;
        double t = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / denom;
        double u = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / denom;
        if (t >= -1e-10 && t <= 1.0 + 1e-10 && u >= -1e-10 && u <= 1.0 + 1e-10) {
            return new DPoint(p1.x + t * d1x, p1.y + t * d1y);
        }
        return null;
    }

    static List<DPoint> clipByHalfPlane(List<DPoint> poly, DPoint p1, DPoint p2) {
        List<DPoint> output = new ArrayList<>();
        int n = poly.size();
        if (n == 0) return output;
        for (int i = 0; i < n; i++) {
            DPoint cur = poly.get(i), prev = poly.get((i - 1 + n) % n);
            double curSide = (p2.x - p1.x) * (cur.y - p1.y) - (p2.y - p1.y) * (cur.x - p1.x);
            double prevSide = (p2.x - p1.x) * (prev.y - p1.y) - (p2.y - p1.y) * (prev.x - p1.x);
            boolean curIn = curSide >= -1e-10;
            boolean prevIn = prevSide >= -1e-10;
            if (curIn) {
                if (!prevIn) { DPoint ip = segIntersect(prev, cur, p1, p2); if (ip != null) output.add(ip); }
                output.add(cur);
            } else if (prevIn) {
                DPoint ip = segIntersect(prev, cur, p1, p2); if (ip != null) output.add(ip);
            }
        }
        return output;
    }

    static List<DPoint> clipPolygonByConvex(List<DPoint> subject, Polygon clipper) {
        List<DPoint> output = new ArrayList<>(subject);
        for (int i = 0; i < clipper.cnt && !output.isEmpty(); i++) {
            int j = (i + 1) % clipper.cnt;
            output = clipByHalfPlane(output, new DPoint(clipper.pts.get(i).x, clipper.pts.get(i).y),
                    new DPoint(clipper.pts.get(j).x, clipper.pts.get(j).y));
        }
        return output;
    }

    static double dpointArea(List<DPoint> pts) {
        int n = pts.size();
        if (n < 3) return 0.0;
        double a = 0;
        for (int i = 0, j = n - 1; i < n; j = i++) a += pts.get(j).x * pts.get(i).y - pts.get(i).x * pts.get(j).y;
        return Math.abs(a) * 0.5;
    }

    static double cross3(Vector2D O, Vector2D A, Vector2D B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    static boolean isEar(List<Vector2D> pts, List<Integer> indices, int i) {
        int n = indices.size();
        int prev = indices.get((i - 1 + n) % n), curr = indices.get(i), nxt = indices.get((i + 1) % n);
        Vector2D a = pts.get(prev), b = pts.get(curr), c = pts.get(nxt);
        double cr = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        if (cr <= 0) return false;
        for (int j = 0; j < n; j++) {
            if (j == (i - 1 + n) % n || j == i || j == (i + 1) % n) continue;
            Vector2D p = pts.get(indices.get(j));
            double d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            double d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
            double d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
            if (!((d1 < 0) || (d2 < 0) || (d3 < 0)) || !((d1 > 0) || (d2 > 0) || (d3 > 0))) return false;
        }
        return true;
    }

    static int[] triangulate(Polygon poly) {
        int n = poly.cnt;
        if (n < 3) return new int[0];
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < n; i++) indices.add(i);
        if (!poly.isCCW()) Collections.reverse(indices);
        int[] triangles = new int[n * 3]; // max size
        int triCnt = 0;
        int remaining = n, i = 0, safety = n * n * 2;
        while (remaining > 3 && safety-- > 0) {
            if (isEar(poly.pts, indices, i % remaining)) {
                int prev = (i - 1 + remaining) % remaining, nxt = (i + 1) % remaining;
                triangles[triCnt++] = indices.get(prev);
                triangles[triCnt++] = indices.get(i % remaining);
                triangles[triCnt++] = indices.get(nxt);
                indices.remove(i % remaining);
                remaining--; i = prev % remaining;
            } else { i = (i + 1) % remaining; }
        }
        if (remaining == 3) {
            triangles[triCnt++] = indices.get(0);
            triangles[triCnt++] = indices.get(1);
            triangles[triCnt++] = indices.get(2);
        }
        return Arrays.copyOf(triangles, triCnt);
    }

    static double intersectionArea(Polygon subject, Polygon clipper) {
        Box sb = subject.boundingBox(), cb = clipper.boundingBox();
        if (!sb.intersects(cb)) return 0.0;
        int[] tris = triangulate(clipper);
        double totalArea = 0.0;
        List<DPoint> subPts = new ArrayList<>();
        for (Vector2D p : subject.pts) subPts.add(new DPoint(p.x, p.y));
        for (int t = 0; t < tris.length; t += 3) {
            List<Vector2D> triPts = Arrays.asList(clipper.pts.get(tris[t]), clipper.pts.get(tris[t+1]), clipper.pts.get(tris[t+2]));
            Polygon triPoly = new Polygon(triPts);
            List<DPoint> clipped = clipPolygonByConvex(subPts, triPoly);
            totalArea += dpointArea(clipped);
        }
        return totalArea;
    }

    static List<Vector2D> convexHull(List<Vector2D> P) {
        int n = P.size();
        if (n <= 1) return new ArrayList<>(P);
        List<Vector2D> sorted = new ArrayList<>(P);
        sorted.sort((a, b) -> a.x < b.x || (a.x == b.x && a.y < b.y) ? -1 : 1);
        Vector2D[] H = new Vector2D[2 * n];
        int k = 0;
        for (int i = 0; i < n; i++) { while (k >= 2 && cross3(H[k-2], H[k-1], sorted.get(i)) <= 0) k--; H[k++] = sorted.get(i); }
        for (int i = n - 2, t = k + 1; i >= 0; i--) { while (k >= t && cross3(H[k-2], H[k-1], sorted.get(i)) <= 0) k--; H[k++] = sorted.get(i); }
        List<Vector2D> result = new ArrayList<>();
        for (int i = 0; i < k; i++) result.add(H[i]);
        if (k > 1 && result.get(result.size()-1).x == result.get(0).x && result.get(result.size()-1).y == result.get(0).y)
            result.remove(result.size() - 1);
        return result;
    }

    static boolean isInsideBound(Polygon poly, Polygon bnd) {
        double polyArea = poly.area();
        if (polyArea < EPS) return true;
        double interArea = intersectionArea(poly, bnd);
        double areaDiff = polyArea - interArea;
        return areaDiff < 1e-10 && areaDiff < polyArea * 1e-6;
    }

    static boolean polygonsOverlap(Polygon A, Polygon B) {
        Box ba = A.boundingBox(), bb = B.boundingBox();
        if (!ba.intersects(bb)) return false;
        double interArea = intersectionArea(A, B);
        if (interArea < 1e-10) return false;
        double minArea = Math.min(A.area(), B.area());
        if (minArea < EPS) return false;
        return interArea >= minArea * 1e-6;
    }

    static boolean quickOverlapCheck(Polygon A, Polygon B) {
        Box ba = A.boundingBox(), bb = B.boundingBox();
        if (!ba.intersects(bb)) return false;
        for (int i = 0; i < A.cnt; i++) {
            int j = (i + 1) % A.cnt;
            Vector2D edge = A.pts.get(j).sub(A.pts.get(i));
            Vector2D normal = new Vector2D(-edge.y, edge.x);
            if (normal.norm2() > EPS_SQ) {
                double minA = 1e18, maxA = -1e18, minB = 1e18, maxB = -1e18;
                for (Vector2D p : A.pts) { double proj = p.dot(normal); minA = Math.min(minA, proj); maxA = Math.max(maxA, proj); }
                for (Vector2D p : B.pts) { double proj = p.dot(normal); minB = Math.min(minB, proj); maxB = Math.max(maxB, proj); }
                if (Math.max(minA, minB) > Math.min(maxA, maxB) + EPS) return false;
            }
        }
        for (int i = 0; i < B.cnt; i++) {
            int j = (i + 1) % B.cnt;
            Vector2D edge = B.pts.get(j).sub(B.pts.get(i));
            Vector2D normal = new Vector2D(-edge.y, edge.x);
            if (normal.norm2() > EPS_SQ) {
                double minA = 1e18, maxA = -1e18, minB = 1e18, maxB = -1e18;
                for (Vector2D p : A.pts) { double proj = p.dot(normal); minA = Math.min(minA, proj); maxA = Math.max(maxA, proj); }
                for (Vector2D p : B.pts) { double proj = p.dot(normal); minB = Math.min(minB, proj); maxB = Math.max(maxB, proj); }
                if (Math.max(minA, minB) > Math.min(maxA, maxB) + EPS) return false;
            }
        }
        return true;
    }

    static Polygon minkowskiSumConvex(Polygon A, Polygon B) {
        int na = A.cnt, nb = B.cnt;
        if (na == 0 || nb == 0) return new Polygon();
        List<Vector2D> a = new ArrayList<>(A.pts), b = new ArrayList<>(B.pts);
        if (!A.isCCW()) Collections.reverse(a);
        if (!B.isCCW()) Collections.reverse(b);
        int ia = 0, ib = 0;
        for (int i = 1; i < na; i++) if (a.get(i).y < a.get(ia).y || (a.get(i).y == a.get(ia).y && a.get(i).x < a.get(ia).x)) ia = i;
        for (int i = 1; i < nb; i++) if (b.get(i).y < b.get(ib).y || (b.get(i).y == b.get(ib).y && b.get(i).x < b.get(ib).x)) ib = i;
        List<Vector2D> result = new ArrayList<>();
        int i = 0, j = 0;
        while (i < na || j < nb) {
            result.add(a.get((ia + i) % na).add(b.get((ib + j) % nb)));
            if (i >= na) { j++; continue; }
            if (j >= nb) { i++; continue; }
            Vector2D va = a.get((ia + i + 1) % na).sub(a.get((ia + i) % na));
            Vector2D vb = b.get((ib + j + 1) % nb).sub(b.get((ib + j) % nb));
            double c = va.cross(vb);
            if (c > EPS) i++; else if (c < -EPS) j++; else { i++; j++; }
        }
        return new Polygon(result);
    }

    static Polygon computeNFP(Polygon A, Polygon B) {
        List<Vector2D> hullA = convexHull(new ArrayList<>(A.pts));
        List<Vector2D> reflectedB = new ArrayList<>();
        for (Vector2D p : B.pts) reflectedB.add(new Vector2D(-p.x, -p.y));
        List<Vector2D> hullB = convexHull(reflectedB);
        if (hullA.isEmpty() || hullB.isEmpty()) return new Polygon();
        return minkowskiSumConvex(new Polygon(hullA), new Polygon(hullB));
    }

    static Vector2D findMinTranslation(Polygon A, Polygon B) {
        if (!polygonsOverlap(A, B)) return new Vector2D(0, 0);
        List<Vector2D> hullA = convexHull(new ArrayList<>(A.pts));
        List<Vector2D> hullB = convexHull(new ArrayList<>(B.pts));
        double minOverlap = 1e18; Vector2D bestAxis = new Vector2D(1, 0);
        Vector2D centerA = new Vector2D(0, 0), centerB = new Vector2D(0, 0);
        for (Vector2D p : hullA) centerA = centerA.add(p);
        for (Vector2D p : hullB) centerB = centerB.add(p);
        centerA = centerA.div(hullA.size()); centerB = centerB.div(hullB.size());

        List<List<Vector2D>> hulls = Arrays.asList(hullA, hullB);
        List<List<Vector2D>> others = Arrays.asList(hullB, hullA);
        for (int pass = 0; pass < 2; pass++) {
            List<Vector2D> h1 = hulls.get(pass);
            int n1 = h1.size();
            for (int ii = 0; ii < n1; ii++) {
                int jj = (ii + 1) % n1;
                Vector2D edge = h1.get(jj).sub(h1.get(ii));
                Vector2D axis = new Vector2D(-edge.y, edge.x);
                double len = axis.norm();
                if (len < EPS) continue;
                axis = axis.mul(1.0 / len);
                double min1 = 1e18, max1 = -1e18, min2 = 1e18, max2 = -1e18;
                for (Vector2D p : hullA) { double proj = p.dot(axis); min1 = Math.min(min1, proj); max1 = Math.max(max1, proj); }
                for (Vector2D p : hullB) { double proj = p.dot(axis); min2 = Math.min(min2, proj); max2 = Math.max(max2, proj); }
                double overlap = Math.min(max1, max2) - Math.max(min1, min2);
                if (overlap < minOverlap) {
                    minOverlap = overlap; bestAxis = axis;
                    if (centerB.sub(centerA).dot(bestAxis) < 0) bestAxis = bestAxis.mul(-1.0);
                }
            }
        }
        return bestAxis.mul(minOverlap + EPS * 10);
    }

    // Global data
    static int n = 0;
    static Polygon bound;
    static int[] vertexNums;
    static Polygon[] polygons;
    static Vector2D[] testOut;

    static boolean isValidPlacement(int polyIdx, Vector2D trans, List<Integer> placedIndices, List<Vector2D> placedTrans) {
        Polygon movedPoly = polygons[polyIdx].moved(trans);
        if (!isInsideBound(movedPoly, bound)) return false;
        Box movedBox = movedPoly.boundingBox();
        for (int k = 0; k < placedIndices.size(); k++) {
            int otherIdx = placedIndices.get(k);
            Polygon otherPoly = polygons[otherIdx].moved(placedTrans.get(k));
            if (!movedBox.intersects(otherPoly.boundingBox())) continue;
            if (quickOverlapCheck(movedPoly, otherPoly)) {
                if (polygonsOverlap(movedPoly, otherPoly)) return false;
            }
        }
        return true;
    }

    static boolean isValidPlacementAll(int polyIdx, Vector2D trans) {
        Polygon movedPoly = polygons[polyIdx].moved(trans);
        if (!isInsideBound(movedPoly, bound)) return false;
        Box movedBox = movedPoly.boundingBox();
        for (int j = 0; j < n; j++) {
            if (j == polyIdx) continue;
            Polygon otherPoly = polygons[j].moved(testOut[j]);
            if (!movedBox.intersects(otherPoly.boundingBox())) continue;
            if (quickOverlapCheck(movedPoly, otherPoly)) {
                if (polygonsOverlap(movedPoly, otherPoly)) return false;
            }
        }
        return true;
    }

    static Vector2D findBestPlacement(int polyIdx, List<Integer> placedIndices, List<Vector2D> placedTrans) {
        Polygon poly = polygons[polyIdx];
        Box polyBB = poly.boundingBox(), boundBB = bound.boundingBox();
        double feasibleLeft = boundBB.left - polyBB.left;
        double feasibleRight = boundBB.right - polyBB.right;
        double feasibleBottom = boundBB.bottom - polyBB.bottom;
        double feasibleTop = boundBB.top - polyBB.top;

        if (isValidPlacement(polyIdx, new Vector2D(0, 0), placedIndices, placedTrans)) return new Vector2D(0, 0);

        List<Vector2D> candidates = new ArrayList<>();

        // NFP-based candidates
        for (int k = 0; k < placedIndices.size(); k++) {
            int otherIdx = placedIndices.get(k);
            Polygon otherMoved = polygons[otherIdx].moved(placedTrans.get(k));
            Polygon nfp = computeNFP(otherMoved, poly);
            for (int ii = 0; ii < nfp.cnt; ii++) {
                int jj = (ii + 1) % nfp.cnt;
                candidates.add(nfp.pts.get(ii));
                candidates.add(new Vector2D((nfp.pts.get(ii).x + nfp.pts.get(jj).x) * 0.5,
                        (nfp.pts.get(ii).y + nfp.pts.get(jj).y) * 0.5));
            }
        }

        // Boundary edge sampling
        Vector2D cen = poly.centroid();
        for (int ii = 0; ii < bound.cnt; ii++) {
            int jj = (ii + 1) % bound.cnt;
            double[] keyFractions = {0.0, 0.25, 1.0/3.0, 0.5, 2.0/3.0, 0.75, 1.0};
            for (double t : keyFractions) {
                Vector2D pos = bound.pts.get(ii).add(bound.pts.get(jj).sub(bound.pts.get(ii)).mul(t));
                candidates.add(new Vector2D(pos.x - cen.x, pos.y - cen.y));
            }
            for (int s = 0; s <= 16; s++) {
                double t = (double) s / 16.0;
                Vector2D pos = bound.pts.get(ii).add(bound.pts.get(jj).sub(bound.pts.get(ii)).mul(t));
                candidates.add(new Vector2D(pos.x - cen.x, pos.y - cen.y));
            }
        }

        // Grid positions
        int gridSteps = 40;
        for (int ii = 0; ii <= gridSteps; ii++) {
            double t = (double) ii / gridSteps;
            double x = feasibleLeft + t * (feasibleRight - feasibleLeft);
            candidates.add(new Vector2D(x, feasibleBottom));
            candidates.add(new Vector2D(x, feasibleTop));
            double y = feasibleBottom + t * (feasibleTop - feasibleBottom);
            candidates.add(new Vector2D(feasibleLeft, y));
            candidates.add(new Vector2D(feasibleRight, y));
        }
        double midX = (feasibleLeft + feasibleRight) * 0.5;
        double midY = (feasibleBottom + feasibleTop) * 0.5;
        candidates.add(new Vector2D(midX, feasibleBottom));
        candidates.add(new Vector2D(midX, feasibleTop));
        candidates.add(new Vector2D(feasibleLeft, midY));
        candidates.add(new Vector2D(feasibleRight, midY));
        candidates.add(new Vector2D(midX, midY));

        // Interior grid
        int innerGrid = 12;
        for (int ii = 1; ii < innerGrid; ii++) {
            for (int jj = 1; jj < innerGrid; jj++) {
                double x = feasibleLeft + (double) ii / innerGrid * (feasibleRight - feasibleLeft);
                double y = feasibleBottom + (double) jj / innerGrid * (feasibleTop - feasibleBottom);
                candidates.add(new Vector2D(x, y));
            }
        }

        double bestDist = 1e18; Vector2D bestVec = new Vector2D(0, 0);
        for (Vector2D cand : candidates) {
            if (cand.x < feasibleLeft - 10 || cand.x > feasibleRight + 10 ||
                cand.y < feasibleBottom - 10 || cand.y > feasibleTop + 10) continue;
            if (!isValidPlacement(polyIdx, cand, placedIndices, placedTrans)) continue;
            double dist = cand.norm();
            if (dist < bestDist) { bestDist = dist; bestVec = cand; }
        }

        // Fallback: iterative separation
        if (bestDist >= 1e17) {
            Vector2D totalTrans = new Vector2D(0, 0);
            Polygon curPoly = poly.moved(new Vector2D(0, 0));
            int maxIter = n > 30 ? 1000 : 500;
            for (int iter = 0; iter < maxIter; iter++) {
                boolean anyOverlap = false;
                for (int k = 0; k < placedIndices.size(); k++) {
                    int otherIdx = placedIndices.get(k);
                    Polygon otherPoly = polygons[otherIdx].moved(placedTrans.get(k));
                    if (polygonsOverlap(otherPoly, curPoly)) {
                        anyOverlap = true;
                        Vector2D mtv = findMinTranslation(otherPoly, curPoly);
                        curPoly.moveBy(mtv);
                        totalTrans = totalTrans.add(mtv);
                    }
                }
                if (!isInsideBound(curPoly, bound)) {
                    anyOverlap = true;
                    Vector2D toward = bound.centroid().sub(curPoly.centroid());
                    double tLen = toward.norm();
                    if (tLen > EPS) { toward = toward.div(tLen).mul(0.05); curPoly.moveBy(toward); totalTrans = totalTrans.add(toward); }
                }
                if (!anyOverlap) break;
            }
            if (isInsideBound(curPoly, bound)) {
                boolean noOverlap = true;
                for (int k = 0; k < placedIndices.size(); k++) {
                    Polygon otherPoly = polygons[placedIndices.get(k)].moved(placedTrans.get(k));
                    if (polygonsOverlap(otherPoly, curPoly)) { noOverlap = false; break; }
                }
                if (noOverlap) bestVec = totalTrans;
            }
        }
        return bestVec;
    }

    static double totalL(Vector2D[] v) {
        double s = 0;
        for (Vector2D p : v) s += p.norm();
        return s;
    }

    static void GenSolution() {
        testOut = new Vector2D[n];
        double bestL = 1e18;
        Vector2D[] bestOut = new Vector2D[n];
        Arrays.fill(bestOut, new Vector2D(0, 0));

        // Compute overlap degrees
        double[] overlapDegree = new double[n];
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (polygonsOverlap(polygons[i], polygons[j])) {
                    double area = intersectionArea(polygons[i], polygons[j]);
                    overlapDegree[i] += area; overlapDegree[j] += area;
                }
            }
        }

        // Strategy 1: Greedy placement with orderings
        {
            List<List<Integer>> orderings = new ArrayList<>();
            List<Integer> order = new ArrayList<>();
            for (int i = 0; i < n; i++) order.add(i);
            order.sort((a, b) -> Double.compare(overlapDegree[b], overlapDegree[a])); orderings.add(new ArrayList<>(order));
            order.sort((a, b) -> Double.compare(overlapDegree[a], overlapDegree[b])); orderings.add(new ArrayList<>(order));
            order.sort((a, b) -> Double.compare(polygons[b].area(), polygons[a].area())); orderings.add(new ArrayList<>(order));
            order.sort((a, b) -> Double.compare(polygons[a].area(), polygons[b].area())); orderings.add(new ArrayList<>(order));
            Vector2D bc = bound.centroid();
            order.sort((a, b) -> Double.compare(polygons[a].centroid().sub(bc).norm2(), polygons[b].centroid().sub(bc).norm2())); orderings.add(new ArrayList<>(order));
            order.sort((a, b) -> {
                double sa = polygons[a].boundingBox().width() * polygons[a].boundingBox().height();
                double sb = polygons[b].boundingBox().width() * polygons[b].boundingBox().height();
                return Double.compare(sb, sa);
            }); orderings.add(new ArrayList<>(order));

            for (int oi = 0; oi < orderings.size(); oi++) {
                if (elapsedSeconds() > g_timeBudget * 0.3) break;
                List<Integer> curOrder = orderings.get(oi);
                Vector2D[] curOut = new Vector2D[n]; Arrays.fill(curOut, new Vector2D(0, 0));
                List<Integer> placedIndices = new ArrayList<>();
                List<Vector2D> placedTrans = new ArrayList<>();

                int idx = curOrder.get(0); Vector2D trans = new Vector2D(0, 0);
                if (!isInsideBound(polygons[idx], bound)) {
                    Vector2D toward = bound.centroid().sub(polygons[idx].centroid());
                    trans = toward;
                    if (!isInsideBound(polygons[idx].moved(trans), bound)) trans = new Vector2D(0, 0);
                }
                curOut[idx] = trans; placedIndices.add(idx); placedTrans.add(trans);

                for (int pi = 1; pi < n; pi++) {
                    if (elapsedSeconds() > g_timeBudget * 0.3) break;
                    idx = curOrder.get(pi);
                    curOut[idx] = findBestPlacement(idx, placedIndices, placedTrans);
                    placedIndices.add(idx); placedTrans.add(curOut[idx]);
                }

                double curL = totalL(curOut);
                if (curL < bestL) { bestL = curL; System.arraycopy(curOut, 0, bestOut, 0, n); }
            }
        }

        // Strategy 2: Iterative resolution from all-zero start
        if (elapsedSeconds() < g_timeBudget * 0.15) {
            Vector2D[] curOut = new Vector2D[n]; Arrays.fill(curOut, new Vector2D(0, 0));
            int maxIter = n > 30 ? 50 : 100;
            for (int iter = 0; iter < maxIter; iter++) {
                if (elapsedSeconds() > g_timeBudget * 0.25) break;
                boolean anyOverlap = false;
                for (int i = 0; i < n; i++) {
                    Polygon movedI = polygons[i].moved(curOut[i]);
                    if (!isInsideBound(movedI, bound)) {
                        anyOverlap = true;
                        Vector2D toward = bound.centroid().sub(movedI.centroid());
                        double tLen = toward.norm();
                        if (tLen > EPS) {
                            toward = toward.div(tLen);
                            double lo = 0, hi = 1.0;
                            for (int s = 0; s < 20; s++) {
                                double mid = (lo + hi) * 0.5;
                                if (isInsideBound(polygons[i].moved(curOut[i].add(toward.mul(mid))), bound)) hi = mid; else lo = mid;
                            }
                            curOut[i] = curOut[i].add(toward.mul(hi + 0.001));
                        }
                    }
                }
                for (int i = 0; i < n; i++) {
                    Polygon movedI = polygons[i].moved(curOut[i]);
                    for (int j = i + 1; j < n; j++) {
                        Polygon movedJ = polygons[j].moved(curOut[j]);
                        if (polygonsOverlap(movedI, movedJ)) {
                            anyOverlap = true;
                            Vector2D mtv = findMinTranslation(movedI, movedJ);
                            if (curOut[i].norm() <= curOut[j].norm()) {
                                curOut[j] = curOut[j].add(mtv); movedJ = polygons[j].moved(curOut[j]);
                            } else {
                                curOut[i] = curOut[i].sub(mtv); movedI = polygons[i].moved(curOut[i]);
                            }
                        }
                    }
                }
                if (!anyOverlap) break;
            }
            double curL = totalL(curOut);
            if (curL < bestL) { bestL = curL; System.arraycopy(curOut, 0, bestOut, 0, n); }
        }

        testOut = bestOut;

        // Phase 2: Fix remaining overlaps
        int maxVerifyRounds = n > 30 ? 10 : 5;
        for (int verify = 0; verify < maxVerifyRounds; verify++) {
            boolean needFix = false;
            for (int i = 0; i < n; i++) {
                Polygon movedI = polygons[i].moved(testOut[i]);
                if (!isInsideBound(movedI, bound)) { needFix = true; break; }
                for (int j = i + 1; j < n; j++) {
                    if (polygonsOverlap(movedI, polygons[j].moved(testOut[j]))) { needFix = true; break; }
                }
                if (needFix) break;
            }
            if (!needFix) break;
            for (int i = 0; i < n; i++) {
                Polygon movedI = polygons[i].moved(testOut[i]);
                boolean hasIssue = !isInsideBound(movedI, bound);
                if (!hasIssue) {
                    for (int j = 0; j < n; j++) {
                        if (j == i) continue;
                        if (polygonsOverlap(movedI, polygons[j].moved(testOut[j]))) { hasIssue = true; break; }
                    }
                }
                if (hasIssue) {
                    List<Integer> otherIndices = new ArrayList<>();
                    List<Vector2D> otherTrans = new ArrayList<>();
                    for (int j = 0; j < n; j++) if (j != i) { otherIndices.add(j); otherTrans.add(testOut[j]); }
                    testOut[i] = findBestPlacement(i, otherIndices, otherTrans);
                }
            }
        }

        // Phase 3: Binary search to reduce each translation
        for (int round = 0; round < 5; round++) {
            if (elapsedSeconds() > g_timeBudget * 0.5) break;
            for (int i = 0; i < n; i++) {
                Vector2D curTrans = testOut[i];
                Vector2D dir = new Vector2D(-curTrans.x, -curTrans.y);
                double dirLen = dir.norm();
                if (dirLen < EPS) continue;
                dir = dir.div(dirLen);
                double lo = 0, hi = dirLen; boolean suc = false;
                for (int step = 0; step < 80; step++) {
                    double mid = (lo + hi) * 0.5;
                    Vector2D tryTrans = curTrans.add(dir.mul(mid));
                    Polygon movedPoly = polygons[i].moved(tryTrans);
                    if (!isInsideBound(movedPoly, bound)) { hi = mid; continue; }
                    boolean overlaps = false;
                    for (int j = 0; j < n; j++) {
                        if (j == i) continue;
                        if (polygonsOverlap(movedPoly, polygons[j].moved(testOut[j]))) { overlaps = true; break; }
                    }
                    if (overlaps) hi = mid; else { suc = true; lo = mid; }
                }
                if (suc) {
                    Vector2D newTrans = curTrans.add(dir.mul(lo));
                    if (newTrans.norm() < curTrans.norm() - 1e-10) testOut[i] = newTrans;
                }
            }
        }

        // Phase 4: Re-place polygons with largest translations
        for (int retry = 0; retry < 2; retry++) {
            if (elapsedSeconds() > g_timeBudget * 0.65) break;
            Integer[] byLenArr = new Integer[n];
            for (int i = 0; i < n; i++) byLenArr[i] = i;
            Arrays.sort(byLenArr, (a, b) -> Double.compare(testOut[b].norm(), testOut[a].norm()));
            for (int idx : byLenArr) {
                if (testOut[idx].norm() < EPS) continue;
                if (elapsedSeconds() > g_timeBudget * 0.65) break;
                List<Integer> otherIndices = new ArrayList<>();
                List<Vector2D> otherTrans = new ArrayList<>();
                for (int j = 0; j < n; j++) if (j != idx) { otherIndices.add(j); otherTrans.add(testOut[j]); }
                Vector2D newTrans = findBestPlacement(idx, otherIndices, otherTrans);
                if (isValidPlacementAll(idx, newTrans) && newTrans.norm() < testOut[idx].norm() - EPS) testOut[idx] = newTrans;
            }
        }

        // Phase 5: Multi-direction local search
        for (int round = 0; round < 10; round++) {
            if (elapsedSeconds() > g_timeBudget * 0.85) break;
            for (int i = 0; i < n; i++) {
                if (elapsedSeconds() > g_timeBudget * 0.85) break;
                Vector2D curTrans = testOut[i]; double curLen = curTrans.norm();
                if (curLen < EPS) continue;
                for (int d = 0; d < 24; d++) {
                    double angle = d * 2.0 * PI / 24.0;
                    Vector2D dir = new Vector2D(Math.cos(angle), Math.sin(angle));
                    double step = 0.01;
                    while (step > 1e-7) {
                        Vector2D tryTrans = curTrans.add(dir.mul(step));
                        if (tryTrans.norm() >= curLen - 1e-10) { step *= 0.5; continue; }
                        Polygon movedPoly = polygons[i].moved(tryTrans);
                        if (!isInsideBound(movedPoly, bound)) { step *= 0.5; continue; }
                        boolean overlaps = false;
                        for (int j = 0; j < n; j++) {
                            if (j == i) continue;
                            if (polygonsOverlap(movedPoly, polygons[j].moved(testOut[j]))) { overlaps = true; break; }
                        }
                        if (!overlaps && tryTrans.norm() < curLen - 1e-10) {
                            curTrans = tryTrans; curLen = tryTrans.norm();
                        } else { step *= 0.5; }
                    }
                }
                testOut[i] = curTrans;
            }
        }

        // Phase 6: Final strict verification
        for (int i = 0; i < n; i++) {
            Polygon movedPoly = polygons[i].moved(testOut[i]);
            if (!isInsideBound(movedPoly, bound)) {
                Vector2D toward = bound.centroid().sub(movedPoly.centroid());
                double tLen = toward.norm();
                if (tLen > EPS) {
                    toward = toward.div(tLen);
                    for (int iter = 0; iter < 200; iter++) {
                        testOut[i] = testOut[i].add(toward.mul(0.02));
                        if (isInsideBound(polygons[i].moved(testOut[i]), bound)) break;
                    }
                }
            }
            movedPoly = polygons[i].moved(testOut[i]);
            for (int iter = 0; iter < 200; iter++) {
                boolean anyOverlap = false;
                for (int j = 0; j < n; j++) {
                    if (j == i) continue;
                    Polygon movedJ = polygons[j].moved(testOut[j]);
                    if (polygonsOverlap(movedPoly, movedJ)) {
                        anyOverlap = true;
                        Vector2D mtv = findMinTranslation(movedJ, movedPoly);
                        movedPoly.moveBy(mtv);
                        testOut[i] = testOut[i].add(mtv);
                    }
                }
                if (!anyOverlap) break;
            }
        }
    }

    public static void main(String[] args) throws Exception {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        StreamTokenizer st = new StreamTokenizer(br);
        st.parseNumbers();

        st.nextToken(); int boundVertexCnt = (int) st.nval;
        List<Vector2D> boundPts = new ArrayList<>();
        for (int i = 0; i < boundVertexCnt; i++) {
            st.nextToken(); double x = st.nval; st.nextToken(); double y = st.nval;
            boundPts.add(new Vector2D(x, y));
        }
        bound = new Polygon(boundPts);

        st.nextToken(); n = (int) st.nval;
        if (n <= 2) { System.err.println("Input data error"); return; }

        if (n > 40) g_timeBudget = 58.0; else if (n > 20) g_timeBudget = 55.0; else g_timeBudget = 55.0;

        vertexNums = new int[n]; polygons = new Polygon[n];
        for (int i = 0; i < n; i++) {
            st.nextToken(); vertexNums[i] = (int) st.nval;
            List<Vector2D> pts = new ArrayList<>();
            for (int j = 0; j < vertexNums[i]; j++) {
                st.nextToken(); double x = st.nval; st.nextToken(); double y = st.nval;
                pts.add(new Vector2D(x, y));
            }
            polygons[i] = new Polygon(pts);
        }

        st.nextToken(); String okResp = st.sval;
        if (okResp == null || !okResp.equals("OK")) { System.err.println("Input data error: waiting for OK"); return; }

        GenSolution();

        StringBuilder sb = new StringBuilder();
        sb.append(n).append("\n");
        for (int i = 0; i < n; i++) {
            sb.append(String.format("%.5f %.5f\n", testOut[i].x, testOut[i].y));
        }
        sb.append("OK\n");
        System.out.print(sb.toString());
        System.out.flush();
    }
}
