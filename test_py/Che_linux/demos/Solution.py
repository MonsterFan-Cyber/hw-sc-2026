import math
import time
import sys

EPS = 1e-9
PI = math.acos(-1.0)
EPS_SQ = EPS * EPS

g_startTime = time.time()
def elapsedSeconds():
    return time.time() - g_startTime

g_timeBudget = 55.0

class Vector2D:
    __slots__ = ('x', 'y')
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x); self.y = float(y)
    def __add__(self, p): return Vector2D(self.x + p.x, self.y + p.y)
    def __sub__(self, p): return Vector2D(self.x - p.x, self.y - p.y)
    def __mul__(self, t): return Vector2D(self.x * t, self.y * t)
    def __rmul__(self, t): return Vector2D(self.x * t, self.y * t)
    def __truediv__(self, t): return Vector2D(self.x / t, self.y / t)
    def __iadd__(self, p): self.x += p.x; self.y += p.y; return self
    def __isub__(self, p): self.x -= p.x; self.y -= p.y; return self
    def dot(self, p): return self.x * p.x + self.y * p.y
    def cross(self, p): return self.x * p.y - self.y * p.x
    def norm(self): return math.sqrt(self.x * self.x + self.y * self.y)
    def norm2(self): return self.x * self.x + self.y * self.y
    def __neg__(self): return Vector2D(-self.x, -self.y)

class Box:
    __slots__ = ('left', 'right', 'bottom', 'top')
    def __init__(self):
        self.left = 1e18; self.right = -1e18; self.bottom = 1e18; self.top = -1e18
    def width(self): return self.right - self.left
    def height(self): return self.top - self.bottom
    def intersects(self, b):
        return not (self.right < b.left - EPS or b.right < self.left - EPS or
                    self.top < b.bottom - EPS or b.top < self.bottom - EPS)
    def center(self): return Vector2D((self.left + self.right) * 0.5, (self.bottom + self.top) * 0.5)

class Polygon:
    __slots__ = ('pts', 'cnt', 'cachedArea', 'cachedBox', 'boxCached')
    def __init__(self, pts=None):
        if pts is not None:
            self.pts = list(pts); self.cnt = len(self.pts)
        else:
            self.pts = []; self.cnt = 0
        self.cachedArea = -1; self.boxCached = False; self.cachedBox = None

    def signedArea(self):
        a = 0; j = self.cnt - 1
        for i in range(self.cnt):
            a += (self.pts[j].x + self.pts[i].x) * (self.pts[j].y - self.pts[i].y)
            j = i
        return -a * 0.5

    def area(self):
        if self.cachedArea < 0: self.cachedArea = abs(self.signedArea())
        return self.cachedArea

    def isCCW(self): return self.signedArea() >= 0

    def centroid(self):
        cx = cy = a = 0
        for i in range(self.cnt):
            j = (i + 1) % self.cnt
            cr = self.pts[i].cross(self.pts[j])
            a += cr; cx += (self.pts[i].x + self.pts[j].x) * cr; cy += (self.pts[i].y + self.pts[j].y) * cr
        a *= 0.5
        if abs(a) < EPS:
            cx = cy = 0
            for p in self.pts: cx += p.x; cy += p.y
            return Vector2D(cx / self.cnt, cy / self.cnt)
        return Vector2D(cx / (6.0 * a), cy / (6.0 * a))

    def boundingBox(self):
        if not self.boxCached:
            b = Box()
            for p in self.pts:
                b.left = min(b.left, p.x); b.right = max(b.right, p.x)
                b.bottom = min(b.bottom, p.y); b.top = max(b.top, p.y)
            self.cachedBox = b; self.boxCached = True
        return self.cachedBox

    def moved(self, v):
        return Polygon([p + v for p in self.pts])

    def moveBy(self, v):
        self.cachedArea = -1; self.boxCached = False
        for i in range(self.cnt): self.pts[i] = self.pts[i] + v

# ---- Segment intersection ----
def segIntersect(p1, p2, p3, p4):
    d1x = p2.x - p1.x; d1y = p2.y - p1.y
    d2x = p4.x - p3.x; d2y = p4.y - p3.y
    denom = d1x * d2y - d1y * d2x
    if abs(denom) < 1e-15: return None
    t = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / denom
    u = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / denom
    if t >= -1e-10 and t <= 1.0 + 1e-10 and u >= -1e-10 and u <= 1.0 + 1e-10:
        return Vector2D(p1.x + t * d1x, p1.y + t * d1y)
    return None

# ---- Clip by half-plane (Sutherland-Hodgman) ----
def clipByHalfPlane(poly, p1, p2):
    output = []
    n = len(poly)
    if n == 0: return output
    for i in range(n):
        cur = poly[i]; prev = poly[(i - 1 + n) % n]
        curSide = (p2.x - p1.x) * (cur.y - p1.y) - (p2.y - p1.y) * (cur.x - p1.x)
        prevSide = (p2.x - p1.x) * (prev.y - p1.y) - (p2.y - p1.y) * (prev.x - p1.x)
        curIn = curSide >= -1e-10
        prevIn = prevSide >= -1e-10
        if curIn:
            if not prevIn:
                ip = segIntersect(prev, cur, p1, p2)
                if ip is not None: output.append(ip)
            output.append(cur)
        elif prevIn:
            ip = segIntersect(prev, cur, p1, p2)
            if ip is not None: output.append(ip)
    return output

def clipPolygonByConvex(subject, clipper):
    output = list(subject)
    for i in range(clipper.cnt):
        if not output: break
        j = (i + 1) % clipper.cnt
        output = clipByHalfPlane(output, clipper.pts[i], clipper.pts[j])
    return output

def dpointArea(pts):
    n = len(pts)
    if n < 3: return 0.0
    a = 0; j = n - 1
    for i in range(n):
        a += pts[j].x * pts[i].y - pts[i].x * pts[j].y
        j = i
    return abs(a) * 0.5

def cross3(O, A, B):
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x)

# ---- Triangulation (ear-clipping) ----
def isEar(pts, indices, i):
    n = len(indices)
    prev = indices[(i - 1 + n) % n]; curr = indices[i]; nxt = indices[(i + 1) % n]
    a = pts[prev]; b = pts[curr]; c = pts[nxt]
    cr = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    if cr <= 0: return False
    for j in range(n):
        if j == (i - 1 + n) % n or j == i or j == (i + 1) % n: continue
        p = pts[indices[j]]
        d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
        d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x)
        d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x)
        if not ((d1 < 0) or (d2 < 0) or (d3 < 0)) or not ((d1 > 0) or (d2 > 0) or (d3 > 0)):
            return False
    return True

def triangulate(poly):
    triangles = []
    n = poly.cnt
    if n < 3: return triangles
    indices = list(range(n))
    if not poly.isCCW(): indices.reverse()
    remaining = n; i = 0; safety = n * n * 2
    while remaining > 3 and safety > 0:
        safety -= 1
        if isEar(poly.pts, indices, i % remaining):
            prev = (i - 1 + remaining) % remaining; nxt = (i + 1) % remaining
            triangles.append((indices[prev], indices[i % remaining], indices[nxt]))
            del indices[i % remaining]
            remaining -= 1; i = prev % remaining
        else:
            i = (i + 1) % remaining
    if remaining == 3: triangles.append((indices[0], indices[1], indices[2]))
    return triangles

def intersectionArea(subject, clipper):
    sb = subject.boundingBox(); cb = clipper.boundingBox()
    if not sb.intersects(cb): return 0.0
    tris = triangulate(clipper)
    totalArea = 0.0
    subPts = list(subject.pts)
    for tri in tris:
        triPoly = Polygon([clipper.pts[tri[0]], clipper.pts[tri[1]], clipper.pts[tri[2]]])
        clipped = clipPolygonByConvex(subPts, triPoly)
        totalArea += dpointArea(clipped)
    return totalArea

# ---- Convex Hull ----
def convexHull(P):
    n = len(P)
    if n <= 1: return list(P)
    P = sorted(P, key=lambda p: (p.x, p.y))
    H = [None] * (2 * n)
    k = 0
    for i in range(n):
        while k >= 2 and cross3(H[k-2], H[k-1], P[i]) <= 0: k -= 1
        H[k] = P[i]; k += 1
    t = k + 1
    for i in range(n - 2, -1, -1):
        while k >= t and cross3(H[k-2], H[k-1], P[i]) <= 0: k -= 1
        H[k] = P[i]; k += 1
    result = H[:k]
    if k > 1 and result[-1].x == result[0].x and result[-1].y == result[0].y:
        result.pop()
    return result

# ---- Overlap and boundary checks ----
def isInsideBound(poly, bnd):
    polyArea = poly.area()
    if polyArea < EPS: return True
    interArea = intersectionArea(poly, bnd)
    areaDiff = polyArea - interArea
    return areaDiff < 1e-10 and areaDiff < polyArea * 1e-6

def polygonsOverlap(A, B):
    ba = A.boundingBox(); bb = B.boundingBox()
    if not ba.intersects(bb): return False
    interArea = intersectionArea(A, B)
    if interArea < 1e-10: return False
    minArea = min(A.area(), B.area())
    if minArea < EPS: return False
    return interArea >= minArea * 1e-6

def quickOverlapCheck(A, B):
    ba = A.boundingBox(); bb = B.boundingBox()
    if not ba.intersects(bb): return False
    def checkAxis(axis):
        projsA = [p.dot(axis) for p in A.pts]; projsB = [p.dot(axis) for p in B.pts]
        return max(min(projsA), min(projsB)) <= min(max(projsA), max(projsB)) + EPS
    for i in range(A.cnt):
        j = (i + 1) % A.cnt
        edge = A.pts[j] - A.pts[i]; normal = Vector2D(-edge.y, edge.x)
        if normal.norm2() > EPS_SQ and not checkAxis(normal): return False
    for i in range(B.cnt):
        j = (i + 1) % B.cnt
        edge = B.pts[j] - B.pts[i]; normal = Vector2D(-edge.y, edge.x)
        if normal.norm2() > EPS_SQ and not checkAxis(normal): return False
    return True

# ---- Minkowski sum / NFP ----
def minkowskiSumConvex(A, B):
    na = A.cnt; nb = B.cnt
    if na == 0 or nb == 0: return Polygon()
    a = list(A.pts); b = list(B.pts)
    if not A.isCCW(): a.reverse()
    if not B.isCCW(): b.reverse()
    ia = 0; ib = 0
    for i in range(1, na):
        if a[i].y < a[ia].y or (a[i].y == a[ia].y and a[i].x < a[ia].x): ia = i
    for i in range(1, nb):
        if b[i].y < b[ib].y or (b[i].y == b[ib].y and b[i].x < b[ib].x): ib = i
    result = []
    i = 0; j = 0
    while i < na or j < nb:
        result.append(a[(ia + i) % na] + b[(ib + j) % nb])
        if i >= na: j += 1; continue
        if j >= nb: i += 1; continue
        va = a[(ia + i + 1) % na] - a[(ia + i) % na]
        vb = b[(ib + j + 1) % nb] - b[(ib + j) % nb]
        c = va.cross(vb)
        if c > EPS: i += 1
        elif c < -EPS: j += 1
        else: i += 1; j += 1
    return Polygon(result)

def computeNFP(A, B):
    hullA = convexHull(list(A.pts))
    reflectedB = [Vector2D(-p.x, -p.y) for p in B.pts]
    hullB = convexHull(reflectedB)
    if not hullA or not hullB: return Polygon()
    return minkowskiSumConvex(Polygon(hullA), Polygon(hullB))

# ---- SAT-based minimum translation vector ----
def findMinTranslation(A, B):
    if not polygonsOverlap(A, B): return Vector2D(0, 0)
    hullA = convexHull(list(A.pts)); hullB = convexHull(list(B.pts))
    minOverlap = 1e18; bestAxis = Vector2D(1, 0)
    centerA = Vector2D(sum(p.x for p in hullA) / len(hullA), sum(p.y for p in hullA) / len(hullA))
    centerB = Vector2D(sum(p.x for p in hullB) / len(hullB), sum(p.y for p in hullB) / len(hullB))

    for hull1, hull2 in [(hullA, hullB), (hullB, hullA)]:
        n1 = len(hull1)
        for i in range(n1):
            j = (i + 1) % n1
            edge = hull1[j] - hull1[i]; axis = Vector2D(-edge.y, edge.x)
            length = axis.norm()
            if length < EPS: continue
            axis = axis * (1.0 / length)
            projsA = [p.dot(axis) for p in hullA]; projsB = [p.dot(axis) for p in hullB]
            overlap = min(max(projsA), max(projsB)) - max(min(projsA), min(projsB))
            if overlap < minOverlap:
                minOverlap = overlap; bestAxis = axis
                if (centerB - centerA).dot(bestAxis) < 0: bestAxis = bestAxis * (-1.0)
    return bestAxis * (minOverlap + EPS * 10)

# ---- Global data ----
n = 0
bound = Polygon()
vertexNums = []
polygons = []
testOut = []

def isValidPlacement(polyIdx, trans, placedIndices, placedTrans):
    movedPoly = polygons[polyIdx].moved(trans)
    if not isInsideBound(movedPoly, bound): return False
    movedBox = movedPoly.boundingBox()
    for k in range(len(placedIndices)):
        otherIdx = placedIndices[k]
        otherPoly = polygons[otherIdx].moved(placedTrans[k])
        if not movedBox.intersects(otherPoly.boundingBox()): continue
        if quickOverlapCheck(movedPoly, otherPoly):
            if polygonsOverlap(movedPoly, otherPoly): return False
    return True

def isValidPlacementAll(polyIdx, trans):
    movedPoly = polygons[polyIdx].moved(trans)
    if not isInsideBound(movedPoly, bound): return False
    movedBox = movedPoly.boundingBox()
    for j in range(n):
        if j == polyIdx: continue
        otherPoly = polygons[j].moved(testOut[j])
        if not movedBox.intersects(otherPoly.boundingBox()): continue
        if quickOverlapCheck(movedPoly, otherPoly):
            if polygonsOverlap(movedPoly, otherPoly): return False
    return True

def findBestPlacement(polyIdx, placedIndices, placedTrans):
    poly = polygons[polyIdx]
    polyBB = poly.boundingBox(); boundBB = bound.boundingBox()
    feasibleLeft = boundBB.left - polyBB.left
    feasibleRight = boundBB.right - polyBB.right
    feasibleBottom = boundBB.bottom - polyBB.bottom
    feasibleTop = boundBB.top - polyBB.top

    if isValidPlacement(polyIdx, Vector2D(0, 0), placedIndices, placedTrans):
        return Vector2D(0, 0)

    candidates = []

    # NFP-based candidates
    for k in range(len(placedIndices)):
        otherIdx = placedIndices[k]
        otherMoved = polygons[otherIdx].moved(placedTrans[k])
        nfp = computeNFP(otherMoved, poly)
        for i in range(nfp.cnt):
            j = (i + 1) % nfp.cnt
            candidates.append(nfp.pts[i])
            candidates.append(Vector2D((nfp.pts[i].x + nfp.pts[j].x) * 0.5,
                                       (nfp.pts[i].y + nfp.pts[j].y) * 0.5))

    # Boundary edge sampling
    cen = poly.centroid()
    for i in range(bound.cnt):
        j = (i + 1) % bound.cnt
        keyFractions = [0.0, 0.25, 1.0/3.0, 0.5, 2.0/3.0, 0.75, 1.0]
        for t in keyFractions:
            pos = bound.pts[i] + (bound.pts[j] - bound.pts[i]) * t
            candidates.append(Vector2D(pos.x - cen.x, pos.y - cen.y))
        for s in range(17):
            t = s / 16.0
            pos = bound.pts[i] + (bound.pts[j] - bound.pts[i]) * t
            candidates.append(Vector2D(pos.x - cen.x, pos.y - cen.y))

    # Grid positions
    gridSteps = 40
    for i in range(gridSteps + 1):
        t = i / gridSteps
        x = feasibleLeft + t * (feasibleRight - feasibleLeft)
        candidates.append(Vector2D(x, feasibleBottom))
        candidates.append(Vector2D(x, feasibleTop))
        y = feasibleBottom + t * (feasibleTop - feasibleBottom)
        candidates.append(Vector2D(feasibleLeft, y))
        candidates.append(Vector2D(feasibleRight, y))

    midX = (feasibleLeft + feasibleRight) * 0.5
    midY = (feasibleBottom + feasibleTop) * 0.5
    candidates.append(Vector2D(midX, feasibleBottom))
    candidates.append(Vector2D(midX, feasibleTop))
    candidates.append(Vector2D(feasibleLeft, midY))
    candidates.append(Vector2D(feasibleRight, midY))
    candidates.append(Vector2D(midX, midY))

    # Interior grid
    innerGrid = 12
    for i in range(1, innerGrid):
        for j in range(1, innerGrid):
            x = feasibleLeft + i / innerGrid * (feasibleRight - feasibleLeft)
            y = feasibleBottom + j / innerGrid * (feasibleTop - feasibleBottom)
            candidates.append(Vector2D(x, y))

    bestDist = 1e18; bestVec = Vector2D(0, 0)
    for cand in candidates:
        if (cand.x < feasibleLeft - 10 or cand.x > feasibleRight + 10 or
            cand.y < feasibleBottom - 10 or cand.y > feasibleTop + 10): continue
        if not isValidPlacement(polyIdx, cand, placedIndices, placedTrans): continue
        dist = cand.norm()
        if dist < bestDist: bestDist = dist; bestVec = cand

    # Fallback: iterative separation
    if bestDist >= 1e17:
        totalTrans = Vector2D(0, 0)
        curPoly = poly.moved(Vector2D(0, 0))
        maxIter = 1000 if n > 30 else 500
        for _ in range(maxIter):
            anyOverlap = False
            for k in range(len(placedIndices)):
                otherIdx = placedIndices[k]
                otherPoly = polygons[otherIdx].moved(placedTrans[k])
                if polygonsOverlap(otherPoly, curPoly):
                    anyOverlap = True
                    mtv = findMinTranslation(otherPoly, curPoly)
                    curPoly.moveBy(mtv)
                    totalTrans = totalTrans + mtv
            if not isInsideBound(curPoly, bound):
                anyOverlap = True
                toward = bound.centroid() - curPoly.centroid()
                tLen = toward.norm()
                if tLen > EPS:
                    toward = (toward / tLen) * 0.05
                    curPoly.moveBy(toward); totalTrans = totalTrans + toward
            if not anyOverlap: break
        if isInsideBound(curPoly, bound):
            noOverlap = True
            for k in range(len(placedIndices)):
                otherPoly = polygons[placedIndices[k]].moved(placedTrans[k])
                if polygonsOverlap(otherPoly, curPoly): noOverlap = False; break
            if noOverlap: bestVec = totalTrans

    return bestVec

def totalL(v):
    return sum(p.norm() for p in v)

def GenSolution():
    global testOut
    testOut = [Vector2D(0, 0) for _ in range(n)]
    bestL = 1e18
    bestOut = [Vector2D(0, 0) for _ in range(n)]

    # Compute overlap degrees
    overlapDegree = [0.0] * n
    for i in range(n):
        for j in range(i + 1, n):
            if polygonsOverlap(polygons[i], polygons[j]):
                area = intersectionArea(polygons[i], polygons[j])
                overlapDegree[i] += area; overlapDegree[j] += area

    # ===== Strategy 1: Greedy placement with orderings =====
    orderings = []
    order = sorted(range(n), key=lambda a: -overlapDegree[a]); orderings.append(order)
    order = sorted(range(n), key=lambda a: overlapDegree[a]); orderings.append(order)
    order = sorted(range(n), key=lambda a: -polygons[a].area()); orderings.append(order)
    order = sorted(range(n), key=lambda a: polygons[a].area()); orderings.append(order)
    bc = bound.centroid()
    order = sorted(range(n), key=lambda a: (polygons[a].centroid() - bc).norm2()); orderings.append(order)
    order = sorted(range(n), key=lambda a: -(polygons[a].boundingBox().width() * polygons[a].boundingBox().height())); orderings.append(order)

    for oi, order in enumerate(orderings):
        if elapsedSeconds() > g_timeBudget * 0.3: break
        curOut = [Vector2D(0, 0) for _ in range(n)]
        placedIndices = []; placedTrans = []

        idx = order[0]; trans = Vector2D(0, 0)
        if not isInsideBound(polygons[idx], bound):
            toward = bound.centroid() - polygons[idx].centroid()
            trans = toward
            if not isInsideBound(polygons[idx].moved(trans), bound): trans = Vector2D(0, 0)
        curOut[idx] = trans; placedIndices.append(idx); placedTrans.append(trans)

        for pi in range(1, n):
            if elapsedSeconds() > g_timeBudget * 0.3: break
            idx = order[pi]
            curOut[idx] = findBestPlacement(idx, placedIndices, placedTrans)
            placedIndices.append(idx); placedTrans.append(curOut[idx])

        curL = totalL(curOut)
        if curL < bestL: bestL = curL; bestOut = list(curOut)

    # ===== Strategy 2: Iterative resolution from all-zero start =====
    if n <= 30 and elapsedSeconds() < g_timeBudget * 0.15:
        curOut = [Vector2D(0, 0) for _ in range(n)]
        for _ in range(100):
            if elapsedSeconds() > g_timeBudget * 0.2: break
            anyOverlap = False

            for i in range(n):
                movedI = polygons[i].moved(curOut[i])
                if not isInsideBound(movedI, bound):
                    anyOverlap = True
                    toward = bound.centroid() - movedI.centroid()
                    tLen = toward.norm()
                    if tLen > EPS:
                        toward = toward / tLen
                        lo = 0; hi = 1.0
                        for _ in range(20):
                            mid = (lo + hi) * 0.5
                            if isInsideBound(polygons[i].moved(curOut[i] + toward * mid), bound): hi = mid
                            else: lo = mid
                        curOut[i] = curOut[i] + toward * (hi + 0.001)

            for i in range(n):
                movedI = polygons[i].moved(curOut[i])
                for j in range(i + 1, n):
                    movedJ = polygons[j].moved(curOut[j])
                    if polygonsOverlap(movedI, movedJ):
                        anyOverlap = True
                        mtv = findMinTranslation(movedI, movedJ)
                        if curOut[i].norm() <= curOut[j].norm():
                            curOut[j] = curOut[j] + mtv; movedJ = polygons[j].moved(curOut[j])
                        else:
                            curOut[i] = curOut[i] - mtv; movedI = polygons[i].moved(curOut[i])
            if not anyOverlap: break
        curL = totalL(curOut)
        if curL < bestL: bestL = curL; bestOut = list(curOut)

    testOut = bestOut

    # ===== Phase 2: Fix any remaining overlaps =====
    for _ in range(5):
        needFix = False
        for i in range(n):
            movedI = polygons[i].moved(testOut[i])
            if not isInsideBound(movedI, bound): needFix = True; break
            for j in range(i + 1, n):
                if polygonsOverlap(movedI, polygons[j].moved(testOut[j])): needFix = True; break
            if needFix: break
        if not needFix: break
        for i in range(n):
            movedI = polygons[i].moved(testOut[i])
            hasIssue = not isInsideBound(movedI, bound)
            if not hasIssue:
                for j in range(n):
                    if j == i: continue
                    if polygonsOverlap(movedI, polygons[j].moved(testOut[j])): hasIssue = True; break
            if hasIssue:
                otherIndices = [j for j in range(n) if j != i]
                otherTrans = [testOut[j] for j in range(n) if j != i]
                testOut[i] = findBestPlacement(i, otherIndices, otherTrans)

    # ===== Phase 3: Binary search to reduce each translation =====
    for _ in range(5):
        if elapsedSeconds() > g_timeBudget * 0.5: break
        for i in range(n):
            curTrans = testOut[i]
            dir = Vector2D(-curTrans.x, -curTrans.y)
            dirLen = dir.norm()
            if dirLen < EPS: continue
            dir = dir / dirLen
            lo = 0; hi = dirLen; suc = False
            for _ in range(80):
                mid = (lo + hi) * 0.5
                tryTrans = curTrans + dir * mid
                movedPoly = polygons[i].moved(tryTrans)
                if not isInsideBound(movedPoly, bound): hi = mid; continue
                overlaps = False
                for j in range(n):
                    if j == i: continue
                    if polygonsOverlap(movedPoly, polygons[j].moved(testOut[j])): overlaps = True; break
                if overlaps: hi = mid
                else: suc = True; lo = mid
            if suc:
                newTrans = curTrans + dir * lo
                if newTrans.norm() < curTrans.norm() - 1e-10: testOut[i] = newTrans

    # ===== Phase 4: Re-place polygons with largest translations =====
    for _ in range(2):
        if elapsedSeconds() > g_timeBudget * 0.65: break
        byLen = sorted(range(n), key=lambda i: -testOut[i].norm())
        for idx in byLen:
            if testOut[idx].norm() < EPS: continue
            if elapsedSeconds() > g_timeBudget * 0.65: break
            otherIndices = [j for j in range(n) if j != idx]
            otherTrans = [testOut[j] for j in range(n) if j != idx]
            newTrans = findBestPlacement(idx, otherIndices, otherTrans)
            if isValidPlacementAll(idx, newTrans) and newTrans.norm() < testOut[idx].norm() - EPS:
                testOut[idx] = newTrans

    # ===== Phase 5: Multi-direction local search =====
    for _ in range(10):
        if elapsedSeconds() > g_timeBudget * 0.85: break
        for i in range(n):
            if elapsedSeconds() > g_timeBudget * 0.85: break
            curTrans = testOut[i]; curLen = curTrans.norm()
            if curLen < EPS: continue
            for d in range(24):
                angle = d * 2.0 * PI / 24.0
                dir = Vector2D(math.cos(angle), math.sin(angle))
                step = 0.01
                while step > 1e-7:
                    tryTrans = curTrans + dir * step
                    if tryTrans.norm() >= curLen - 1e-10: step *= 0.5; continue
                    movedPoly = polygons[i].moved(tryTrans)
                    if not isInsideBound(movedPoly, bound): step *= 0.5; continue
                    overlaps = False
                    for j in range(n):
                        if j == i: continue
                        if polygonsOverlap(movedPoly, polygons[j].moved(testOut[j])): overlaps = True; break
                    if not overlaps and tryTrans.norm() < curLen - 1e-10:
                        curTrans = tryTrans; curLen = tryTrans.norm()
                    else: step *= 0.5
            testOut[i] = curTrans

    # ===== Phase 6: Final strict verification =====
    for i in range(n):
        movedPoly = polygons[i].moved(testOut[i])
        if not isInsideBound(movedPoly, bound):
            toward = bound.centroid() - movedPoly.centroid()
            tLen = toward.norm()
            if tLen > EPS:
                toward = toward / tLen
                for _ in range(200):
                    testOut[i] = testOut[i] + toward * 0.02
                    if isInsideBound(polygons[i].moved(testOut[i]), bound): break
        movedPoly = polygons[i].moved(testOut[i])
        for _ in range(200):
            anyOverlap = False
            for j in range(n):
                if j == i: continue
                movedJ = polygons[j].moved(testOut[j])
                if polygonsOverlap(movedPoly, movedJ):
                    anyOverlap = True
                    mtv = findMinTranslation(movedJ, movedPoly)
                    movedPoly.moveBy(mtv)
                    testOut[i] = testOut[i] + mtv
            if not anyOverlap: break

def main():
    global n, bound, vertexNums, polygons, testOut, g_timeBudget

    input_data = sys.stdin.read().split()
    pos = 0

    def readInt():
        nonlocal pos; v = int(input_data[pos]); pos += 1; return v
    def readFloat():
        nonlocal pos; v = float(input_data[pos]); pos += 1; return v

    boundVertexCnt = readInt()
    boundPts = [Vector2D(readFloat(), readFloat()) for _ in range(boundVertexCnt)]
    bound = Polygon(boundPts)

    n = readInt()
    if n <= 2:
        print("Input data error", file=sys.stderr); return 1

    if n > 50: g_timeBudget = 50.0
    elif n > 30: g_timeBudget = 52.0
    else: g_timeBudget = 55.0

    vertexNums = []
    polygons = []
    for _ in range(n):
        vn = readInt()
        vertexNums.append(vn)
        polygons.append(Polygon([Vector2D(readFloat(), readFloat()) for _ in range(vn)]))

    if pos < len(input_data):
        okResp = input_data[pos]; pos += 1
        if okResp != "OK":
            print("Input data error: waiting for OK", file=sys.stderr); return 1

    GenSolution()

    print(n)
    sys.stdout.flush()
    for i in range(n):
        print(f"{testOut[i].x:.5f} {testOut[i].y:.5f}")
        sys.stdout.flush()
    return 0

if __name__ == "__main__":
    main()
