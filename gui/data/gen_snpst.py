import struct, os

def pack_f32(v): return struct.pack('<f', v)
def pack_u32(v): return struct.pack('<I', v)
def pack_u8(v):  return struct.pack('<B', v)

buf = bytearray()

# ── 可行域多边形：10×10 矩形 ──────────────────────────────
boundary = [(0,0),(10,0),(10,10),(0,10)]
buf += pack_u32(len(boundary))
for x,y in boundary:
    buf += pack_f32(x); buf += pack_f32(y)

# ── 3 个初始多边形（起始位置全部叠在左下角附近）────────────
polygons = [
    # 三角形
    [(0.0, 0.0), (2.0, 0.0), (1.0, 1.8)],
    # 正方形
    [(0.0, 0.0), (2.5, 0.0), (2.5, 2.5), (0.0, 2.5)],
    # 五边形
    [(0.0,0.0),(2.0,0.0),(2.5,1.5),(1.0,2.5),(-0.5,1.5)],
]
buf += pack_u32(len(polygons))
for poly in polygons:
    buf += pack_u32(len(poly))
    for x,y in poly:
        buf += pack_f32(x); buf += pack_f32(y)

# ── 快照帧：模拟多边形逐渐分散落位 ─────────────────────────
# 最终目标位置（dx,dy）：多边形中心移动到各自目标
# 三角形 → 左下  正方形 → 中  五边形 → 右上
final_dx = [1.0, 3.5, 6.5]
final_dy = [0.5, 0.5, 4.5]

N_FRAMES = 20
frames = []
for f in range(N_FRAMES):
    t = (f+1) / N_FRAMES   # 0→1
    trans = [(final_dx[i]*t, final_dy[i]*t) for i in range(3)]
    # 前半段第2个多边形"非法"（模拟穿透），后半段全合法
    if f < N_FRAMES // 2:
        validity = [1, 0, 1]
    else:
        validity = [1, 1, 1]
    frames.append((trans, validity))

buf += pack_u32(len(frames))
for trans, validity in frames:
    for dx, dy in trans:
        buf += pack_f32(dx); buf += pack_f32(dy)
    for v in validity:
        buf += pack_u8(v)

outpath = os.path.join(os.path.dirname(__file__), 'sample.snpst')
with open(outpath, 'wb') as f:
    f.write(buf)

print(f"生成完毕: {outpath}")
print(f"文件大小: {len(buf)} bytes")
print(f"  可行域: {len(boundary)} 顶点")
print(f"  多边形: {len(polygons)} 个")
print(f"  快照帧: {len(frames)} 帧")
