#!/usr/bin/env python3
import struct
import sys

def check_frame_sizes(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    file_size = len(data)
    offset = 0
    
    # 读取可行域
    feasible_vertex_count = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4 + feasible_vertex_count * 2 * 4
    
    # 读取多边形数量
    polygon_count = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    
    # 跳过所有多边形
    for _ in range(polygon_count):
        vertex_count = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4 + vertex_count * 2 * 4
    
    # 读取快照数量
    snapshot_count = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    
    print(f'多边形数量: {polygon_count}')
    print(f'快照数量: {snapshot_count}')
    print(f'每帧应该有 {polygon_count} 个平移向量 (即 {polygon_count * 2} 个 float)')
    
    # 检查每一帧
    expected_translation_count = polygon_count * 2
    errors = []
    
    for frame_idx in range(snapshot_count):
        # 尝试读取平移向量
        remaining = file_size - offset
        bytes_needed = expected_translation_count * 4
        
        if bytes_needed > remaining:
            errors.append({
                'frame': frame_idx + 1,
                'offset': offset,
                'needed': bytes_needed,
                'remaining': remaining,
                'type': 'translation'
            })
            break
        
        # 读取平移向量（但不解析）
        offset += bytes_needed
        
        # 尝试读取合法性标记
        bytes_needed = polygon_count
        if bytes_needed > file_size - offset:
            errors.append({
                'frame': frame_idx + 1,
                'offset': offset,
                'needed': bytes_needed,
                'remaining': file_size - offset,
                'type': 'validity'
            })
            break
        
        offset += bytes_needed
    
    if errors:
        print(f'\n发现错误:')
        for err in errors:
            print(f"  帧 {err['frame']}: 偏移 {err['offset']}, 需要 {err['needed']} 字节, 剩余 {err['remaining']} 字节 ({err['type']})")
    else:
        print(f'\n解析成功！最终偏移: {offset}, 文件大小: {file_size}')
        if offset != file_size:
            print(f'警告：还有 {file_size - offset} 字节未解析')
    
    # 尝试找出问题帧
    # 计算从出错位置往前推，看看是否有某帧长度异常
    if errors and len(errors) > 0:
        err = errors[0]
        problem_offset = err['offset']
        
        # 从文件开头重新解析，打印最后几帧的偏移
        offset = 0
        offset += 4 + feasible_vertex_count * 2 * 4
        offset += 4
        for _ in range(polygon_count):
            vertex_count = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4 + vertex_count * 2 * 4
        offset += 4  # 快照数量
        
        frame_offsets = []
        for frame_idx in range(err['frame'] - 1):
            frame_offsets.append((frame_idx + 1, offset))
            offset += expected_translation_count * 4 + polygon_count
        
        print(f'\n最后5帧的偏移位置:')
        for frame_num, frame_offset in frame_offsets[-5:]:
            print(f'  帧 {frame_num}: 偏移 {frame_offset}')

if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else "test_py/Che_linux/dist/data/snapshoot_data.snpst"
    check_frame_sizes(filepath)
