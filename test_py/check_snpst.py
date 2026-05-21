#!/usr/bin/env python3
import struct
import sys

def check_snpst_file(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    file_size = len(data)
    print(f"文件大小: {file_size} 字节 ({file_size/1024:.2f} KB)")
    
    offset = 0
    
    # 读取可行域多边形顶点数量
    if offset + 4 > file_size:
        print(f"错误：偏移 {offset} 处需要 4 字节，但文件已结束")
        return
    feasible_vertex_count = struct.unpack('<I', data[offset:offset+4])[0]
    print(f"\n可行域多边形顶点数量: {feasible_vertex_count}")
    offset += 4
    
    # 读取可行域多边形坐标序列
    feasible_coords_count = feasible_vertex_count * 2
    bytes_needed = feasible_coords_count * 4
    if offset + bytes_needed > file_size:
        print(f"错误：偏移 {offset} 处需要 {bytes_needed} 字节，但剩余 {file_size - offset} 字节")
        return
    feasible_coords = struct.unpack('<{}f'.format(feasible_coords_count), 
                                   data[offset:offset + bytes_needed])
    offset += bytes_needed
    print(f"可行域多边形坐标: {feasible_coords}")
    
    # 读取多边形数量
    if offset + 4 > file_size:
        print(f"错误：偏移 {offset} 处需要 4 字节，但文件已结束")
        return
    polygon_count = struct.unpack('<I', data[offset:offset+4])[0]
    print(f"\n多边形数量: {polygon_count}")
    offset += 4
    
    # 读取所有多边形
    for i in range(polygon_count):
        if offset + 4 > file_size:
            print(f"错误：多边形 {i+1} 顶点数量偏移 {offset} 处需要 4 字节，但文件已结束")
            return
        vertex_count = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        
        coords_count = vertex_count * 2
        bytes_needed = coords_count * 4
        if offset + bytes_needed > file_size:
            print(f"错误：多边形 {i+1} 坐标偏移 {offset} 处需要 {bytes_needed} 字节，但剩余 {file_size - offset} 字节")
            return
        coords = struct.unpack('<{}f'.format(coords_count), 
                              data[offset:offset + bytes_needed])
        offset += bytes_needed
        print(f"多边形 {i+1}: {vertex_count} 个顶点")
    
    # 读取快照数量
    if offset + 4 > file_size:
        print(f"错误：偏移 {offset} 处快照数量需要 4 字节，但文件已结束")
        return
    snapshot_count = struct.unpack('<I', data[offset:offset+4])[0]
    print(f"\n快照数量: {snapshot_count}")
    offset += 4
    
    # 读取所有快照帧
    for frame_idx in range(snapshot_count):
        translation_count = polygon_count * 2
        bytes_needed = translation_count * 4
        
        if offset + bytes_needed > file_size:
            print(f"错误：快照帧 {frame_idx+1} 平移向量偏移 {offset} 处需要 {bytes_needed} 字节，但剩余 {file_size - offset} 字节")
            print(f"  translation_count = {translation_count}")
            print(f"  需要字节数 = {bytes_needed}")
            return
        
        translations = struct.unpack('<{}f'.format(translation_count),
                                   data[offset:offset + bytes_needed])
        offset += bytes_needed
        
        # 读取合法性标记数组
        if offset + polygon_count > file_size:
            print(f"错误：快照帧 {frame_idx+1} 合法性标记偏移 {offset} 处需要 {polygon_count} 字节，但剩余 {file_size - offset} 字节")
            return
        validity_flags = struct.unpack('<{}B'.format(polygon_count),
                                      data[offset:offset + polygon_count])
        offset += polygon_count
        
        if frame_idx < 3:  # 只打印前3帧的详细信息
            print(f"快照帧 {frame_idx+1}: 平移向量数={translation_count}, 合法性标记={validity_flags[:5]}...")
    
    print(f"\n解析成功！")
    print(f"最终偏移: {offset}")
    print(f"文件大小: {file_size}")
    if offset != file_size:
        print(f"警告：还有 {file_size - offset} 字节未解析")

if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else "test_py/Che_linux/dist/data/snapshoot_data.snpst"
    check_snpst_file(filepath)
