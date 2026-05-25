import struct
from typing import BinaryIO, Tuple, List, Any
import os

from web.models.polygon import Polygon
from web.models.snapshot import Snapshot
from web.models.feasible_region import FeasibleRegion
from web.utils.logger import Logger


class ParseError(Exception):
    pass


class BinaryParser:
    def __init__(self, logger: Logger = None):
        self.logger = logger or Logger("BinaryParser")
        self.polygon_count = 0
    
    def read_u32(self, file: BinaryIO) -> int:
        data = file.read(4)
        if len(data) != 4:
            raise ParseError(f"Failed to read u32: expected 4 bytes, got {len(data)}")
        return struct.unpack('<I', data)[0]
    
    def read_f32(self, file: BinaryIO) -> float:
        data = file.read(4)
        if len(data) != 4:
            raise ParseError(f"Failed to read f32: expected 4 bytes, got {len(data)}")
        return struct.unpack('<f', data)[0]
    
    def read_u8(self, file: BinaryIO) -> int:
        data = file.read(1)
        if len(data) != 1:
            raise ParseError(f"Failed to read u8: expected 1 byte, got {len(data)}")
        return struct.unpack('B', data)[0]
    
    def read_f32_array(self, file: BinaryIO, count: int) -> List[float]:
        data = file.read(4 * count)
        if len(data) != 4 * count:
            raise ParseError(f"Failed to read f32 array: expected {4 * count} bytes, got {len(data)}")
        return list(struct.unpack(f'<{count}f', data))
    
    def read_coordinate_sequence(self, file: BinaryIO, vertex_count: int) -> Tuple[Tuple[float, float], ...]:
        coords = self.read_f32_array(file, vertex_count * 2)
        vertices = []
        for i in range(vertex_count):
            x = coords[i * 2]
            y = coords[i * 2 + 1]
            vertices.append((x, y))
        return tuple(vertices)
    
    def parse_feasible_region(self, file: BinaryIO) -> FeasibleRegion:
        vertex_count = self.read_u32(file)
        self.logger.info(f"Feasible region vertex count: {vertex_count}")
        
        if vertex_count < 3:
            raise ParseError(f"Feasible region must have at least 3 vertices, got {vertex_count}")
        
        vertices = self.read_coordinate_sequence(file, vertex_count)
        self.logger.info(f"Feasible region vertices parsed: {len(vertices)}")
        
        return FeasibleRegion(vertices=vertices)
    
    def parse_polygon(self, file: BinaryIO, polygon_id: int) -> Polygon:
        vertex_count = self.read_u32(file)
        self.logger.info(f"Polygon {polygon_id} vertex count: {vertex_count}")
        
        if vertex_count < 3:
            raise ParseError(f"Polygon {polygon_id} must have at least 3 vertices, got {vertex_count}")
        
        vertices = self.read_coordinate_sequence(file, vertex_count)
        
        polygon = Polygon(polygon_id=polygon_id, vertices=vertices)
        self.logger.info(f"Polygon {polygon_id} parsed: {polygon.get_vertex_count()} vertices, center: {polygon.get_bbox_center()}")
        
        return polygon
    
    def parse_snapshot(self, file: BinaryIO, frame_id: int, polygon_count: int) -> Snapshot:
        translations = []
        for _ in range(polygon_count):
            dx = self.read_f32(file)
            dy = self.read_f32(file)
            translations.append((dx, dy))
        
        legality_flags = []
        for _ in range(polygon_count):
            flag = self.read_u8(file)
            if flag not in (0, 1):
                raise ParseError(f"Invalid legality flag {flag} in frame {frame_id}")
            legality_flags.append(flag)
        
        snapshot = Snapshot(
            frame_id=frame_id,
            translations=tuple(translations),
            legality_flags=tuple(legality_flags)
        )
        
        legal_count = snapshot.get_legal_count()
        illegal_count = snapshot.get_illegal_count()
        self.logger.info(f"Frame {frame_id} parsed: {legal_count} legal, {illegal_count} illegal")
        
        return snapshot
    
    def parse_file(self, file_path: str) -> dict:
        self.logger.info(f"Parsing file: {file_path}")
        
        if not os.path.exists(file_path):
            raise ParseError(f"File not found: {file_path}")
        
        if not file_path.endswith('.snpst'):
            raise ParseError(f"Invalid file extension: {file_path}")
        
        file_size = os.path.getsize(file_path)
        self.logger.info(f"File size: {file_size} bytes")
        
        with open(file_path, 'rb') as file:
            feasible_region = self.parse_feasible_region(file)
            
            polygon_count = self.read_u32(file)
            self.polygon_count = polygon_count
            self.logger.info(f"Polygon count: {polygon_count}")
            
            if polygon_count < 1:
                raise ParseError(f"Must have at least 1 polygon, got {polygon_count}")
            
            polygons = []
            for i in range(polygon_count):
                polygon = self.parse_polygon(file, i + 1)
                polygons.append(polygon)
            
            snapshot_count = self.read_u32(file)
            self.logger.info(f"Snapshot count: {snapshot_count}")
            
            if snapshot_count < 1:
                raise ParseError(f"Must have at least 1 snapshot, got {snapshot_count}")
            
            snapshots = []
            for i in range(snapshot_count):
                snapshot = self.parse_snapshot(file, i, polygon_count)
                snapshots.append(snapshot)
            
            remaining = file.read()
            if len(remaining) > 0:
                self.logger.warning(f"File has {len(remaining)} extra bytes after parsing")
        
        result = {
            'feasible_region': feasible_region,
            'polygons': polygons,
            'snapshots': snapshots,
            'metadata': {
                'polygon_count': polygon_count,
                'snapshot_count': snapshot_count,
                'feasible_region_vertices': feasible_region.get_vertex_count()
            }
        }
        
        self.logger.info(f"File parsing completed successfully")
        return result
