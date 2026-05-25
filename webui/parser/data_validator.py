import os
from typing import List

from web.utils.logger import Logger


class ValidationError(Exception):
    pass


class DataValidator:
    def __init__(self, logger: Logger = None):
        self.logger = logger or Logger("DataValidator")
    
    def validate_file_extension(self, file_path: str) -> bool:
        if not file_path.endswith('.snpst'):
            error_msg = f"Invalid file extension: {file_path}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
    
    def validate_polygon_vertex_count(self, vertex_count: int, polygon_id: int = None) -> bool:
        if vertex_count < 3:
            error_msg = f"Polygon {polygon_id if polygon_id else ''} must have at least 3 vertices, got {vertex_count}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
    
    def validate_snapshot_count(self, snapshot_count: int) -> bool:
        if snapshot_count < 1:
            error_msg = f"Must have at least 1 snapshot, got {snapshot_count}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
    
    def validate_legality_flag(self, flag: int, frame_id: int = None, polygon_id: int = None) -> bool:
        if flag not in (0, 1):
            error_msg = f"Invalid legality flag {flag} at frame {frame_id}, polygon {polygon_id}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
    
    def check_file_integrity(self, file_path: str) -> bool:
        if not os.path.exists(file_path):
            error_msg = f"File not found: {file_path}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        
        if not os.path.isfile(file_path):
            error_msg = f"Not a file: {file_path}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        
        file_size = os.path.getsize(file_path)
        if file_size == 0:
            error_msg = f"File is empty: {file_path}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        
        min_file_size = 12
        if file_size < min_file_size:
            error_msg = f"File too small ({file_size} bytes), expected at least {min_file_size} bytes"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        
        self.logger.info(f"File integrity check passed: {file_path} ({file_size} bytes)")
        return True
    
    def validate_polygon_count(self, polygon_count: int) -> bool:
        if polygon_count < 1:
            error_msg = f"Must have at least 1 polygon, got {polygon_count}"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
    
    def validate_frame_id(self, frame_id: int, total_frames: int) -> bool:
        if frame_id < 0 or frame_id >= total_frames:
            error_msg = f"Frame ID {frame_id} out of range [0, {total_frames})"
            self.logger.error(error_msg)
            raise ValidationError(error_msg)
        return True
