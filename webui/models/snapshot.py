from dataclasses import dataclass
from typing import Tuple, List


@dataclass(frozen=True)
class Snapshot:
    frame_id: int
    translations: Tuple[Tuple[float, float], ...]
    legality_flags: Tuple[int, ...]
    
    def get_polygon_state(self, polygon_idx: int) -> Tuple[Tuple[float, float], int]:
        if polygon_idx < 0 or polygon_idx >= len(self.translations):
            raise IndexError(f"Polygon index {polygon_idx} out of range")
        
        translation = self.translations[polygon_idx]
        legality = self.legality_flags[polygon_idx]
        
        return (translation, legality)
    
    def get_illegal_polygon_ids(self) -> List[int]:
        illegal_ids = []
        for idx, flag in enumerate(self.legality_flags):
            if flag == 0:
                illegal_ids.append(idx + 1)
        return illegal_ids
    
    def get_legal_count(self) -> int:
        return sum(1 for flag in self.legality_flags if flag == 1)
    
    def get_illegal_count(self) -> int:
        return sum(1 for flag in self.legality_flags if flag == 0)
    
    def get_total_count(self) -> int:
        return len(self.legality_flags)
    
    def is_polygon_legal(self, polygon_idx: int) -> bool:
        if polygon_idx < 0 or polygon_idx >= len(self.legality_flags):
            return False
        return self.legality_flags[polygon_idx] == 1
