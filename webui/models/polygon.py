from dataclasses import dataclass
from typing import Tuple, List


@dataclass(frozen=True)
class Polygon:
    polygon_id: int
    vertices: Tuple[Tuple[float, float], ...]
    _bbox_center: Tuple[float, float] = None
    
    def __post_init__(self):
        if self._bbox_center is None:
            center = self._compute_bbox_center()
            object.__setattr__(self, '_bbox_center', center)
    
    def _compute_bbox_center(self) -> Tuple[float, float]:
        if not self.vertices:
            return (0.0, 0.0)
        
        x_coords = [v[0] for v in self.vertices]
        y_coords = [v[1] for v in self.vertices]
        
        min_x = min(x_coords)
        max_x = max(x_coords)
        min_y = min(y_coords)
        max_y = max(y_coords)
        
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        
        return (center_x, center_y)
    
    def get_bbox_center(self) -> Tuple[float, float]:
        return self._bbox_center
    
    def get_translated_vertices(self, dx: float, dy: float) -> List[Tuple[float, float]]:
        return [(v[0] + dx, v[1] + dy) for v in self.vertices]
    
    def get_vertex_count(self) -> int:
        return len(self.vertices)
