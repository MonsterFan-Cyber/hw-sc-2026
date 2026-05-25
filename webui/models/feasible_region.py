from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class FeasibleRegion:
    vertices: Tuple[Tuple[float, float], ...]
    
    def get_vertex_count(self) -> int:
        return len(self.vertices)
    
    def get_vertices_list(self) -> list:
        return list(self.vertices)
    
    def is_empty(self) -> bool:
        return len(self.vertices) == 0
