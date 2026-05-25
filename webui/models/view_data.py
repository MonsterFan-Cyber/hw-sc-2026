from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class ViewData:
    zoom: float = 1.0
    offset_x: float = 0.0
    offset_y: float = 0.0
    canvas_width: int = 800
    canvas_height: int = 600
    
    MIN_ZOOM = 0.1
    MAX_ZOOM = 10.0
    
    def clamp_zoom(self, zoom: float) -> float:
        return max(self.MIN_ZOOM, min(self.MAX_ZOOM, zoom))
    
    def set_zoom(self, zoom: float):
        self.zoom = self.clamp_zoom(zoom)
    
    def world_to_screen(self, world_x: float, world_y: float) -> Tuple[float, float]:
        screen_x = (world_x - self.offset_x) * self.zoom + self.canvas_width / 2
        screen_y = (world_y - self.offset_y) * self.zoom + self.canvas_height / 2
        return (screen_x, screen_y)
    
    def screen_to_world(self, screen_x: float, screen_y: float) -> Tuple[float, float]:
        world_x = (screen_x - self.canvas_width / 2) / self.zoom + self.offset_x
        world_y = (screen_y - self.canvas_height / 2) / self.zoom + self.offset_y
        return (world_x, world_y)
    
    def zoom_at_point(self, zoom_factor: float, screen_x: float, screen_y: float):
        world_x, world_y = self.screen_to_world(screen_x, screen_y)
        
        new_zoom = self.clamp_zoom(self.zoom * zoom_factor)
        
        if new_zoom == self.zoom:
            return
        
        self.zoom = new_zoom
        
        new_screen_x, new_screen_y = self.world_to_screen(world_x, world_y)
        
        self.offset_x += (new_screen_x - screen_x) / self.zoom
        self.offset_y += (new_screen_y - screen_y) / self.zoom
    
    def pan(self, dx: float, dy: float):
        self.offset_x += dx / self.zoom
        self.offset_y += dy / self.zoom
    
    def reset(self):
        self.zoom = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
    
    def fit_bounding_box(self, min_x: float, max_x: float, min_y: float, max_y: float, padding: float = 0.1):
        width = max_x - min_x
        height = max_y - min_y
        
        if width == 0 or height == 0:
            return
        
        zoom_x = (self.canvas_width * (1 - padding)) / width
        zoom_y = (self.canvas_height * (1 - padding)) / height
        
        self.zoom = self.clamp_zoom(min(zoom_x, zoom_y))
        
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        self.offset_x = center_x
        self.offset_y = center_y
