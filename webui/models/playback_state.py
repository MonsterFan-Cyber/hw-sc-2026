from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class PlaybackSpeed(Enum):
    SLOW_0_5X = 0.5
    NORMAL_1X = 1.0
    FAST_2X = 2.0
    FAST_4X = 4.0
    
    @classmethod
    def from_value(cls, value: float) -> 'PlaybackSpeed':
        for speed in cls:
            if speed.value == value:
                return speed
        return cls.NORMAL_1X


@dataclass
class PlaybackState:
    total_frames: int = 0
    current_frame: int = 0
    is_playing: bool = False
    speed: PlaybackSpeed = PlaybackSpeed.NORMAL_1X
    loop_enabled: bool = False
    start_frame: int = 0
    
    def set_total_frames(self, total: int):
        self.total_frames = total
        if total > 0 and self.current_frame >= total:
            self.current_frame = 0
    
    def advance_frame(self) -> bool:
        if self.total_frames == 0:
            return False
        
        next_frame = self.current_frame + 1
        
        if next_frame >= self.total_frames:
            if self.loop_enabled:
                self.current_frame = 0
                return True
            else:
                return False
        
        self.current_frame = next_frame
        return True
    
    def retreat_frame(self) -> bool:
        if self.total_frames == 0:
            return False
        
        prev_frame = self.current_frame - 1
        
        if prev_frame < 0:
            if self.loop_enabled:
                self.current_frame = self.total_frames - 1
                return True
            else:
                return False
        
        self.current_frame = prev_frame
        return True
    
    def jump_to_frame(self, frame_id: int) -> bool:
        if frame_id < 0 or frame_id >= self.total_frames:
            return False
        
        self.current_frame = frame_id
        return True
    
    def get_frame_interval_ms(self) -> int:
        base_interval_ms = 100
        return int(base_interval_ms / self.speed.value)
    
    def set_speed(self, speed: PlaybackSpeed):
        self.speed = speed
    
    def toggle_loop(self):
        self.loop_enabled = not self.loop_enabled
    
    def set_start_frame(self, frame_id: int) -> bool:
        if frame_id < 0 or frame_id >= self.total_frames:
            return False
        
        self.start_frame = frame_id
        self.current_frame = frame_id
        self.is_playing = False
        return True
    
    def play(self):
        self.is_playing = True
    
    def pause(self):
        self.is_playing = False
