"""命令解析器 - 解析 C++ 求解器 stderr 输出的命令"""

import re
import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class IterCommand:
    """it 命令: 第 d 轮迭代，嵌入总长度 f，移动总距离 f"""
    iteration: int
    embed_length: float
    move_distance: float


@dataclass
class ResultSuccess:
    """res succ 命令: 得到解 f，先前最优解为 f"""
    current: float
    previous_best: float


@dataclass
class ResultFail:
    """res fail 命令: 本次未求出可行解"""
    pass


@dataclass
class EndCommand:
    """end 命令: 结束求解"""
    pass


@dataclass
class FrameCommand:
    """frame 命令: 开始接收 N 个多边形的中间帧数据"""
    polygon_count: int


@dataclass
class FrameDataCommand:
    """frame 数据行: 一个多边形的顶点数据"""
    vertices: list  # List[Tuple[float, float]]


@dataclass
class PlenCommand:
    """plen 命令: 总穿透深度"""
    value: float


@dataclass
class MlenCommand:
    """mlen 命令: 总移动距离"""
    value: float


class CommandParser:
    """解析 stderr 流中的命令
     
    命令格式:
        it  %d %f %f      （第 %d 轮迭代，嵌入总长度 %f，移动总距离 %f）
        res fail          （本次未求出可行解）
        res succ %f %f    （得到解 %f，先前最优解为 %f）
        end               （结束求解）
        frame N           （开始接收 N 个多边形的中间帧数据）
        后续 N 行: M x1 y1 x2 y2 ... （每个多边形的顶点数据）
        可选跟随: plen %f （总穿透深度）
        可选跟随: mlen %f （总移动距离）
     
    无法匹配的行作为日志输出。
    """
    
    # 预编译正则表达式
    PATTERN_IT = re.compile(r'^it\s+(\d+)\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s*$')
    PATTERN_RES_FAIL = re.compile(r'^res\s+fail\s*$')
    PATTERN_RES_SUCC = re.compile(r'^res\s+succ\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s*$')
    PATTERN_END = re.compile(r'^end\s*$')
    PATTERN_FRAME = re.compile(r'^frame\s+(\d+)\s*$')
    PATTERN_PLEN = re.compile(r'^plen\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s*$')
    PATTERN_MLEN = re.compile(r'^mlen\s+([\-\d.]+(?:e[\+\-]?\d+)?)\s*$')
    
    def __init__(self):
        pass
    
    def parse_line(self, line: str) -> Optional[object]:
        """解析一行 stderr 输出
        
        Args:
            line: 去掉首尾空白后的一行文本
        
        Returns:
            解析出的命令对象（IterCommand / ResultSuccess / ResultFail / EndCommand），
            若无法匹配则返回 None（应视为日志）
        """
        line = line.strip()
        if not line:
            return None
        
        # 尝试匹配 it 命令
        m = self.PATTERN_IT.match(line)
        if m:
            return IterCommand(
                iteration=int(m.group(1)),
                embed_length=float(m.group(2)),
                move_distance=float(m.group(3))
            )
        
        # 尝试匹配 res fail
        m = self.PATTERN_RES_FAIL.match(line)
        if m:
            return ResultFail()
        
        # 尝试匹配 res succ
        m = self.PATTERN_RES_SUCC.match(line)
        if m:
            return ResultSuccess(
                current=float(m.group(1)),
                previous_best=float(m.group(2))
            )
        
        # 尝试匹配 end
        m = self.PATTERN_END.match(line)
        if m:
            return EndCommand()
        
        # 尝试匹配 frame 命令
        m = self.PATTERN_FRAME.match(line)
        if m:
            return FrameCommand(polygon_count=int(m.group(1)))
        
        # 无法匹配，视为日志
        return None
    
    @staticmethod
    def parse_frame_data_line(line: str) -> Optional[FrameDataCommand]:
        """解析 frame 数据行: M x1 y1 x2 y2 ...（一个多边形的顶点数据）
        
        Args:
            line: 去掉首尾空白后的一行文本
        
        Returns:
            若成功解析则返回 FrameDataCommand，否则返回 None
        """
        line = line.strip()
        if not line:
            return None
        tokens = line.split()
        if len(tokens) < 3:
            return None
        try:
            m = int(tokens[0])
            if len(tokens) < 1 + 2 * m:
                return None
            vertices = []
            for i in range(m):
                x = float(tokens[1 + 2 * i])
                y = float(tokens[1 + 2 * i + 1])
                vertices.append((x, y))
            return FrameDataCommand(vertices=vertices)
        except (ValueError, IndexError):
            return None
