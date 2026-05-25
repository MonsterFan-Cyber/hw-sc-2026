"""求解器运行器 - 封装 QProcess 调用 C++ 求解器"""

import os
import logging
from typing import List, Tuple, Optional

from PyQt5.QtCore import QProcess, pyqtSignal, QObject

from gui.core.command_parser import (
    CommandParser,
    IterCommand,
    ResultSuccess,
    ResultFail,
    EndCommand,
    FrameCommand,
    FrameDataCommand,
    PlenCommand,
    MlenCommand,
)

logger = logging.getLogger(__name__)


class SolverRunner(QObject):
    """封装 QProcess 调用 poly.exe，解析 stderr/stdout"""
    
    # 信号定义
    iteration_update = pyqtSignal(int, float, float)   # iter, embed_len, move_dist
    result_success = pyqtSignal(float, float)            # current, previous_best
    result_fail = pyqtSignal()                           # 求解失败
    solver_started = pyqtSignal()                        # 求解开始
    solver_finished = pyqtSignal(list, str)              # 求解完成 (translations列表, output_file_path)
    solver_error = pyqtSignal(str)                       # 求解出错
    log_message = pyqtSignal(str)                        # 日志消息
    frame_received = pyqtSignal(int, list, float, float)  # (frame_index, polygons, plen, mlen) 中间帧
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._process: Optional[QProcess] = None
        self._parser = CommandParser()
        self._stdout_buffer = b''
        self._stderr_buffer = b''
        self._translations: List[Tuple[float, float]] = []
        self._input_file_path: Optional[str] = None
        
        # frame 收集状态
        self._frame_collecting = False      # 是否正在收集 frame 数据
        self._frame_expected = 0            # 期望收集的多边形数量
        self._frame_polys: list = []        # 当前正在收集的多边形列表
        self._frame_index = 0               # 帧序号计数器
        
        # frame 元数据（plen/mlen）
        self._frame_meta_done = False       # 多边形收齐后，等待 plen/mlen 元数据
        self._frame_plen: Optional[float] = None
        self._frame_mlen: Optional[float] = None
        
        # 求解器路径 - 优先当前目录下的 poly.exe / Checker.exe 等
        self._solver_path = self._find_solver()
    
    def _find_solver(self) -> Optional[str]:
        """查找求解器可执行文件"""
        candidates = ['poly.exe', 'Checker.exe', 'Runner.exe']
        # 在项目根目录和当前目录查找
        search_dirs = [
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        ]
        
        for d in search_dirs:
            for name in candidates:
                path = os.path.join(d, name)
                if os.path.isfile(path):
                    logger.info(f"Found solver: {path}")
                    return path
        return None
    
    @property
    def solver_path(self) -> Optional[str]:
        return self._solver_path
    
    def set_solver_path(self, path: str):
        self._solver_path = path
    
    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.state() != QProcess.NotRunning
    
    def start(self, input_text: str, input_file_path: str = None):
        """启动求解器
        
        Args:
            input_text: 输入数据文本，将写入求解器 stdin
            input_file_path: 输入文件路径，用于生成输出文件路径
        """
        if not self._solver_path or not os.path.isfile(self._solver_path):
            self.solver_error.emit(f"求解器未找到: {self._solver_path}")
            return
        
        if self.is_running:
            self.kill()
        
        self._translations = []
        self._stdout_buffer = b''
        self._stderr_buffer = b''
        self._input_file_path = input_file_path
        
        # 重置 frame 收集状态
        self._frame_collecting = False
        self._frame_expected = 0
        self._frame_polys = []
        self._frame_index = 0
        self._frame_meta_done = False
        self._frame_plen = None
        self._frame_mlen = None
        
        self._process = QProcess(self)
        self._process.setProgram(self._solver_path)
        self._process.setProcessChannelMode(QProcess.SeparateChannels)
        
        # 连接信号
        self._process.readyReadStandardOutput.connect(self._on_stdout_ready)
        self._process.readyReadStandardError.connect(self._on_stderr_ready)
        self._process.finished.connect(self._on_process_finished)
        self._process.errorOccurred.connect(self._on_process_error)
        
        # 启动进程
        self._process.start()
        
        # 写入 stdin 然后关闭输入通道
        if self._process.waitForStarted(5000):
            # 写入多边形数据，末尾追加 OK 表示输入结束
            # C++ 端在读完所有多边形后会等待读取 "OK" 字符串
            full_input = input_text.strip() + "\nOK\n"
            self._process.write(full_input.encode('utf-8'))
            self._process.closeWriteChannel()
            self.solver_started.emit()
            logger.info("Solver started, input written to stdin")
        else:
            self.solver_error.emit("求解器启动超时")
    
    def kill(self):
        """终止求解器"""
        if self._process and self._process.state() != QProcess.NotRunning:
            self._process.kill()
            self._process.waitForFinished(3000)
    
    def _on_stdout_ready(self):
        """读取 stdout 数据"""
        data = self._process.readAllStandardOutput()
        self._stdout_buffer += bytes(data)
    
    def _on_stderr_ready(self):
        """读取 stderr 数据，逐行解析命令"""
        data = self._process.readAllStandardError()
        self._stderr_buffer += bytes(data)
        
        # 尝试按行解析
        text = self._stderr_buffer.decode('utf-8', errors='replace')
        lines = text.split('\n')
        
        # 保留最后一个可能不完整的行
        for line in lines[:-1]:
            self._parse_stderr_line(line)
        
        self._stderr_buffer = lines[-1].encode('utf-8', errors='replace')
    
    def _try_collect_meta(self, line: str) -> bool:
        """尝试收集 plen/mlen 元数据行
        
        Returns:
            True 如果该行是 plen 或 mlen（已收集），False 表示不是元数据行
        """
        if not line.strip():
            return True  # 空行在 meta 等待期忽略
        
        m = CommandParser.PATTERN_PLEN.match(line)
        if m:
            self._frame_plen = float(m.group(1))
            return True
        
        m = CommandParser.PATTERN_MLEN.match(line)
        if m:
            self._frame_mlen = float(m.group(1))
            return True
        
        return False
    
    def _emit_frame(self):
        """发射已收集的帧数据（含 plen/mlen），并重置收集状态"""
        self._frame_index += 1
        self.frame_received.emit(
            self._frame_index,
            self._frame_polys.copy(),
            self._frame_plen if self._frame_plen is not None else -1.0,
            self._frame_mlen if self._frame_mlen is not None else -1.0,
        )
        self._frame_meta_done = False
        self._frame_polys = []
        self._frame_expected = 0
        self._frame_plen = None
        self._frame_mlen = None
    
    def _parse_stderr_line(self, line: str):
        """解析一行 stderr"""
        # ----- 状态 1: 等待 plen/mlen 元数据（多边形已收齐） -----
        if self._frame_meta_done:
            if self._try_collect_meta(line):
                return
            # 不是 plen/mlen，发射已收集的帧数据，然后回退该行给普通命令解析
            self._emit_frame()
            # fall through 到普通命令解析
        
        # ----- 状态 2: 正在收集多边形数据行 -----
        if self._frame_collecting:
            data_cmd = CommandParser.parse_frame_data_line(line)
            if data_cmd is not None:
                self._frame_polys.append(data_cmd.vertices)
                # 检查是否收集完毕
                if len(self._frame_polys) >= self._frame_expected:
                    self._frame_collecting = False
                    self._frame_meta_done = True
                    # 继续等待 plen/mlen，不立即发射
                return
            else:
                # 数据行解析失败，退出收集模式并作为日志处理
                self._frame_collecting = False
                self._frame_polys = []
                self._frame_expected = 0
                if line.strip():
                    self.log_message.emit(line.strip())
                return
        
        cmd = self._parser.parse_line(line)
        
        if cmd is None:
            # 无法匹配命令，视为日志
            if line.strip():
                self.log_message.emit(line.strip())
        elif isinstance(cmd, IterCommand):
            self.iteration_update.emit(
                cmd.iteration,
                cmd.embed_length,
                cmd.move_distance
            )
        elif isinstance(cmd, ResultSuccess):
            self.result_success.emit(cmd.current, cmd.previous_best)
        elif isinstance(cmd, ResultFail):
            self.result_fail.emit()
        elif isinstance(cmd, EndCommand):
            pass  # end 由进程结束来处理
        elif isinstance(cmd, FrameCommand):
            # 开始收集 frame 数据
            self._frame_collecting = True
            self._frame_expected = cmd.polygon_count
            self._frame_polys = []
            logger.info(f"Frame collecting started: expecting {cmd.polygon_count} polygons")
    
    def _save_output_file(self, stdout_text: str) -> str:
        """将 stdout 文本保存为 .out 文件
        
        Returns:
            output_file_path: 输出文件路径，如果无法保存则返回空字符串
        """
        if not self._input_file_path:
            logger.warning("No input file path, cannot determine output path")
            return ""
        
        # 从输入文件路径生成输出文件路径
        # 例如: data/practice_1.in -> data/practice_1.out
        base, _ = os.path.splitext(self._input_file_path)
        output_path = base + ".out"
        
        try:
            # 确保数据目录存在
            output_dir = os.path.dirname(output_path)
            if output_dir and not os.path.isdir(output_dir):
                os.makedirs(output_dir, exist_ok=True)
            
            with open(output_path, 'w', encoding='utf-8') as f:
                save_text = stdout_text.replace("\r\n", "\n")
                save_text = save_text.replace("OK", "")
                f.write(save_text.strip())
            
            logger.info(f"Output file saved: {output_path}")
            return output_path
        except Exception as e:
            logger.error(f"Failed to save output file: {e}")
            return ""
    
    def _on_process_finished(self, exit_code, exit_status):
        """进程结束处理"""
        # 处理 stderr 缓冲区中剩余的内容
        if self._stderr_buffer:
            text = self._stderr_buffer.decode('utf-8', errors='replace')
            for line in text.split('\n'):
                self._parse_stderr_line(line)
        
        # 如果还有未发射的帧数据（meta_done 状态），发射之
        if self._frame_meta_done and self._frame_polys:
            self._emit_frame()
        
        # 解析 stdout 中的平移向量
        stdout_text = self._stdout_buffer.decode('utf-8', errors='replace').strip()
        self._translations = self._parse_translations(stdout_text)
        
        # 将 stdout 写入 .out 文件
        output_file_path = self._save_output_file(stdout_text)
        
        if exit_status == QProcess.NormalExit and exit_code == 0:
            logger.info(f"Solver finished, got {len(self._translations)} translations")
            self.solver_finished.emit(self._translations, output_file_path)
        else:
            error_msg = f"求解器异常退出 (code={exit_code}, status={exit_status})"
            logger.error(error_msg)
            self.solver_error.emit(error_msg)
            self.solver_finished.emit([], output_file_path)
    
    def _parse_translations(self, text: str) -> List[Tuple[float, float]]:
        """解析 stdout 中的 N 个平移向量
        
        每行格式: x y
        """
        translations = []
        for line in text.split('\n'):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    translations.append((x, y))
                except ValueError:
                    pass
        return translations
    
    def _on_process_error(self, error):
        """进程错误处理"""
        error_map = {
            QProcess.FailedToStart: "无法启动求解器（程序不存在或权限不足）",
            QProcess.Crashed: "求解器崩溃",
            QProcess.Timedout: "求解器超时",
            QProcess.WriteError: "写入求解器stdin时出错",
            QProcess.ReadError: "读取求解器输出时出错",
            QProcess.UnknownError: "未知错误",
        }
        msg = error_map.get(error, f"求解器错误: {error}")
        logger.error(msg)
        self.solver_error.emit(msg)