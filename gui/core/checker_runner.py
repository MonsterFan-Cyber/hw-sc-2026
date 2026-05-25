"""验证运行器 - 封装 QProcess 调用 Checker.exe 评测输出文件"""

import os
import logging
import re
from typing import Optional

from PyQt5.QtCore import QProcess, pyqtSignal, QObject

logger = logging.getLogger(__name__)


class CheckerRunner(QObject):
    """封装 QProcess 调用 Checker.exe，解析验证结果"""

    # 信号定义
    check_finished = pyqtSignal(bool, str)        # 验证完成 (success, message)
    check_started = pyqtSignal()                   # 验证开始
    check_error = pyqtSignal(str)                  # 验证出错

    # 匹配 score 输出的正则：score: <数值>
    PATTERN_SCORE = re.compile(r'score:\s*([\d.eE+\-]+)', re.IGNORECASE)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._process: Optional[QProcess] = None
        self._checker_path: Optional[str] = self._find_checker()

    def _find_checker(self) -> Optional[str]:
        """查找 Checker.exe 可执行文件"""
        candidates = ['Checker.exe']
        # 在项目根目录和上级目录查找
        search_dirs = [
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        ]

        for d in search_dirs:
            for name in candidates:
                path = os.path.join(d, name)
                if os.path.isfile(path):
                    logger.info(f"Found checker: {path}")
                    return path
        return None

    @property
    def checker_path(self) -> Optional[str]:
        return self._checker_path

    def set_checker_path(self, path: str):
        self._checker_path = path

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.state() != QProcess.NotRunning

    def check(self, input_file: str, output_file: str):
        """启动验证

        调用 Checker.exe <input_file> <output_file>

        Args:
            input_file: 输入文件路径 (.in)
            output_file: 输出文件路径 (.out)
        """
        if not self._checker_path or not os.path.isfile(self._checker_path):
            self.check_error.emit(f"Checker未找到: {self._checker_path}")
            return

        if self.is_running:
            self.kill()

        if not os.path.isfile(input_file):
            self.check_error.emit(f"输入文件不存在: {input_file}")
            return

        if not os.path.isfile(output_file):
            self.check_error.emit(f"输出文件不存在: {output_file}")
            return

        self._stdout_buffer = b''
        self._stderr_buffer = b''

        self._process = QProcess(self)
        self._process.setProgram(self._checker_path)
        self._process.setArguments([input_file, output_file, output_file])
        self._process.setProcessChannelMode(QProcess.SeparateChannels)

        # 连接信号
        self._process.readyReadStandardOutput.connect(self._on_stdout_ready)
        self._process.readyReadStandardError.connect(self._on_stderr_ready)
        self._process.finished.connect(self._on_process_finished)
        self._process.errorOccurred.connect(self._on_process_error)

        self._process.start()
        self.check_started.emit()
        logger.info(f"Checker started: {self._checker_path} {input_file} {output_file}")

    def kill(self):
        """终止验证器"""
        if self._process and self._process.state() != QProcess.NotRunning:
            self._process.kill()
            self._process.waitForFinished(3000)

    def _on_stdout_ready(self):
        """读取 stdout 数据"""
        data = self._process.readAllStandardOutput()
        self._stdout_buffer += bytes(data)

    def _on_stderr_ready(self):
        """读取 stderr 数据"""
        data = self._process.readAllStandardError()
        self._stderr_buffer += bytes(data)

    def _on_process_finished(self, exit_code, exit_status):
        """进程结束处理"""
        stdout_text = self._stdout_buffer.decode('utf-8', errors='replace').strip()
        stderr_text = self._stderr_buffer.decode('utf-8', errors='replace').strip()

        logger.info(f"Checker finished: exit_code={exit_code}, exit_status={exit_status}")
        logger.info(f"Checker stdout: {stdout_text}")
        if stderr_text:
            logger.info(f"Checker stderr: {stderr_text}")

        if exit_status == QProcess.NormalExit:
            # 解析 score
            score_value = self._parse_score(stdout_text)
            full_message = stdout_text

            if stderr_text:
                full_message += ("\n" if full_message else "") + stderr_text

            self.check_finished.emit(True, full_message)
        else:
            error_msg = f"Checker异常退出 (code={exit_code})"
            if stderr_text:
                error_msg += f"\n{stderr_text}"
            if stdout_text:
                error_msg += f"\n{stdout_text}"
            self.check_error.emit(error_msg)

    def _parse_score(self, text: str) -> Optional[float]:
        """从输出文本中解析 score 数值"""
        m = self.PATTERN_SCORE.search(text)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                pass
        return None

    def _on_process_error(self, error):
        """进程错误处理"""
        error_map = {
            QProcess.FailedToStart: "无法启动Checker（程序不存在或权限不足）",
            QProcess.Crashed: "Checker崩溃",
            QProcess.Timedout: "Checker超时",
            QProcess.WriteError: "写入Checker时出错",
            QProcess.ReadError: "读取Checker输出时出错",
            QProcess.UnknownError: "未知错误",
        }
        msg = error_map.get(error, f"Checker错误: {error}")
        logger.error(msg)
        self.check_error.emit(msg)