from datetime import datetime
from typing import List, Optional
from enum import Enum
import sys


class LogLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Logger:
    MAX_LOG_ENTRIES = 1000
    
    def __init__(self, name: str = "Logger"):
        self.name = name
        self.entries: List[str] = []
    
    def _format_message(self, level: LogLevel, message: str) -> str:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] [{level.value}] {message}"
        return formatted
    
    def _add_entry(self, formatted_message: str):
        self.entries.append(formatted_message)
        if len(self.entries) > self.MAX_LOG_ENTRIES:
            self.entries.pop(0)
    
    def info(self, message: str):
        formatted = self._format_message(LogLevel.INFO, message)
        self._add_entry(formatted)
        print(formatted, file=sys.stdout)
    
    def warning(self, message: str):
        formatted = self._format_message(LogLevel.WARNING, message)
        self._add_entry(formatted)
        print(formatted, file=sys.stdout)
    
    def error(self, message: str):
        formatted = self._format_message(LogLevel.ERROR, message)
        self._add_entry(formatted)
        print(formatted, file=sys.stderr)
    
    def get_entries(self) -> List[str]:
        return self.entries.copy()
    
    def clear(self):
        self.entries.clear()
    
    def get_recent_entries(self, count: int = 50) -> List[str]:
        return self.entries[-count:] if count < len(self.entries) else self.entries.copy()
