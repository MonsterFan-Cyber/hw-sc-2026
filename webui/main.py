#!/usr/bin/env python3
"""
Web 快照播放器主入口

使用方法:
    python main.py

然后在浏览器访问 http://localhost:8000
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app import main

if __name__ == '__main__':
    main()
