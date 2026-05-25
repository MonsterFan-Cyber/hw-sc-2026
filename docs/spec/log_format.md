# SA Logger 输出格式说明

## 一、日志文件结构

```
dist/logs/
└── sa_main.log    # 主日志文件，包含所有日志信息
```

日志采用**单文件分段结构**，通过标签区分不同类型的日志内容。

---

## 二、日志段落类型

### 2.1 阶段进入标记

```
[PHASE_ENTER] <phase_id> <phase_name> time=<timestamp>
```

| 字段 | 说明 |
|-----|------|
| phase_id | 阶段编号 (0-5) |
| phase_name | 阶段名称 |
| timestamp | 进入时间 (秒) |

**阶段编号对照表：**

| phase_id | 阶段名称 | 对应函数 |
|---------|---------|---------|
| 0 | saOnPerm | 排列 SA |
| 1 | positionSAHigh | 高温位置 SA (最关键) |
| 2 | positionSALow | 低温位置 SA |
| 3 | exactPullback_mid | 中间精确拉回 |
| 4 | exactPullback_final | 最终精确拉回 |
| 5 | aabbPullback | AABB 拉回 |

**示例：**
```
[PHASE_ENTER] 1 positionSAHigh time=0.235
```

---

### 2.2 阶段退出标记

```
[PHASE_EXIT] <phase_id> iters=<iter_count> duration=<seconds>
```

| 字段 | 说明 |
|-----|------|
| phase_id | 阶段编号 |
| iter_count | 该阶段迭代次数 |
| seconds | 阶段耗时 (秒) |

**示例：**
```
[PHASE_EXIT] 1 iters=5000000 duration=12.567
```

---

### 2.3 算子统计

```
[OP_STATS] phase=<phase_id>
  <op_name> total=<total_calls> accepted=<accepted_calls> rate=<accept_rate>
  ...
[OP_STATS_END]
```

| 字段 | 说明 |
|-----|------|
| op_name | 算子名称 (SWAP/MOVE/JUMP/RANDOM_JUMP) |
| total_calls | 总调用次数 |
| accepted_calls | 接受次数 |
| accept_rate | 接受率 (0.0-1.0) |

**示例：**
```
[OP_STATS] phase=1
  SWAP total=1000000 accepted=320000 rate=0.3200
  MOVE total=4500000 accepted=2520000 rate=0.5600
  JUMP total=1500000 accepted=675000 rate=0.4500
  RANDOM_JUMP total=500000 accepted=150000 rate=0.3000
[OP_STATS_END]
```

---

### 2.4 L 值曲线采样

```
[L_CURVE]
  <timestamp> <phase_id> <global_iter> <L> <bestL>
  ...
[L_CURVE_END]
```

| 字段 | 说明 |
|-----|------|
| timestamp | 采样时刻 (秒) |
| phase_id | 当前阶段编号 |
| global_iter | 全局迭代计数 |
| L | 当前 L 值 |
| bestL | 历史最优 L 值 |

**采样间隔：** 默认每 1000 次迭代采样一次。

**示例：**
```
[L_CURVE]
  0.250 1 1000 152.345678 152.345678
  0.255 1 2000 151.234567 151.234567
  0.260 1 3000 150.987654 150.987654
[L_CURVE_END]
```

---

### 2.5 L 下降事件

```
[L_DROP] phase=<phase_id> global_iter=<iter> phase_iter=<p_iter> old=<oldL> new=<newL> delta=<delta> time=<timestamp>
```

| 字段 | 说明 |
|-----|------|
| phase_id | 下降发生的阶段 |
| global_iter | 全局迭代次数 |
| phase_iter | 阶段内迭代次数 |
| oldL | 下降前 L 值 |
| newL | 下降后 L 值 |
| delta | 下降量 (= oldL - newL) |
| timestamp | 事件时刻 (秒) |

**特点：** L 下降事件**实时写入**，不缓冲。

**示例：**
```
[L_DROP] phase=1 global_iter=15234 phase_iter=15234 old=151.234567 new=150.123456 delta=1.111111 time=0.345
```

---

### 2.6 positionSAHigh 周期详情

```
[CYCLE_DETAILS]
  cycle=<cycle_id> T_init=<T_init> T_end=<T_end> C=<C> feasible=<start>-><end> L=<L_start>-><L_end> penalty=<pen_start>-><pen_end> iters=<iter_count>
  ...
[CYCLE_DETAILS_END]
```

| 字段 | 说明 |
|-----|------|
| cycle_id | 周期编号 |
| T_init | 周期初始温度 |
| T_end | 周期结束温度 |
| C | 惩罚系数 |
| feasible | 可行性状态变化 (0/1 -> 0/1) |
| L_start/L_end | L 值变化 |
| pen_start/pen_end | 惩罚值变化 |
| iter_count | 周期内迭代次数 |

**示例：**
```
[CYCLE_DETAILS]
  cycle=0 T_init=4.5678e+00 T_end=1.5226e-01 C=3.5000 feasible=0->0 L=152.3456->148.2345 penalty=0.5678->0.2345 iters=500000
  cycle=1 T_init=4.4567e+00 T_end=1.4891e-01 C=3.5000 feasible=0->1 L=148.2345->145.1234 penalty=0.1234->0.0000 iters=500000
[CYCLE_DETAILS_END]
```

---

## 三、完整日志示例

```
# SA Logger Output
# Start Time: 0.000123
# ====================

[PHASE_ENTER] 0 saOnPerm time=0.150
[PHASE_EXIT] 0 iters=50000 duration=0.085

[OP_STATS] phase=0
  SWAP total=16667 accepted=8333 rate=0.5000
[OP_STATS_END]

[PHASE_ENTER] 1 positionSAHigh time=0.250
[L_CURVE]
  0.255 1 1000 152.345678 152.345678
  0.260 1 2000 151.234567 151.234567
[L_CURVE_END]
[L_DROP] phase=1 global_iter=1523 phase_iter=1523 old=152.345678 new=151.234567 delta=1.111111 time=0.278
[OP_STATS] phase=1
  SWAP total=1000000 accepted=320000 rate=0.3200
  MOVE total=4500000 accepted=2520000 rate=0.5600
  JUMP total=1500000 accepted=675000 rate=0.4500
  RANDOM_JUMP total=500000 accepted=150000 rate=0.3000
[OP_STATS_END]
[CYCLE_DETAILS]
  cycle=0 T_init=4.5678e+00 T_end=1.5226e-01 C=3.5000 feasible=0->1 L=152.3456->145.1234 penalty=0.5678->0.0000 iters=500000
[CYCLE_DETAILS_END]
[PHASE_EXIT] 1 iters=5000000 duration=12.567

[PHASE_ENTER] 3 exactPullback_mid time=12.850
[PHASE_EXIT] 3 iters=2000 duration=0.120

[PHASE_ENTER] 2 positionSALow time=13.000
[PHASE_EXIT] 2 iters=3000000 duration=8.234

[PHASE_ENTER] 4 exactPullback_final time=21.300
[PHASE_EXIT] 4 iters=3000 duration=0.180
```

---

## 四、Python 解析示例

```python
import re

def parse_sa_log(filename):
    """解析 SA 日志文件"""
    result = {
        'phases': [],
        'l_curve': [],
        'l_drops': [],
        'op_stats': {},
        'cycles': []
    }
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # 解析阶段进入
        if line.startswith('[PHASE_ENTER]'):
            m = re.match(r'\[PHASE_ENTER\] (\d+) (\S+) time=(\S+)', line)
            result['phases'].append({
                'id': int(m.group(1)),
                'name': m.group(2),
                'enter_time': float(m.group(3))
            })
        
        # 解析阶段退出
        elif line.startswith('[PHASE_EXIT]'):
            m = re.match(r'\[PHASE_EXIT\] (\d+) iters=(\d+) duration=(\S+)', line)
            phase = next(p for p in result['phases'] if p['id'] == int(m.group(1)))
            phase['iters'] = int(m.group(2))
            phase['duration'] = float(m.group(3))
        
        # 解析 L 下降
        elif line.startswith('[L_DROP]'):
            m = re.match(r'\[L_DROP\] phase=(\d+) global_iter=(\d+) phase_iter=(\d+) old=(\S+) new=(\S+) delta=(\S+) time=(\S+)', line)
            result['l_drops'].append({
                'phase': int(m.group(1)),
                'global_iter': int(m.group(2)),
                'phase_iter': int(m.group(3)),
                'old': float(m.group(4)),
                'new': float(m.group(5)),
                'delta': float(m.group(6)),
                'time': float(m.group(7))
            })
        
        # 解析 L 曲线
        elif line == '[L_CURVE]':
            i += 1
            while i < len(lines) and lines[i].strip() != '[L_CURVE_END]':
                parts = lines[i].strip().split()
                result['l_curve'].append({
                    'time': float(parts[0]),
                    'phase': int(parts[1]),
                    'iter': int(parts[2]),
                    'L': float(parts[3]),
                    'bestL': float(parts[4])
                })
                i += 1
        
        i += 1
    
    return result

# 使用示例
data = parse_sa_log('dist/logs/sa_main.log')
print(f"总 L 下降次数: {len(data['l_drops'])}")
print(f"positionSAHigh 迭代次数: {next(p['iters'] for p in data['phases'] if p['id'] == 1)}")
```

---

## 五、日志分析建议

### 5.1 算子接受率分析

- **SWAP 接受率过低**：排列可能已陷入局部最优，考虑增加扰动
- **MOVE 接受率过高**：温度可能过高，收敛过慢
- **JUMP 接受率**：跳跃算子的探索效率指标
- **RANDOM_JUMP 接受率**：非法状态逃离能力的指标

### 5.2 L 下降曲线分析

- 绘制 `l_curve` 中的 `L` 和 `bestL` 曲线
- 统计各阶段的 L 下降次数和下降量
- 分析 L 下降的时间分布，判断收敛阶段

### 5.3 positionSAHigh 专项分析

- 追踪 `feasible` 状态转变时刻
- 分析 `C` 参数的动态调整过程
- 观察 `penalty` 值的下降趋势
- 对比不同 `cycle` 的 L 下降效果

---

## 六、配置参数

可在 `LoggerConfig` 中调整：

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| enabled | true | 是否启用日志 |
| lCurveSampleInterval | 1000 | L 曲线采样间隔 |
| statsFlushInterval | 10000 | 统计刷新间隔 |
| cycleDetailInterval | 1 | 周期详情记录间隔 |
| enableCycleDetail | true | 是否记录周期详情 |
