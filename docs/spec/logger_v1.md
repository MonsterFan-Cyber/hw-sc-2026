# SA Logger 完善设计方案 v1

## 一、设计目标

为 `src/main.cpp` 中的模拟退火算法提供专用日志系统，用于：
1. 分析各算子的接受率和贡献度
2. 追踪最优解 L 的下降曲线及环节归属
3. 深入分析关键环节 `positionSAHigh` 的迭代行为
4. 辅助排查算法性能瓶颈和边界条件遗漏

## 二、核心设计原则

1. **最小侵入**：面向对象设计，在原始代码中仅需添加极简调用
2. **低性能开销**：通过采样间隔和缓冲机制控制开销
3. **结构化输出**：便于后续 Python 脚本分析和可视化
4. **阶段隔离**：不同 SA 阶段统计独立，便于对比分析

## 三、代码结构设计

### 3.1 头文件定义 (`src/logger.h`) - Header-Only

```cpp
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <iomanip>
#include "lg.h"

using namespace std;

// 算子类型枚举
enum class OpType { 
    SWAP = 0,        // saSwapOp
    MOVE = 1,        // saMoveOp  
    JUMP = 2,        // saJumpOp
    RANDOM_JUMP = 3, // randomJumpOp
    COUNT = 4        // 算子总数
};

// SA 阶段枚举
enum class Phase { 
    PERM_SA = 0,         // saOnPerm
    POS_SA_HIGH = 1,     // positionSAHigh (最关键)
    POS_SA_LOW = 2,      // positionSALow
    PULLBACK_MID = 3,    // exactPullback_mid
    PULLBACK_FINAL = 4,  // exactPullback_final
    AABB_PULL = 5,       // aabbPullback
    UNKNOWN = -1 
};

// 算子统计结构
struct OpStats {
    long long totalCalls = 0;
    long long acceptedCalls = 0;
    
    double acceptRate() const { 
        return totalCalls > 0 ? (double)acceptedCalls / totalCalls : 0.0; 
    }
    
    void reset() { totalCalls = 0; acceptedCalls = 0; }
};

// L 下降记录
struct LDropRecord {
    int phaseId;
    long long globalIter;
    long long phaseIter;
    double oldL;
    double newL;
    double delta;
    double timestamp;
};

// L 曲线采样点
struct LSample {
    double timestamp;
    int phaseId;
    long long globalIter;
    double L;
    double bestL;
};

// positionSAHigh 周期详情
struct CycleDetail {
    int cycleId;
    double T_init;
    double T_end;
    double C;
    bool feasibleStart;
    bool feasibleEnd;
    double L_start;
    double L_end;
    double penalty_start;
    double penalty_end;
    long long iterInCycle;
    map<OpType, OpStats> opStats;
};

// Logger 配置
struct LoggerConfig {
    bool enabled = true;
    int lCurveSampleInterval = 1000;
    int statsFlushInterval = 10000;
    int cycleDetailInterval = 1;
    bool enableCycleDetail = true;
};

class SALogger {
private:
    ofstream logFile;
    LoggerConfig config;
    bool initialized = false;
    
    Phase currentPhase = Phase::UNKNOWN;
    int currentPhaseId = -1;
    long long phaseIter = 0;
    double phaseStartTime = 0.0;
    string phaseName;
    
    long long globalIter = 0;
    double startTime = 0.0;
    double bestL = 1e18;
    
    map<int, map<OpType, OpStats>> phaseOpStats;
    vector<LSample> lCurve;
    vector<LDropRecord> lDrops;
    map<int, double> phaseTime;
    map<int, long long> phaseIterCount;
    
    vector<CycleDetail> cycleDetails;
    int currentCycleId = 0;
    
    double elapsed() const {
        return ::elapsed();
    }
    
    void flushLCurve() {
        if (lCurve.empty()) return;
        logFile << "[L_CURVE]\n";
        for (const auto& s : lCurve) {
            logFile << "  " << fixed << setprecision(3) << s.timestamp
                    << " " << s.phaseId
                    << " " << s.globalIter
                    << " " << setprecision(6) << s.L
                    << " " << s.bestL << "\n";
        }
        logFile << "[L_CURVE_END]\n\n";
        lCurve.clear();
    }
    
    void flushLDrops() {
        lDrops.clear();
    }
    
    void flushOpStats() {
        if (phaseOpStats.find(currentPhaseId) == phaseOpStats.end()) return;
        logFile << "[OP_STATS] phase=" << currentPhaseId << "\n";
        const char* opNames[] = {"SWAP", "MOVE", "JUMP", "RANDOM_JUMP"};
        for (int i = 0; i < static_cast<int>(OpType::COUNT); i++) {
            OpType op = static_cast<OpType>(i);
            const auto& stats = phaseOpStats[currentPhaseId][op];
            if (stats.totalCalls == 0) continue;
            logFile << "  " << opNames[i]
                    << " total=" << stats.totalCalls
                    << " accepted=" << stats.acceptedCalls
                    << " rate=" << fixed << setprecision(4) << stats.acceptRate() << "\n";
        }
        logFile << "[OP_STATS_END]\n\n";
        phaseOpStats[currentPhaseId].clear();
    }
    
    void flushCycleDetails() {
        if (cycleDetails.empty()) return;
        logFile << "[CYCLE_DETAILS]\n";
        for (const auto& cd : cycleDetails) {
            logFile << "  cycle=" << cd.cycleId
                    << " T_init=" << scientific << setprecision(4) << cd.T_init
                    << " T_end=" << cd.T_end
                    << " C=" << fixed << cd.C
                    << " feasible=" << (cd.feasibleStart ? 1 : 0) << "->" << (cd.feasibleEnd ? 1 : 0)
                    << " L=" << setprecision(4) << cd.L_start << "->" << cd.L_end
                    << " penalty=" << cd.penalty_start << "->" << cd.penalty_end
                    << " iters=" << cd.iterInCycle << "\n";
        }
        logFile << "[CYCLE_DETAILS_END]\n\n";
        cycleDetails.clear();
    }
    
public:
    void init(const string& filename, const LoggerConfig& cfg = LoggerConfig()) {
        config = cfg;
        if (!config.enabled) return;
        logFile.open(filename);
        if (!logFile.is_open()) {
            fprintf(stderr, "Logger: 无法打开日志文件 %s\n", filename.c_str());
            return;
        }
        startTime = elapsed();
        initialized = true;
        logFile << "# SA Logger Output\n";
        logFile << "# Start Time: " << startTime << "\n";
        logFile << "# ====================\n\n";
        logFile.flush();
    }
    
    void close() {
        if (!initialized) return;
        flush();
        logFile.close();
        initialized = false;
    }
    
    bool isInitialized() const { return initialized; }
    bool isEnabled() const { return initialized && config.enabled; }
    
    void enterPhase(Phase phase, const string& name) {
        if (!isEnabled()) return;
        if (currentPhase != Phase::UNKNOWN) exitPhase();
        currentPhase = phase;
        currentPhaseId = static_cast<int>(phase);
        phaseName = name;
        phaseIter = 0;
        phaseStartTime = elapsed();
        logFile << "[PHASE_ENTER] " << currentPhaseId << " " << name 
                << " time=" << phaseStartTime << "\n";
        logFile.flush();
    }
    
    void exitPhase() {
        if (!isEnabled() || currentPhase == Phase::UNKNOWN) return;
        double duration = elapsed() - phaseStartTime;
        phaseTime[currentPhaseId] = duration;
        phaseIterCount[currentPhaseId] = phaseIter;
        logFile << "[PHASE_EXIT] " << currentPhaseId 
                << " iters=" << phaseIter 
                << " duration=" << duration << "\n\n";
        flushOpStats();
        currentPhase = Phase::UNKNOWN;
        currentPhaseId = -1;
        logFile.flush();
    }
    
    Phase getCurrentPhase() const { return currentPhase; }
    
    void incIter() { globalIter++; phaseIter++; }
    long long getGlobalIter() const { return globalIter; }
    long long getPhaseIter() const { return phaseIter; }
    
    void logOpCall(OpType op, bool accepted) {
        if (!isEnabled()) return;
        auto& stats = phaseOpStats[currentPhaseId][op];
        stats.totalCalls++;
        if (accepted) stats.acceptedCalls++;
    }
    
    void logLValue(double L) {
        if (!isEnabled()) return;
        if (globalIter % config.lCurveSampleInterval != 0) return;
        lCurve.push_back({elapsed(), currentPhaseId, globalIter, L, bestL});
        if (lCurve.size() >= 1000) flushLCurve();
    }
    
    void logLDrop(double oldL, double newL) {
        if (!isEnabled()) return;
        double delta = oldL - newL;
        if (delta <= 0) return;
        lDrops.push_back({currentPhaseId, globalIter, phaseIter, oldL, newL, delta, elapsed()});
        logFile << "[L_DROP] phase=" << currentPhaseId
                << " global_iter=" << globalIter
                << " phase_iter=" << phaseIter
                << " old=" << fixed << setprecision(6) << oldL
                << " new=" << newL
                << " delta=" << delta
                << " time=" << elapsed() << "\n";
        logFile.flush();
    }
    
    void updateBestL(double L) { if (L < bestL) bestL = L; }
    double getBestL() const { return bestL; }
    
    void enterCycle(int cycleId, double T_init, double T_end, double C,
                    bool feasible, double L, double penalty) {
        if (!isEnabled()) return;
        currentCycleId = cycleId;
        if (config.enableCycleDetail && cycleId % config.cycleDetailInterval == 0) {
            CycleDetail cd;
            cd.cycleId = cycleId;
            cd.T_init = T_init;
            cd.T_end = T_end;
            cd.C = C;
            cd.feasibleStart = feasible;
            cd.L_start = L;
            cd.penalty_start = penalty;
            cd.iterInCycle = 0;
            cycleDetails.push_back(cd);
        }
    }
    
    void exitCycle(bool feasible, double L, double penalty) {
        if (!isEnabled()) return;
        if (!cycleDetails.empty() && cycleDetails.back().cycleId == currentCycleId) {
            auto& cd = cycleDetails.back();
            cd.feasibleEnd = feasible;
            cd.L_end = L;
            cd.penalty_end = penalty;
            cd.opStats = phaseOpStats[currentPhaseId];
        }
        if (cycleDetails.size() >= 10) flushCycleDetails();
    }
    
    void flush() {
        if (!initialized) return;
        flushLCurve();
        flushLDrops();
        flushOpStats();
        flushCycleDetails();
        logFile.flush();
    }
    
    const map<int, map<OpType, OpStats>>& getOpStats() const { return phaseOpStats; }
    const vector<LDropRecord>& getLDrops() const { return lDrops; }
};

static SALogger g_saLogger;

#define LOG_INIT(fn) g_saLogger.init(fn)
#define LOG_CLOSE() g_saLogger.close()
#define LOG_PHASE_ENTER(phase, name) g_saLogger.enterPhase(phase, name)
#define LOG_PHASE_EXIT() g_saLogger.exitPhase()
#define LOG_ITER() g_saLogger.incIter()
#define LOG_OP_CALL(op, accepted) g_saLogger.logOpCall(op, accepted)
#define LOG_L(L) g_saLogger.logLValue(L)
#define LOG_L_DROP(oldL, newL) g_saLogger.logLDrop(oldL, newL)
#define LOG_BEST(L) g_saLogger.updateBestL(L)
#define LOG_CYCLE_ENTER(cid, Ti, Te, C, feas, L, pen) \
    g_saLogger.enterCycle(cid, Ti, Te, C, feas, L, pen)
#define LOG_CYCLE_EXIT(feas, L, pen) g_saLogger.exitCycle(feas, L, pen)
#define LOG_FLUSH() g_saLogger.flush()
```

## 四、侵入式修改清单

在以下位置添加日志调用，总计约 20 行：

### 4.1 main.cpp

```cpp
int main(int argc, char *argv[]) {
    // ... 现有代码 ...
    
    LOG_INIT("dist/logs/sa_main.log");  // 新增
    
    Polygon feasiblePoly; Boundary bd; vector<Polygon> P;
    readInput(feasiblePoly, bd, P);
    // ... 现有代码 ...
    
    LOG_PHASE_ENTER(Phase::PERM_SA, "saOnPerm");  // 新增
    saOnPerm(P, perm, bd);
    LOG_PHASE_EXIT();  // 新增
    
    clampAll(P, bd);
    aabbPullback(P, bd);
    buildNFPMatrix(P);
    
    {
        double L0 = calcLPrime(P);
        LOG_PHASE_ENTER(Phase::POS_SA_HIGH, "positionSAHigh");  // 新增
        positionSAHigh(P, bd, L0 * 0.03, L0 * 0.001, T_SA_HIGH_END);
        LOG_PHASE_EXIT();  // 新增
    }
    
    // ... 其他阶段同理 ...
    
    LOG_CLOSE();  // 新增
    return 0;
}
```

### 4.2 sa.h 中的 SA 函数

在 `positionSAHigh` 的循环中：

```cpp
while (true) {
    LOG_ITER();  // 新增
    
    // 在周期开始处
    if (iteration_in_cycle == 1) {
        LOG_CYCLE_ENTER(cycleId, T_init_cycle, T_end_cycle, C, 
                        globalFeasible, calcLPrime(P), penaltyValue);  // 新增
    }
    
    // ... 算子调用 ...
    
    double randV = randDouble();
    if (randV < 0.20 && n >= 2) {
        bool accepted = saSwapOp(...);
        LOG_OP_CALL(OpType::SWAP, accepted);  // 新增
    } else if (...) {
        bool accepted = saJumpOp(...);
        LOG_OP_CALL(OpType::JUMP, accepted);  // 新增
    }
    // ... 其他算子同理 ...
    
    // 在周期结束处
    if (iteration_in_cycle == iterPerCycle) {
        LOG_CYCLE_EXIT(globalFeasible, calcLPrime(P), calcOverlapPenalty(P,n));  // 新增
    }
    
    // L 值采样
    if (iteration % 1000 == 0) {
        LOG_L(calcLPrime(P));  // 新增
    }
    
    // 最优解更新
    if (curCost < bestCost) {
        // ... 现有更新逻辑 ...
        LOG_L_DROP(oldBestCost, curCost);  // 新增 (如有 L 下降)
    }
}
```

## 五、输出文件格式说明

详见 `docs/spec/log_format.md`。

## 六、文件组织结构

```
src/
├── logger.h      # Logger 类定义与实现 (Header-Only)
├── main.cpp      # 添加 LOG 宏调用
└── sa.h          # 添加 LOG 宏调用

docs/spec/
└── log_format.md # 输出格式文档

dist/logs/        # 日志输出目录
└── sa_main.log   # 主日志文件
```

## 七、实现优先级

| 优先级 | 内容 | 预计工作量 |
|-------|------|-----------|
| P0 | Logger 基础框架 + 算子接受率统计 | 2h |
| P0 | L 曲线采样 + L 下降记录 | 1h |
| P0 | 阶段切换管理 | 0.5h |
| P1 | log_format.md 文档 | 0.5h |
| P1 | positionSAHigh 周期详情 | 1h |
| P2 | Python 分析脚本 | 2h |

## 八、性能影响评估

| 操作 | 开销 | 频率 | 总影响 |
|-----|------|-----|-------|
| logOpCall | ~10ns | 每次算子调用 | <0.01% |
| logLValue | ~100ns | 每1000次迭代 | <0.001% |
| logLDrop | ~1μs | 下降时 | <0.1% |
| 文件 flush | ~1ms | 每10000次迭代 | <0.5% |

总体性能影响预估 **<1%**。
