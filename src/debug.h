#pragma once
#include <cstdio>
#include <ctime>
#include <chrono>

extern double elapsed();
extern long long g_cntGetCandidates, g_sumCands, g_cntFastOverlap, g_cntExactOverlap, g_cntShelfPlace;

// ═══════════════════════════════════════════════════════════════
// ★ 调试日志系统（仅 DEBUG_LOG 时编译）
// ═══════════════════════════════════════════════════════════════

inline long long g_cntExactOverlap  = 0;
inline long long g_cntFastOverlap   = 0;
inline long long g_cntGetCandidates = 0;
inline long long g_sumCands         = 0;
inline long long g_cntShelfPlace    = 0;

// ═══════════════════════════════════════════════════════════════
// DebugSAStats — SA调试统计结构体
// ═══════════════════════════════════════════════════════════════
#ifdef DEBUG_LOG
struct DebugSAStats {
    long long dbgIter;
    double    dbgBestL;
    long long snapSwapTrials, snapSwapAccepts;
    long long snapMoveTrials, snapMoveAccepts;
    long long snapSlideTrials, snapSlideAccepts;
    long long snapJumpTrials, snapJumpAccepts;
    long long snapRandomJumpTrials, snapRandomJumpAccepts;
    long long snapIters;
    double    lastT, lastStep;
    double    lastSnapshotTime;
    const char* phaseName;
};
#endif

#ifdef DEBUG_LOG
  #define DEBUG_SA_STATS_PARAM  , DebugSAStats& dbg
  #define DEBUG_SA_STATS_ARG    , dbg
#else
  #define DEBUG_SA_STATS_PARAM
  #define DEBUG_SA_STATS_ARG
#endif

#ifdef DEBUG_LOG
// ── 原有细节日志（debug_log.txt）──────────────────────────────
static FILE* g_logFile = nullptr;

static void logInitFile() {
    g_logFile = fopen("debug_log.txt", "w");
    if (!g_logFile) { fprintf(stderr, "[DEBUG_LOG] 无法创建 debug_log.txt\n"); return; }
    fprintf(g_logFile,
        "# debug_log.txt — 多边形重叠消除调试日志 v3\n"
        "# phase / iter / round / elapsed / cur_L / best_L / tag\n#\n");
    fflush(g_logFile);
}
static void logCloseFile() {
    if (g_logFile) { fflush(g_logFile); fclose(g_logFile); g_logFile = nullptr; }
}
static void logPhaseHeader(const char* phaseName) {
    if (!g_logFile) return;
    fprintf(g_logFile,
        "\n================================================\nPhase: %s\n================================================\n",
        phaseName);
    fflush(g_logFile);
}
static void logLine(const char* phase, long long iter, int round,
                    double elapsedSec, double curL, double bestL, const char* tag) {
    if (!g_logFile) return;
    if (iter >= 0 && round >= 0)
        fprintf(g_logFile, "phase=%-24s  iter=%-10lld  round=%-4d  elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, iter, round, elapsedSec, curL, bestL, tag);
    else if (iter >= 0)
        fprintf(g_logFile, "phase=%-24s  iter=%-10lld             elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, iter, elapsedSec, curL, bestL, tag);
    else
        fprintf(g_logFile, "phase=%-24s  round=%-4d               elapsed=%8.4f  cur_L=%12.6f  best_L=%12.6f  [%s]\n",
                phase, round, elapsedSec, curL, bestL, tag);
    fflush(g_logFile);
}
static void logPhaseSummary(const char* phaseName, long long totalIter, int totalRound, double elapsedSec) {
    if (!g_logFile) return;
    if (totalIter >= 0)
        fprintf(g_logFile, ">>> %s  total_iters=%lld  elapsed_at_end=%.4f\n", phaseName, totalIter, elapsedSec);
    else
        fprintf(g_logFile, ">>> %s  total_rounds=%d  elapsed_at_end=%.4f\n", phaseName, totalRound, elapsedSec);
    fflush(g_logFile);
}

// ── 新增：计时日志（timer_log.txt）────────────────────────────
static FILE* g_timerFile = nullptr;

template<typename B>
static void timerInitFile(int n, const B& bd) {
    g_timerFile = fopen("timer_log.txt", "w");
    // g_timerFile = fopen("timer_log.txt", "a");
    if (!g_timerFile) { fprintf(stderr, "[DEBUG_LOG] 无法创建 timer_log.txt\n"); return; }

    time_t now_t = time(nullptr);
    struct tm* tm_info = localtime(&now_t);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info);

    double bw = bd.xMax - bd.xMin;
    double bh = bd.yMax - bd.yMin;

    fprintf(g_timerFile,
        "\n"
        "════════════════════════════════════════════════════════\n"
        "  Run start : %s\n"
        "  n         = %d 个多边形\n"
        "  boundary  = x∈[%.4f, %.4f]  y∈[%.4f, %.4f]\n"
        "  宽×高     = %.4f × %.4f  面积=%.4f\n"
        "════════════════════════════════════════════════════════\n"
        "%-12s  %-28s  %s\n"
        "%-12s  %-28s  %s\n",
        time_str,
        n,
        bd.xMin, bd.xMax, bd.yMin, bd.yMax,
        bw, bh, bw * bh,
        "elapsed(s)", "event", "detail",
        "------------", "----------------------------", "------"
    );
    fflush(g_timerFile);
}

static void timerCloseFile() {
    if (g_timerFile) { fflush(g_timerFile); fclose(g_timerFile); g_timerFile = nullptr; }
}

static void timerPhaseStart(const char* name, double lBefore) {
    if (!g_timerFile) return;
    fprintf(g_timerFile, "%12.4f  [PHASE  start] %-20s  L'_before=%12.6f\n",
            elapsed(), name, lBefore);
    fflush(g_timerFile);
}

static void timerPhaseEnd(const char* name, double tStart, double lBefore, double lAfter,
                          long long iters = -1, int rounds = -1) {
    if (!g_timerFile) return;
    double now = elapsed();
    double cost = now - tStart;
    double gain = lBefore - lAfter;
    double gainPct = (lBefore > 1e-9) ? gain / lBefore * 100.0 : 0.0;

    if (iters >= 0)
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)  iters=%lld\n",
            now, name, cost, lBefore, lAfter, gain, gainPct, iters);
    else if (rounds >= 0)
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)  rounds=%d\n",
            now, name, cost, lBefore, lAfter, gain, gainPct, rounds);
    else
        fprintf(g_timerFile,
            "%12.4f  [PHASE  end  ] %-20s  cost=%7.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f(%+.2f%%)\n",
            now, name, cost, lBefore, lAfter, gain, gainPct);
    fflush(g_timerFile);
}

static void timerRound(const char* funcName, int round,
                       double tRoundStart, double lBefore, double lAfter) {
    if (!g_timerFile) return;
    double now = elapsed();
    double roundCost = now - tRoundStart;
    double gain = lBefore - lAfter;
    fprintf(g_timerFile,
        "%12.4f  [ROUND  r=%-4d] %-20s  round_cost=%6.3fs  L'_before=%12.6f  L'_after=%12.6f  gain=%10.6f\n",
        now, round, funcName, roundCost, lBefore, lAfter, gain);
    fflush(g_timerFile);
}

static void timerSASnapshot(const char* phaseName, double T, double step,
                            double curCost, double bestCost, long long iters,
                            long long swapTrials, long long swapAccepts,
                            long long moveTrials, long long moveAccepts,
                            long long slideTrials = 0, long long slideAccepts = 0,
                            long long jumpTrials = 0, long long jumpAccepts = 0) {
    if (!g_timerFile) return;
    double swapRate  = (swapTrials  > 0) ? swapAccepts  * 100.0 / swapTrials  : 0.0;
    double moveRate  = (moveTrials  > 0) ? moveAccepts  * 100.0 / moveTrials  : 0.0;
    double slideRate = (slideTrials > 0) ? slideAccepts * 100.0 / slideTrials : 0.0;
    double jumpRate  = (jumpTrials  > 0) ? jumpAccepts  * 100.0 / jumpTrials  : 0.0;
    fprintf(g_timerFile,
        "%12.4f  [SNAPSHOT     ] %-20s"
        "  T=%10.6f  step=%8.6f"
        "  curCost=%12.6f  bestCost=%12.6f  iters=%9lld"
        "  swap_acc=%.1f%%(%lld/%lld)"
        "  move_acc=%.1f%%(%lld/%lld)"
        "  slide_acc=%.1f%%(%lld/%lld)"
        "  jump_acc=%.1f%%(%lld/%lld)\n",
        elapsed(), phaseName,
        T, step,
        curCost, bestCost, iters,
        swapRate, swapAccepts, swapTrials,
        moveRate, moveAccepts, moveTrials,
        slideRate, slideAccepts, slideTrials,
        jumpRate, jumpAccepts, jumpTrials);
    fflush(g_timerFile);
}

static void timerHotspotSummary() {
    if (!g_timerFile) return;
    double avgCands = (g_cntGetCandidates > 0) ? (double)g_sumCands / g_cntGetCandidates : 0.0;
    fprintf(g_timerFile,
            "\n%s\n"
            "# [HOTSPOT] 热点操作统计\n"
            "%s\n"
            "  fastOverlap   calls : %lld\n"
            "  exactOverlap  calls : %lld\n"
            "  getCandidates calls : %lld  (avg_cands_per_call=%.2f)\n"
            "  shelfPlace    calls : %lld\n",
            "================================================",
            "================================================",
            g_cntFastOverlap,
        g_cntExactOverlap,
        g_cntGetCandidates, avgCands,
        g_cntShelfPlace);
    fflush(g_timerFile);
}

#endif // DEBUG_LOG
