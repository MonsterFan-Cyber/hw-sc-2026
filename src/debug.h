#pragma once
#include <cstdio>
#include <ctime>
#include <chrono>

extern double elapsed();
extern long long g_cntGetCandidates, g_sumCands, g_cntFastOverlap, g_cntExactOverlap, g_cntShelfPlace;


inline long long g_cntExactOverlap  = 0;
inline long long g_cntFastOverlap   = 0;
inline long long g_cntGetCandidates = 0;
inline long long g_sumCands         = 0;
inline long long g_cntShelfPlace    = 0;


#define DEBUG_SA_STATS_PARAM
#define DEBUG_SA_STATS_ARG


