#pragma once

// ═══════════════════════════════════════════════════════════════
// 常量
// ═══════════════════════════════════════════════════════════════
const double EXPAND_EPS    = 4e-5;
const double AABB_PREC     = 1e-4;
const double EXACT_PREC    = 1e-5;
const double SA_PERM_ALPHA = 0.9999;
const double SA_TMIN       = 1e-6;
const double WALL_EPS      = 1e-4;
const double TIME_SCALE = 1;
const double JUMP_SIZE_WEIGHT = 1.0;
// 各阶段时间节点（秒）
const double T_SA_PERM      = 22.0*TIME_SCALE ;
const double T_AABB_PULL    = 23.0*TIME_SCALE;
const double T_SA_HIGH_END  = 46.0*TIME_SCALE;
const double T_EXACT_MID    = 47.5*TIME_SCALE;
const double T_SA_LOW_END   = 57.0*TIME_SCALE;
const double T_EXACT_FINAL  = 59.5*TIME_SCALE;

// positionSAHigh 参数
// JUMP 算子
const double JUMP_BEGIN_PROG_HIGH = 0.5;
const double JUMP_BACK_PROB_HIGH = 0.1;
// MOVE 算子
const double MOVE_BACK_PROB_HIGH = 0.25;
// SWAP 算子

// 循环退火
const int RONND_PRE_CYC = 2500;
// 
// NFP
// 是否开启内环
const bool OPEN_INNER = true;
// 下面两个条件满足一个即可
// 两个多边形包围盒面积占比 min/max = 0.02~0.1 ，填 1 表示全开
const double OPEN_INNER_AREA_RATIO = 0.1;
// 开启内环的条件，两多边形顶点数乘积小于某个值，则也可以开启内环
const double OPEN_INNER_VERTEX_NUMBER = 50*50;
